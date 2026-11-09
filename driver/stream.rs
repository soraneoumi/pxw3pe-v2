// SPDX-License-Identifier: GPL-2.0

//! TS DMA engine of the bridge, its interrupt, packet routing and descrambling.
//!
//! Each bridge port carries the streams of the demodulator cores wired to it into two ping-pong
//! DMA buffers; on this board all four tuners arrive on one port (`ts::Routing`). The hard
//! interrupt handler only acknowledges the interrupt; the threaded handler copies each completed
//! buffer out, re-arms the channel, and routes, descrambles and delivers the packets.
//!
//! The bridge has no input buffer worth the name: once a channel has completed, the other one
//! fills, and when that one completes too before the first is re-armed, packets are lost. The
//! slack is therefore one transfer, and the transfer length is chosen so that it lasts
//! [`TARGET_US`] at the rate the port is seeing (`next_length`). On an idle recording server
//! the interrupt thread was measured to run up to 17 ms after its interrupt, behind kernel work
//! that is not preempted (display connector polling, for one); 58 KB transfers lose packets
//! then. A port too slow to fill even the shortest transfer within [`TARGET_US`] has its packets
//! delivered while the transfer fills, by the stream's periodic poll (`platform::Poll`). The
//! bridge reports lost input with an interrupt of its own, which is counted and logged.

use crate::{
    dvb::Feed,
    hw::Board,
    platform::{self, mb, IrqLock, LatencyLimit},
    proto::ts::{self, Descrambler, Port, Routing, TRANSFER, TUNERS},
};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use kernel::{
    device::{Bound, Device},
    dma::CoherentAllocation,
    irq::{IrqReturn, ThreadedHandler, ThreadedIrqReturn},
    new_mutex,
    prelude::*,
    sync::{aref::ARef, Arc, Mutex},
    time::{delay::fsleep, Delta},
};

/// Interrupt registers (INT_STATUS, INT_STATUS_CLR, INT_ENA_CLR, INT_ENA_SET). A posted clear is
/// flushed by reading the status register back; 0x810 is INT_STATUS_SET and must not be written.
const IRQ_STATUS: usize = 0x808;
const IRQ_CLEAR: usize = 0x80C;
const IRQ_DISABLE: usize = 0x818;
const IRQ_ENABLE: usize = 0x81C;

/// Interrupt bit of an input overflow (DATA_BUF_OVERFLOW_INT): the bridge raises it when packets
/// arrive that neither channel of a running port can take, that is, when packets are lost. Measured
/// on the card: never raised idle or while the DMA keeps up, and raised once or twice per episode
/// of input lost on purpose. It is not tied to a port, so it is enabled while any port runs.
const IRQ_OVERFLOW: u32 = 1 << 8;

/// Period of the stream's poll, in milliseconds: packets of a slow port wait at most about
/// [`TARGET_US`] plus this.
pub(crate) const POLL_MS: u32 = 50;

/// Channel control: transfer length in bits 21:0, TLP transfer unit 7 (188 bytes) in bits 24:22,
/// TLP TD in bit 28; with the length 0xE520 the value is 0x11C0E520 (the revealed value). The
/// length may differ from one arming to the next (measured).
const fn control(length: usize) -> u32 {
    0x11C0_0000 | length as u32
}
const TRANSFERRED_MASK: u32 = 0x3F_FFFF;

/// Shortest transfer, in bytes: 312 packets (revealed), used for the first transfers after a start,
/// when the rate is not known yet, and on a port that carries little.
const MIN_TRANSFER: usize = 312 * ts::PACKET;

/// Shortest transfer the interrupt thread may lose when it runs late, in microseconds, and the
/// delay the packets of a slow port see at most. 100 ms covers the latencies measured on the
/// card's host several times over.
const TARGET_US: u64 = 100_000;

/// Limit on the time the CPUs may take to leave an idle state while a port's DMA runs, in
/// microseconds (the kernel's CPU latency QoS). The bridge keeps no packet in reserve: a DMA
/// write that waits for the memory path to wake from a deep idle state loses the packet that
/// arrives meanwhile, and the four tuners of a port can deliver one every 19 µs. On the card's
/// host (Core i3-9100) only the states that wake within this limit lose nothing: with its 70 µs
/// state allowed a few packets a minute were lost, with deeper ones hundreds a second whenever
/// the machine was otherwise idle.
const IDLE_EXIT_LATENCY_US: u32 = 20;

/// Time a port is given to settle after its DMA is stopped (revealed).
const STOP_SETTLE_MS: i64 = 100;

/// Polling period while waiting for the channels of a port to stop.
const IDLE_POLL_US: i64 = 100;

/// TS mode register bit that enables the TS input of a port.
const TS_INPUT_ENABLE: u32 = 0x80;

/// Registers of one DMA channel.
struct Channel {
    addr_lo: usize,
    addr_hi: usize,
    state: usize,
    control: usize,
}

/// Registers of one port.
pub(crate) struct PortRegs {
    channels: [Channel; 2],
    management: usize,
    ts_mode: usize,
    /// Interrupt status/enable bits of channel 0 and 1.
    irq: [u32; 2],
}

impl PortRegs {
    /// Interrupt bits of both channels.
    const fn irq_mask(&self) -> u32 {
        self.irq[0] | self.irq[1]
    }
}

/// DMA registers of both ports.
struct Layout {
    a: PortRegs,
    b: PortRegs,
}

const fn port(base: usize, management: usize, ts_mode: usize, irq: [u32; 2]) -> PortRegs {
    const fn ch(base: usize) -> Channel {
        Channel {
            addr_lo: base,
            addr_hi: base + 4,
            state: base + 8,
            control: base + 0xC,
        }
    }
    PortRegs {
        channels: [ch(base), ch(base + 0x10)],
        management,
        ts_mode,
        irq,
    }
}

/// The ASV5220's DMA ports A and B, their TS control registers and completion interrupt bits. The
/// alternate map (0xD00/0xD40) is the port C/D block of this layout, whose interrupts are bits 4..7
/// and not bits 0..3; the driver does not run on a bridge that needs it
/// (`bringup::probe_i2c_engine`).
const LAYOUT: Layout = Layout {
    a: port(0xAC0, 0xAE0, 0xA00, [1 << 0, 1 << 1]),
    b: port(0xC00, 0xC20, 0xB40, [1 << 2, 1 << 3]),
};

/// Completion interrupt bits of both ports.
const PORT_IRQS: u32 = LAYOUT.a.irq_mask() | LAYOUT.b.irq_mask();

impl Layout {
    fn port(&self, port: Port) -> &PortRegs {
        match port {
            Port::A => &self.a,
            Port::B => &self.b,
        }
    }
}

/// Management register bits of channel `ch`: enable (bit 0), transfer start (bit 1), the 2-bit
/// state field (bits 3:2, 1 while the channel is transferring) and the completion field (bits 5:4,
/// [`COMPLETED`], 2 = cancelled, 3 = ended by the latency timer, [`LATENCY_TIMERS`]). Channel 1
/// uses the same layout 16 bits up; its state field was seen at bits 19:18 on the card.
const fn enable_bit(ch: usize) -> u32 {
    1 << (16 * ch)
}
const fn run_bit(ch: usize) -> u32 {
    2 << (16 * ch)
}
const fn state_field(m: u32, ch: usize) -> u32 {
    (m >> (2 + 16 * ch)) & 3
}
const fn done_field(m: u32, ch: usize) -> u32 {
    (m >> (4 + 16 * ch)) & 3
}

/// Completion field: the transfer filled its armed length.
const COMPLETED: u32 = 1;

/// The latency timer fields of MANAGEMENT (bits 15:8 of channel 0, 31:24 of channel 1). With a
/// value N other than 0, the bridge ends a transfer that has taken data but is not full N times
/// about 8.4 ms after it began to fill, with completion field 3 (measured on the card). Such a
/// transfer usually ends between packets, but under load it can end at a 1 KiB boundary inside a
/// packet, and the rest of that packet is lost: the next transfer starts with a whole packet.
/// With 0 the timer is off and a transfer only ends full, so every start clears both fields: they
/// survive driver loads (a value written by a test stayed for days), and a stale one made the
/// port of a weak channel deliver nothing, every transfer ending early and being taken for a
/// failure.
const LATENCY_TIMERS: u32 = 0xFF00_FF00;

/// Clears the run and enable bits of both channels of a port, whatever state they are in: the
/// stopping only channel 0, and only when its state field reads 1, would leave the waiting
/// ping-pong channel, which need not be in that state, to start again at its turn.
///
/// The status fields are not write-1-to-clear (measured), so writing the register back as read
/// leaves them alone; it is still read immediately before the write to keep the window in which
/// the hardware can change the start bits short.
fn halt_channels(board: &Board, regs: &PortRegs) -> Option<()> {
    let m = board.read32(regs.management)?;
    let off = run_bit(0) | enable_bit(0) | run_bit(1) | enable_bit(1);
    board.write32(m & !off, regs.management)
}

/// Why a port could not be stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StopError {
    /// The device is gone, so it does not transfer any more.
    Gone,
    /// A channel was still running after [`STOP_SETTLE_MS`]; it may still write to memory.
    Timeout,
}

/// Waits until neither channel of a port is in the running state, but at most
/// [`STOP_SETTLE_MS`]; the wait ends early as soon as the port is idle so that the caller does not
/// sleep a fixed time.
fn wait_idle(board: &Board, regs: &PortRegs) -> Result<(), StopError> {
    let deadline = board.now_us() + (STOP_SETTLE_MS * 1000) as u64;
    loop {
        let m = board.read32(regs.management).ok_or(StopError::Gone)?;
        if (0..2).all(|ch| state_field(m, ch) != 1) {
            return Ok(());
        }
        if board.now_us() >= deadline {
            return Err(StopError::Timeout);
        }
        fsleep(Delta::from_micros(IDLE_POLL_US));
    }
}

/// Stops the DMA of every port and the interrupts of the card, and disables the TS inputs. Safe
/// before a [`Stream`] exists, so it can bring a card that a previous user left running into a
/// known state before bus mastering is enabled, and stop the device before its DMA buffer is
/// freed.
///
/// Every port is given the chance to stop even if another one did not; the result reports a
/// timeout of any of them, unless the device is gone.
pub(crate) fn shutdown(board: &Board) -> Result<(), StopError> {
    let ports = [&LAYOUT.a, &LAYOUT.b];
    for regs in ports {
        halt_channels(board, regs).ok_or(StopError::Gone)?;
    }
    board.write32(0xFFFF, IRQ_DISABLE).ok_or(StopError::Gone)?;
    let mut result = Ok(());
    for regs in ports {
        let mode = board.read32(regs.ts_mode).ok_or(StopError::Gone)?;
        board
            .write32(mode & !TS_INPUT_ENABLE, regs.ts_mode)
            .ok_or(StopError::Gone)?;
        match wait_idle(board, regs) {
            Err(StopError::Gone) => return Err(StopError::Gone),
            Err(StopError::Timeout) => result = Err(StopError::Timeout),
            Ok(()) => {}
        }
    }
    // Only now: a transfer that was still running when the channels were halted completes
    // while waiting and raises its status bit.
    let status = board.read32(IRQ_STATUS).ok_or(StopError::Gone)?;
    board.write32(status, IRQ_CLEAR).ok_or(StopError::Gone)?;
    result
}

/// Counts an event that forces a port restart and warns when the count reaches a power of two,
/// so that a persistent fault does not flood the log.
fn count_fault(port: Port, counter: &mut u32, what: &str) {
    *counter = counter.wrapping_add(1);
    if counter.is_power_of_two() {
        pr_warn!(
            "port {:?}: {} ({} so far), restarting the port\n",
            port,
            what,
            counter
        );
    }
}

/// Mutable state of one port, guarded by its mutex.
struct PortState {
    /// DMA running.
    running: bool,
    /// The stream was shut down for good: the port cannot be started any more.
    halted: bool,
    /// Per tuner: streaming, and where to.
    sinks: [Option<Feed>; TUNERS],
    /// Copy of the last completed DMA buffer.
    scratch: KVec<u8>,
    /// Packets collected for each tuner from one transfer; empty buffers for the tuners that are
    /// not wired to this port.
    out: [KVec<u8>; TUNERS],
    /// Channel whose completion is expected next. The hardware alternates the two channels, so
    /// the transfers are only in stream order when they are delivered in that order.
    next: usize,
    /// When the port's DMA was last stopped (`board.now_us()`), until the next start has waited
    /// out the settle time.
    stopped_at_us: Option<u64>,
    /// Transfers that completed with a bad status or length and caused a restart.
    bad_transfers: u32,
    /// Transfers in which fewer than half of the packets carry a known tag, which caused a
    /// restart: the data is not packet aligned.
    lost_sync: u32,
    /// Starts refused because a channel of the port was still running.
    busy_starts: u32,
    /// Length of the transfer each channel is armed with, in bytes; a completion with any other
    /// length is a failed transfer.
    armed_length: [usize; 2],
    /// Length the next arming uses: what the port delivered in [`TARGET_US`] at the rate of
    /// the last completed transfer, within [`MIN_TRANSFER`] and [`TRANSFER`].
    next_length: usize,
    /// When the last transfer was served (`board.now_us()`), for the rate; 0 after a start.
    last_served_us: u64,
    /// Bytes of channel `next`'s transfer that the poll has delivered while it was filling, and
    /// how many of their packets carried a tag; the sync check covers the whole transfer.
    delivered: usize,
    delivered_tagged: usize,
    /// When channel `next` began to fill (`board.now_us()`): at the start of the port or at the
    /// other channel's completion.
    filling_since_us: u64,
    /// Held while the port's DMA runs ([`IDLE_EXIT_LATENCY_US`]).
    latency: LatencyLimit,
}

impl PortState {
    fn new(routing: &Routing, port: Port) -> Result<Self> {
        let mut scratch = KVec::with_capacity(TRANSFER, GFP_KERNEL)?;
        scratch.resize(TRANSFER, 0, GFP_KERNEL)?;
        // Each buffer has room for a whole transfer, so that `dispatch` never allocates.
        let buffer = |tuner| {
            if routing.port(tuner) == port {
                KVec::with_capacity(TRANSFER, GFP_KERNEL)
            } else {
                Ok(KVec::new())
            }
        };
        Ok(PortState {
            running: false,
            halted: false,
            sinks: [None; TUNERS],
            scratch,
            out: [buffer(0)?, buffer(1)?, buffer(2)?, buffer(3)?],
            next: 0,
            stopped_at_us: None,
            bad_transfers: 0,
            lost_sync: 0,
            busy_starts: 0,
            armed_length: [MIN_TRANSFER; 2],
            next_length: MIN_TRANSFER,
            last_served_us: 0,
            delivered: 0,
            delivered_tagged: 0,
            filling_since_us: 0,
            latency: LatencyLimit::new()?,
        })
    }

    /// Records a served transfer of `length` bytes at `now` and sets the length of the next
    /// arming from the rate it shows. Two completions served together are the same moment: the
    /// second says nothing about the rate.
    fn note_served(&mut self, length: usize, now: u64) {
        if self.last_served_us != 0 && now > self.last_served_us {
            let packets = (length / ts::PACKET) as u64;
            let wanted = packets * TARGET_US / (now - self.last_served_us);
            self.next_length = (wanted as usize * ts::PACKET).clamp(MIN_TRANSFER, TRANSFER)
                / ts::PACKET
                * ts::PACKET;
        }
        self.last_served_us = now;
    }
}

/// The TS path of one card.
#[pin_data]
pub(crate) struct Stream {
    board: Arc<Board>,
    /// Four transfer buffers: port A channel 0/1, then port B channel 0/1 (the revealed layout).
    dma: DmaBuffers,
    #[pin]
    port_a: Mutex<PortState>,
    #[pin]
    port_b: Mutex<PortState>,
    routing: Routing,
    /// Status bits taken by the hard handler and not yet processed by the thread.
    pending: AtomicU32,
    /// Status bits of the channels whose interrupts are enabled, which the hard handler may
    /// claim. The hardware is not relied on to keep masked completions out of the status
    /// register (on the card it did).
    armed: AtomicU32,
    /// Makes the hard handler's claim (status read, `armed`, clear, publication to `pending`)
    /// one step with respect to a port's arming and disarming, which change the status register,
    /// `armed` and `pending` together. The atomics alone cannot: another CPU's status read is
    /// not ordered against this CPU's posted clear, and the handler could publish a bit it read
    /// before a stop after the following start had cleared `pending`.
    irq_lock: IrqLock,
    /// For log messages.
    dev: ARef<Device>,
    descrambler: Descrambler,
    /// Per tuner: its packets are dropped instead of delivered. Set by the frontend from a tune
    /// until the demodulator has settled on the new signal, because what it emits before then
    /// (while acquiring, and for about a second after a gain change) has packets missing.
    muted: [AtomicBool; TUNERS],
    /// Input overflows ([`IRQ_OVERFLOW`]) counted by the hard handler, and the count the poll
    /// last looked at for its log message.
    overflows: AtomicU32,
    overflows_seen: AtomicU32,
}

/// The DMA buffers, shareable with the interrupt thread.
struct DmaBuffers(CoherentAllocation<u8>);

// SAFETY: `CoherentAllocation` is only `Send` because its contents may be accessed through shared
// references. Here the contents are only read through `as_slice`, under the mutex of the port that
// owns the buffer, for bytes the device has written and does not write again before the channel
// is re-armed: a buffer whose channel has completed, or the part of a filling buffer below the
// channel's TRANS_STATE. Every other method used only reads fields fixed at allocation.
unsafe impl Sync for DmaBuffers {}

/// Index of the first buffer of `port` in the DMA allocation.
fn buffer_index(port: Port) -> usize {
    match port {
        Port::A => 0,
        Port::B => 2,
    }
}

impl Stream {
    /// Allocates the DMA buffers. The device's DMA mask must already be set to 32 bits, since
    /// the engine's high address registers are not used.
    pub(crate) fn new(
        dev: &Device<Bound>,
        board: Arc<Board>,
        routing: Routing,
        descrambler: Descrambler,
    ) -> Result<Arc<Self>> {
        let dma = CoherentAllocation::<u8>::alloc_coherent(dev, 4 * TRANSFER, GFP_KERNEL)?;
        Arc::pin_init(
            try_pin_init!(Stream {
                board,
                dma: DmaBuffers(dma),
                port_a <- new_mutex!(PortState::new(&routing, Port::A)?),
                port_b <- new_mutex!(PortState::new(&routing, Port::B)?),
                routing,
                pending: AtomicU32::new(0),
                armed: AtomicU32::new(0),
                irq_lock: IrqLock::new()?,
                dev: dev.into(),
                descrambler,
                muted: [const { AtomicBool::new(false) }; TUNERS],
                overflows: AtomicU32::new(0),
                overflows_seen: AtomicU32::new(0),
            }),
            GFP_KERNEL,
        )
    }

    /// Drops (`true`) or delivers (`false`) the packets of `tuner` from the next transfer on.
    pub(crate) fn set_muted(&self, tuner: usize, muted: bool) {
        self.muted[tuner].store(muted, Ordering::Relaxed);
    }

    fn state(&self, port: Port) -> &Mutex<PortState> {
        match port {
            Port::A => &self.port_a,
            Port::B => &self.port_b,
        }
    }

    fn read(&self, offset: usize) -> Result<u32> {
        self.board.read32(offset).ok_or(ENODEV)
    }

    fn write(&self, value: u32, offset: usize) -> Result {
        self.board.write32(value, offset).ok_or(ENODEV)
    }

    /// Starts delivering the packets of `tuner` to `feed`, starting the port's DMA if needed.
    pub(crate) fn start(&self, tuner: usize, feed: Feed) -> Result {
        let port = self.routing.port(tuner);
        let result = self.attach(port, tuner, feed);
        // Completions that arrived while the port was locked were left to be served now. This
        // only fails once the device is gone, when there is nothing left to do.
        let _ = self.service(port);
        result
    }

    fn attach(&self, port: Port, tuner: usize, feed: Feed) -> Result {
        let mut st = self.state(port).lock();
        if st.halted {
            return Err(ENODEV);
        }
        if !st.running {
            self.start_dma(port, &mut st)?;
            st.running = true;
        }
        st.sinks[tuner] = Some(feed);
        Ok(())
    }

    /// Stops delivering the packets of `tuner`; stops the port when no tuner of it is left.
    /// No packet reaches the tuner's feed once this returns.
    pub(crate) fn stop(&self, tuner: usize) {
        let port = self.routing.port(tuner);
        self.detach(port, tuner);
        // As in `start`.
        let _ = self.service(port);
    }

    fn detach(&self, port: Port, tuner: usize) {
        let mut st = self.state(port).lock();
        st.sinks[tuner] = None;
        if st.running && st.sinks.iter().all(Option::is_none) {
            // The port is stopped under its lock, so that a sibling's `start` cannot re-arm it
            // while it settles; the rest of the settle time is waited out by the next start
            // instead of sleeping here, which would also hold up the other port's interrupts.
            st.running = false;
            st.stopped_at_us = Some(self.board.now_us());
            // A device that is gone has stopped anyway. A channel that did not stop is caught by
            // the next start, which does not arm a port that is not idle.
            if self.stop_dma(port, &mut st) == Err(StopError::Timeout) {
                dev_err!(self.dev, "port {:?}: DMA did not stop\n", port);
            }
        }
    }

    /// Ends the stream for good, for removal and shutdown: no feed receives packets any more, the
    /// DMA of both ports is stopped, and `start` fails. Waits for a start or stop that is in
    /// progress. Safe to repeat. Reports a channel that did not stop within the settle time;
    /// until it has stopped, the DMA buffers must not be freed.
    pub(crate) fn quiesce(&self) -> Result<(), StopError> {
        let mut result = Ok(());
        for port in [Port::A, Port::B] {
            let mut st = self.state(port).lock();
            // Under the lock, so that neither a start nor the interrupt thread's restart can
            // arm the port again.
            st.halted = true;
            st.running = false;
            st.sinks = [None; TUNERS];
            result = result.and(self.stop_dma(port, &mut st));
        }
        result.and(shutdown(&self.board))
    }

    /// Waits until the channels of `port` are idle, so that no running channel takes the new
    /// addresses of a start.
    fn require_idle(&self, port: Port, st: &mut PortState) -> Result {
        match wait_idle(&self.board, LAYOUT.port(port)) {
            Ok(()) => Ok(()),
            Err(StopError::Gone) => Err(ENODEV),
            Err(StopError::Timeout) => {
                // `recover` retries every status poll, so only powers of two are logged.
                st.busy_starts = st.busy_starts.wrapping_add(1);
                if st.busy_starts.is_power_of_two() {
                    dev_err!(
                        self.dev,
                        "port {:?}: DMA still running, not starting ({} so far)\n",
                        port,
                        st.busy_starts
                    );
                }
                Err(EIO)
            }
        }
    }

    /// Starts the port of `tuner` again if a failed restart left it stopped while its feeds still
    /// expect data. The interrupt thread cannot retry that: with the DMA stopped, no interrupt
    /// comes. Called from the frontend's status poll, which runs every 200 ms while the frontend
    /// is open.
    pub(crate) fn recover(&self, tuner: usize) {
        let port = self.routing.port(tuner);
        {
            let mut st = self.state(port).lock();
            // Only a failed restart stops a port that still has sinks; `detach` stops it only
            // once the last sink is gone.
            if st.halted || st.running || st.sinks.iter().all(Option::is_none) {
                return;
            }
            if self.start_dma(port, &mut st).is_ok() {
                st.running = true;
            }
        }
        // As in `start`.
        let _ = self.service(port);
    }

    /// Starts both channels of `port`, after the settle time of an earlier stop has elapsed.
    fn start_dma(&self, port: Port, st: &mut PortState) -> Result {
        if let Some(stopped_at) = st.stopped_at_us.take() {
            // Process context, with the port locked: nothing else may touch the port meanwhile.
            let ready = stopped_at + (STOP_SETTLE_MS * 1000) as u64;
            let now = self.board.now_us();
            if now < ready {
                fsleep(Delta::from_micros((ready - now) as i64));
            }
        }
        self.require_idle(port, st)?;
        let regs = LAYOUT.port(port);
        // Set the TS input enable bit without clearing the rest of the register, which a
        // byte-wide write would do.
        let mode = self.read(regs.ts_mode)?;
        if mode & TS_INPUT_ENABLE == 0 {
            self.write(mode | TS_INPUT_ENABLE, regs.ts_mode)?;
        }
        let mask = regs.irq_mask();
        // What an earlier run left, still latched in the status register or already taken by the
        // handler, would otherwise be served as the completion of a new transfer. Reading the
        // status back makes the clear reach the device before the lock is released, so the next
        // handler reads the cleared status. The overflow bit comes with the first running port;
        // while another port runs it is armed already, and clearing it could drop a count.
        self.irq_lock.with(|| {
            let first = self.armed.load(Ordering::Relaxed) & PORT_IRQS == 0;
            let bits = mask | IRQ_OVERFLOW;
            self.write(if first { bits } else { mask }, IRQ_CLEAR)?;
            self.read(IRQ_STATUS)?;
            self.pending.fetch_and(!mask, Ordering::AcqRel);
            self.armed.fetch_or(bits, Ordering::Release);
            self.write(bits, IRQ_ENABLE)
        })?;
        // After a start the hardware fills channel 0 first, then alternates (measured).
        st.next = 0;
        st.delivered = 0;
        st.delivered_tagged = 0;
        st.filling_since_us = self.board.now_us();
        // Short first transfers: the rate is unknown, and the first packets should not wait.
        st.armed_length = [MIN_TRANSFER; 2];
        st.next_length = MIN_TRANSFER;
        st.last_served_us = 0;
        let first = buffer_index(port);
        for (ch, c) in regs.channels.iter().enumerate() {
            let addr = self.dma.0.dma_handle() + ((first + ch) * TRANSFER) as u64;
            // The allocation lies below 4 GiB: the device's DMA mask is 32 bits.
            self.write(addr as u32, c.addr_lo)?;
            self.write(0, c.addr_hi)?;
            self.write(control(MIN_TRANSFER), c.control)?;
            // Read right before the write, see `halt_channels`.
            let m = self.read(regs.management)?;
            self.write(
                (m & !LATENCY_TIMERS) | enable_bit(ch) | run_bit(ch),
                regs.management,
            )?;
        }
        st.latency.hold(IDLE_EXIT_LATENCY_US);
        Ok(())
    }

    /// Stops both channels of `port`, masks its interrupts and waits until it is idle.
    fn stop_dma(&self, port: Port, st: &mut PortState) -> Result<(), StopError> {
        st.latency.release();
        let regs = LAYOUT.port(port);
        let mask = regs.irq_mask();
        // Masked before the bits leave `armed`, so that the handler is not left with a line
        // that it refuses to claim; the status bits that latch from here on are ignored. The
        // last running port takes the overflow bit with it.
        self.irq_lock.with(|| {
            let others = self.armed.load(Ordering::Relaxed) & PORT_IRQS & !mask;
            let bits = if others == 0 {
                mask | IRQ_OVERFLOW
            } else {
                mask
            };
            self.board
                .write32(bits, IRQ_DISABLE)
                .ok_or(StopError::Gone)?;
            self.armed.fetch_and(!bits, Ordering::AcqRel);
            Ok(())
        })?;
        halt_channels(&self.board, regs).ok_or(StopError::Gone)?;
        wait_idle(&self.board, regs)
    }

    /// Serves the completions of `port` that are pending, and those that arrive while doing so.
    ///
    /// The interrupt thread serves both ports, and a start or stop may hold this port's lock for
    /// up to the settle time. Waiting for it would let the other port overflow, so the lock is
    /// only tried. That is safe because everything that holds the lock calls this after
    /// releasing it (except `quiesce`, after which nothing is served): a bit that was pending
    /// when the attempt failed is still pending then (or has been served meanwhile), and bits
    /// that arrive later wake the interrupt thread again. A stop discards the port's data
    /// anyway.
    fn service(&self, port: Port) -> Result {
        let mask = LAYOUT.port(port).irq_mask();
        while self.pending.load(Ordering::Acquire) & mask != 0 {
            let Some(mut guard) = self.state(port).try_lock() else {
                return Ok(());
            };
            self.service_locked(port, &mut guard)?;
        }
        Ok(())
    }

    /// Handles the completions of `port` that are pending; `st` is the port's locked state.
    fn service_locked(&self, port: Port, st: &mut PortState) -> Result {
        let regs = LAYOUT.port(port);
        let mask = regs.irq_mask();
        // Taken under the port's lock: a bit that a stop or restart raced with is then either
        // consumed here and dropped because the port is not running, or cleared by the start.
        // The bits are taken before the status is read, so every taken bit has its completion
        // visible in the snapshot.
        let bits = self.pending.fetch_and(!mask, Ordering::AcqRel) & mask;
        if !st.running || bits == 0 {
            return Ok(());
        }
        // One snapshot judges every channel's completion; later channels are not judged on a
        // value that is older or newer than the others'.
        let snapshot = self.read(regs.management)?;
        // Every pending channel is handled; taking one per interrupt loses the other's
        // completion. A good buffer is delivered even when the other channel failed, and only then
        // is the port restarted. The expected channel goes first so that the transfers reach the
        // demux in the order the hardware filled them.
        let mut failed = false;
        let mut desynced = false;
        let now = self.board.now_us();
        for ch in [st.next, st.next ^ 1] {
            if bits & regs.irq[ch] == 0 {
                continue;
            }
            let length = (self.read(regs.channels[ch].state)? & TRANSFERRED_MASK) as usize;
            if done_field(snapshot, ch) != COMPLETED || length != st.armed_length[ch] {
                failed = true;
                continue;
            }
            // The poll may have delivered the start of the transfer while it filled.
            let (skip, earlier) = if ch == st.next {
                st.filling_since_us = now;
                let skip = core::mem::take(&mut st.delivered);
                (
                    skip,
                    (core::mem::take(&mut st.delivered_tagged), skip / ts::PACKET),
                )
            } else {
                (0, (0, 0))
            };
            let rest = length - skip;
            if rest != 0 {
                let offset = (buffer_index(port) + ch) * TRANSFER + skip;
                // SAFETY: The channel has completed and is not re-armed before the copy is done,
                // so the device does not access this buffer while the slice lives.
                let data = unsafe { self.dma.0.as_slice(offset, rest)? };
                st.scratch[..rest].copy_from_slice(data);
            }
            st.note_served(length, now);
            if !failed {
                // The reads of the buffer must be complete before the write that hands it back to
                // the device, or the device could overwrite what is still being read. `writel`
                // only orders the writes before it, and `smp_mb()` is a compiler barrier alone
                // on a kernel without SMP, so the mandatory full barrier is used.
                mb();
                // The length of the next transfer is set while the channel is stopped, like at a
                // start; the address stays.
                self.write(control(st.next_length), regs.channels[ch].control)?;
                st.armed_length[ch] = st.next_length;
                // MANAGEMENT is read again right before the write: the value read before the
                // copy, which takes long, would write back stale run bits of the other channel.
                // The status fields are not write-1-to-clear, so writing them back as read
                // changes nothing.
                let m = self.read(regs.management)?;
                self.write(m | run_bit(ch), regs.management)?;
            }
            if self.dispatch(port, st, rest, earlier).is_none() {
                desynced = true;
            }
            st.next = ch ^ 1;
        }
        if failed {
            count_fault(port, &mut st.bad_transfers, "DMA transfer failed");
        }
        if desynced {
            count_fault(port, &mut st.lost_sync, "lost packet sync");
        }
        if failed || desynced {
            self.restart(port, st)?;
        }
        Ok(())
    }

    /// Restarts a port after a transfer completed with a bad status or length, or out of packet
    /// sync.
    ///
    /// The port is restarted by cancelling it, waiting, setting the TS mode and setting up the
    /// DMA again. The wait is not a spin loop, whose duration depends on the CPU, nor a fixed
    /// sleep, which would run in the interrupt thread both ports share, so the other port would
    /// overflow while it lasts. Instead the wait ends as soon as both channels have left the
    /// running state, bounded by `STOP_SETTLE_MS`.
    ///
    /// If the port cannot be restarted it is left stopped; `recover` retries.
    fn restart(&self, port: Port, st: &mut PortState) -> Result {
        let result = match self.stop_dma(port, st) {
            Ok(()) => self.start_dma(port, st),
            Err(StopError::Gone) => Err(ENODEV),
            Err(StopError::Timeout) => {
                dev_err!(
                    self.dev,
                    "port {:?}: DMA did not stop, restart failed\n",
                    port
                );
                Err(EIO)
            }
        };
        if result.is_err() {
            st.running = false;
            st.stopped_at_us = Some(self.board.now_us());
        }
        result
    }

    /// Routes and descrambles `length` bytes of a transfer, copied to the scratch buffer, then
    /// delivers them. `earlier` counts the tagged and all packets of the same transfer delivered
    /// before, which the sync check (`ts::in_sync`) takes together with these. Returns the number
    /// of tagged packets, or `None` if the transfer is not packet aligned, when nothing is
    /// delivered.
    ///
    /// There is no packet filter. Null packets never arrive: the demodulator or the bridge drops
    /// them before the DMA. Packets flagged with a transport error are delivered, because dropping
    /// them would hide the errors from user space: the DVB demux flags them, skips their continuity
    /// check and passes them on.
    fn dispatch(
        &self,
        port: Port,
        st: &mut PortState,
        length: usize,
        earlier: (usize, usize),
    ) -> Option<usize> {
        let PortState {
            scratch,
            out,
            sinks,
            ..
        } = st;
        let transfer = &mut scratch[..length];
        let tagged = ts::tagged_packets(&self.routing, port, transfer);
        if !ts::in_sync(earlier.0 + tagged, earlier.1 + length / ts::PACKET) {
            return None;
        }
        let deliver: [bool; TUNERS] =
            core::array::from_fn(|t| sinks[t].is_some() && !self.muted[t].load(Ordering::Relaxed));
        for packet in transfer.chunks_exact_mut(ts::PACKET) {
            let packet: &mut [u8; ts::PACKET] = match packet.try_into() {
                Ok(p) => p,
                Err(_) => continue,
            };
            let Some(tuner) = self.routing.tuner(port, packet[0]) else {
                continue;
            };
            if !deliver[tuner] {
                continue;
            }
            packet[0] = 0x47;
            self.descrambler.packet(packet);
            // The tuner's buffer has room for a whole transfer, so this never allocates.
            let _ = out[tuner].extend_from_slice(packet, GFP_ATOMIC);
        }
        for (tuner, buf) in out.iter_mut().enumerate() {
            if let Some(feed) = sinks[tuner] {
                // The demux holds a spinlock with interrupts off for a whole call, so a transfer
                // goes in pieces no longer than the shortest one.
                for piece in buf.chunks(MIN_TRANSFER) {
                    feed.packets(piece);
                }
            }
            buf.clear();
        }
        Some(tagged)
    }
}

impl ThreadedHandler for Stream {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let claimed = self.irq_lock.with(|| {
            // Reads of a device that is gone or whose link is down return all ones; they are not
            // completions.
            let status = self.board.read32(IRQ_STATUS).filter(|&s| s != u32::MAX)?;
            // A status bit of a masked channel, should the register ever show one, must not be
            // taken for a completion of the next run.
            let ours = status & self.armed.load(Ordering::Acquire);
            if ours == 0 {
                return None;
            }
            let _ = self.board.write32(ours, IRQ_CLEAR);
            let _ = self.board.read32(IRQ_STATUS);
            // An overflow is only counted here; the poll logs it from process context, because
            // a message costs the interrupt thread milliseconds on a serial console.
            if ours & IRQ_OVERFLOW != 0 {
                self.overflows.fetch_add(1, Ordering::Relaxed);
            }
            let completions = ours & !IRQ_OVERFLOW;
            self.pending.fetch_or(completions, Ordering::AcqRel);
            Some(completions != 0)
        });
        // The line is shared; foreign interrupts are reported as such and not claimed.
        match claimed {
            Some(true) => ThreadedIrqReturn::WakeThread,
            Some(false) => ThreadedIrqReturn::Handled,
            None => ThreadedIrqReturn::None,
        }
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        for port in [Port::A, Port::B] {
            // Only fails once the device is gone, when there is nothing left to do.
            let _ = self.service(port);
        }
        IrqReturn::Handled
    }
}

impl Stream {
    /// Delivers what the channel now filling has taken so far, once it has been filling for
    /// [`TARGET_US`]: a port slower than one shortest transfer per `TARGET_US` would otherwise
    /// hold its packets until the transfer is full (1.5 s for a partial-reception layer alone,
    /// measured). The completion delivers the rest. Shorter transfers would deliver as early, but
    /// they leave less slack when the port's rate rises at once, as when another tuner locks,
    /// and the slack is what keeps packets from being lost.
    ///
    /// TRANS_STATE grows while a transfer runs, always by whole packets, and the bytes below it
    /// are in memory when it is read (measured on the card, also with the buffer cleared before
    /// every arming, to show bytes read before the device wrote them).
    fn deliver_filling(&self, port: Port) -> Result {
        let mut guard = self.state(port).lock();
        let st = &mut *guard;
        if !st.running || self.board.now_us() < st.filling_since_us + TARGET_US {
            return Ok(());
        }
        let regs = LAYOUT.port(port);
        let ch = st.next;
        // A completed transfer is the interrupt thread's to deliver.
        if state_field(self.read(regs.management)?, ch) != 1 {
            return Ok(());
        }
        let filled = (self.read(regs.channels[ch].state)? & TRANSFERRED_MASK) as usize;
        if filled > st.armed_length[ch] || filled % ts::PACKET != 0 {
            // Not what the hardware does; the completion judges the transfer.
            return Err(EIO);
        }
        if filled <= st.delivered {
            return Ok(());
        }
        let length = filled - st.delivered;
        let offset = (buffer_index(port) + ch) * TRANSFER + st.delivered;
        // SAFETY: The device has written these bytes, which lie below the channel's TRANS_STATE,
        // and does not write them again before the channel is re-armed, which takes this port's
        // lock.
        let data = unsafe { self.dma.0.as_slice(offset, length)? };
        st.scratch[..length].copy_from_slice(data);
        let earlier = (st.delivered_tagged, st.delivered / ts::PACKET);
        match self.dispatch(port, st, length, earlier) {
            Some(tagged) => {
                st.delivered = filled;
                st.delivered_tagged += tagged;
            }
            None => {
                count_fault(port, &mut st.lost_sync, "lost packet sync");
                self.restart(port, st)?;
            }
        }
        Ok(())
    }

    /// Logs the input overflows counted since the last call, when their total reaches a power of
    /// two, so that a persistent cause does not flood the log.
    fn report_overflows(&self) {
        let total = self.overflows.load(Ordering::Relaxed);
        let seen = self.overflows_seen.swap(total, Ordering::Relaxed);
        if total != seen
            && seen
                .wrapping_add(1)
                .checked_next_power_of_two()
                .is_some_and(|p| p <= total)
        {
            dev_warn!(
                self.dev,
                "TS input overflowed, packets were lost ({} times so far)\n",
                total
            );
        }
    }
}

impl platform::Poll for Stream {
    fn poll(&self) {
        for port in [Port::A, Port::B] {
            // Only fails once the device is gone, or on a TRANS_STATE that the completion of the
            // transfer judges anyway.
            let _ = self.deliver_filling(port);
            // Completions that arrived while the port was locked are served now, as in `start`.
            let _ = self.service(port);
        }
        self.report_overflows();
    }
}
