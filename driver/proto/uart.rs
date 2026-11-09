// SPDX-License-Identifier: GPL-2.0

//! Smart-card UART of the ASV5220 bridge (BAR0 0x884, 0x890 and 0xC80..0xC9C).
//!
//! The register values and their order are revealed truth. The timing is not counted in polling
//! loops, whose duration depends on the kernel's HZ or on the scheduler: every wait here is
//! measured on a monotonic clock against limits taken from ISO/IEC 7816-3 and the ATR, every wait
//! is bounded, and card removal is checked on each polling step and each time characters are taken
//! from the FIFO. Waiting also ends as soon as [`Bus::check_abort`] says the operation must not go
//! on.

use super::atr::{self, Atr, AtrError, Scan, T1Params};
use super::t1::{RecvError, Transport};

/// UART reset: byte 0 is pulsed to 4 and back to 0.
const UART_RESET: usize = 0x884;
/// GPIO register shared with the rest of the bridge.
const GPIO: usize = 0x890;
/// Card reset line (active low).
pub(crate) const GPIO_CARD_RST: u32 = 1 << 5;
/// Card detect switch; reads 0 while a card is inserted.
pub(crate) const GPIO_CARD_ABSENT: u32 = 1 << 20;
const CONFIG: usize = 0xC80;
const DIVISOR: usize = 0xC84;
const CONTROL: usize = 0xC88;
const TX_LENGTH: usize = 0xC8C;
const FIFO: usize = 0xC90;
/// Byte 0: characters waiting in the receive FIFO. Byte 1: characters queued for transmission.
const STATUS: usize = 0xC94;
const TX_BYTE_ENABLE: usize = 0xC98;
const RX_POP: usize = 0xC9C;

/// Transmit FIFO fill level up to which another 32-bit word may be pushed.
const TX_PUSH_LEVEL: u32 = 0x1C;

/// Register values for the two operating points the hardware is known to work with (revealed). Only
/// these are known to be correct for the hardware, so only these are used.
struct OperatingPoint {
    config: u32,
    divisor: u32,
    /// Upper bound for the transmission of one character, in microseconds.
    char_us: u32,
}

/// Clock cycles of one etu at the ATR rate (F = 372, D = 1).
const ATR_ETU_CLOCKS: u64 = 372;

/// Etu of the longest character frame the UART produces.
const CHAR_ETU: u64 = 12;

/// Bound for the transmission of one character at divisor `d`, in microseconds.
///
/// It exists only to notice a dead transmitter, so it is twice the real duration of a 12-etu
/// frame rather than a tight figure; deriving it from [`CARD_CLOCK_HZ`] keeps it valid if the
/// clock ever changes.
const fn char_us(d: u64) -> u32 {
    (2 * CHAR_ETU * ATR_ETU_CLOCKS * 1_000_000).div_ceil(CARD_CLOCK_HZ as u64 * d) as u32
}

/// F = 372, D = 1: the rate of every ATR.
const ATR_RATE: OperatingPoint = OperatingPoint {
    config: 0x0076_0022,
    divisor: 0x3117,
    char_us: char_us(1),
};

/// F = 372, D = 2 with the 11-etu character frame of N = 255: the B-CAS specific mode.
const BCAS_RATE: OperatingPoint = OperatingPoint {
    config: 0x007E_0022,
    divisor: 0x310B,
    char_us: char_us(2),
};

/// Clock supplied to the card. ARIB STD-B25 4.3.2 requires the first reset at 4 MHz, and the clock
/// divider is never changed (the upper byte of 0xC84 stays 0x31, as revealed).
pub(crate) const CARD_CLOCK_HZ: u32 = 4_000_000;

/// Time the reset line is held low. ISO/IEC 7816-3 requires at least 400 clock cycles; this is the
/// revealed value, which leaves a wide margin.
const RESET_HOLD_US: u32 = 50_000;

/// Polling period while waiting for characters. The receive FIFO holds [`RX_FIFO`] characters,
/// about 15 ms at the B-CAS rate, so the FIFO only overflows if the task is kept off the CPU
/// that long; the characters lost then fail the block's EDC and T=1 error recovery asks again.
const POLL_US: u32 = 250;

/// Depth of the receive FIFO in characters.
const RX_FIFO: u32 = 32;

/// How long the detect switch has to read "absent" without interruption before the card counts as
/// removed. A contact that bounces is not a removal: the characters a bounce corrupts are repaired
/// by T=1 error recovery, while a session that is ended has to be set up again by the application.
/// With samples 100 ms apart, as the reader's idle polling takes them, this is two consecutive
/// samples.
pub(crate) const REMOVAL_US: u64 = 100_000;

/// Slack added to every protocol time limit to absorb timer and scheduling latency. It only makes
/// a dead card take slightly longer to be noticed; it never shortens a time the card is granted.
const SLACK_US: u32 = 2_000;

/// Register access to the bridge.
pub(crate) trait Bus {
    /// Reads a 32-bit register. Fails once the device is gone.
    fn read32(&self, offset: usize) -> Result<u32, Fault>;
    /// Writes a 32-bit register. Fails once the device is gone.
    fn write32(&self, value: u32, offset: usize) -> Result<(), Fault>;
    /// Atomically sets and clears bits of the GPIO register, which other parts of the driver
    /// modify concurrently.
    fn update_gpio(&self, set: u32, clear: u32) -> Result<(), Fault>;
    /// Sleeps for at least `us` microseconds.
    fn sleep_us(&self, us: u32);
    /// Monotonic time in microseconds.
    fn now_us(&self) -> u64;
    /// Fails when the operation in progress must end at once: [`Fault::DeviceGone`] when the
    /// reader is shut down, [`Fault::Interrupted`] when the caller has been killed. Called on
    /// every step of a wait, so it must neither block nor touch the hardware. Holding the card
    /// in reset ([`Uart::deactivate`]) does not call it, as it is what recovers from an abort.
    fn check_abort(&self) -> Result<(), Fault>;
}

impl<B: Bus + ?Sized> Bus for &B {
    fn read32(&self, offset: usize) -> Result<u32, Fault> {
        (**self).read32(offset)
    }
    fn write32(&self, value: u32, offset: usize) -> Result<(), Fault> {
        (**self).write32(value, offset)
    }
    fn update_gpio(&self, set: u32, clear: u32) -> Result<(), Fault> {
        (**self).update_gpio(set, clear)
    }
    fn sleep_us(&self, us: u32) {
        (**self).sleep_us(us)
    }
    fn now_us(&self) -> u64 {
        (**self).now_us()
    }
    fn check_abort(&self) -> Result<(), Fault> {
        (**self).check_abort()
    }
}

/// Failures of the card interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Fault {
    /// The PCI device has been removed, or the reader is shutting down.
    DeviceGone,
    /// The calling task has been killed.
    Interrupted,
    /// No card is inserted (or it was pulled during the operation).
    NoCard,
    /// The transmitter did not drain in the time the frame needs.
    TxStuck,
    /// The receive FIFO did not empty.
    RxStuck,
    /// The card did not answer the reset.
    Mute,
    /// The ATR is malformed or announces parameters this reader cannot operate.
    Atr(AtrError),
}

/// The card UART: activation and character transport.
pub(crate) struct Uart<B: Bus> {
    bus: B,
    /// Characters popped from the FIFO but not yet consumed.
    pending: [u8; 4],
    pending_len: usize,
    pending_pos: usize,
    /// Time the last character was popped, for the block guard time.
    last_rx_us: u64,
    /// Upper bound for the transmission of one character, in microseconds.
    char_us: u32,
    /// Since when the detect switch has read "absent" without a "present" in between.
    absent_since_us: Option<u64>,
    /// A card is active, so a removal has to last [`REMOVAL_US`]. Before that, during
    /// activation, no session is at stake and an absent card is reported at once.
    debounce_removal: bool,
}

/// Result of an activation.
pub(crate) struct Activation {
    /// The raw ATR.
    pub(crate) atr: [u8; atr::MAX_LEN],
    /// Its length.
    pub(crate) atr_len: usize,
    /// The T=1 parameters in effect.
    pub(crate) params: T1Params,
}

impl<B: Bus> Uart<B> {
    /// Takes over the bus; the UART is not touched until [`Uart::activate`].
    pub(crate) fn new(bus: B) -> Self {
        Uart {
            bus,
            pending: [0; 4],
            pending_len: 0,
            pending_pos: 0,
            last_rx_us: 0,
            char_us: ATR_RATE.char_us,
            absent_since_us: None,
            debounce_removal: false,
        }
    }

    /// Whether the detect switch reads "card inserted" now.
    pub(crate) fn card_present(&self) -> Result<bool, Fault> {
        Ok(self.bus.read32(GPIO)? & GPIO_CARD_ABSENT == 0)
    }

    /// Samples the detect switch and tells whether the card has been absent for [`REMOVAL_US`].
    /// Every sample taken during a session goes through here, so that all of them, whoever
    /// takes them, judge removal by the same rule.
    pub(crate) fn card_removed(&mut self) -> Result<bool, Fault> {
        if self.card_present()? {
            self.absent_since_us = None;
            return Ok(false);
        }
        let now = self.bus.now_us();
        let since = *self.absent_since_us.get_or_insert(now);
        Ok(now - since >= REMOVAL_US)
    }

    fn check_card(&mut self) -> Result<(), Fault> {
        let removed = if self.debounce_removal {
            self.card_removed()?
        } else {
            !self.card_present()?
        };
        if removed {
            Err(Fault::NoCard)
        } else {
            Ok(())
        }
    }

    fn reset_uart(&self) -> Result<(), Fault> {
        let v = self.bus.read32(UART_RESET)?;
        self.bus.write32((v & !0xFF) | 4, UART_RESET)?;
        self.bus.sleep_us(10_000);
        self.bus.write32(v & !0xFF, UART_RESET)
    }

    fn set_rate(&mut self, rate: &OperatingPoint) -> Result<(), Fault> {
        self.bus.write32(rate.divisor, DIVISOR)?;
        self.bus.write32(rate.config, CONFIG)?;
        self.char_us = rate.char_us;
        Ok(())
    }

    /// Resets the card and reads its ATR, then switches the UART to the card's specific mode.
    ///
    /// The card supply cannot be switched by this hardware, so every activation is a warm reset
    /// in ISO/IEC 7816-3 terms.
    pub(crate) fn activate(&mut self) -> Result<Activation, Fault> {
        // No session is at stake yet, so an empty slot is reported at once rather than after
        // the removal time: the ATR wait would otherwise end first and call the card mute.
        self.debounce_removal = false;
        self.check_card()?;
        self.absent_since_us = None;
        self.reset_uart()?;
        self.set_rate(&ATR_RATE)?;
        self.bus.write32(0x0001_0003, CONTROL)?;
        self.bus.write32(0x0123_0000, TX_LENGTH)?;
        self.bus.write32(0x0000_000F, TX_BYTE_ENABLE)?;
        self.bus.write32(0x000F_0004, RX_POP)?;
        self.bus.update_gpio(0, GPIO_CARD_RST)?;
        self.bus.sleep_us(RESET_HOLD_US);
        // The card is silent while held in reset, so nothing received now belongs to the ATR.
        self.purge()?;
        self.bus.update_gpio(GPIO_CARD_RST, 0)?;
        self.bus.write32(0x0001_0103, CONTROL)?;

        let atr = self.read_atr()?;
        let parsed = Atr::parse(&atr.0[..atr.1]).map_err(Fault::Atr)?;
        let params = parsed.t1_params().map_err(Fault::Atr)?;
        if (params.f, params.d, params.n) != (372, 2, 0xFF) {
            return Err(Fault::Atr(AtrError::Unsupported(
                "transmission factors other than the B-CAS specific mode",
            )));
        }
        self.set_rate(&BCAS_RATE)?;
        self.debounce_removal = true;
        Ok(Activation {
            atr: atr.0,
            atr_len: atr.1,
            params,
        })
    }

    fn read_atr(&mut self) -> Result<([u8; atr::MAX_LEN], usize), Fault> {
        // ISO/IEC 7816-3 8.1: the leading edge of TS comes within 40000 clock cycles of the reset
        // release, and each further character within the initial waiting time of 9600 etu. TS is
        // only seen once the whole character has been received and popped, so one character time
        // at the ATR rate is added; the poll granularity is covered by SLACK_US in `recv`.
        let first_us = ((40_000 + CHAR_ETU * ATR_ETU_CLOCKS) * 1_000_000)
            .div_ceil(u64::from(CARD_CLOCK_HZ)) as u32;
        let wait_us = (9_600 * 372 * 1_000_000u64).div_ceil(u64::from(CARD_CLOCK_HZ)) as u32;
        let mut buf = [0u8; atr::MAX_LEN];
        let mut len = 0;
        loop {
            let need = match atr::scan(&buf[..len]).map_err(Fault::Atr)? {
                Scan::Complete(n) => return Ok((buf, n)),
                Scan::NeedMore(n) => n,
            };
            for _ in 0..need {
                let timeout = if len == 0 { first_us } else { wait_us };
                buf[len] = match self.recv(timeout) {
                    Ok(b) => b,
                    Err(RecvError::Timeout) => return Err(Fault::Mute),
                    Err(RecvError::Io(e)) => return Err(e),
                };
                len += 1;
            }
        }
    }

    /// Holds the card in reset, which ends any session.
    pub(crate) fn deactivate(&mut self) -> Result<(), Fault> {
        self.bus.update_gpio(0, GPIO_CARD_RST)?;
        self.discard_pending();
        self.absent_since_us = None;
        self.debounce_removal = false;
        self.reset_uart()
    }

    fn discard_pending(&mut self) {
        self.pending_len = 0;
        self.pending_pos = 0;
    }

    fn rx_count(&self) -> Result<u32, Fault> {
        Ok(self.bus.read32(STATUS)? & 0xFF)
    }

    fn tx_level(&self) -> Result<u32, Fault> {
        Ok((self.bus.read32(STATUS)? >> 8) & 0xFF)
    }

    /// Moves up to four received characters from the FIFO into `pending`.
    fn pop(&mut self) -> Result<bool, Fault> {
        let count = self.rx_count()?;
        if count == 0 {
            return Ok(false);
        }
        // `wait_until` only checks for removal while it is idle, so a card that is pulled while
        // characters keep arriving would otherwise go unnoticed until the stream stops.
        self.check_card()?;
        let n = count.min(4);
        let word = if n == 4 {
            self.bus.read32(FIFO)?
        } else {
            // Byte 0 of RX_POP selects how many characters the next FIFO read removes.
            self.bus.write32(0x000F_0000 | n, RX_POP)?;
            let w = self.bus.read32(FIFO)?;
            self.bus.write32(0x000F_0004, RX_POP)?;
            w
        };
        self.pending = word.to_le_bytes();
        self.pending_len = n as usize;
        self.pending_pos = 0;
        self.last_rx_us = self.bus.now_us();
        Ok(true)
    }

    /// Waits until `deadline_us` while `ready` is false, checking for card removal (see
    /// [`REMOVAL_US`]).
    fn wait_until(
        &mut self,
        deadline_us: u64,
        mut ready: impl FnMut(&mut Self) -> Result<bool, Fault>,
    ) -> Result<bool, Fault> {
        loop {
            self.bus.check_abort()?;
            if ready(self)? {
                return Ok(true);
            }
            self.check_card()?;
            let now = self.bus.now_us();
            if now >= deadline_us {
                // A late wake-up must not turn into a timeout the card did not cause.
                return ready(self);
            }
            self.bus.sleep_us(POLL_US.min((deadline_us - now) as u32));
        }
    }
}

impl<B: Bus> Transport for Uart<B> {
    type Error = Fault;

    fn now_us(&self) -> u64 {
        self.bus.now_us()
    }

    fn purge(&mut self) -> Result<(), Fault> {
        self.discard_pending();
        // Characters may keep arriving, so this loop is a wait like any other. Each pop takes at
        // least one character and a few register accesses, far less than the character time, so
        // emptying a full FIFO takes at most `RX_FIFO` pops plus the few characters that arrive
        // meanwhile. A count that does not go down within twice that is a broken receiver.
        for _ in 0..2 * RX_FIFO {
            self.bus.check_abort()?;
            if !self.pop()? {
                return Ok(());
            }
            self.discard_pending();
        }
        Err(Fault::RxStuck)
    }

    fn send(&mut self, frame: &[u8], guard_us: u32) -> Result<(), Fault> {
        let start = self.last_rx_us + u64::from(guard_us);
        let now = self.bus.now_us();
        if start > now {
            self.bus.sleep_us((start - now) as u32);
        }
        // In half-duplex T=1 nothing valid can be received while it is our turn to send, so any
        // character here is stray or late; left in the FIFO it would become the NAD of the answer.
        self.purge()?;
        self.check_card()?;
        // Frames are at most 258 bytes (prologue, 254-byte INF, LRC).
        let len = frame.len() as u32;
        self.bus.write32(0x0023_0000 | len, TX_LENGTH)?;
        self.bus.write32(0x0001_0203, CONTROL)?;
        let drain_us = len * self.char_us + SLACK_US;
        let deadline = self.bus.now_us() + u64::from(drain_us);
        for chunk in frame.chunks(4) {
            let room = self.wait_until(deadline, |u| Ok(u.tx_level()? <= TX_PUSH_LEVEL))?;
            if !room {
                return Err(Fault::TxStuck);
            }
            let mut word = [0u8; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            if chunk.len() < 4 {
                self.bus
                    .write32((1u32 << chunk.len()) - 1, TX_BYTE_ENABLE)?;
            }
            self.bus.write32(u32::from_le_bytes(word), FIFO)?;
        }
        if !self.wait_until(deadline, |u| Ok(u.tx_level()? == 0))? {
            return Err(Fault::TxStuck);
        }
        self.bus.write32(0x0000_000F, TX_BYTE_ENABLE)?;
        // The control register is read back at this point (as revealed); kept so the register
        // access pattern matches the one the hardware is known to work with.
        self.bus.read32(CONTROL)?;
        Ok(())
    }

    fn recv(&mut self, timeout_us: u32) -> Result<u8, RecvError<Fault>> {
        if self.pending_pos == self.pending_len {
            let deadline = self.bus.now_us() + u64::from(timeout_us) + u64::from(SLACK_US);
            match self.wait_until(deadline, |u| u.pop()) {
                Ok(true) => {}
                Ok(false) => return Err(RecvError::Timeout),
                Err(e) => return Err(RecvError::Io(e)),
            }
        }
        let b = self.pending[self.pending_pos];
        self.pending_pos += 1;
        Ok(b)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::super::t1::{self, Session, IFSD};
    use super::*;
    use core::cell::{Cell, RefCell};
    use std::collections::{BTreeMap, VecDeque};
    use std::vec::Vec;

    const BCAS: [u8; 13] = [
        0x3B, 0xF0, 0x12, 0x00, 0xFF, 0x91, 0x81, 0xB1, 0x7C, 0x45, 0x1F, 0x03, 0x99,
    ];

    /// Register-level model of the bridge UART with a card that answers IFS and echoes I-blocks.
    struct Sim {
        regs: RefCell<BTreeMap<usize, u32>>,
        gpio: Cell<u32>,
        now: Cell<u64>,
        rx: RefCell<VecDeque<u8>>,
        tx: RefCell<Vec<u8>>,
        tx_expected: Cell<u32>,
        byte_enable: Cell<u32>,
        pop_width: Cell<u32>,
        /// When the card starts talking after reset release, and whether it ever does.
        atr: Vec<u8>,
        mute: bool,
        frames: RefCell<Vec<Vec<u8>>>,
        removed_at: Cell<Option<u64>>,
        /// When a pulled card is back in the slot.
        back_at: Cell<Option<u64>>,
        /// FIFO reads return characters without taking them out: a broken receiver.
        stuck_rx: Cell<bool>,
        /// From this time on, `check_abort` fails with this fault.
        abort_at: Cell<Option<(u64, Fault)>>,
    }

    impl Sim {
        fn new(atr: &[u8]) -> Self {
            Sim {
                regs: RefCell::new(BTreeMap::new()),
                gpio: Cell::new(0),
                now: Cell::new(0),
                rx: RefCell::new(VecDeque::new()),
                tx: RefCell::new(Vec::new()),
                tx_expected: Cell::new(0),
                byte_enable: Cell::new(0),
                pop_width: Cell::new(4),
                atr: atr.to_vec(),
                mute: false,
                frames: RefCell::new(Vec::new()),
                removed_at: Cell::new(None),
                back_at: Cell::new(None),
                stuck_rx: Cell::new(false),
                abort_at: Cell::new(None),
            }
        }

        fn card_answer(&self, frame: &[u8]) {
            let pcb = frame[1];
            let inf = &frame[3..frame.len() - 1];
            let mut reply = match pcb {
                0xC1 => std::vec![0x00, 0xE1, 0x01, inf[0]],
                0x00 | 0x40 => {
                    let mut r = std::vec![0x00, pcb, inf.len() as u8 + 2];
                    r.extend_from_slice(inf);
                    r.extend_from_slice(&[0x90, 0x00]);
                    r
                }
                _ => panic!("unexpected PCB {pcb:#04x}"),
            };
            reply.push(reply.iter().fold(0, |a, b| a ^ b));
            self.rx.borrow_mut().extend(reply);
        }
    }

    impl Bus for Sim {
        fn read32(&self, off: usize) -> Result<u32, Fault> {
            Ok(match off {
                GPIO => {
                    let now = self.now.get();
                    let removed = self.removed_at.get().is_some_and(|t| now >= t)
                        && !self.back_at.get().is_some_and(|t| now >= t);
                    self.gpio.get() | if removed { GPIO_CARD_ABSENT } else { 0 }
                }
                // The transmitter empties instantly in this model.
                STATUS => (self.rx.borrow().len() as u32).min(RX_FIFO),
                FIFO => {
                    let n = self.pop_width.get();
                    let mut rx = self.rx.borrow_mut();
                    let mut w = [0u8; 4];
                    for (i, b) in w.iter_mut().take(n as usize).enumerate() {
                        *b = if self.stuck_rx.get() {
                            rx[i]
                        } else {
                            rx.pop_front().expect("FIFO read with too few characters")
                        };
                    }
                    u32::from_le_bytes(w)
                }
                _ => self.regs.borrow().get(&off).copied().unwrap_or(0),
            })
        }

        fn write32(&self, v: u32, off: usize) -> Result<(), Fault> {
            match off {
                TX_LENGTH => {
                    self.tx_expected.set(v & 0xFFFF);
                    self.tx.borrow_mut().clear();
                }
                TX_BYTE_ENABLE => self.byte_enable.set(v),
                RX_POP => {
                    assert_eq!(v >> 16, 0xF);
                    self.pop_width.set(v & 0xFF);
                }
                FIFO => {
                    let n = match self.byte_enable.get() {
                        1 => 1,
                        3 => 2,
                        7 => 3,
                        0xF => 4,
                        e => panic!("byte enable {e:#x}"),
                    };
                    self.tx
                        .borrow_mut()
                        .extend_from_slice(&v.to_le_bytes()[..n]);
                    let done = self.tx.borrow().len() as u32 == self.tx_expected.get();
                    if done {
                        let frame = self.tx.borrow().clone();
                        self.frames.borrow_mut().push(frame.clone());
                        self.card_answer(&frame);
                    }
                }
                _ => {
                    self.regs.borrow_mut().insert(off, v);
                }
            }
            Ok(())
        }

        fn update_gpio(&self, set: u32, clear: u32) -> Result<(), Fault> {
            let old = self.gpio.get();
            let new = (old & !clear) | set;
            self.gpio.set(new);
            if old & GPIO_CARD_RST == 0 && new & GPIO_CARD_RST != 0 && !self.mute {
                self.rx.borrow_mut().extend(self.atr.iter().copied());
            }
            Ok(())
        }

        fn sleep_us(&self, us: u32) {
            self.now.set(self.now.get() + u64::from(us));
        }

        fn now_us(&self) -> u64 {
            self.now.get()
        }

        fn check_abort(&self) -> Result<(), Fault> {
            match self.abort_at.get() {
                Some((at, fault)) if self.now.get() >= at => Err(fault),
                _ => Ok(()),
            }
        }
    }

    #[test]
    fn activation_reads_the_atr_and_switches_rate() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        let a = uart.activate().unwrap();
        assert_eq!(&a.atr[..a.atr_len], &BCAS);
        assert_eq!(sim.regs.borrow()[&DIVISOR], 0x310B);
        assert_eq!(sim.regs.borrow()[&CONFIG], 0x007E_0022);
        assert_ne!(sim.gpio.get() & GPIO_CARD_RST, 0);
    }

    #[test]
    fn frames_of_every_length_are_packed_and_answered() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        let a = uart.activate().unwrap();
        let mut s = Session::new(&a.params, a.params.timing(CARD_CLOCK_HZ));
        s.negotiate_ifsd(&mut uart).unwrap();
        assert_eq!(sim.frames.borrow()[0], [0x00, 0xC1, 0x01, IFSD, 0x3E]);
        for len in 1..=usize::from(a.params.ifsc) {
            let cmd: Vec<u8> = (0..len).map(|i| i as u8).collect();
            let mut out = [0u8; 256];
            let n = s.transceive(&mut uart, &cmd, &mut out).unwrap();
            assert_eq!(&out[..n - 2], &cmd[..]);
            assert_eq!(&out[n - 2..n], &[0x90, 0x00]);
        }
        assert_eq!(s.stats(), Default::default());
    }

    #[test]
    fn mute_card_is_reported_within_the_iso_limit() {
        let mut sim = Sim::new(&BCAS);
        sim.mute = true;
        let mut uart = Uart::new(&sim);
        assert_eq!(uart.activate().err(), Some(Fault::Mute));
        // UART reset, reset hold, and the 10 ms TS limit plus one ATR character (about 1.1 ms)
        // and slack.
        assert!(sim.now.get() < 10_000 + 50_000 + 10_000 + 1_200 + 2 * u64::from(SLACK_US));
    }

    #[test]
    fn unsupported_card_is_rejected_after_the_atr() {
        // T=1 specific mode but with D = 1: no known register setting for it.
        let mut atr = BCAS;
        atr[2] = 0x11;
        atr[12] = atr[1..12].iter().fold(0, |a, &b| a ^ b);
        let sim = Sim::new(&atr);
        let mut uart = Uart::new(&sim);
        assert!(matches!(
            uart.activate().err(),
            Some(Fault::Atr(AtrError::Unsupported(_)))
        ));
    }

    #[test]
    fn removal_aborts_a_wait() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        let removed = sim.now.get() + 5_000;
        sim.removed_at.set(Some(removed));
        assert_eq!(uart.recv(1_000_000), Err(RecvError::Io(Fault::NoCard)));
        let now = sim.now.get();
        assert!(now >= removed + REMOVAL_US);
        assert!(now < removed + REMOVAL_US + 2 * u64::from(POLL_US));
    }

    #[test]
    fn a_bouncing_contact_is_not_a_removal() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        let start = sim.now.get();
        // Absent for just under the removal time, twice, with a present sample in between.
        for at in [start + 5_000, start + 5_000 + REMOVAL_US] {
            sim.removed_at.set(Some(at));
            sim.back_at
                .set(Some(at + REMOVAL_US - 10 * u64::from(POLL_US)));
            assert_eq!(uart.recv(REMOVAL_US as u32), Err(RecvError::Timeout));
        }
        assert_eq!(uart.card_removed(), Ok(false));
    }

    #[test]
    fn removal_needs_the_card_absent_throughout() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.removed_at.set(Some(sim.now.get()));
        assert_eq!(uart.card_removed(), Ok(false));
        sim.now.set(sim.now.get() + REMOVAL_US / 2);
        sim.back_at.set(Some(sim.now.get()));
        // A present sample starts the count again.
        assert_eq!(uart.card_removed(), Ok(false));
        sim.removed_at.set(Some(sim.now.get()));
        sim.back_at.set(None);
        assert_eq!(uart.card_removed(), Ok(false));
        sim.now.set(sim.now.get() + REMOVAL_US / 2);
        assert_eq!(uart.card_removed(), Ok(false));
        sim.now.set(sim.now.get() + REMOVAL_US / 2);
        assert_eq!(uart.card_removed(), Ok(true));
    }

    #[test]
    fn a_card_pulled_during_activation_is_reported_at_once() {
        let mut sim = Sim::new(&BCAS);
        // Pulled during the wait for TS, which would end as a mute card long before 100 ms.
        sim.mute = true;
        sim.removed_at.set(Some(65_000));
        let mut uart = Uart::new(&sim);
        assert_eq!(uart.activate().err(), Some(Fault::NoCard));
    }

    #[test]
    fn a_receiver_that_does_not_empty_is_reported() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.rx.borrow_mut().extend([0u8; 8]);
        sim.stuck_rx.set(true);
        assert_eq!(uart.purge(), Err(Fault::RxStuck));
    }

    #[test]
    fn purge_drops_partially_consumed_characters() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.rx.borrow_mut().extend([1, 2, 3, 4, 5]);
        assert_eq!(uart.recv(1000), Ok(1));
        uart.purge().unwrap();
        assert!(sim.rx.borrow().is_empty());
        assert_eq!(uart.recv(1000), Err(RecvError::Timeout));
        sim.rx.borrow_mut().push_back(9);
        assert_eq!(uart.recv(1000), Ok(9));
    }

    #[test]
    fn characters_left_in_the_fifo_are_not_part_of_the_answer() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        let a = uart.activate().unwrap();
        let mut s = Session::new(&a.params, a.params.timing(CARD_CLOCK_HZ));
        // One character is consumed from a popped word, the rest sits in `pending` and in the FIFO.
        sim.rx
            .borrow_mut()
            .extend([0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6]);
        assert_eq!(uart.recv(1000), Ok(0xA1));
        s.negotiate_ifsd(&mut uart).unwrap();
        let cmd = [0x90, 0x34, 0x00, 0x00, 0x01];
        sim.rx.borrow_mut().push_back(0xFF);
        let mut out = [0u8; 256];
        let n = s.transceive(&mut uart, &cmd, &mut out).unwrap();
        assert_eq!(&out[..n - 2], &cmd[..]);
        assert_eq!(s.stats(), Default::default());
    }

    #[test]
    fn removal_is_noticed_while_characters_keep_arriving() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.rx.borrow_mut().extend([0u8; 64]);
        sim.removed_at.set(Some(sim.now.get()));
        // The first sample starts the removal time; one taken after it, with characters still
        // arriving, ends the wait.
        assert_eq!(uart.recv(1000), Ok(0));
        sim.now.set(sim.now.get() + REMOVAL_US);
        let mut fault = None;
        for _ in 0..64 {
            if let Err(RecvError::Io(e)) = uart.recv(1000) {
                fault = Some(e);
                break;
            }
        }
        assert_eq!(fault, Some(Fault::NoCard));
    }

    #[test]
    fn character_bounds_follow_the_card_clock() {
        // 12 etu at F = 372: 1116 us at 4 MHz and D = 1, half of that at D = 2; both doubled.
        assert_eq!(ATR_RATE.char_us, 2_232);
        assert_eq!(BCAS_RATE.char_us, 1_116);
    }

    #[test]
    fn absent_card_is_not_reset() {
        let sim = Sim::new(&BCAS);
        sim.removed_at.set(Some(0));
        let mut uart = Uart::new(&sim);
        assert_eq!(uart.activate().err(), Some(Fault::NoCard));
        assert_eq!(sim.gpio.get() & GPIO_CARD_RST, 0);
        assert_eq!(sim.now.get(), 0);
    }

    #[test]
    fn abort_ends_a_wait_within_one_poll() {
        for fault in [Fault::DeviceGone, Fault::Interrupted] {
            let sim = Sim::new(&BCAS);
            let mut uart = Uart::new(&sim);
            uart.activate().unwrap();
            let abort = sim.now.get() + 5_000;
            sim.abort_at.set(Some((abort, fault)));
            assert_eq!(uart.recv(1_000_000), Err(RecvError::Io(fault)));
            assert!(sim.now.get() < abort + u64::from(POLL_US));
        }
    }

    #[test]
    fn abort_ends_an_exchange_before_anything_is_sent() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        let a = uart.activate().unwrap();
        let mut s = Session::new(&a.params, a.params.timing(CARD_CLOCK_HZ));
        s.negotiate_ifsd(&mut uart).unwrap();
        let sent = sim.frames.borrow().len();
        sim.abort_at.set(Some((0, Fault::Interrupted)));
        let mut out = [0u8; 256];
        assert_eq!(
            s.transceive(&mut uart, &[0x90, 0x34, 0x00, 0x00, 0x00], &mut out),
            Err(t1::Error::Io(Fault::Interrupted))
        );
        assert_eq!(sim.frames.borrow().len(), sent);
    }

    #[test]
    fn abort_ends_a_purge_of_a_babbling_card() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.rx.borrow_mut().extend([0u8; 64]);
        sim.abort_at.set(Some((0, Fault::DeviceGone)));
        assert_eq!(uart.purge(), Err(Fault::DeviceGone));
        assert!(!sim.rx.borrow().is_empty());
    }

    #[test]
    fn the_card_can_be_held_in_reset_after_an_abort() {
        let sim = Sim::new(&BCAS);
        let mut uart = Uart::new(&sim);
        uart.activate().unwrap();
        sim.abort_at.set(Some((0, Fault::DeviceGone)));
        assert_eq!(uart.deactivate(), Ok(()));
        assert_eq!(sim.gpio.get() & GPIO_CARD_RST, 0);
    }
}
