// SPDX-License-Identifier: GPL-2.0

//! I2C master engine of the ASV5220 bridge (bus A, BAR0 0x940.., see [`MAP`]).
//!
//! The register protocol is the revealed one: clear CTL_STATUS, write ADDR, fill the TX FIFO, start
//! with a 16-bit write of `len << 8 | cmd` to CTL_STATUS, feed or drain the FIFO while the engine
//! runs, and finish on the done bit. Around that protocol the engine is robust: the FIFOs are reset
//! before a write that finds either of them dirty and before a read that finds the RX FIFO dirty
//! (such a read is not refused), and after every failed transaction, so a NACK cannot poison later
//! transfers; every wait is measured on a monotonic clock and bounded; the TX FIFO is refilled as
//! soon as it reports space instead of after a fixed delay; and an empty payload never pushes a
//! dword.
//!
//! A read does not reset on TX residue. The last dword of a write is zero padded, and whether the
//! engine discards the padding once it has sent `len` bytes is unknown; a read often continues a
//! write without STOP with the bus held open (gateway reads), and a reset there would break the
//! sequence. The residue cannot reach the wire during a read, which sends nothing from the FIFO,
//! and the next write clears it, which is the state space the hardware is known to work in (as
//! revealed).

/// Register access needed by the engine.
pub(crate) trait Regs {
    /// Reads a 32-bit register. Fails once the device is gone.
    fn read32(&self, offset: usize) -> Result<u32, Error>;
    /// Writes a 32-bit register. Fails once the device is gone.
    fn write32(&self, value: u32, offset: usize) -> Result<(), Error>;
    /// The engine starts a transaction with a 16-bit write of CTL_STATUS.
    fn write16(&self, value: u16, offset: usize) -> Result<(), Error>;
    /// Busy-waits (udelay); only used for waits of at most 50 us.
    fn delay_us(&self, us: u32);
    /// Sleeps (may schedule) for at least `us`.
    fn sleep_us(&self, us: u32);
    /// Monotonic time in microseconds.
    fn now_us(&self) -> u64;
}

impl<R: Regs + ?Sized> Regs for &R {
    fn read32(&self, offset: usize) -> Result<u32, Error> {
        (**self).read32(offset)
    }
    fn write32(&self, value: u32, offset: usize) -> Result<(), Error> {
        (**self).write32(value, offset)
    }
    fn write16(&self, value: u16, offset: usize) -> Result<(), Error> {
        (**self).write16(value, offset)
    }
    fn delay_us(&self, us: u32) {
        (**self).delay_us(us)
    }
    fn sleep_us(&self, us: u32) {
        (**self).sleep_us(us)
    }
    fn now_us(&self) -> u64 {
        (**self).now_us()
    }
}

/// Failures of an I2C transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error {
    /// Device unbound (returned by Regs).
    Gone,
    /// The engine reported an error status (no ACK).
    Nack,
    /// The transaction did not complete in time.
    Timeout,
    /// Payload longer than one transaction can carry (255 bytes).
    TooLong,
    /// The request cannot be expressed as a transaction: a write without sub-address and without
    /// data, or a read into an empty buffer. The length field of the start command would be 0 for a
    /// transfer that has nothing to move, and what the engine does with it is unknown.
    /// A write with a sub-address and no data is valid: it is the sub-address-only write that
    /// precedes a raw read.
    Invalid,
}

/// Register offsets of an engine.
pub(crate) struct Map {
    /// CTL_STATUS: written with 0, then with the start command; read for the status.
    pub(crate) ctl: usize,
    /// ADDR: `addr8 << 8 | sub-address`.
    pub(crate) addr: usize,
    /// SW_CTL: bit 5 resets the FIFOs and the engine.
    pub(crate) sw_ctl: usize,
    /// FIFO_STATUS: TX free bytes in bits 4:0, RX bytes in bits 12:8.
    pub(crate) fifo_status: usize,
    /// FIFO_DATA: 32-bit window on both FIFOs.
    pub(crate) fifo_data: usize,
}

/// Bus A of the ASV5220. The alternate map at 0xA80 is the AES block of this same register layout,
/// i.e. another bridge variant; the driver does not support it (`bringup::probe_i2c_engine`).
pub(crate) const MAP: Map = Map {
    ctl: 0x940,
    addr: 0x944,
    sw_ctl: 0x948,
    fifo_status: 0x950,
    fifo_data: 0x960,
};

/// Start command bits.
const CMD_GO: u16 = 0x80;
const CMD_SUB: u16 = 0x40;
const CMD_READ: u16 = 0x20;
const CMD_NO_STOP: u16 = 0x04;

/// Error bits of CTL_STATUS bits 31:16 (bits 19 and 21 of the register).
const STATUS_ERROR: u32 = 0x28;
const STATUS_WRITE_DONE: u32 = 1 << 22;
const STATUS_READ_DONE: u32 = 1 << 23;

/// SW_CTL bit that resets the FIFOs and the engine.
const SW_RESET: u32 = 1 << 5;
/// Hold and settle time of the reset pulse.
const RESET_PULSE_US: u32 = 50;

/// Size of each FIFO in bytes. FIFO_STATUS reports it as the TX free count of an idle engine.
const FIFO_BYTES: usize = 16;
const FIFO_FIELD_MASK: u32 = 0x1F;
const FIFO_RX_SHIFT: u32 = 8;

/// FIFO writes are spaced by this much (the revealed spacing); the hardware's real need is unknown,
/// so every dword push keeps the spacing.
const PUSH_SPACING_US: u32 = 50;

/// Polling period while waiting for the engine. A 16-byte FIFO holds more than a millisecond of
/// traffic even at 400 kHz, so this period cannot starve it.
const POLL_US: u32 = 50;

/// Time limit of a transaction: a fixed part for the start-up and the ACK, and the time of a slow
/// 10 kHz bus for each payload byte.
const TIMEOUT_BASE_US: u64 = 20_000;
const TIMEOUT_PER_BYTE_US: u64 = 1_000;

/// An I2C engine on bus A.
pub(crate) struct Engine<R: Regs> {
    regs: R,
    map: &'static Map,
}

impl<R: Regs> Engine<R> {
    /// Takes over the register window; nothing is touched until the first transaction.
    pub(crate) fn new(regs: R, map: &'static Map) -> Self {
        Engine { regs, map }
    }

    /// Write transaction: S addr [sub] data... [P].
    ///
    /// `addr` is the 8-bit write address. `sub` set sends a sub-address byte (command bit 6);
    /// `stop` false leaves the bus without STOP (command bit 2) so that a following transaction
    /// continues with a repeated start. The commands are 0xC0 (sub, stop), 0xC4 (sub, no stop),
    /// 0x80 (raw, stop) and 0x84 (raw, no stop).
    ///
    /// `data` may be empty only together with a sub-address; otherwise this fails with
    /// [`Error::Invalid`]. More than 255 bytes fail with [`Error::TooLong`]. Neither touches the
    /// hardware.
    pub(crate) fn write(
        &mut self,
        addr: u8,
        sub: Option<u8>,
        data: &[u8],
        stop: bool,
    ) -> Result<(), Error> {
        let len = u8::try_from(data.len()).map_err(|_| Error::TooLong)?;
        if sub.is_none() && data.is_empty() {
            return Err(Error::Invalid);
        }
        let mut cmd = CMD_GO;
        if sub.is_some() {
            cmd |= CMD_SUB;
        }
        if !stop {
            cmd |= CMD_NO_STOP;
        }
        self.transaction(fifos_idle, |e| e.run_write(addr, sub, data, len, cmd))
    }

    /// Read transaction into `buf` (1..=255 bytes).
    ///
    /// With a sub-address this is S addr sub Sr addr|1 data P (0xE0), without it S (or Sr) addr|1
    /// data P (0xA0). An empty `buf` fails with [`Error::Invalid`], more than 255 bytes with
    /// [`Error::TooLong`]; neither touches the hardware.
    pub(crate) fn read(&mut self, addr: u8, sub: Option<u8>, buf: &mut [u8]) -> Result<(), Error> {
        let len = u8::try_from(buf.len()).map_err(|_| Error::TooLong)?;
        if buf.is_empty() {
            return Err(Error::Invalid);
        }
        let cmd = CMD_GO | CMD_READ | if sub.is_some() { CMD_SUB } else { 0 };
        self.transaction(rx_idle, |e| e.run_read(addr, sub, buf, len, cmd))
    }

    /// Read transaction that never touches the engine's reset: for probing whether the engine
    /// answers at all, before anything else trusts the register map. A busy FIFO is reported as
    /// [`Error::Timeout`].
    pub(crate) fn probe_read(
        &mut self,
        addr: u8,
        sub: Option<u8>,
        buf: &mut [u8],
    ) -> Result<(), Error> {
        if !fifos_idle(self.fifo_status()?) {
            return Err(Error::Timeout);
        }
        let len = u8::try_from(buf.len()).map_err(|_| Error::TooLong)?;
        if len == 0 {
            return Err(Error::Invalid);
        }
        let cmd = CMD_GO | CMD_READ | if sub.is_some() { CMD_SUB } else { 0 };
        self.run_read(addr, sub, buf, len, cmd)
    }

    /// Runs one transaction between the entry check and the failure recovery. `idle` judges the
    /// FIFO_STATUS value: what residue the transaction must not start with (module documentation).
    fn transaction(
        &mut self,
        idle: fn(u32) -> bool,
        run: impl FnOnce(&Self) -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.ensure_idle(idle)?;
        run(self).map_err(|e| self.recover(e))
    }

    /// Pulses the reset when a previous transfer left residue that `idle` rejects.
    ///
    /// A stuck FIFO that survives the reset means the engine itself is wedged. Starting anyway
    /// would mix the old bytes into the new transfer, so the transaction is refused.
    fn ensure_idle(&self, idle: fn(u32) -> bool) -> Result<(), Error> {
        if idle(self.fifo_status()?) {
            return Ok(());
        }
        self.reset_pulse()?;
        if idle(self.fifo_status()?) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn fifo_status(&self) -> Result<u32, Error> {
        self.regs.read32(self.map.fifo_status)
    }

    /// Resets the FIFOs and the engine. The other SW_CTL bits are not ours and are preserved.
    fn reset_pulse(&self) -> Result<(), Error> {
        let sw = self.regs.read32(self.map.sw_ctl)?;
        self.regs.write32(sw | SW_RESET, self.map.sw_ctl)?;
        self.regs.delay_us(RESET_PULSE_US);
        self.regs.write32(sw & !SW_RESET, self.map.sw_ctl)?;
        self.regs.delay_us(RESET_PULSE_US);
        Ok(())
    }

    /// Leaves the engine clean after a failed transaction. A vanished device has nothing to clean
    /// and a failing reset means it vanished, so that is what the caller learns.
    fn recover(&self, err: Error) -> Error {
        if err == Error::Gone {
            return err;
        }
        match self.reset_pulse() {
            Ok(()) => err,
            Err(gone) => gone,
        }
    }

    /// Clears the engine and loads the address; the first two steps of every transaction.
    fn setup(&self, addr: u8, sub: Option<u8>) -> Result<(), Error> {
        self.regs.write32(0, self.map.ctl)?;
        let value = u32::from(addr) << 8 | u32::from(sub.unwrap_or(0));
        self.regs.write32(value, self.map.addr)
    }

    /// Starts the transaction and returns the time at which it has to be complete.
    fn start(&self, len: u8, cmd: u16) -> Result<u64, Error> {
        self.regs.write16(u16::from(len) << 8 | cmd, self.map.ctl)?;
        Ok(self.regs.now_us() + TIMEOUT_BASE_US + TIMEOUT_PER_BYTE_US * u64::from(len))
    }

    fn push_dword(&self, bytes: &[u8]) -> Result<(), Error> {
        let mut word = [0u8; 4];
        word[..bytes.len()].copy_from_slice(bytes);
        self.regs
            .write32(u32::from_le_bytes(word), self.map.fifo_data)
    }

    fn run_write(
        &self,
        addr: u8,
        sub: Option<u8>,
        data: &[u8],
        len: u8,
        cmd: u16,
    ) -> Result<(), Error> {
        self.setup(addr, sub)?;
        // The first 16 bytes go in before the start so that the engine never waits for the first
        // data byte. A short last chunk is zero padded to a whole dword.
        let mut sent = 0;
        let prefill = data.len().min(FIFO_BYTES);
        while sent < prefill {
            if sent != 0 {
                self.regs.delay_us(PUSH_SPACING_US);
            }
            let n = (data.len() - sent).min(4);
            self.push_dword(&data[sent..sent + n])?;
            sent += n;
        }
        let deadline = self.start(len, cmd)?;
        let done = self.wait(deadline, |e| {
            let st = e.regs.read32(e.map.ctl)?;
            if (st >> 16) & STATUS_ERROR != 0 {
                return Err(Error::Nack);
            }
            if st & STATUS_WRITE_DONE != 0 {
                return Ok(true);
            }
            // Space is counted in bytes while the engine drains a byte at a time, so a count that
            // is not a whole number of dwords means the engine is mid-dword. The count is read
            // again for every dword because the spacing delay lets the engine move on.
            while sent < data.len() {
                let free = tx_free(e.regs.read32(e.map.fifo_status)?);
                if free < 4 || free & 3 != 0 {
                    break;
                }
                let n = (data.len() - sent).min(4);
                e.push_dword(&data[sent..sent + n])?;
                sent += n;
                if sent < data.len() {
                    e.regs.delay_us(PUSH_SPACING_US);
                }
            }
            Ok(false)
        })?;
        if done {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn run_read(
        &self,
        addr: u8,
        sub: Option<u8>,
        buf: &mut [u8],
        len: u8,
        cmd: u16,
    ) -> Result<(), Error> {
        self.setup(addr, sub)?;
        let deadline = self.start(len, cmd)?;
        let mut got = 0;
        let done = self.wait(deadline, |e| {
            let st = e.regs.read32(e.map.ctl)?;
            if (st >> 16) & STATUS_ERROR != 0 {
                return Err(Error::Nack);
            }
            let done = st & STATUS_READ_DONE != 0;
            if got < buf.len() {
                let rx = rx_bytes(e.regs.read32(e.map.fifo_status)?);
                // A count that is not a whole number of dwords is a dword still being assembled.
                // Only when the engine is done can it be the final, short dword.
                if rx != 0 && (rx & 3 == 0 || done) {
                    for _ in 0..rx.div_ceil(4) {
                        if got == buf.len() {
                            break;
                        }
                        let word = e.regs.read32(e.map.fifo_data)?.to_le_bytes();
                        let n = (buf.len() - got).min(4);
                        buf[got..got + n].copy_from_slice(&word[..n]);
                        got += n;
                    }
                }
            }
            Ok(done && got == buf.len())
        })?;
        if done {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    /// Runs `step` every [`POLL_US`] until it reports completion, fails, or `deadline_us` passes.
    ///
    /// Returns whether it completed.
    fn wait(
        &self,
        deadline_us: u64,
        mut step: impl FnMut(&Self) -> Result<bool, Error>,
    ) -> Result<bool, Error> {
        loop {
            if step(self)? {
                return Ok(true);
            }
            let now = self.regs.now_us();
            if now >= deadline_us {
                // A late wake-up (or preemption between the status read and the clock read) must
                // not turn into a timeout the engine did not cause.
                return step(self);
            }
            let nap = (deadline_us - now).min(u64::from(POLL_US));
            self.regs.sleep_us(nap as u32);
        }
    }
}

/// TX free bytes of a FIFO_STATUS value.
fn tx_free(status: u32) -> usize {
    (status & FIFO_FIELD_MASK) as usize
}

/// RX bytes of a FIFO_STATUS value.
fn rx_bytes(status: u32) -> usize {
    ((status >> FIFO_RX_SHIFT) & FIFO_FIELD_MASK) as usize
}

/// Both FIFOs empty: what a write must start from.
fn fifos_idle(status: u32) -> bool {
    tx_free(status) == FIFO_BYTES && rx_bytes(status) == 0
}

/// RX FIFO empty: what a read must start from.
fn rx_idle(status: u32) -> bool {
    rx_bytes(status) == 0
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::cell::RefCell;
    use std::collections::VecDeque;
    use std::vec;
    use std::vec::Vec;

    /// An event on the I2C wire.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Wire {
        Start,
        Restart,
        Addr(u8),
        Sub(u8),
        Data(u8),
        Stop,
    }

    /// A register write or a wait, as the driver issued it.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Op {
        W32(usize, u32),
        W16(usize, u16),
        Delay(u32),
        Sleep(u32),
    }

    /// One transaction as started by the driver.
    struct Txn {
        cmd: u8,
        len: u8,
        addr: u32,
        /// TX FIFO bytes present when the start command arrived.
        prefill: usize,
        /// Dwords written to the TX FIFO for this transaction, before and after the start.
        dwords: usize,
        started_at: u64,
        wire: Vec<Wire>,
    }

    #[derive(Clone, Copy)]
    enum Step {
        Addr(u8),
        Sub(u8),
        Restart,
        Data(usize),
    }

    /// The engine's progress through one transaction.
    struct Run {
        steps: Vec<Step>,
        next: usize,
        next_at: u64,
        read: bool,
        stop: bool,
        stuck: bool,
        nack_step: Option<usize>,
    }

    /// Byte `i` that the simulated slave returns.
    fn pat(i: usize) -> u8 {
        (i as u8).wrapping_mul(7).wrapping_add(3)
    }

    /// Filler of the unused bytes of a short dword; the driver must never hand it out.
    const GARBAGE: u8 = 0xA5;
    const SW_CTL_INIT: u32 = 0x0000_0102;

    struct State {
        map: &'static Map,
        now: u64,
        access_us: u64,
        byte_us: u64,
        start_delay_us: u64,
        ops: Vec<(u64, Op)>,
        accesses: usize,
        fail_after: Option<usize>,
        ctl: u32,
        addr: u32,
        sw_ctl: u32,
        tx: VecDeque<u8>,
        rx: VecDeque<u8>,
        run: Option<Run>,
        bus_open: bool,
        txns: Vec<Txn>,
        pending_dwords: usize,
        resets: usize,
        reset_set_at: u64,
        quiet_until: u64,
        odd_tx_reads: usize,
        odd_rx_reads: usize,
        // Fault injection.
        stuck: bool,
        nack_step: Option<usize>,
        error_bit: u32,
        reset_noop: bool,
        /// The engine leaves the padding of a short last dword in the TX FIFO after a write (the
        /// hardware's behaviour is unknown; the driver must work either way).
        keep_padding: bool,
        /// First sleep that starts at or after the time oversleeps by the amount.
        oversleep: Option<(u64, u64)>,
        /// First clock read at or after the time jumps ahead by the amount.
        preempt: Option<(u64, u64)>,
    }

    /// Register-level model of the engine and of a slave that answers every address.
    struct Sim {
        st: RefCell<State>,
    }

    impl State {
        fn access(&mut self) -> Result<(), Error> {
            if self.fail_after.is_some_and(|n| self.accesses >= n) {
                return Err(Error::Gone);
            }
            self.accesses += 1;
            self.sync();
            assert!(
                self.now >= self.quiet_until,
                "register access during the reset settle time"
            );
            self.now += self.access_us;
            Ok(())
        }

        /// Runs the engine up to the present time.
        fn sync(&mut self) {
            while let Some((stuck, next_at)) = self.run.as_ref().map(|r| (r.stuck, r.next_at)) {
                if stuck || next_at > self.now {
                    break;
                }
                if self.step() {
                    if let Some(r) = self.run.as_mut() {
                        r.next_at = next_at + self.byte_us;
                    }
                } else {
                    // Stalled on a full or empty FIFO; try again at the next register access.
                    self.run.as_mut().unwrap().next_at = self.now;
                    break;
                }
            }
        }

        /// One wire step. Returns false if it is stalled on the FIFO.
        fn step(&mut self) -> bool {
            let mut run = self.run.take().unwrap();
            let idx = run.next;
            let txn = self.txns.last_mut().unwrap();
            match run.steps[idx] {
                Step::Addr(a) => txn.wire.push(Wire::Addr(a)),
                Step::Sub(s) => txn.wire.push(Wire::Sub(s)),
                Step::Restart => txn.wire.push(Wire::Restart),
                Step::Data(i) if run.read => {
                    if self.rx.len() >= FIFO_BYTES {
                        self.run = Some(run);
                        return false;
                    }
                    self.rx.push_back(pat(i));
                    txn.wire.push(Wire::Data(pat(i)));
                }
                Step::Data(_) => match self.tx.pop_front() {
                    Some(b) => txn.wire.push(Wire::Data(b)),
                    None => {
                        self.run = Some(run);
                        return false;
                    }
                },
            }
            if run.nack_step == Some(idx) {
                // The slave does not acknowledge this byte: the engine flags it and stops.
                txn.wire.push(Wire::Stop);
                self.ctl |= 1 << self.error_bit;
                self.bus_open = false;
                return true;
            }
            run.next += 1;
            if run.next == run.steps.len() {
                if run.stop {
                    txn.wire.push(Wire::Stop);
                }
                self.bus_open = !run.stop;
                if run.read {
                    self.ctl |= STATUS_READ_DONE;
                } else {
                    self.ctl |= STATUS_WRITE_DONE;
                    // The engine takes only `len` bytes; whether the padding of the last dword
                    // goes with them is a property of the hardware.
                    if !self.keep_padding {
                        self.tx.clear();
                    }
                }
                return true;
            }
            self.run = Some(run);
            true
        }

        fn start(&mut self, value: u16) {
            let cmd = (value & 0xFF) as u8;
            let len = (value >> 8) as u8;
            assert!(self.run.is_none(), "start while the engine is busy");
            assert!(self.tx.len() <= FIFO_BYTES);
            let c = u16::from(cmd);
            assert_ne!(c & CMD_GO, 0, "start without the go bit");
            assert_eq!(c & !(CMD_GO | CMD_SUB | CMD_READ | CMD_NO_STOP), 0);
            let read = c & CMD_READ != 0;
            let sub = c & CMD_SUB != 0;
            let stop = c & CMD_NO_STOP == 0;
            assert!(!read || stop, "no-stop read");
            assert!(!read || len > 0, "empty read");
            assert_eq!(self.addr >> 16, 0);
            let addr = (self.addr >> 8) as u8;
            let subaddr = self.addr as u8;
            assert!(
                sub || subaddr == 0,
                "sub-address bits set for a raw transfer"
            );

            let mut steps = Vec::new();
            if read {
                if sub {
                    steps.extend([Step::Addr(addr), Step::Sub(subaddr), Step::Restart]);
                }
                steps.push(Step::Addr(addr | 1));
            } else {
                steps.push(Step::Addr(addr));
                if sub {
                    steps.push(Step::Sub(subaddr));
                }
            }
            steps.extend((0..usize::from(len)).map(Step::Data));
            let first = if self.bus_open {
                Wire::Restart
            } else {
                Wire::Start
            };
            self.txns.push(Txn {
                cmd,
                len,
                addr: self.addr,
                prefill: self.tx.len(),
                dwords: self.pending_dwords,
                started_at: self.now,
                wire: vec![first],
            });
            self.pending_dwords = 0;
            self.run = Some(Run {
                steps,
                next: 0,
                next_at: self.now + self.start_delay_us + self.byte_us,
                read,
                stop,
                stuck: core::mem::take(&mut self.stuck),
                nack_step: self.nack_step.take(),
            });
        }

        fn reset(&mut self) {
            self.resets += 1;
            if !self.reset_noop {
                self.tx.clear();
                self.rx.clear();
                self.run = None;
                self.bus_open = false;
            }
        }
    }

    impl Regs for Sim {
        fn read32(&self, offset: usize) -> Result<u32, Error> {
            let mut s = self.st.borrow_mut();
            s.access()?;
            let map = s.map;
            Ok(if offset == map.ctl {
                s.ctl
            } else if offset == map.addr {
                s.addr
            } else if offset == map.sw_ctl {
                s.sw_ctl
            } else if offset == map.fifo_status {
                let (tx, rx) = (FIFO_BYTES - s.tx.len(), s.rx.len());
                s.odd_tx_reads += usize::from(tx % 4 != 0);
                s.odd_rx_reads += usize::from(rx % 4 != 0);
                (tx as u32) | (rx as u32) << 8
            } else if offset == map.fifo_data {
                let n = s.rx.len();
                assert!(n > 0, "FIFO read from an empty RX FIFO");
                // A dword with fewer than four bytes can only be the end of the transfer.
                assert!(
                    n >= 4 || s.ctl & STATUS_READ_DONE != 0,
                    "short dword popped while the engine is still receiving"
                );
                let mut w = [GARBAGE; 4];
                for b in w.iter_mut().take(n.min(4)) {
                    *b = s.rx.pop_front().unwrap();
                }
                u32::from_le_bytes(w)
            } else {
                panic!("read of unknown register {offset:#x}")
            })
        }

        fn write32(&self, value: u32, offset: usize) -> Result<(), Error> {
            let mut s = self.st.borrow_mut();
            s.access()?;
            let at = s.now;
            s.ops.push((at, Op::W32(offset, value)));
            let map = s.map;
            if offset == map.ctl {
                assert_eq!(value, 0);
                assert!(s.run.is_none(), "CTL cleared while the engine is busy");
                s.ctl = 0;
            } else if offset == map.addr {
                s.addr = value;
            } else if offset == map.sw_ctl {
                let rising = value & SW_RESET != 0 && s.sw_ctl & SW_RESET == 0;
                let falling = value & SW_RESET == 0 && s.sw_ctl & SW_RESET != 0;
                s.sw_ctl = value;
                if rising {
                    s.reset_set_at = at;
                    s.reset();
                }
                if falling {
                    assert!(
                        at >= s.reset_set_at + u64::from(RESET_PULSE_US),
                        "reset pulse shorter than 50 us"
                    );
                    s.quiet_until = at + u64::from(RESET_PULSE_US);
                }
            } else if offset == map.fifo_data {
                let free = FIFO_BYTES - s.tx.len();
                assert!(
                    free >= 4 && free & 3 == 0,
                    "dword pushed with {free} bytes free"
                );
                s.tx.extend(value.to_le_bytes());
                if s.run.is_some() {
                    s.txns.last_mut().unwrap().dwords += 1;
                } else {
                    s.pending_dwords += 1;
                }
            } else {
                panic!("write of unknown register {offset:#x}");
            }
            Ok(())
        }

        fn write16(&self, value: u16, offset: usize) -> Result<(), Error> {
            let mut s = self.st.borrow_mut();
            s.access()?;
            assert_eq!(offset, s.map.ctl);
            let at = s.now;
            s.ops.push((at, Op::W16(offset, value)));
            s.start(value);
            Ok(())
        }

        fn delay_us(&self, us: u32) {
            assert!(us <= 50, "busy-wait of {us} us");
            let mut s = self.st.borrow_mut();
            let at = s.now;
            s.ops.push((at, Op::Delay(us)));
            s.now += u64::from(us);
        }

        fn sleep_us(&self, us: u32) {
            let mut s = self.st.borrow_mut();
            let at = s.now;
            s.ops.push((at, Op::Sleep(us)));
            s.now += u64::from(us);
            if s.oversleep.is_some_and(|(from, _)| at >= from) {
                s.now += s.oversleep.take().unwrap().1;
            }
        }

        fn now_us(&self) -> u64 {
            let mut s = self.st.borrow_mut();
            if s.preempt.is_some_and(|(from, _)| s.now >= from) {
                s.now += s.preempt.take().unwrap().1;
            }
            s.now
        }
    }

    impl Sim {
        fn with_map(map: &'static Map) -> Self {
            Sim {
                st: RefCell::new(State {
                    map,
                    now: 1_000,
                    access_us: 1,
                    byte_us: 90,
                    start_delay_us: 0,
                    ops: Vec::new(),
                    accesses: 0,
                    fail_after: None,
                    ctl: 0,
                    addr: 0,
                    sw_ctl: SW_CTL_INIT,
                    tx: VecDeque::new(),
                    rx: VecDeque::new(),
                    run: None,
                    bus_open: false,
                    txns: Vec::new(),
                    pending_dwords: 0,
                    resets: 0,
                    reset_set_at: 0,
                    quiet_until: 0,
                    odd_tx_reads: 0,
                    odd_rx_reads: 0,
                    stuck: false,
                    nack_step: None,
                    error_bit: 19,
                    reset_noop: false,
                    keep_padding: false,
                    oversleep: None,
                    preempt: None,
                }),
            }
        }

        fn new() -> Self {
            Self::with_map(&MAP)
        }

        fn engine(&self) -> Engine<&Sim> {
            Engine::new(self, self.st.borrow().map)
        }

        fn now(&self) -> u64 {
            self.st.borrow().now
        }

        fn set_byte_us(&self, us: u64) {
            self.st.borrow_mut().byte_us = us;
        }

        fn dirty(&self, tx: usize, rx: usize) {
            let mut s = self.st.borrow_mut();
            s.tx.extend(core::iter::repeat_n(0xDD, tx));
            s.rx.extend(core::iter::repeat_n(0xEE, rx));
        }

        fn ops(&self) -> Vec<Op> {
            self.st.borrow().ops.iter().map(|&(_, o)| o).collect()
        }

        fn fifos_empty(&self) -> bool {
            let s = self.st.borrow();
            s.tx.is_empty() && s.rx.is_empty()
        }

        fn resets(&self) -> usize {
            self.st.borrow().resets
        }
    }

    fn payload(n: usize) -> Vec<u8> {
        (0..n)
            .map(|i| (i as u8).wrapping_mul(31).wrapping_add(0x11))
            .collect()
    }

    /// The reset pulse as it appears among the register writes.
    fn pulse(set: u32) -> [Op; 4] {
        [
            Op::W32(MAP.sw_ctl, set | SW_RESET),
            Op::Delay(50),
            Op::W32(MAP.sw_ctl, set),
            Op::Delay(50),
        ]
    }

    fn expected_write_wire(
        reopen: bool,
        addr: u8,
        sub: Option<u8>,
        data: &[u8],
        stop: bool,
    ) -> Vec<Wire> {
        let mut w = vec![
            if reopen { Wire::Restart } else { Wire::Start },
            Wire::Addr(addr),
        ];
        w.extend(sub.map(Wire::Sub));
        w.extend(data.iter().map(|&b| Wire::Data(b)));
        if stop {
            w.push(Wire::Stop);
        }
        w
    }

    fn expected_read_wire(reopen: bool, addr: u8, sub: Option<u8>, n: usize) -> Vec<Wire> {
        let mut w = vec![if reopen { Wire::Restart } else { Wire::Start }];
        match sub {
            Some(s) => w.extend([
                Wire::Addr(addr),
                Wire::Sub(s),
                Wire::Restart,
                Wire::Addr(addr | 1),
            ]),
            None => w.push(Wire::Addr(addr | 1)),
        }
        w.extend((0..n).map(|i| Wire::Data(pat(i))));
        w.push(Wire::Stop);
        w
    }

    /// Number of wire steps the simulated engine needs.
    fn wire_steps(read: bool, sub: bool, n: usize) -> u64 {
        let head = match (read, sub) {
            (true, true) => 4,
            (false, true) => 2,
            _ => 1,
        };
        (head + n) as u64
    }

    fn assert_waits_are_short(sim: &Sim) {
        for op in sim.ops() {
            match op {
                Op::Sleep(us) => assert!((1..=50).contains(&us)),
                Op::Delay(us) => assert!(us <= 50),
                _ => {}
            }
        }
    }

    /// Runs one write and checks everything the register protocol requires of it.
    fn check_write(byte_us: u64, len: usize, sub: Option<u8>, stop: bool) {
        let sim = Sim::new();
        sim.set_byte_us(byte_us);
        let data = payload(len);
        let t0 = sim.now();
        sim.engine().write(0x30, sub, &data, stop).unwrap();
        let elapsed = sim.now() - t0;

        let s = sim.st.borrow();
        assert_eq!(s.txns.len(), 1);
        let t = &s.txns[0];
        let cmd = 0x80 | if sub.is_some() { 0x40 } else { 0 } | if stop { 0 } else { 0x04 };
        assert_eq!(t.cmd, cmd);
        assert_eq!(usize::from(t.len), len);
        assert_eq!(t.addr, 0x3000 | u32::from(sub.unwrap_or(0)));
        assert_eq!(t.wire, expected_write_wire(false, 0x30, sub, &data, stop));
        // Nothing is pushed beyond the payload, and the prefill is the first 16 bytes.
        assert_eq!(t.dwords, len.div_ceil(4));
        assert_eq!(t.prefill, (len.div_ceil(4) * 4).min(FIFO_BYTES));
        // Clear, ADDR, prefill dwords spaced by 50 us, then the start.
        let d = t.prefill / 4;
        let ops: Vec<Op> = s.ops.iter().map(|&(_, o)| o).collect();
        assert_eq!(ops[0], Op::W32(MAP.ctl, 0));
        assert_eq!(ops[1], Op::W32(MAP.addr, t.addr));
        let start_idx = 2 + (2 * d).saturating_sub(1);
        assert_eq!(
            ops[start_idx],
            Op::W16(MAP.ctl, (len as u16) << 8 | u16::from(cmd))
        );
        for k in 0..d {
            let mut word = [0u8; 4];
            let chunk = &data[4 * k..(4 * k + 4).min(len)];
            word[..chunk.len()].copy_from_slice(chunk);
            assert_eq!(
                ops[2 + 2 * k],
                Op::W32(MAP.fifo_data, u32::from_le_bytes(word))
            );
            if k + 1 < d {
                assert_eq!(ops[3 + 2 * k], Op::Delay(50));
            }
        }
        assert_eq!(s.resets, 0);
        assert!(s.tx.is_empty() && s.rx.is_empty());
        // No fixed refill waits: the transfer takes about as long as the wire needs.
        let wire = wire_steps(false, sub.is_some(), len) * byte_us;
        assert!(
            elapsed <= wire + 1_000,
            "{elapsed} us for {wire} us of wire time"
        );
        drop(s);
        assert_waits_are_short(&sim);
    }

    /// Runs one read and checks everything the register protocol requires of it.
    fn check_read(byte_us: u64, len: usize, sub: Option<u8>) {
        let sim = Sim::new();
        sim.set_byte_us(byte_us);
        // Bytes past the requested length must stay untouched.
        let mut arr = [0xEE_u8; 260];
        let t0 = sim.now();
        sim.engine().read(0x32, sub, &mut arr[..len]).unwrap();
        let elapsed = sim.now() - t0;
        let want: Vec<u8> = (0..len).map(pat).collect();
        assert_eq!(&arr[..len], &want[..]);
        assert!(arr[len..].iter().all(|&b| b == 0xEE));

        let s = sim.st.borrow();
        let t = &s.txns[0];
        let cmd = if sub.is_some() { 0xE0 } else { 0xA0 };
        assert_eq!(t.cmd, cmd);
        assert_eq!(usize::from(t.len), len);
        assert_eq!(t.addr, 0x3200 | u32::from(sub.unwrap_or(0)));
        assert_eq!(t.wire, expected_read_wire(false, 0x32, sub, len));
        assert_eq!(t.prefill, 0);
        assert_eq!(t.dwords, 0);
        let ops: Vec<Op> = s.ops.iter().map(|&(_, o)| o).collect();
        assert_eq!(ops[0], Op::W32(MAP.ctl, 0));
        assert_eq!(ops[1], Op::W32(MAP.addr, t.addr));
        assert_eq!(ops[2], Op::W16(MAP.ctl, (len as u16) << 8 | u16::from(cmd)));
        assert_eq!(ops.iter().filter(|o| matches!(o, Op::W32(..))).count(), 2);
        assert_eq!(s.resets, 0);
        assert!(s.tx.is_empty() && s.rx.is_empty());
        let wire = wire_steps(true, sub.is_some(), len) * byte_us;
        assert!(
            elapsed <= wire + 1_000,
            "{elapsed} us for {wire} us of wire time"
        );
        drop(s);
        assert_waits_are_short(&sim);
    }

    const LENGTHS: [usize; 42] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 255,
    ];

    #[test]
    fn the_register_map_matches_the_documented_layout() {
        assert_eq!(
            [
                MAP.ctl,
                MAP.addr,
                MAP.sw_ctl,
                MAP.fifo_status,
                MAP.fifo_data
            ],
            [0x940, 0x944, 0x948, 0x950, 0x960]
        );
    }

    #[test]
    fn every_write_command_with_every_length() {
        // 13 us is faster than the poll period and the dword spacing, 90 us is 100 kHz, 700 us
        // leaves the FIFO empty.
        for byte_us in [13, 90, 700] {
            for len in LENGTHS {
                for sub in [Some(0x12), None] {
                    for stop in [true, false] {
                        if len == 0 && sub.is_none() {
                            continue;
                        }
                        check_write(byte_us, len, sub, stop);
                    }
                }
            }
        }
    }

    #[test]
    fn write_command_codes() {
        for (sub, stop, cmd) in [
            (Some(1), true, 0xC0),
            (Some(1), false, 0xC4),
            (None, true, 0x80),
            (None, false, 0x84),
        ] {
            let sim = Sim::new();
            sim.engine().write(0x30, sub, &[9], stop).unwrap();
            assert_eq!(sim.st.borrow().txns[0].cmd, cmd);
        }
    }

    #[test]
    fn every_read_command_with_every_length() {
        for byte_us in [13, 90, 700] {
            for len in LENGTHS.into_iter().filter(|&n| n != 0) {
                for sub in [Some(0x34), None] {
                    check_read(byte_us, len, sub);
                }
            }
        }
    }

    #[test]
    fn read_command_codes() {
        for (sub, cmd) in [(Some(1), 0xE0), (None, 0xA0)] {
            let sim = Sim::new();
            sim.engine().read(0x30, sub, &mut [0; 3]).unwrap();
            assert_eq!(sim.st.borrow().txns[0].cmd, cmd);
        }
    }

    #[test]
    fn sub_address_only_write_is_legal_and_pushes_nothing() {
        for stop in [true, false] {
            let sim = Sim::new();
            sim.engine().write(0x30, Some(0x55), &[], stop).unwrap();
            let s = sim.st.borrow();
            assert_eq!(s.txns[0].cmd, if stop { 0xC0 } else { 0xC4 });
            assert_eq!(s.txns[0].len, 0);
            assert_eq!(s.txns[0].dwords, 0);
            let mut wire = vec![Wire::Start, Wire::Addr(0x30), Wire::Sub(0x55)];
            wire.extend(stop.then_some(Wire::Stop));
            assert_eq!(s.txns[0].wire, wire);
            assert!(!s
                .ops
                .iter()
                .any(|&(_, o)| matches!(o, Op::W32(off, _) if off == MAP.fifo_data)));
        }
    }

    #[test]
    fn empty_requests_without_data_are_invalid_and_touch_nothing() {
        for stop in [true, false] {
            let sim = Sim::new();
            let t0 = sim.now();
            assert_eq!(
                sim.engine().write(0x30, None, &[], stop),
                Err(Error::Invalid)
            );
            assert_eq!(sim.engine().read(0x30, None, &mut []), Err(Error::Invalid));
            assert_eq!(
                sim.engine().read(0x30, Some(1), &mut []),
                Err(Error::Invalid)
            );
            assert_eq!(sim.st.borrow().accesses, 0);
            assert_eq!(sim.now(), t0);
        }
    }

    #[test]
    fn too_long_requests_fail_before_touching_hardware() {
        for len in [256, 257, 300, 4096, 70_000] {
            let sim = Sim::new();
            let t0 = sim.now();
            let data = payload(len);
            for sub in [Some(1), None] {
                assert_eq!(
                    sim.engine().write(0x30, sub, &data, true),
                    Err(Error::TooLong)
                );
                let mut buf = vec![0; len];
                assert_eq!(sim.engine().read(0x30, sub, &mut buf), Err(Error::TooLong));
            }
            assert_eq!(sim.st.borrow().accesses, 0);
            assert_eq!(sim.now(), t0);
        }
    }

    #[test]
    fn dwords_are_little_endian_and_the_tail_is_padded() {
        let sim = Sim::new();
        sim.engine()
            .write(0x30, Some(0x10), &[1, 2, 3, 4, 5, 6], true)
            .unwrap();
        let ops = sim.ops();
        assert_eq!(
            &ops[..6],
            [
                Op::W32(MAP.ctl, 0),
                Op::W32(MAP.addr, 0x3010),
                Op::W32(MAP.fifo_data, 0x0403_0201),
                Op::Delay(50),
                Op::W32(MAP.fifo_data, 0x0000_0605),
                Op::W16(MAP.ctl, 0x06C0),
            ]
        );
    }

    /// The engine code takes its offsets from the map alone (here bus B of the bridge, which the
    /// driver does not use).
    #[test]
    fn another_register_map_works() {
        const BUS_B: Map = Map {
            ctl: 0x980,
            addr: 0x984,
            sw_ctl: 0x988,
            fifo_status: 0x990,
            fifo_data: 0x9A0,
        };
        let sim = Sim::with_map(&BUS_B);
        let data = payload(33);
        sim.engine().write(0x34, Some(3), &data, true).unwrap();
        let mut buf = [0; 33];
        sim.engine().read(0x34, Some(3), &mut buf).unwrap();
        assert_eq!(buf[..], (0..33).map(pat).collect::<Vec<_>>()[..]);
        let ops = sim.ops();
        assert_eq!(ops[0], Op::W32(BUS_B.ctl, 0));
        assert_eq!(ops[1], Op::W32(BUS_B.addr, 0x3403));
    }

    #[test]
    fn a_transfer_without_stop_continues_with_a_repeated_start() {
        let sim = Sim::new();
        let mut e = sim.engine();
        e.write(0x30, None, &[0xFE, 0xC7], false).unwrap();
        let mut buf = [0; 2];
        e.read(0x30, None, &mut buf).unwrap();
        let s = sim.st.borrow();
        assert_eq!(s.txns[1].wire, expected_read_wire(true, 0x30, None, 2));
        assert_eq!(s.txns[1].wire[..2], [Wire::Restart, Wire::Addr(0x31)]);
    }

    #[test]
    fn the_simulator_exposes_partial_dword_counts() {
        let sim = Sim::new();
        sim.engine().write(0x30, None, &payload(40), true).unwrap();
        let mut buf = [0; 40];
        sim.engine().read(0x30, None, &mut buf).unwrap();
        let s = sim.st.borrow();
        assert!(s.odd_tx_reads > 0 && s.odd_rx_reads > 0);
    }

    #[test]
    fn a_dirty_fifo_is_reset_before_a_write_and_rx_residue_before_a_read() {
        for (tx, rx) in [(4, 0), (1, 0), (16, 0), (0, 1), (0, 4), (0, 16), (8, 3)] {
            let data = payload(20);
            let sim = Sim::new();
            sim.dirty(tx, rx);
            sim.engine().write(0x30, Some(2), &data, true).unwrap();
            let ops = sim.ops();
            assert_eq!(ops[..4], pulse(SW_CTL_INIT), "write, tx {tx} rx {rx}");
            assert_eq!(ops[4], Op::W32(MAP.ctl, 0));
            assert_eq!(sim.resets(), 1);
            assert_eq!(
                sim.st.borrow().txns[0].wire,
                expected_write_wire(false, 0x30, Some(2), &data, true)
            );

            let sim = Sim::new();
            sim.dirty(tx, rx);
            let mut buf = [0; 20];
            sim.engine().read(0x30, Some(2), &mut buf).unwrap();
            let ops = sim.ops();
            if rx != 0 {
                assert_eq!(ops[..4], pulse(SW_CTL_INIT), "read, tx {tx} rx {rx}");
                assert_eq!(ops[4], Op::W32(MAP.ctl, 0));
                assert_eq!(sim.resets(), 1);
            } else {
                assert_eq!(ops[0], Op::W32(MAP.ctl, 0), "read, tx {tx}");
                assert_eq!(sim.resets(), 0);
            }
            assert_eq!(buf[..], (0..20).map(pat).collect::<Vec<_>>()[..]);
        }
    }

    /// An engine that keeps the padding of a short last dword: the read that continues a stopless
    /// write must not reset the engine with the bus held open, and the next write cleans up.
    #[test]
    fn a_read_after_a_short_stopless_write_keeps_the_held_bus() {
        let sim = Sim::new();
        sim.st.borrow_mut().keep_padding = true;
        let mut e = sim.engine();
        e.write(0x30, Some(0xFE), &[0xA8, 0x12], true).unwrap();
        assert_eq!(sim.resets(), 0);
        e.write(0x30, Some(0xFE), &[0xA9], false).unwrap();
        // The first write left two padding bytes, which the second write reset away before
        // starting; its own padding is in the FIFO now.
        assert_eq!(sim.resets(), 1);
        assert!(!sim.fifos_empty());
        let mut buf = [0; 1];
        e.read(0x30, None, &mut buf).unwrap();
        assert_eq!(sim.resets(), 1);
        assert_eq!(buf, [pat(0)]);
        {
            let s = sim.st.borrow();
            assert_eq!(s.txns.len(), 3);
            assert_eq!(s.txns[2].wire, expected_read_wire(true, 0x30, None, 1));
        }
        e.write(0x30, Some(2), &[1], true).unwrap();
        assert_eq!(sim.resets(), 2);
        assert_eq!(sim.st.borrow().txns.len(), 4);
    }

    #[test]
    fn an_idle_fifo_is_not_reset() {
        let sim = Sim::new();
        sim.engine()
            .write(0x30, Some(2), &payload(8), true)
            .unwrap();
        sim.engine().read(0x30, Some(2), &mut [0; 8]).unwrap();
        assert_eq!(sim.resets(), 0);
        assert!(!sim
            .ops()
            .iter()
            .any(|o| matches!(o, Op::W32(off, _) if *off == MAP.sw_ctl)));
    }

    #[test]
    fn a_fifo_that_survives_the_reset_refuses_the_transaction() {
        for read in [false, true] {
            let sim = Sim::new();
            sim.dirty(0, 2);
            sim.st.borrow_mut().reset_noop = true;
            let r = if read {
                sim.engine().read(0x30, None, &mut [0; 4])
            } else {
                sim.engine().write(0x30, None, &[1], true)
            };
            assert_eq!(r, Err(Error::Timeout));
            assert_eq!(sim.ops(), pulse(SW_CTL_INIT));
            assert!(sim.st.borrow().txns.is_empty());
        }
    }

    /// Failure positions for a transfer: the address, the sub-address and several data bytes.
    #[test]
    fn probe_read_never_resets_the_engine() {
        let sim = Sim::new();
        let mut buf = [0; 1];
        sim.engine().probe_read(0xA0, Some(0), &mut buf).unwrap();
        assert_eq!(buf[0], pat(0));
        // A busy FIFO is reported, not reset.
        let sim = Sim::new();
        sim.dirty(0, 4);
        assert_eq!(
            sim.engine().probe_read(0xA0, Some(0), &mut buf),
            Err(Error::Timeout)
        );
        assert_eq!(sim.resets(), 0);
        // A NACK is reported and left as it is.
        let sim = Sim::new();
        sim.st.borrow_mut().nack_step = Some(0);
        sim.st.borrow_mut().error_bit = 19;
        assert_eq!(
            sim.engine().probe_read(0xA0, Some(0), &mut buf),
            Err(Error::Nack)
        );
        assert_eq!(sim.resets(), 0);
    }

    fn nack_steps(total: usize) -> [usize; 5] {
        [0, 1, 2, total / 2, total - 1]
    }

    #[test]
    fn a_nack_resets_the_fifos_and_the_next_transaction_starts_clean() {
        for error_bit in [19, 21] {
            for k in nack_steps(2 + 40) {
                let sim = Sim::new();
                sim.st.borrow_mut().nack_step = Some(k);
                sim.st.borrow_mut().error_bit = error_bit;
                let r = sim.engine().write(0x30, Some(1), &payload(40), true);
                assert_eq!(r, Err(Error::Nack), "write, step {k}, bit {error_bit}");
                assert_eq!(sim.resets(), 1);
                assert!(sim.fifos_empty());
                let ops = sim.ops();
                let tail = &ops[ops.iter().rposition(|o| matches!(o, Op::W16(..))).unwrap()..];
                assert!(tail.ends_with(&pulse(SW_CTL_INIT)));
                // The next transaction needs no reset of its own and is not disturbed.
                let mut buf = [0; 9];
                sim.engine().read(0x30, Some(1), &mut buf).unwrap();
                assert_eq!(buf[..], (0..9).map(pat).collect::<Vec<_>>()[..]);
                assert_eq!(sim.resets(), 1);
            }
            for k in nack_steps(4 + 20) {
                let sim = Sim::new();
                sim.st.borrow_mut().nack_step = Some(k);
                sim.st.borrow_mut().error_bit = error_bit;
                let r = sim.engine().read(0x30, Some(1), &mut [0; 20]);
                assert_eq!(r, Err(Error::Nack), "read, step {k}, bit {error_bit}");
                assert_eq!(sim.resets(), 1);
                assert!(sim.fifos_empty());
                // Stale RX bytes of the failed read must not reach the next one.
                let mut buf = [0; 6];
                sim.engine().read(0x30, None, &mut buf).unwrap();
                assert_eq!(buf[..], (0..6).map(pat).collect::<Vec<_>>()[..]);
                assert_eq!(sim.resets(), 1);
            }
        }
    }

    #[test]
    fn a_nack_after_a_stopless_write_leaves_a_clean_bus_state() {
        let sim = Sim::new();
        let mut e = sim.engine();
        e.write(0x30, None, &[1], false).unwrap();
        sim.st.borrow_mut().nack_step = Some(0);
        assert_eq!(e.read(0x30, None, &mut [0; 1]), Err(Error::Nack));
        assert_eq!(
            sim.st.borrow().txns[1].wire[..2],
            [Wire::Restart, Wire::Addr(0x31)]
        );
        e.read(0x30, None, &mut [0; 1]).unwrap();
        assert_eq!(sim.st.borrow().txns[2].wire[0], Wire::Start);
    }

    #[test]
    fn a_stuck_transaction_times_out_in_real_time_and_is_reset() {
        for (read, len) in [
            (false, 0),
            (false, 1),
            (false, 40),
            (false, 255),
            (true, 1),
            (true, 40),
            (true, 255),
        ] {
            let sim = Sim::new();
            sim.st.borrow_mut().stuck = true;
            let r = if read {
                sim.engine().read(0x30, Some(1), &mut vec![0; len])
            } else {
                sim.engine().write(0x30, Some(1), &payload(len), true)
            };
            assert_eq!(r, Err(Error::Timeout));
            let limit = 20_000 + 1_000 * len as u64;
            let s = sim.st.borrow();
            let spent = s.now - s.txns[0].started_at;
            // The limit, plus the reset pulse, plus the last poll step.
            assert!(
                (limit + 100..=limit + 200).contains(&spent),
                "{spent} us for a {limit} us limit"
            );
            assert_eq!(s.resets, 1);
            assert!(s.tx.is_empty() && s.rx.is_empty());
            assert!(s.run.is_none());
            drop(s);
            assert_waits_are_short(&sim);
            // The engine is usable again at once.
            sim.engine().write(0x30, Some(1), &[7], true).unwrap();
            assert_eq!(sim.resets(), 1);
        }
    }

    #[test]
    fn a_late_wake_up_is_not_a_timeout() {
        // 0: on time, 1: the last sleep overshoots, 2: the clock read after the last status read
        // is delayed.
        for read in [false, true] {
            let run = |late: u8| {
                let sim = Sim::new();
                let t0 = sim.now();
                {
                    let mut s = sim.st.borrow_mut();
                    // The engine finishes 0.5 ms after the 21 ms limit of a one byte transfer.
                    s.start_delay_us = 21_500;
                    match late {
                        1 => s.oversleep = Some((t0 + 20_000, 2_000)),
                        2 => s.preempt = Some((t0 + 20_990, 2_000)),
                        _ => {}
                    }
                }
                let r = if read {
                    sim.engine().read(0x30, None, &mut [0; 1])
                } else {
                    sim.engine().write(0x30, Some(1), &[1], true)
                };
                (r, sim.now() - t0)
            };
            assert_eq!(run(0).0, Err(Error::Timeout));
            for late in [1, 2] {
                let (r, spent) = run(late);
                assert_eq!(r, Ok(()), "read {read}, late {late}");
                assert!(spent > 21_000);
            }
        }
    }

    #[test]
    fn a_vanished_device_is_reported_at_every_register_access() {
        type Case = (&'static str, fn(&Sim) -> Result<(), Error>, bool, bool);
        let cases: [Case; 4] = [
            (
                "write",
                |s| s.engine().write(0x30, Some(1), &payload(21), true),
                false,
                false,
            ),
            (
                "read",
                |s| s.engine().read(0x30, Some(1), &mut [0; 21]),
                false,
                false,
            ),
            (
                "dirty write",
                |s| s.engine().write(0x30, Some(1), &payload(5), true),
                true,
                false,
            ),
            (
                "nack read",
                |s| s.engine().read(0x30, Some(1), &mut [0; 5]),
                false,
                true,
            ),
        ];
        for (name, op, dirty, nack) in cases {
            let setup = || {
                let sim = Sim::new();
                if dirty {
                    sim.dirty(4, 4);
                }
                if nack {
                    sim.st.borrow_mut().nack_step = Some(2);
                }
                sim
            };
            let baseline = setup();
            let expect = if nack { Err(Error::Nack) } else { Ok(()) };
            assert_eq!(op(&baseline), expect, "{name}");
            let total = baseline.st.borrow().accesses;
            for n in 0..total {
                let sim = setup();
                sim.st.borrow_mut().fail_after = Some(n);
                assert_eq!(op(&sim), Err(Error::Gone), "{name}, access {n} of {total}");
            }
        }
    }
}
