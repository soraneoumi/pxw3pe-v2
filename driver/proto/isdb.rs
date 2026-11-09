// SPDX-License-Identifier: GPL-2.0

//! The demodulator cores and the tuners behind them as one chip set: gateway framing, finding the
//! demodulator group, initialisation, tuning and status.
//!
//! The register sequences and conversions live in [`tc90522`], [`tda2014x`] and [`nmi`]; this
//! module orders them and puts them on the wire through [`Hw`]. Every call takes the 7-bit
//! address of one demodulator core, so it serves any of the four cores of the board.

use super::{
    nmi::{self, Nmi, NmiBus, NmiError},
    tc90522 as tc,
    tda2014x::{self, Lna, TdaBus, TdaError},
    ts::BASES,
};

/// Bus access to the demodulator cores and, through their I2C gateway, to the tuners. Every method
/// is one bus hold, so no other transaction comes between the parts of a gateway read. Addresses
/// are 7-bit; `tuner` is the 8-bit write address of the tuner chip as the gateway takes it.
pub(crate) trait Hw {
    /// Error of a failed bus transaction.
    type Error;

    /// `S addr reg val P`.
    fn demod_write(&mut self, addr: u8, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// `S addr reg Sr addr+1 buf P`: a burst of `buf.len()` registers from `reg` on, which the
    /// demodulator reads with auto-increment.
    fn demod_read(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// `S addr FE tuner data P`.
    fn gate_write(&mut self, addr: u8, tuner: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// `S addr FE tuner sub P`, then `S addr FE tuner|1 Sr addr+1 buf P`, in one bus hold.
    fn gate_read(
        &mut self,
        addr: u8,
        tuner: u8,
        sub: &[u8],
        buf: &mut [u8],
    ) -> Result<(), Self::Error>;

    /// Sleeps for `ms` milliseconds without holding the bus.
    fn sleep_ms(&mut self, ms: u32);
}

/// Why an operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error<E> {
    /// A bus transaction failed. The chips may be partly programmed.
    Bus(E),
    /// The satellite tuner's VCO calibration did not finish.
    Calibration,
    /// The terrestrial tuner is not an NM120; carries the chip ID it reported.
    UnsupportedChip(u32),
    /// The frequency is outside what the adapter accepts. Nothing was written.
    Frequency,
}

impl<E> From<TdaError<E>> for Error<E> {
    fn from(e: TdaError<E>) -> Self {
        match e {
            TdaError::Bus(e) => Error::Bus(e),
            TdaError::Calibration => Error::Calibration,
            TdaError::Frequency => Error::Frequency,
        }
    }
}

impl<E> From<NmiError<E>> for Error<E> {
    fn from(e: NmiError<E>) -> Self {
        match e {
            NmiError::Bus(e) => Error::Bus(e),
            NmiError::UnsupportedChip(id) => Error::UnsupportedChip(id),
            NmiError::Frequency => Error::Frequency,
        }
    }
}

/// 8-bit write address of the satellite tuner behind the gateway of an ISDB-S core.
const TDA_ADDR: u8 = 0xA8;
/// 8-bit write address of the terrestrial tuner behind the gateway of an ISDB-T core.
const NMI_ADDR: u8 = 0xCE;

/// Wait between the tuner programming and the post-tune writes that start the demodulator's
/// acquisition. The reference waits 250 ms; on the card 100, 50 and 10 ms gave the same C/N, and
/// the lock came that much sooner, so 100 ms is used: the tuner is already locked (VCO calibration,
/// PLL check) when the wait starts.
const SETTLE_MS: u32 = 100;

/// Lowest ISDB-S intermediate frequency the adapter accepts, in kHz. The tuner takes a wider range
/// (`tda2014x::tune`); the adapter limits are the bounds of the channel tables.
pub(crate) const S_IF_MIN_KHZ: u32 = 1_049_480;
/// Highest ISDB-S intermediate frequency the adapter accepts, in kHz.
pub(crate) const S_IF_MAX_KHZ: u32 = 2_053_000;

/// The ISDB-S intermediate frequency `khz` if the adapter accepts it.
pub(crate) fn s_if_khz(khz: u32) -> Option<u32> {
    (S_IF_MIN_KHZ..=S_IF_MAX_KHZ).contains(&khz).then_some(khz)
}

/// The frequency the ISDB-T tuner is programmed with for the request `hz`: rounded to the nearest
/// kHz, `None` outside the 93 to 768 MHz the adapter declares. The request is checked after
/// rounding, as the tuner only sees the rounded value.
pub(crate) fn t_rf_hz(hz: u32) -> Option<u32> {
    let rounded = hz.checked_add(500)? / 1000 * 1000;
    (nmi::RF_MIN_HZ..=nmi::RF_MAX_HZ)
        .contains(&rounded)
        .then_some(rounded)
}

/// Register access to the TDA2014x behind the satellite core at `addr`.
struct TdaGate<'a, H: Hw> {
    hw: &'a mut H,
    addr: u8,
}

impl<H: Hw> TdaBus for TdaGate<'_, H> {
    type Error = H::Error;

    fn read(&mut self, reg: u8) -> Result<u8, H::Error> {
        let mut value = [0];
        self.hw.gate_read(self.addr, TDA_ADDR, &[reg], &mut value)?;
        Ok(value[0])
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), H::Error> {
        self.hw.gate_write(self.addr, TDA_ADDR, &[reg, value])
    }

    fn sleep_ms(&mut self, ms: u32) {
        self.hw.sleep_ms(ms);
    }
}

/// Register access to the NM120 behind the terrestrial core at `addr`. The 8-bit
/// registers are the 16-bit addresses 0x0000..0x00FF, so both widths share the framing: the
/// 16-bit address big endian, then the value (little endian, 8 or 32 bits).
struct NmiGate<'a, H: Hw> {
    hw: &'a mut H,
    addr: u8,
}

impl<H: Hw> NmiBus for NmiGate<'_, H> {
    type Error = H::Error;

    fn read8(&mut self, reg: u8) -> Result<u8, H::Error> {
        let mut value = [0];
        self.hw
            .gate_read(self.addr, NMI_ADDR, &[0x00, reg], &mut value)?;
        Ok(value[0])
    }

    fn write8(&mut self, reg: u8, value: u8) -> Result<(), H::Error> {
        self.hw.gate_write(self.addr, NMI_ADDR, &[0x00, reg, value])
    }

    fn read32(&mut self, addr: u16) -> Result<u32, H::Error> {
        let mut value = [0; 4];
        self.hw
            .gate_read(self.addr, NMI_ADDR, &addr.to_be_bytes(), &mut value)?;
        Ok(u32::from_le_bytes(value))
    }

    fn write32(&mut self, addr: u16, value: u32) -> Result<(), H::Error> {
        let [hi, lo] = addr.to_be_bytes();
        let [b0, b1, b2, b3] = value.to_le_bytes();
        self.hw
            .gate_write(self.addr, NMI_ADDR, &[hi, lo, b0, b1, b2, b3])
    }

    fn sleep_ms(&mut self, ms: u32) {
        self.hw.sleep_ms(ms);
    }
}

/// First 7-bit address of the demodulator cores that can exist.
const FIRST_CORE: u8 = 0x10;
/// Cores per group.
const GROUP_LEN: u8 = 4;

/// No demodulator group was found: none, several or an incomplete one answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ProbeError {
    /// Bit `n` is set if the core at the 7-bit address `0x10 + n` answered.
    pub(crate) answered: u8,
}

/// Finds the demodulator group: reads the status register of the cores at 0x10..0x17, T
/// register 0x80 at the even and S register 0xC3 at the odd addresses, and returns the base
/// address of the one group, 0x10 or 0x14, whose four cores all answer. A core that fails to
/// answer for any reason counts as absent; the error says what answered.
pub(crate) fn probe<H: Hw>(hw: &mut H) -> Result<u8, ProbeError> {
    let mut answered = 0u8;
    for n in 0..2 * GROUP_LEN {
        let addr = FIRST_CORE + n;
        let reg = if addr & 1 == 0 {
            tc::REG_T_STATUS
        } else {
            tc::REG_S_STATUS
        };
        if hw.demod_read(addr, reg, &mut [0]).is_ok() {
            answered |= 1 << n;
        }
    }
    let group = (1u8 << GROUP_LEN) - 1;
    let mut complete = BASES
        .into_iter()
        .filter(|&base| (answered >> (base - FIRST_CORE)) & group == group);
    match (complete.next(), complete.next()) {
        (Some(base), None) => Ok(base),
        _ => Err(ProbeError { answered }),
    }
}

fn write_all<H: Hw>(hw: &mut H, addr: u8, writes: &[tc::Write]) -> Result<(), Error<H::Error>> {
    for &(reg, value) in writes {
        hw.demod_write(addr, reg, value).map_err(Error::Bus)?;
    }
    Ok(())
}

fn read_reg<H: Hw>(hw: &mut H, addr: u8, reg: u8) -> Result<u8, Error<H::Error>> {
    let mut value = [0];
    hw.demod_read(addr, reg, &mut value).map_err(Error::Bus)?;
    Ok(value[0])
}

/// Initialises the ISDB-S core at `addr` and its tuner (every step is checked).
pub(crate) fn init_s<H: Hw>(hw: &mut H, addr: u8) -> Result<(), Error<H::Error>> {
    write_all(hw, addr, &tc::INIT_S)?;
    tda2014x::init(&mut TdaGate { hw: &mut *hw, addr })?;
    Ok(())
}

/// Initialises the ISDB-T core at `addr` and its tuner, which must be an NM120. Returns
/// the tuner's state for [`tune_t`].
pub(crate) fn init_t<H: Hw>(hw: &mut H, addr: u8) -> Result<Nmi, Error<H::Error>> {
    write_all(hw, addr, &tc::INIT_T)?;
    Ok(Nmi::init(&mut NmiGate { hw: &mut *hw, addr })?)
}

/// Tunes the ISDB-S core at `addr` to the intermediate frequency `if_khz` with LNA code `lna`
/// and selects the TSID `tsid` (0xFFFF for any TS).
///
/// The TSID is written on every tune, before the reacquire that ends the sequence, so that the
/// demodulator acquires the stream that was asked for. There is no lock wait: the caller polls
/// [`status_s`].
pub(crate) fn tune_s<H: Hw>(
    hw: &mut H,
    addr: u8,
    if_khz: u32,
    tsid: u16,
    lna: Lna,
) -> Result<(), Error<H::Error>> {
    let if_khz = s_if_khz(if_khz).ok_or(Error::Frequency)?;
    write_all(hw, addr, &tc::PRE_TUNE_S)?;
    tda2014x::tune(&mut TdaGate { hw: &mut *hw, addr }, if_khz, lna)?;
    hw.sleep_ms(SETTLE_MS);
    write_all(hw, addr, &tc::POST_TUNE_S)?;
    write_all(hw, addr, &tc::tsid_writes(tsid))?;
    write_all(hw, addr, &[tc::REACQUIRE_S])
}

/// Stops the ISDB-T core at `addr` and then the tuner behind it: the core first, so that its TS
/// ends before the tuner's signal does.
pub(crate) fn sleep_t<H: Hw>(hw: &mut H, nmi: &mut Nmi, addr: u8) -> Result<(), Error<H::Error>> {
    write_all(hw, addr, &[tc::SLEEP_T])?;
    Ok(nmi.sleep(&mut NmiGate { hw: &mut *hw, addr })?)
}

/// Restarts the tuner behind the ISDB-T core at `addr` and then the core, which resumes the
/// channel it was tuned to; the next tune programs the frequency.
pub(crate) fn wake_t<H: Hw>(hw: &mut H, nmi: &mut Nmi, addr: u8) -> Result<(), Error<H::Error>> {
    nmi.wake(&mut NmiGate { hw: &mut *hw, addr })?;
    write_all(hw, addr, &[tc::WAKE_T])
}

/// Stops the ISDB-S core at `addr` and then the tuner behind it, as [`sleep_t`]. The tuner's RF
/// input stage and loop-through output stay on (`tda2014x::sleep`).
pub(crate) fn sleep_s<H: Hw>(hw: &mut H, addr: u8) -> Result<(), Error<H::Error>> {
    write_all(hw, addr, &[tc::SLEEP_S])?;
    tda2014x::sleep(&mut TdaGate { hw: &mut *hw, addr }).map_err(Error::Bus)
}

/// Restarts the tuner behind the ISDB-S core at `addr` and then the core, as [`wake_t`].
pub(crate) fn wake_s<H: Hw>(hw: &mut H, addr: u8) -> Result<(), Error<H::Error>> {
    tda2014x::wake(&mut TdaGate { hw: &mut *hw, addr })?;
    write_all(hw, addr, &[tc::WAKE_S])
}

/// Changes the LNA code of the satellite tuner behind `addr` without retuning.
pub(crate) fn set_lna_s<H: Hw>(hw: &mut H, addr: u8, lna: Lna) -> Result<(), Error<H::Error>> {
    tda2014x::set_lna(&mut TdaGate { hw: &mut *hw, addr }, lna).map_err(Error::Bus)
}

/// Tunes the ISDB-T core at `addr` to `hz`, rounded to the nearest kHz. `nmi` is the state
/// [`init_t`] returned for this core. There is no lock wait: the caller polls [`status_t`].
pub(crate) fn tune_t<H: Hw>(
    hw: &mut H,
    nmi: &mut Nmi,
    addr: u8,
    hz: u32,
) -> Result<(), Error<H::Error>> {
    let rf_hz = t_rf_hz(hz).ok_or(Error::Frequency)?;
    write_all(hw, addr, &tc::PRE_TUNE_T)?;
    nmi.tune(&mut NmiGate { hw: &mut *hw, addr }, rf_hz)?;
    hw.sleep_ms(SETTLE_MS);
    write_all(hw, addr, &tc::POST_TUNE_T)
}

/// What a core reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Status {
    /// `enum fe_status` bits.
    pub(crate) fe_status: u32,
    /// C/N in 0.001 dB; `None` while the demodulator has no lock, and when it reports no
    /// measurement.
    pub(crate) cnr_mdb: Option<i32>,
}

/// Status of the ISDB-S core at `addr`, tuned to the TSID `tsid` (`None` for any TS).
///
/// The demodulator's lock is the transponder's: it locks whatever TSID is selected and then
/// delivers the transponder's first TS if the selected one is not carried there. With a TSID
/// selected, `FE_HAS_LOCK` therefore also requires the TMCC TSID list to name it, so that a
/// stream the transponder does not carry reports no lock instead of another stream's packets.
/// The C/N is read whenever the demodulator is locked: it measures the
/// transponder, which is received either way.
pub(crate) fn status_s<H: Hw>(
    hw: &mut H,
    addr: u8,
    tsid: Option<u16>,
) -> Result<Status, Error<H::Error>> {
    let c3 = read_reg(hw, addr, tc::REG_S_STATUS)?;
    let c5 = read_reg(hw, addr, tc::REG_S_TMCC)?;
    let mut fe_status = tc::status_s(c3, c5);
    let mut cnr_mdb = None;
    if fe_status & tc::FE_HAS_LOCK != 0 {
        let mut raw = [0; tc::CN_S_LEN];
        hw.demod_read(addr, tc::REG_CN_S, &mut raw)
            .map_err(Error::Bus)?;
        cnr_mdb = tc::cn_s_mdb(u16::from_be_bytes(raw));
        if let Some(tsid) = tsid {
            let mut list = [0; tc::TSID_LIST_LEN];
            hw.demod_read(addr, tc::REG_S_TSID_LIST, &mut list)
                .map_err(Error::Bus)?;
            if !tc::tsid_listed(&list, tsid) {
                fe_status &= !tc::FE_HAS_LOCK;
            }
        }
    }
    Ok(Status { fe_status, cnr_mdb })
}

/// Status of the ISDB-T core at `addr`. The C/N is read only with lock, after the statistics latch
/// that is written before every statistics read.
pub(crate) fn status_t<H: Hw>(hw: &mut H, addr: u8) -> Result<Status, Error<H::Error>> {
    let r96 = read_reg(hw, addr, tc::REG_T_LOCK)?;
    let r80 = read_reg(hw, addr, tc::REG_T_STATUS)?;
    let rb0 = read_reg(hw, addr, tc::REG_T_MODE)?;
    let fe_status = tc::status_t(r96, r80, rb0);
    let mut cnr_mdb = None;
    if fe_status & tc::FE_HAS_LOCK != 0 {
        let (reg, value) = tc::LATCH_T_STATS;
        hw.demod_write(addr, reg, value).map_err(Error::Bus)?;
        let mut raw = [0; tc::CN_T_LEN];
        hw.demod_read(addr, tc::REG_CN_T, &mut raw)
            .map_err(Error::Bus)?;
        let [b2, b1, b0] = raw;
        cnr_mdb = tc::cn_t_mdb(u32::from_be_bytes([0, b2, b1, b0]));
    }
    Ok(Status { fe_status, cnr_mdb })
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::HashMap;
    use std::vec;
    use std::vec::Vec;

    /// One call of the bus, as the board would see it.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Op {
        DemodWrite(u8, u8, u8),
        DemodRead(u8, u8, usize),
        GateWrite(u8, u8, Vec<u8>),
        GateRead(u8, u8, Vec<u8>, usize),
        Sleep(u32),
    }
    use Op::*;

    /// Error of a failed call: its number among the bus calls.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Fail(usize);

    /// NM120 chip ID: `id & 0xFFF00` = 0x12000.
    const NM120: u32 = 0x0001_2003;

    /// A bus that logs every call and models the chips: demodulator registers, the TDA2014x
    /// behind `TDA_ADDR` and the NM120 behind `NMI_ADDR` (both reachable through any core).
    /// Bus calls are numbered from 0; the call `fail_at` fails with its number, is logged and
    /// changes nothing. Cores outside `present` do not answer (every call to them fails).
    struct Fake {
        log: Vec<Op>,
        present: Vec<u8>,
        demod: HashMap<(u8, u8), u8>,
        tda: [u8; 256],
        nmi8: [u8; 256],
        nmi32: HashMap<u16, u32>,
        calls: usize,
        fail_at: Option<usize>,
    }

    impl Fake {
        fn new() -> Self {
            let mut nmi32 = HashMap::new();
            nmi32.insert(0x3FC, NM120);
            Fake {
                log: Vec::new(),
                present: (0x10..0x18).collect(),
                demod: HashMap::new(),
                tda: [0x5A; 256],
                nmi8: [0; 256],
                nmi32,
                calls: 0,
                fail_at: None,
            }
        }

        fn with_cores(cores: &[u8]) -> Self {
            Fake {
                present: cores.to_vec(),
                ..Fake::new()
            }
        }

        fn failing_at(n: usize) -> Self {
            Fake {
                fail_at: Some(n),
                ..Fake::new()
            }
        }

        fn call(&mut self, addr: u8) -> Result<(), Fail> {
            let n = self.calls;
            self.calls += 1;
            if self.fail_at == Some(n) || !self.present.contains(&addr) {
                Err(Fail(n))
            } else {
                Ok(())
            }
        }

        fn set_demod(&mut self, addr: u8, reg: u8, value: u8) {
            self.demod.insert((addr, reg), value);
        }

        fn take_log(&mut self) -> Vec<Op> {
            core::mem::take(&mut self.log)
        }
    }

    impl Hw for Fake {
        type Error = Fail;

        fn demod_write(&mut self, addr: u8, reg: u8, value: u8) -> Result<(), Fail> {
            self.log.push(DemodWrite(addr, reg, value));
            self.call(addr)?;
            self.demod.insert((addr, reg), value);
            Ok(())
        }

        fn demod_read(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), Fail> {
            self.log.push(DemodRead(addr, reg, buf.len()));
            self.call(addr)?;
            for (i, b) in buf.iter_mut().enumerate() {
                *b = self.demod.get(&(addr, reg + i as u8)).copied().unwrap_or(0);
            }
            Ok(())
        }

        fn gate_write(&mut self, addr: u8, tuner: u8, data: &[u8]) -> Result<(), Fail> {
            self.log.push(GateWrite(addr, tuner, data.to_vec()));
            self.call(addr)?;
            match (tuner, data) {
                (TDA_ADDR, &[reg, value]) => self.tda[usize::from(reg)] = value,
                (NMI_ADDR, &[0, reg, value]) => self.nmi8[usize::from(reg)] = value,
                (NMI_ADDR, &[hi, lo, b0, b1, b2, b3]) => {
                    let value = u32::from_le_bytes([b0, b1, b2, b3]);
                    self.nmi32.insert(u16::from_be_bytes([hi, lo]), value);
                }
                _ => panic!("malformed gateway write {tuner:#x} {data:x?}"),
            }
            Ok(())
        }

        fn gate_read(
            &mut self,
            addr: u8,
            tuner: u8,
            sub: &[u8],
            buf: &mut [u8],
        ) -> Result<(), Fail> {
            self.log
                .push(GateRead(addr, tuner, sub.to_vec(), buf.len()));
            self.call(addr)?;
            match (tuner, sub, buf.len()) {
                (TDA_ADDR, &[reg], 1) => {
                    // The VCO calibration reports completion at once.
                    let done = if reg == 0x11 || reg == 0x15 { 0x10 } else { 0 };
                    buf[0] = self.tda[usize::from(reg)] | done;
                }
                (NMI_ADDR, &[0, reg], 1) => buf[0] = self.nmi8[usize::from(reg)],
                (NMI_ADDR, &[hi, lo], 4) => {
                    let value = self
                        .nmi32
                        .get(&u16::from_be_bytes([hi, lo]))
                        .copied()
                        .unwrap_or(0xFFFF_FFFF);
                    buf.copy_from_slice(&value.to_le_bytes());
                }
                _ => panic!("malformed gateway read {tuner:#x} {sub:x?} {}", buf.len()),
            }
            Ok(())
        }

        fn sleep_ms(&mut self, ms: u32) {
            self.log.push(Sleep(ms));
        }
    }

    /// Calls that reach a tuner, or wait for it.
    fn is_tuner_op(op: &Op) -> bool {
        matches!(op, GateWrite(..) | GateRead(..) | Sleep(_))
    }

    fn is_gate(op: &Op) -> bool {
        matches!(op, GateWrite(..) | GateRead(..))
    }

    fn position_of_wait(log: &[Op]) -> usize {
        log.iter().position(|op| *op == Sleep(100)).unwrap()
    }

    #[test]
    fn tda_accesses_are_framed_for_the_gateway() {
        let mut hw = Fake::new();
        let mut bus = TdaGate {
            hw: &mut hw,
            addr: 0x11,
        };
        bus.write(0x12, 0x34).unwrap();
        assert_eq!(bus.read(0x12), Ok(0x34));
        bus.sleep_ms(3);
        assert_eq!(
            hw.log,
            [
                GateWrite(0x11, 0xA8, vec![0x12, 0x34]),
                GateRead(0x11, 0xA8, vec![0x12], 1),
                Sleep(3),
            ]
        );
    }

    #[test]
    fn nmi_accesses_are_framed_for_the_gateway() {
        let mut hw = Fake::new();
        let mut bus = NmiGate {
            hw: &mut hw,
            addr: 0x12,
        };
        bus.write8(0x2B, 0x91).unwrap();
        assert_eq!(bus.read8(0x2B), Ok(0x91));
        bus.write32(0x1C0, 0x2D8C_19C7).unwrap();
        assert_eq!(bus.read32(0x1C0), Ok(0x2D8C_19C7));
        assert_eq!(bus.read32(0x3FC), Ok(NM120));
        bus.sleep_ms(7);
        assert_eq!(
            hw.log,
            [
                GateWrite(0x12, 0xCE, vec![0x00, 0x2B, 0x91]),
                GateRead(0x12, 0xCE, vec![0x00, 0x2B], 1),
                // The value goes out little endian after the big endian address.
                GateWrite(0x12, 0xCE, vec![0x01, 0xC0, 0xC7, 0x19, 0x8C, 0x2D]),
                GateRead(0x12, 0xCE, vec![0x01, 0xC0], 4),
                GateRead(0x12, 0xCE, vec![0x03, 0xFC], 4),
                Sleep(7),
            ]
        );
    }

    #[test]
    fn a_failed_gateway_access_is_reported() {
        let mut hw = Fake::failing_at(0);
        let mut bus = TdaGate {
            hw: &mut hw,
            addr: 0x11,
        };
        assert_eq!(bus.read(0x01), Err(Fail(0)));
        let mut hw = Fake::failing_at(0);
        let mut bus = NmiGate {
            hw: &mut hw,
            addr: 0x10,
        };
        assert_eq!(bus.write8(0x01, 0x02), Err(Fail(0)));
        let mut hw = Fake::failing_at(0);
        let mut bus = NmiGate {
            hw: &mut hw,
            addr: 0x10,
        };
        assert_eq!(bus.read32(0x3FC), Err(Fail(0)));
    }

    #[test]
    fn probe_finds_the_group_at_0x10() {
        let mut hw = Fake::with_cores(&[0x10, 0x11, 0x12, 0x13]);
        assert_eq!(probe(&mut hw), Ok(0x10));
        // T register 0x80 at the even and S register 0xC3 at the odd addresses, one byte each.
        let expected: Vec<Op> = (0x10..0x18)
            .map(|a| DemodRead(a, if a & 1 == 0 { 0x80 } else { 0xC3 }, 1))
            .collect();
        assert_eq!(hw.log, expected);
    }

    #[test]
    fn probe_finds_the_group_at_0x14() {
        let mut hw = Fake::with_cores(&[0x14, 0x15, 0x16, 0x17]);
        assert_eq!(probe(&mut hw), Ok(0x14));
    }

    #[test]
    fn sleep_stops_the_core_first_and_wake_restarts_it_last() {
        let gateway = |op: &Op| matches!(op, GateWrite(..) | GateRead(..) | Sleep(_));
        // ISDB-T core with its NM120.
        let mut hw = Fake::new();
        let mut nmi = init_t(&mut hw, 0x12).unwrap();
        hw.log.clear();
        sleep_t(&mut hw, &mut nmi, 0x12).unwrap();
        assert_eq!(hw.log[0], DemodWrite(0x12, 0x03, 0x10));
        assert!(
            hw.log.len() > 1 && hw.log[1..].iter().all(gateway),
            "{:?}",
            hw.log
        );
        hw.log.clear();
        wake_t(&mut hw, &mut nmi, 0x12).unwrap();
        assert_eq!(hw.log.last(), Some(&DemodWrite(0x12, 0x03, 0x00)));
        let n = hw.log.len();
        assert!(n > 1 && hw.log[..n - 1].iter().all(gateway), "{:?}", hw.log);
        // ISDB-S core with its TDA2014x.
        let mut hw = Fake::new();
        init_s(&mut hw, 0x11).unwrap();
        hw.log.clear();
        sleep_s(&mut hw, 0x11).unwrap();
        assert_eq!(hw.log[0], DemodWrite(0x11, 0x17, 0x01));
        assert!(
            hw.log.len() > 1 && hw.log[1..].iter().all(gateway),
            "{:?}",
            hw.log
        );
        hw.log.clear();
        wake_s(&mut hw, 0x11).unwrap();
        assert_eq!(hw.log.last(), Some(&DemodWrite(0x11, 0x17, 0x00)));
        let n = hw.log.len();
        assert!(n > 1 && hw.log[..n - 1].iter().all(gateway), "{:?}", hw.log);
    }

    #[test]
    fn probe_reports_what_answered_when_nothing_does() {
        let mut hw = Fake::with_cores(&[]);
        assert_eq!(probe(&mut hw), Err(ProbeError { answered: 0 }));
    }

    #[test]
    fn probe_refuses_two_complete_groups() {
        let mut hw = Fake::new();
        assert_eq!(probe(&mut hw), Err(ProbeError { answered: 0xFF }));
    }

    #[test]
    fn probe_refuses_partial_groups() {
        // Bit n of `answered` is the core at 0x10 + n.
        for (cores, answered) in [
            (vec![0x10, 0x11, 0x12], 0x07),
            (vec![0x11, 0x12, 0x13], 0x0E),
            (vec![0x14, 0x15, 0x16], 0x70),
            (vec![0x10, 0x13, 0x14, 0x17], 0x99),
            // Four cores that do not form a group.
            (vec![0x12, 0x13, 0x14, 0x15], 0x3C),
        ] {
            let mut hw = Fake::with_cores(&cores);
            assert_eq!(probe(&mut hw), Err(ProbeError { answered }), "{cores:x?}");
        }
    }

    #[test]
    fn probe_takes_the_one_complete_group_beside_a_partial_one() {
        let mut hw = Fake::with_cores(&[0x10, 0x11, 0x12, 0x13, 0x16]);
        assert_eq!(probe(&mut hw), Ok(0x10));
        let mut hw = Fake::with_cores(&[0x11, 0x14, 0x15, 0x16, 0x17]);
        assert_eq!(probe(&mut hw), Ok(0x14));
    }

    #[test]
    fn probe_takes_a_failing_read_for_a_missing_core() {
        // The read of the second core fails, so the group at 0x10 is incomplete.
        let mut hw = Fake::with_cores(&[0x10, 0x11, 0x12, 0x13]);
        hw.fail_at = Some(1);
        assert_eq!(probe(&mut hw), Err(ProbeError { answered: 0x0D }));
    }

    #[test]
    fn init_s_writes_the_core_then_initialises_the_tuner() {
        let mut hw = Fake::new();
        init_s(&mut hw, 0x13).unwrap();
        let log = hw.take_log();
        let writes: Vec<_> = tc::INIT_S
            .iter()
            .map(|&(r, v)| DemodWrite(0x13, r, v))
            .collect();
        assert_eq!(log[..writes.len()], writes);
        // Everything after the core's registers goes to the tuner at 0xA8 of the same core.
        let rest = &log[writes.len()..];
        assert!(!rest.is_empty());
        for op in rest {
            match op {
                GateWrite(0x13, 0xA8, _) | GateRead(0x13, 0xA8, _, 1) | Sleep(_) => {}
                other => panic!("unexpected {other:?}"),
            }
        }
    }

    #[test]
    fn init_t_writes_the_core_then_identifies_the_tuner() {
        let mut hw = Fake::new();
        init_t(&mut hw, 0x12).unwrap();
        let log = hw.take_log();
        let writes: Vec<_> = tc::INIT_T
            .iter()
            .map(|&(r, v)| DemodWrite(0x12, r, v))
            .collect();
        assert_eq!(log[..writes.len()], writes);
        assert_eq!(log[writes.len()], GateRead(0x12, 0xCE, vec![0x03, 0xFC], 4));
        assert!(log[writes.len() + 1..].iter().all(is_tuner_op));
    }

    #[test]
    fn init_t_rejects_other_tuners() {
        let mut hw = Fake::new();
        hw.nmi32.insert(0x3FC, 0x0001_3100);
        assert_eq!(
            init_t(&mut hw, 0x12).err(),
            Some(Error::UnsupportedChip(0x0001_3100))
        );
    }

    #[test]
    fn init_stops_at_the_first_failure() {
        // Every call is checked: a failure at any call ends the init with that error and no
        // further call.
        let mut hw = Fake::new();
        init_s(&mut hw, 0x11).unwrap();
        for n in 0..hw.calls {
            let mut hw = Fake::failing_at(n);
            assert_eq!(init_s(&mut hw, 0x11), Err(Error::Bus(Fail(n))));
            assert_eq!(hw.calls, n + 1);
        }
        let mut hw = Fake::new();
        init_t(&mut hw, 0x10).unwrap();
        for n in 0..hw.calls {
            let mut hw = Fake::failing_at(n);
            assert_eq!(init_t(&mut hw, 0x10).err(), Some(Error::Bus(Fail(n))));
            assert_eq!(hw.calls, n + 1);
        }
    }

    #[test]
    fn tune_s_orders_demodulator_tuner_wait_and_acquisition() {
        let mut hw = Fake::new();
        init_s(&mut hw, 0x11).unwrap();
        hw.take_log();
        tune_s(&mut hw, 0x11, 1_318_000, 0x4010, Lna::Least).unwrap();
        let log = hw.take_log();

        let pre: Vec<_> = tc::PRE_TUNE_S
            .iter()
            .map(|&(r, v)| DemodWrite(0x11, r, v))
            .collect();
        assert_eq!(log[..pre.len()], pre);
        // The tuner programming lies between the pre-tune writes and the settle wait, and the
        // demodulator is not touched meanwhile.
        let wait = position_of_wait(&log);
        assert!(wait > pre.len());
        for op in &log[pre.len()..wait] {
            assert!(
                matches!(
                    op,
                    GateWrite(0x11, 0xA8, _) | GateRead(0x11, 0xA8, _, 1) | Sleep(_)
                ),
                "{op:?}"
            );
        }
        assert!(log[pre.len()..wait].iter().any(is_gate));
        // Then the post-tune writes, the TSID and the reacquire, in this order and nothing else.
        let post = [
            DemodWrite(0x11, 0x0A, 0xFF),
            DemodWrite(0x11, 0x10, 0xB2),
            DemodWrite(0x11, 0x11, 0x00),
            DemodWrite(0x11, 0x8F, 0x40),
            DemodWrite(0x11, 0x90, 0x10),
            DemodWrite(0x11, 0x03, 0x01),
        ];
        assert_eq!(log[wait + 1..], post);
    }

    #[test]
    fn tune_s_writes_the_tsid_on_every_tune() {
        let mut hw = Fake::new();
        init_s(&mut hw, 0x13).unwrap();
        for tsid in [0x4010, 0xFFFF, 0x0001] {
            hw.take_log();
            tune_s(&mut hw, 0x13, 1_318_000, tsid, Lna::Least).unwrap();
            let [high, low] = tsid.to_be_bytes();
            assert!(hw.log.contains(&DemodWrite(0x13, 0x8F, high)));
            assert!(hw.log.contains(&DemodWrite(0x13, 0x90, low)));
            // The reacquire is the last write.
            assert_eq!(hw.log.last(), Some(&DemodWrite(0x13, 0x03, 0x01)));
        }
    }

    #[test]
    fn tune_s_checks_the_frequency_before_the_first_write() {
        for khz in [0, 1_049_479, 2_053_001, 3_000_000, u32::MAX] {
            let mut hw = Fake::new();
            assert_eq!(
                tune_s(&mut hw, 0x11, khz, 0xFFFF, Lna::Least),
                Err(Error::Frequency)
            );
            assert!(hw.log.is_empty(), "{khz} kHz");
        }
        for khz in [1_049_480, 1_318_000, 2_053_000] {
            let mut hw = Fake::new();
            init_s(&mut hw, 0x11).unwrap();
            assert_eq!(
                tune_s(&mut hw, 0x11, khz, 0xFFFF, Lna::Least),
                Ok(()),
                "{khz} kHz"
            );
        }
    }

    #[test]
    fn tune_s_stops_at_the_first_failure() {
        let mut hw = Fake::new();
        init_s(&mut hw, 0x11).unwrap();
        let before = hw.calls;
        hw.take_log();
        tune_s(&mut hw, 0x11, 1_318_000, 0xFFFF, Lna::Least).unwrap();
        // The calls up to the wait, which only a complete tuner programming reaches.
        let log = hw.take_log();
        let until_wait = log[..position_of_wait(&log)]
            .iter()
            .filter(|op| !matches!(op, Sleep(_)))
            .count();
        for n in before..hw.calls {
            let mut hw = Fake::new();
            init_s(&mut hw, 0x11).unwrap();
            hw.fail_at = Some(n);
            hw.take_log();
            assert_eq!(
                tune_s(&mut hw, 0x11, 1_318_000, 0xFFFF, Lna::Least),
                Err(Error::Bus(Fail(n)))
            );
            assert_eq!(hw.calls, n + 1);
            assert_eq!(hw.log.contains(&Sleep(100)), n - before >= until_wait);
        }
    }

    #[test]
    fn tune_t_orders_demodulator_tuner_wait_and_post_tune() {
        let mut hw = Fake::new();
        let mut nmi = init_t(&mut hw, 0x10).unwrap();
        hw.take_log();
        tune_t(&mut hw, &mut nmi, 0x10, 473_142_857).unwrap();
        let log = hw.take_log();

        let pre: Vec<_> = tc::PRE_TUNE_T
            .iter()
            .map(|&(r, v)| DemodWrite(0x10, r, v))
            .collect();
        let post: Vec<_> = tc::POST_TUNE_T
            .iter()
            .map(|&(r, v)| DemodWrite(0x10, r, v))
            .collect();
        assert_eq!(log[..pre.len()], pre);
        let wait = position_of_wait(&log);
        assert!(wait > pre.len());
        assert!(log[pre.len()..wait].iter().all(is_tuner_op));
        assert!(log[pre.len()..wait].iter().any(is_gate));
        assert_eq!(log[wait + 1..], post);
    }

    #[test]
    fn tune_t_passes_the_request_rounded_to_the_nearest_khz() {
        // The same tune made directly on the tuner shows what the tuner gets.
        let direct = |hz: u32| {
            let mut hw = Fake::new();
            let mut nmi = Nmi::init(&mut NmiGate {
                hw: &mut hw,
                addr: 0x10,
            })
            .unwrap();
            hw.take_log();
            nmi.tune(
                &mut NmiGate {
                    hw: &mut hw,
                    addr: 0x10,
                },
                hz,
            )
            .unwrap();
            hw.take_log()
                .into_iter()
                .filter(is_gate)
                .collect::<Vec<_>>()
        };
        let through_tune_t = |hz: u32| {
            let mut hw = Fake::new();
            let mut nmi = init_t(&mut hw, 0x10).unwrap();
            hw.take_log();
            tune_t(&mut hw, &mut nmi, 0x10, hz).unwrap();
            hw.take_log()
                .into_iter()
                .filter(is_gate)
                .collect::<Vec<_>>()
        };
        for (request, rounded) in [
            (473_142_857, 473_143_000),
            (473_142_499, 473_142_000),
            (473_142_500, 473_143_000),
            (93_000_000, 93_000_000),
            (92_999_500, 93_000_000),
            (767_999_999, 768_000_000),
        ] {
            assert_eq!(through_tune_t(request), direct(rounded), "{request} Hz");
        }
        assert_ne!(direct(473_143_000), direct(473_144_000));
    }

    #[test]
    fn tune_t_checks_the_frequency_before_the_first_write() {
        let mut hw = Fake::new();
        let mut nmi = init_t(&mut hw, 0x10).unwrap();
        for hz in [0, 92_999_499, 768_000_500, 800_000_000, u32::MAX] {
            hw.take_log();
            assert_eq!(
                tune_t(&mut hw, &mut nmi, 0x10, hz),
                Err(Error::Frequency),
                "{hz} Hz"
            );
            assert!(hw.log.is_empty());
        }
    }

    #[test]
    fn tune_t_stops_at_the_first_failure() {
        let mut hw = Fake::new();
        let mut nmi = init_t(&mut hw, 0x10).unwrap();
        let before = hw.calls;
        hw.take_log();
        tune_t(&mut hw, &mut nmi, 0x10, 473_143_000).unwrap();
        let log = hw.take_log();
        let until_wait = log[..position_of_wait(&log)]
            .iter()
            .filter(|op| !matches!(op, Sleep(_)))
            .count();
        for n in before..hw.calls {
            let mut hw = Fake::new();
            let mut nmi = init_t(&mut hw, 0x10).unwrap();
            hw.fail_at = Some(n);
            hw.take_log();
            assert_eq!(
                tune_t(&mut hw, &mut nmi, 0x10, 473_143_000),
                Err(Error::Bus(Fail(n)))
            );
            assert_eq!(hw.calls, n + 1);
            assert_eq!(hw.log.contains(&Sleep(100)), n - before >= until_wait);
        }
    }

    #[test]
    fn status_s_reads_the_c_n_only_with_lock() {
        let mut hw = Fake::new();
        // Locked: 0xC3 bit 4 clear, 0xC5 bit 0 set (TMCC valid); 0xBC:0xBD = 0x1031 = 4145.
        hw.set_demod(0x11, 0xC5, 0x01);
        hw.set_demod(0x11, 0xBC, 0x10);
        hw.set_demod(0x11, 0xBD, 0x31);
        let status = status_s(&mut hw, 0x11, None).unwrap();
        assert_eq!(status.fe_status, tc::status_s(0x00, 0x01));
        assert_ne!(status.fe_status & tc::FE_HAS_LOCK, 0);
        assert_eq!(status.cnr_mdb, tc::cn_s_mdb(0x1031));
        assert!(status.cnr_mdb.is_some());
        // One burst of two registers for the C/N, after the two status reads.
        assert_eq!(
            hw.log,
            [
                DemodRead(0x11, 0xC3, 1),
                DemodRead(0x11, 0xC5, 1),
                DemodRead(0x11, 0xBC, 2),
            ]
        );

        // Carrier but no lock (bit 4 set): the C/N registers are not read.
        let mut hw = Fake::new();
        hw.set_demod(0x13, 0xC3, 0x10);
        hw.set_demod(0x13, 0xC5, 0x03);
        let status = status_s(&mut hw, 0x13, None).unwrap();
        assert_eq!(status.fe_status, tc::status_s(0x10, 0x03));
        assert_eq!(status.fe_status & tc::FE_HAS_LOCK, 0);
        assert_eq!(status.cnr_mdb, None);
        assert_eq!(hw.log.len(), 2);
    }

    #[test]
    fn status_s_locks_only_on_a_tsid_the_transponder_carries() {
        // Locked on BS-15, whose TMCC lists 0x40F1, 0x40F2 and 0x48F3; C/N raw 0x1031.
        let bs15 = |hw: &mut Fake| {
            hw.set_demod(0x11, 0xC5, 0x01);
            hw.set_demod(0x11, 0xBC, 0x10);
            hw.set_demod(0x11, 0xBD, 0x31);
            for (i, b) in [0x40, 0xF1, 0x40, 0xF2, 0x48, 0xF3].into_iter().enumerate() {
                hw.set_demod(0x11, 0xCE + i as u8, b);
            }
        };
        let locked = tc::status_s(0x00, 0x01);

        // A listed TSID: locked, the list read in one burst after the C/N.
        let mut hw = Fake::new();
        bs15(&mut hw);
        let status = status_s(&mut hw, 0x11, Some(0x48F3)).unwrap();
        assert_eq!(status.fe_status, locked);
        assert_eq!(status.cnr_mdb, tc::cn_s_mdb(0x1031));
        assert_eq!(
            hw.log,
            [
                DemodRead(0x11, 0xC3, 1),
                DemodRead(0x11, 0xC5, 1),
                DemodRead(0x11, 0xBC, 2),
                DemodRead(0x11, 0xCE, 16),
            ]
        );

        // A TSID the transponder does not carry: everything but the lock, with the C/N of the
        // transponder.
        let mut hw = Fake::new();
        bs15(&mut hw);
        let status = status_s(&mut hw, 0x11, Some(0x40F0)).unwrap();
        assert_eq!(status.fe_status, locked & !tc::FE_HAS_LOCK);
        assert_eq!(status.cnr_mdb, tc::cn_s_mdb(0x1031));

        // Any TS: the list is not read.
        let mut hw = Fake::new();
        bs15(&mut hw);
        assert_eq!(status_s(&mut hw, 0x11, None).unwrap().fe_status, locked);
        assert!(!hw.log.contains(&DemodRead(0x11, 0xCE, 16)));

        // Without the demodulator's lock nothing beyond the status registers is read.
        let mut hw = Fake::new();
        bs15(&mut hw);
        hw.set_demod(0x11, 0xC3, 0x10);
        let status = status_s(&mut hw, 0x11, Some(0x40F1)).unwrap();
        assert_eq!(status.fe_status, tc::status_s(0x10, 0x01));
        assert_eq!(status.cnr_mdb, None);
        assert_eq!(hw.log.len(), 2);
    }

    #[test]
    fn status_s_without_a_measurement_has_no_c_n() {
        let mut hw = Fake::new();
        hw.set_demod(0x11, 0xC5, 0x01);
        // Raw 0 is below the offset of 3000.
        let status = status_s(&mut hw, 0x11, None).unwrap();
        assert_ne!(status.fe_status & tc::FE_HAS_LOCK, 0);
        assert_eq!(status.cnr_mdb, None);
    }

    #[test]
    fn status_t_latches_before_reading_the_c_n_with_lock() {
        let mut hw = Fake::new();
        // Full lock in 0x96 and mode register 0xB0 = 9 (>= 8); 0x8B..0x8D = 0x00DA4C.
        hw.set_demod(0x10, 0x96, 0xE0);
        hw.set_demod(0x10, 0xB0, 0x09);
        hw.set_demod(0x10, 0x8B, 0x00);
        hw.set_demod(0x10, 0x8C, 0xDA);
        hw.set_demod(0x10, 0x8D, 0x4C);
        let status = status_t(&mut hw, 0x10).unwrap();
        assert_eq!(status.fe_status, tc::status_t(0xE0, 0x00, 0x09));
        assert_ne!(status.fe_status & tc::FE_HAS_LOCK, 0);
        assert_eq!(status.cnr_mdb, tc::cn_t_mdb(0x0000_DA4C));
        assert!(status.cnr_mdb.is_some());
        assert_eq!(
            hw.log,
            [
                DemodRead(0x10, 0x96, 1),
                DemodRead(0x10, 0x80, 1),
                DemodRead(0x10, 0xB0, 1),
                // The statistics latch comes before the burst of the three C/N bytes.
                DemodWrite(0x10, 0xBA, 0x00),
                DemodRead(0x10, 0x8B, 3),
            ]
        );
    }

    #[test]
    fn status_t_needs_the_mode_condition_for_lock() {
        let mut hw = Fake::new();
        hw.set_demod(0x12, 0x96, 0xE0);
        hw.set_demod(0x12, 0xB0, 0x07);
        let status = status_t(&mut hw, 0x12).unwrap();
        assert_eq!(status.fe_status & tc::FE_HAS_LOCK, 0);
        assert_ne!(status.fe_status & tc::FE_HAS_SIGNAL, 0);
        assert_eq!(status.cnr_mdb, None);
        // Neither the latch nor the C/N registers are touched without lock.
        assert_eq!(
            hw.log,
            [
                DemodRead(0x12, 0x96, 1),
                DemodRead(0x12, 0x80, 1),
                DemodRead(0x12, 0xB0, 1),
            ]
        );
    }

    #[test]
    fn status_t_without_signal_reports_nothing() {
        let mut hw = Fake::new();
        hw.set_demod(0x10, 0x80, 0xF0);
        let status = status_t(&mut hw, 0x10).unwrap();
        assert_eq!(
            status,
            Status {
                fe_status: 0,
                cnr_mdb: None
            }
        );
    }

    #[test]
    fn status_reports_failed_reads() {
        for n in 0..3 {
            let mut hw = Fake::failing_at(n);
            assert_eq!(status_t(&mut hw, 0x10), Err(Error::Bus(Fail(n))));
            assert_eq!(hw.calls, n + 1);
        }
        for n in 0..2 {
            let mut hw = Fake::failing_at(n);
            assert_eq!(status_s(&mut hw, 0x11, None), Err(Error::Bus(Fail(n))));
            assert_eq!(hw.calls, n + 1);
        }
        // With lock, a failure of the C/N burst (or of the latch before it) is an error too.
        for n in [3, 4] {
            let mut hw = Fake::failing_at(n);
            hw.set_demod(0x10, 0x96, 0xE0);
            hw.set_demod(0x10, 0xB0, 0x09);
            assert_eq!(status_t(&mut hw, 0x10), Err(Error::Bus(Fail(n))));
        }
        let mut hw = Fake::failing_at(2);
        hw.set_demod(0x11, 0xC5, 0x01);
        assert_eq!(status_s(&mut hw, 0x11, None), Err(Error::Bus(Fail(2))));
        // With lock and a TSID, so is a failure of the TSID list burst.
        let mut hw = Fake::failing_at(3);
        hw.set_demod(0x11, 0xC5, 0x01);
        assert_eq!(
            status_s(&mut hw, 0x11, Some(0x40F1)),
            Err(Error::Bus(Fail(3)))
        );
    }

    #[test]
    fn frequency_helpers() {
        assert_eq!(s_if_khz(1_049_480), Some(1_049_480));
        assert_eq!(s_if_khz(2_053_000), Some(2_053_000));
        assert_eq!(s_if_khz(1_049_479), None);
        assert_eq!(s_if_khz(2_053_001), None);

        assert_eq!(t_rf_hz(473_142_857), Some(473_143_000));
        assert_eq!(t_rf_hz(473_142_499), Some(473_142_000));
        assert_eq!(t_rf_hz(473_142_500), Some(473_143_000));
        assert_eq!(t_rf_hz(93_000_000), Some(93_000_000));
        assert_eq!(t_rf_hz(92_999_500), Some(93_000_000));
        assert_eq!(t_rf_hz(92_999_499), None);
        assert_eq!(t_rf_hz(768_000_000), Some(768_000_000));
        assert_eq!(t_rf_hz(768_000_499), Some(768_000_000));
        assert_eq!(t_rf_hz(768_000_500), None);
        assert_eq!(t_rf_hz(u32::MAX), None);
        assert_eq!(t_rf_hz(u32::MAX - 499), None);
    }
}
