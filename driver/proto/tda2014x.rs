// SPDX-License-Identifier: GPL-2.0

//! NXP TDA2014x (TDA20142) satellite tuner, as fitted behind the satellite cores of the PX-W3PE
//! V2.0: initialisation and tuning over single-register reads and writes.
//!
//! The register sequences are the revealed ones.
//! The tuner is always driven with the one parameter set the board uses (symbol rate 28 860 kBd,
//! roll-off index 2, LPF offset 5 000 kHz) and the chosen gains, so the low-pass filter and gain
//! registers are constants. The known pitfalls of the sequences are avoided here, and each is
//! marked where it matters.
//!
//! Read-modify-write registers keep the bits the sequences do not name: their values after reset
//! are only known to the card, so each one is read through the bus and written back once.

/// Register access to the tuner, one transaction per call.
///
/// An implementation holds the I2C bus for the whole of one `read` or `write` and releases it in
/// `sleep_ms`, so that the other tuners keep working while a calibration runs.
pub(crate) trait TdaBus {
    /// Error of a failed bus transaction.
    type Error;

    /// Reads register `reg` (`FE A8 reg`, then `FE A9` and one byte).
    fn read(&mut self, reg: u8) -> Result<u8, Self::Error>;

    /// Writes `value` to register `reg` (`FE A8 reg value`).
    fn write(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Sleeps for `ms` milliseconds, uninterruptibly.
    fn sleep_ms(&mut self, ms: u32);
}

/// Why initialisation or tuning failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TdaError<E> {
    /// A bus transaction failed. The registers may be partly programmed.
    Bus(E),
    /// A VCO calibration did not report completion within [`CAL_LIMIT_MS`]. Going on as if it had
    /// finished would be a mistake.
    Calibration,
    /// The intermediate frequency is outside [`IF_MIN_KHZ`]..=[`IF_MAX_KHZ`]. Nothing was written.
    Frequency,
}

impl<E> From<E> for TdaError<E> {
    fn from(e: E) -> Self {
        TdaError::Bus(e)
    }
}

/// Lowest intermediate frequency the PLL is programmed for.
const IF_MIN_KHZ: u32 = 300_000;

/// Highest intermediate frequency the PLL is programmed for.
///
/// Inside this range every frequency yields a valid PLL setting: the divider selection, the
/// reference ratio selection and the 8-bit integer part never fail (checked for every kHz by the
/// tests), so this range is the whole of what `tune` accepts. The DVB frequency limits of the
/// front end (1 049 480 to 2 053 000 kHz) are narrower and are enforced by its caller.
const IF_MAX_KHZ: u32 = 3_000_000;

/// Interval between two reads of a calibration done bit.
///
/// Reading the bit two or three times back to back spans a few gateway transactions and so a few
/// milliseconds of bus time at most. The revealed sequences give no calibration time; 1 ms steps
/// and a 20 ms limit are used. 1 ms is the finest step `sleep_ms` offers.
const CAL_POLL_MS: u32 = 1;

/// Time after which a calibration that has not reported completion is a failure.
///
/// 20 ms is several times the span of a few back-to-back reads, and small against the 100 ms the
/// caller waits after every tune, so a calibration that really needs longer than this is broken.
const CAL_LIMIT_MS: u32 = 20;

/// Crystal frequency in MHz.
const XTAL_MHZ: u64 = 27;

/// Limits of the integer part of the PLL ratio, in tenths, that the reference ratio selection
/// aims for.
///
/// The limits depend on 0x21[6] and 0x1B[0]. `init` sets 0x21[6] and clears 0x1B[0] (see
/// [`INIT`]), which selects (1310, 2510).
const RATIO_MIN_N10: u64 = 1310;
const RATIO_MAX_N10: u64 = 2510;

/// LPF registers for symbol rate 28 860 kBd, roll-off index 2 (factor 1.35) and offset 5 000 kHz:
/// `bw = (28860 * 135 / 2 + 5000 * 100) / 72 + 2000 = 36000` kHz, which selects 0x0A[3:0] = 0xA
/// (35 000 <= bw < 45 000); `bw - 2000 = 34000` falls in the table entry below 39 000, code 0x7C
/// for 0x0B[7:1].
const LPF_0A_CODE: u8 = 0x0A;
const LPF_0B_CODE: u8 = 0x7C;

/// The LNA code, the one gain that is not a constant: the frontend compares the two values on the
/// first lock of each tuner and keeps the better one. Code 4
/// (`Least`, register 0x06 bits 0x01, the one code with bit 7 clear) is the least gain and the
/// better choice on a strong input; code 0 (`Full`, 0x80) is the reference's value. Codes 1 to 3
/// lie between the two and measured worse than either. The chip's own level
/// word cannot make the choice: it is saturated at the inputs where the choice matters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Lna {
    Least,
    Full,
}

impl Lna {
    /// The value ORed into register 0x06: LNA(4) = 0x01, LNA(0) = 0x80, with LPT code 0 in
    /// bits 2:1.
    const fn reg06_or(self) -> u8 {
        match self {
            Lna::Least => 0x01,
            Lna::Full => 0x80,
        }
    }
}

/// Gain registers for the chosen gains: LPT code 0, AMPOUT code 0, h2..h4 = 5, 4, 3 and
/// g0..g3 = 0. The four AGC profiles are identical, so no level is read and nothing is switched.
///
/// - 0x06[6] = g0 & 1 = 0.
/// - 0x07 = (R & 0xAC) | 0x02: g1 != 1, g2 != 1 gives bit 1 and g3 != 1, g0 not in {2, 3} nothing
///   else.
/// - 0x06 = (R & 0x48) | LNA | (LPT 0 << 1) = (R & 0x48) | [`Lna::reg06_or`].
/// - 0x09 = (R & 0x03) | (5 << 5) | (4 << 2) = (R & 0x03) | 0xB0.
/// - 0x0A[7:5] = h4 = 3.
/// - 0x0C[7:4] = h5 = AMPOUT code 0.
const GAIN_07_OR: u8 = 0x02;
const GAIN_09_OR: u8 = 0xB0;
const GAIN_0A_CODE: u8 = 3;
const GAIN_0C_CODE: u8 = 0;

/// Reads register `reg` and writes back `(R & keep) | or`.
fn update<B: TdaBus>(bus: &mut B, reg: u8, keep: u8, or: u8) -> Result<(), B::Error> {
    let r = bus.read(reg)?;
    bus.write(reg, (r & keep) | or)
}

/// One step of the initialisation table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Init {
    /// `reg = value`, without reading.
    Write(u8, u8),
    /// `reg = (R & keep) | or`.
    Update(u8, u8, u8),
}

/// Register initialisation, in the revealed order.
///
/// The update of 0x1B also clears bit 0, which the revealed `R & 0x7F` keeps: the PLL ratio limits
/// this module uses are those of 0x1B[0] = 0 ([`RATIO_MIN_N10`]), so the bit is set to match them
/// instead of relying on its reset value (0 on the card). Sleep and wake keep it.
const INIT: [Init; 21] = [
    Init::Update(0x02, 0xFF, 0x81),
    Init::Update(0x06, 0x46, 0x39),
    Init::Update(0x07, 0xFF, 0xAE),
    Init::Update(0x0F, 0xFF, 0x80),
    Init::Update(0x18, 0x7F, 0x00),
    Init::Update(0x1A, 0xFF, 0xC0),
    Init::Write(0x22, 0xFF),
    Init::Update(0x23, 0xFE, 0x00),
    Init::Update(0x25, 0xFF, 0x08),
    Init::Update(0x27, 0x1F, 0xC0),
    Init::Update(0x24, 0xCB, 0x04),
    Init::Update(0x0D, 0xDF, 0x00),
    Init::Update(0x09, 0x01, 0xB0),
    Init::Update(0x0A, 0x10, 0x6F),
    Init::Update(0x0B, 0x01, 0x7A),
    Init::Write(0x0C, 0x00),
    Init::Write(0x19, 0xFA),
    Init::Update(0x1B, 0x7E, 0x00),
    Init::Update(0x21, 0xFF, 0x40),
    Init::Update(0x10, 0x2F, 0x90),
    Init::Update(0x14, 0xCF, 0x20),
];

/// Reads `reg` until a bit of `mask` is set, sleeping [`CAL_POLL_MS`] between reads, for at most
/// [`CAL_LIMIT_MS`] after the first read. Returns whether the bit was seen.
///
/// Polling without a delay or a time bound would be a mistake; the poll here is bounded.
fn wait_done<B: TdaBus>(bus: &mut B, reg: u8, mask: u8) -> Result<bool, B::Error> {
    let mut waited = 0;
    loop {
        if bus.read(reg)? & mask != 0 {
            return Ok(true);
        }
        if waited >= CAL_LIMIT_MS {
            return Ok(false);
        }
        bus.sleep_ms(CAL_POLL_MS);
        waited += CAL_POLL_MS;
    }
}

/// VCO power-on calibration.
fn vco_power_on<B: TdaBus>(bus: &mut B) -> Result<(), TdaError<B::Error>> {
    update(bus, 0x0F, 0x9F, 0x80)?;
    update(bus, 0x13, 0xEF, 0x20)?;
    update(bus, 0x12, 0xFF, 0xC0)?;
    // Written twice; kept, since the card cannot show whether the write itself triggers
    // anything.
    update(bus, 0x10, 0xFF, 0x20)?;
    update(bus, 0x10, 0xFF, 0x20)?;
    update(bus, 0x0F, 0xFF, 0x20)?;
    if !wait_done(bus, 0x11, 0x10)? {
        return Err(TdaError::Calibration);
    }
    let code = bus.read(0x10)? & 0x0F;
    update(bus, 0x0F, 0xF0, code)?;
    update(bus, 0x0F, 0xFF, 0x40)?;
    update(bus, 0x0F, 0xDF, 0x00)?;
    update(bus, 0x12, 0x7F, 0x00)?;
    update(bus, 0x0D, 0x9F, 0x20)?;
    Ok(())
}

/// PLL power-on.
fn pll_power_on<B: TdaBus>(bus: &mut B) -> Result<(), B::Error> {
    update(bus, 0x1A, 0xFF, 0x40)?;
    update(bus, 0x18, 0xFF, 0x01)?;
    update(bus, 0x18, 0xFF, 0x80)?;
    update(bus, 0x1B, 0xFF, 0x80)?;
    update(bus, 0x18, 0xFE, 0x00)
}

/// Initialises the tuner: register defaults, PLL power-on, VCO power-on calibration and the
/// loop-through output.
///
/// No default frequency is tuned afterwards; the driver tunes on request. A bus error or a
/// calibration that does not finish stops the initialisation, since ignoring the result of an
/// attach would be a mistake.
pub(crate) fn init<B: TdaBus>(bus: &mut B) -> Result<(), TdaError<B::Error>> {
    for step in INIT {
        match step {
            Init::Write(reg, value) => bus.write(reg, value)?,
            Init::Update(reg, keep, or) => update(bus, reg, keep, or)?,
        }
    }

    pll_power_on(bus)?;
    vco_power_on(bus)?;

    // Loop-through output enable.
    update(bus, 0x06, 0xF7, 0x08)?;
    Ok(())
}

/// Stops the tuner by switching its PLL off: the power bits of the PLL power-on, in the reverse
/// order. No standby sequence is known for the TDA2014x; without the PLL the tuner has no local
/// oscillator and delivers nothing (measured on the card).
///
/// The RF input stage stays on: bits 7 and 0 of register 0x02, which `init` sets, would stop the
/// tuner too, but switching them changes what the antenna input sees. BS and UHF often share one
/// outlet and a diplexer, and on such a feed clearing them on the tuner at the antenna disturbed
/// both terrestrial demodulators (in a step-by-step measurement the only steps followed by
/// terrestrial disturbances), while the PLL steps did not. The second tuner takes its signal from
/// the first one's loop-through output and is treated alike. The loop-through output (register
/// 0x06 bit 3) stays on as well: with the first tuner stopped, the second one keeps its signal.
pub(crate) fn sleep<B: TdaBus>(bus: &mut B) -> Result<(), B::Error> {
    update(bus, 0x1B, 0x7F, 0x00)?;
    update(bus, 0x18, 0x7F, 0x00)?;
    update(bus, 0x1A, 0xBF, 0x00)
}

/// Restarts a stopped tuner as init starts its PLL: PLL power-on and the VCO power-on calibration.
/// The next tune programs the frequency.
pub(crate) fn wake<B: TdaBus>(bus: &mut B) -> Result<(), TdaError<B::Error>> {
    pll_power_on(bus)?;
    vco_power_on(bus)
}

/// LO divider and LO configuration for one intermediate frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LoConfig {
    /// Divider M between the VCO and the LO.
    divider: u8,
    /// Value of register 0x22.
    reg22: u8,
    /// Bits ORed into `R & 0x53` for register 0x23.
    reg23_or: u8,
}

/// LO configuration for `if_khz`, thresholds 1 075 000, 1 228 000, 1 433 000 and 1 720 000 kHz.
const fn lo_config(if_khz: u32) -> LoConfig {
    let (divider, reg22, reg23_or) = if if_khz <= 1_075_000 {
        (8, 0x1E, 0x04)
    } else if if_khz <= 1_228_000 {
        (7, 0xFF, 0xA8)
    } else if if_khz <= 1_433_000 {
        (6, 0xFF, 0x28)
    } else if if_khz <= 1_720_000 {
        (5, 0xFF, 0x88)
    } else {
        (4, 0xFF, 0x08)
    };
    LoConfig {
        divider,
        reg22,
        reg23_or,
    }
}

/// VCO frequency over the crystal frequency, times 10^6, truncated:
/// `M * f * 100 * 10^6 / (Fxtal / 10) = M * f * 1000 / 27` with f in kHz.
///
/// At most 4.5e8 for `f <= IF_MAX_KHZ`, so it also fits 32 bits. A 64-bit division that stops at
/// a remainder not above the divisor can come out one short for an exact odd multiple of the
/// divisor; a plain division does not, and the two agree here because the quotient always carries
/// the factor 1000.
const fn res_lsb(divider: u8, if_khz: u32) -> u64 {
    (divider as u64) * (if_khz as u64) * 1000 / XTAL_MHZ
}

/// Ratio of reference divider R and predivider P.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ratio {
    /// R = 1, P = 2.
    R1P2,
    /// R = 2, P = 2.
    R2P2,
    /// R = 3, P = 2.
    R3P2,
    /// R = 3, P = 4.
    R3P4,
}

impl Ratio {
    /// Value of the field 0x03[7:6]: R - 1.
    const fn reg03_field(self) -> u8 {
        match self {
            Ratio::R1P2 => 0,
            Ratio::R2P2 => 1,
            Ratio::R3P2 | Ratio::R3P4 => 2,
        }
    }

    /// Whether the predivider is 4 (0x1A[5]).
    const fn predivider_4(self) -> bool {
        matches!(self, Ratio::R3P4)
    }

    /// `res_lsb * R / P`, by shifts (the truncation is part of the value).
    const fn scale(self, res_lsb: u64) -> u64 {
        match self {
            Ratio::R1P2 => res_lsb >> 1,
            Ratio::R2P2 => res_lsb,
            Ratio::R3P2 => (res_lsb >> 1) + res_lsb,
            Ratio::R3P4 => (res_lsb >> 2) + (res_lsb >> 1),
        }
    }
}

/// `round(10 * N)` for R/P = 1.
const fn ratio_n10(res_lsb: u64) -> u64 {
    (res_lsb + 50_000) / 100_000
}

/// Picks the reference ratio that brings `10 * N` into [`RATIO_MIN_N10`]..=[`RATIO_MAX_N10`];
/// `None` if none does.
///
/// The revealed selection tries R/P = 1/2 first and falls back to 1/4 and 3/4 when that is above
/// the upper limit. That needs `n10 > 5020`, i.e. a frequency far above [`IF_MAX_KHZ`]
/// (`n10 <= 4444` there), so the case cannot occur and is left out: it would end in the range
/// check below.
fn select_ratio(res_lsb: u64) -> Option<Ratio> {
    let n10 = ratio_n10(res_lsb);
    let (ratio, n) = if n10 / 2 >= RATIO_MIN_N10 {
        (Ratio::R1P2, n10 / 2)
    } else if n10 > RATIO_MAX_N10 {
        (Ratio::R3P4, 3 * n10 / 4)
    } else if n10 < RATIO_MIN_N10 {
        (Ratio::R3P2, 3 * n10 / 2)
    } else {
        (Ratio::R2P2, n10)
    };
    (RATIO_MIN_N10..=RATIO_MAX_N10)
        .contains(&n)
        .then_some(ratio)
}

/// `K`, the PLL ratio times 10^6, rounded to a multiple of 10.
const fn ratio_k(ratio: Ratio, res_lsb: u64) -> u64 {
    (ratio.scale(res_lsb) + 5) / 10 * 10
}

/// Register values of the PLL for one intermediate frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Plan {
    lo: LoConfig,
    ratio: Ratio,
    /// Register 0x1E: integer part of the ratio minus 128.
    n_reg: u8,
    /// Registers 0x1F (high byte) and 0x20: fractional part in 1/65536.
    frac: u16,
}

/// Computes the PLL setting for `if_khz` with 0x21[6] set, or `None` if the
/// frequency is outside [`IF_MIN_KHZ`]..=[`IF_MAX_KHZ`].
///
/// All arithmetic is 64-bit; the largest intermediate value is `M * f * 1000 <= 1.2e10`.
fn plan(if_khz: u32) -> Option<Plan> {
    if !(IF_MIN_KHZ..=IF_MAX_KHZ).contains(&if_khz) {
        return None;
    }
    let lo = lo_config(if_khz);
    let res = res_lsb(lo.divider, if_khz);
    let ratio = select_ratio(res)?;
    let k = ratio_k(ratio, res);
    // With 0x21[6] set: N = K / 10^6 with 128 <= N <= 383 (the register holds N - 128).
    let n_reg = u8::try_from((k / 1_000_000).checked_sub(128)?).ok()?;
    // The fraction: frac * 65536 / 10^6 is built in 16 doublings, dividing by 10 whenever the
    // numerator grows too large. A single exact division gives the same value for every frequency
    // in the range (checked by the tests) and cannot lose digits.
    let frac = u16::try_from((k % 1_000_000) * 65_536 / 1_000_000).ok()?;
    Some(Plan {
        lo,
        ratio,
        n_reg,
        frac,
    })
}

/// Programs the PLL.
fn program_pll<B: TdaBus>(bus: &mut B, plan: &Plan) -> Result<(), B::Error> {
    // The revealed sequence reads 0x22 and 0x23 first and ignores both values, since 0x22 is
    // written whole and 0x23 is read again; only 0x25[3], which is saved and written back after
    // 0x23, is kept. Not checking the result of this step would be a mistake; it is checked here.
    let r25 = bus.read(0x25)?;
    bus.write(0x22, plan.lo.reg22)?;
    update(bus, 0x23, 0x53, plan.lo.reg23_or)?;
    update(bus, 0x25, !0x08, r25 & 0x08)?;

    update(bus, 0x03, 0x3F, plan.ratio.reg03_field() << 6)?;
    let p4 = if plan.ratio.predivider_4() { 0x20 } else { 0 };
    update(bus, 0x1A, !0x20, p4)?;
    bus.write(0x1E, plan.n_reg)?;
    let [hi, lo] = plan.frac.to_be_bytes();
    bus.write(0x1F, hi)?;
    bus.write(0x20, lo)
}

/// VCO calibration after a channel change.
///
/// A calibration that does not finish is an error, and the sequence ends the same way in both
/// cases (0x13[4] cleared, 0x12 bit 7 cleared); returning success and leaving both set would be a
/// mistake.
fn vco_channel_change<B: TdaBus>(bus: &mut B) -> Result<(), TdaError<B::Error>> {
    update(bus, 0x12, 0x7F, 0x40)?;
    update(bus, 0x13, 0xFC, 0x02)?;
    update(bus, 0x13, 0x7F, 0x00)?;
    // Written twice, as in `vco_power_on`.
    update(bus, 0x13, 0xFF, 0x20)?;
    update(bus, 0x13, 0xFF, 0x20)?;
    update(bus, 0x13, 0x7F, 0x00)?;
    update(bus, 0x13, 0xFF, 0x10)?;
    let done = wait_done(bus, 0x15, 0x10)?;
    update(bus, 0x13, 0xEF, 0x00)?;
    update(bus, 0x12, 0x7F, 0x00)?;
    if done {
        Ok(())
    } else {
        Err(TdaError::Calibration)
    }
}

/// Low-pass filter.
fn set_lpf<B: TdaBus>(bus: &mut B) -> Result<(), B::Error> {
    update(bus, 0x0A, 0xF0, LPF_0A_CODE)?;
    update(bus, 0x0B, 0x01, LPF_0B_CODE << 1)
}

/// The chosen gains, written with the revealed register logic.
fn set_gains<B: TdaBus>(bus: &mut B, lna: Lna) -> Result<(), B::Error> {
    // 0x07 is read before 0x06[6] is written, and written after.
    let r07 = bus.read(0x07)?;
    update(bus, 0x06, !0x40, 0x00)?;
    bus.write(0x07, (r07 & 0xAC) | GAIN_07_OR)?;
    update(bus, 0x06, 0x48, lna.reg06_or())?;
    update(bus, 0x09, 0x03, GAIN_09_OR)?;
    update(bus, 0x0A, 0x1F, GAIN_0A_CODE << 5)?;
    update(bus, 0x0C, 0x0F, GAIN_0C_CODE << 4)
}

/// Changes the LNA code of a tuned chip: the 0x06 write of `set_gains` alone, so that the two
/// codes can be compared on a live signal without retuning.
pub(crate) fn set_lna<B: TdaBus>(bus: &mut B, lna: Lna) -> Result<(), B::Error> {
    update(bus, 0x06, 0x48, lna.reg06_or())
}

/// Tunes to the intermediate frequency `if_khz` with LNA code `lna`: PLL, VCO calibration, low-pass
/// filter and gains, in the revealed order, each stage only after the previous one
/// succeeded.
///
/// `init` must have run before: the PLL setting assumes 0x21[6] is set. A frequency outside
/// [`IF_MIN_KHZ`]..=[`IF_MAX_KHZ`] fails before the first bus access.
pub(crate) fn tune<B: TdaBus>(
    bus: &mut B,
    if_khz: u32,
    lna: Lna,
) -> Result<(), TdaError<B::Error>> {
    let plan = plan(if_khz).ok_or(TdaError::Frequency)?;
    program_pll(bus, &plan)?;
    vco_channel_change(bus)?;
    set_lpf(bus)?;
    set_gains(bus, lna)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Op {
        R(u8),
        W(u8, u8),
        Sleep(u32),
    }
    use Op::*;

    /// Assumed register values after reset: arbitrary, different for every register, with bits
    /// set and clear in both polarities, so that a wrong mask or a missing read shows. The card
    /// is the only source of the real values.
    fn reset(reg: u8) -> u8 {
        reg.wrapping_mul(0x35) ^ 0xA7
    }

    /// Register file of the chip. Bit 4 of 0x11 and 0x15 (calibration done) is not stored: it
    /// reads 1 once `now_ms` has reached `done11_at` / `done15_at`, and never if that is `None`.
    /// Only `sleep_ms` advances the time. Bus calls are numbered from 0; the call `fail_at`
    /// fails with its number as the error, is recorded, and changes nothing.
    struct Fake {
        regs: [u8; 256],
        ops: Vec<Op>,
        now_ms: u32,
        done11_at: Option<u32>,
        done15_at: Option<u32>,
        calls: usize,
        fail_at: Option<usize>,
    }

    impl Fake {
        fn new() -> Self {
            Fake {
                regs: core::array::from_fn(|i| reset(i as u8)),
                ops: Vec::new(),
                now_ms: 0,
                done11_at: Some(0),
                done15_at: Some(0),
                calls: 0,
                fail_at: None,
            }
        }

        fn failing_at(n: usize) -> Self {
            Fake {
                fail_at: Some(n),
                ..Fake::new()
            }
        }

        fn call(&mut self) -> Result<(), usize> {
            let n = self.calls;
            self.calls += 1;
            if self.fail_at == Some(n) {
                Err(n)
            } else {
                Ok(())
            }
        }

        fn sleeps(&self) -> u32 {
            self.ops
                .iter()
                .map(|op| if let Sleep(ms) = op { *ms } else { 0 })
                .sum()
        }
    }

    impl TdaBus for Fake {
        type Error = usize;

        fn read(&mut self, reg: u8) -> Result<u8, usize> {
            self.ops.push(R(reg));
            self.call()?;
            let done = match reg {
                0x11 => Some(self.done11_at),
                0x15 => Some(self.done15_at),
                _ => None,
            };
            let value = self.regs[usize::from(reg)];
            Ok(match done {
                Some(at) if at.is_some_and(|at| self.now_ms >= at) => value | 0x10,
                Some(_) => value & !0x10,
                None => value,
            })
        }

        fn write(&mut self, reg: u8, value: u8) -> Result<(), usize> {
            self.ops.push(W(reg, value));
            self.call()?;
            self.regs[usize::from(reg)] = value;
            Ok(())
        }

        fn sleep_ms(&mut self, ms: u32) {
            self.ops.push(Sleep(ms));
            self.now_ms += ms;
        }
    }

    // Expected bus traffic for the reset values above, derived from the register sequences by an
    // independent script: all calibration done bits already set, so there is no sleep. `INIT_OPS`
    // is `init`, `TUNE_BS1_OPS` a `tune` of BS-1 after it and `TUNE_ND8_OPS` a `tune` of ND8
    // (the one transponder with R = 3, P = 4) after that.
    const INIT_OPS: &[Op] = &[
        R(0x02),
        W(0x02, 0xCD),
        R(0x06),
        W(0x06, 0x39),
        R(0x07),
        W(0x07, 0xFE),
        R(0x0F),
        W(0x0F, 0xBC),
        R(0x18),
        W(0x18, 0x5F),
        R(0x1A),
        W(0x1A, 0xC5),
        W(0x22, 0xFF),
        R(0x23),
        W(0x23, 0x98),
        R(0x25),
        W(0x25, 0x0E),
        R(0x27),
        W(0x27, 0xD4),
        R(0x24),
        W(0x24, 0xC7),
        R(0x0D),
        W(0x0D, 0x16),
        R(0x09),
        W(0x09, 0xB0),
        R(0x0A),
        W(0x0A, 0x7F),
        R(0x0B),
        W(0x0B, 0x7A),
        W(0x0C, 0x00),
        W(0x19, 0xFA),
        R(0x1B),
        W(0x1B, 0x30),
        R(0x21),
        W(0x21, 0x72),
        R(0x10),
        W(0x10, 0xB7),
        R(0x14),
        W(0x14, 0xA3),
        R(0x1A),
        W(0x1A, 0xC5),
        R(0x18),
        W(0x18, 0x5F),
        R(0x18),
        W(0x18, 0xDF),
        R(0x1B),
        W(0x1B, 0xB0),
        R(0x18),
        W(0x18, 0xDE),
        R(0x0F),
        W(0x0F, 0x9C),
        R(0x13),
        W(0x13, 0x68),
        R(0x12),
        W(0x12, 0xDD),
        R(0x10),
        W(0x10, 0xB7),
        R(0x10),
        W(0x10, 0xB7),
        R(0x0F),
        W(0x0F, 0xBC),
        R(0x11),
        R(0x10),
        R(0x0F),
        W(0x0F, 0xB7),
        R(0x0F),
        W(0x0F, 0xF7),
        R(0x0F),
        W(0x0F, 0xD7),
        R(0x12),
        W(0x12, 0x5D),
        R(0x0D),
        W(0x0D, 0x36),
        R(0x06),
        W(0x06, 0x39),
    ];
    const TUNE_BS1_OPS: &[Op] = &[
        R(0x25),
        W(0x22, 0x1E),
        R(0x23),
        W(0x23, 0x14),
        R(0x25),
        W(0x25, 0x0E),
        R(0x03),
        W(0x03, 0x38),
        R(0x1A),
        W(0x1A, 0xC5),
        W(0x1E, 0x1B),
        W(0x1F, 0x7A),
        W(0x20, 0x80),
        R(0x12),
        W(0x12, 0x5D),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x7A),
        R(0x15),
        R(0x13),
        W(0x13, 0x6A),
        R(0x12),
        W(0x12, 0x5D),
        R(0x0A),
        W(0x0A, 0x7A),
        R(0x0B),
        W(0x0B, 0xF8),
        R(0x07),
        R(0x06),
        W(0x06, 0x39),
        W(0x07, 0xAE),
        R(0x06),
        W(0x06, 0x09),
        R(0x09),
        W(0x09, 0xB0),
        R(0x0A),
        W(0x0A, 0x7A),
        R(0x0C),
        W(0x0C, 0x00),
    ];
    const TUNE_ND8_OPS: &[Op] = &[
        R(0x25),
        W(0x22, 0xFF),
        R(0x23),
        W(0x23, 0x18),
        R(0x25),
        W(0x25, 0x0E),
        R(0x03),
        W(0x03, 0xB8),
        R(0x1A),
        W(0x1A, 0xE5),
        W(0x1E, 0x40),
        W(0x1F, 0x8E),
        W(0x20, 0x39),
        R(0x12),
        W(0x12, 0x5D),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x6A),
        R(0x13),
        W(0x13, 0x7A),
        R(0x15),
        R(0x13),
        W(0x13, 0x6A),
        R(0x12),
        W(0x12, 0x5D),
        R(0x0A),
        W(0x0A, 0x7A),
        R(0x0B),
        W(0x0B, 0xF8),
        R(0x07),
        R(0x06),
        W(0x06, 0x09),
        W(0x07, 0xAE),
        R(0x06),
        W(0x06, 0x09),
        R(0x09),
        W(0x09, 0xB0),
        R(0x0A),
        W(0x0A, 0x7A),
        R(0x0C),
        W(0x0C, 0x00),
    ];

    fn initialised() -> Fake {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        fake.ops.clear();
        fake
    }

    // ---- worked examples --------------------------------------------------------------------

    struct Row {
        channel: &'static str,
        rf: u32,
        if_khz: u32,
        divider: u8,
        reg22: u8,
        reg23_or: u8,
        res_lsb: u64,
        n10: u64,
        r: u8,
        p: u8,
        reg03: u8,
        reg1a: u8,
        k: u64,
        n_reg: u8,
        frac: [u8; 2],
    }

    /// The 24 BS and ND transponders with their expected PLL values.
    const ROWS: [Row; 24] = [
        Row {
            channel: "BS-1",
            rf: 11_727_480,
            if_khz: 1_049_480,
            divider: 8,
            reg22: 0x1E,
            reg23_or: 0x04,
            res_lsb: 310_957_037,
            n10: 3110,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 155_478_520,
            n_reg: 0x1B,
            frac: [0x7A, 0x80],
        },
        Row {
            channel: "BS-3",
            rf: 11_765_840,
            if_khz: 1_087_840,
            divider: 7,
            reg22: 0xFF,
            reg23_or: 0xA8,
            res_lsb: 282_032_592,
            n10: 2820,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 141_016_300,
            n_reg: 0x0D,
            frac: [0x04, 0x2C],
        },
        Row {
            channel: "BS-5",
            rf: 11_804_200,
            if_khz: 1_126_200,
            divider: 7,
            reg22: 0xFF,
            reg23_or: 0xA8,
            res_lsb: 291_977_777,
            n10: 2920,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 145_988_890,
            n_reg: 0x11,
            frac: [0xFD, 0x27],
        },
        Row {
            channel: "BS-7",
            rf: 11_842_560,
            if_khz: 1_164_560,
            divider: 7,
            reg22: 0xFF,
            reg23_or: 0xA8,
            res_lsb: 301_922_962,
            n10: 3019,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 150_961_480,
            n_reg: 0x16,
            frac: [0xF6, 0x23],
        },
        Row {
            channel: "BS-9",
            rf: 11_880_920,
            if_khz: 1_202_920,
            divider: 7,
            reg22: 0xFF,
            reg23_or: 0xA8,
            res_lsb: 311_868_148,
            n10: 3119,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 155_934_070,
            n_reg: 0x1B,
            frac: [0xEF, 0x1F],
        },
        Row {
            channel: "BS-11",
            rf: 11_919_280,
            if_khz: 1_241_280,
            divider: 6,
            reg22: 0xFF,
            reg23_or: 0x28,
            res_lsb: 275_840_000,
            n10: 2758,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 137_920_000,
            n_reg: 0x09,
            frac: [0xEB, 0x85],
        },
        Row {
            channel: "BS-13",
            rf: 11_957_640,
            if_khz: 1_279_640,
            divider: 6,
            reg22: 0xFF,
            reg23_or: 0x28,
            res_lsb: 284_364_444,
            n10: 2844,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 142_182_220,
            n_reg: 0x0E,
            frac: [0x2E, 0xA5],
        },
        Row {
            channel: "BS-15",
            rf: 11_996_000,
            if_khz: 1_318_000,
            divider: 6,
            reg22: 0xFF,
            reg23_or: 0x28,
            res_lsb: 292_888_888,
            n10: 2929,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 146_444_440,
            n_reg: 0x12,
            frac: [0x71, 0xC6],
        },
        Row {
            channel: "BS-17",
            rf: 12_034_360,
            if_khz: 1_356_360,
            divider: 6,
            reg22: 0xFF,
            reg23_or: 0x28,
            res_lsb: 301_413_333,
            n10: 3014,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 150_706_670,
            n_reg: 0x16,
            frac: [0xB4, 0xE8],
        },
        Row {
            channel: "BS-19",
            rf: 12_072_720,
            if_khz: 1_394_720,
            divider: 6,
            reg22: 0xFF,
            reg23_or: 0x28,
            res_lsb: 309_937_777,
            n10: 3099,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 154_968_890,
            n_reg: 0x1A,
            frac: [0xF8, 0x09],
        },
        Row {
            channel: "BS-21",
            rf: 12_111_080,
            if_khz: 1_433_080,
            divider: 5,
            reg22: 0xFF,
            reg23_or: 0x88,
            res_lsb: 265_385_185,
            n10: 2654,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 132_692_590,
            n_reg: 0x04,
            frac: [0xB1, 0x4D],
        },
        Row {
            channel: "BS-23",
            rf: 12_149_440,
            if_khz: 1_471_440,
            divider: 5,
            reg22: 0xFF,
            reg23_or: 0x88,
            res_lsb: 272_488_888,
            n10: 2725,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 136_244_440,
            n_reg: 0x08,
            frac: [0x3E, 0x93],
        },
        Row {
            channel: "ND2",
            rf: 12_291_000,
            if_khz: 1_613_000,
            divider: 5,
            reg22: 0xFF,
            reg23_or: 0x88,
            res_lsb: 298_703_703,
            n10: 2987,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 149_351_850,
            n_reg: 0x15,
            frac: [0x5A, 0x12],
        },
        Row {
            channel: "ND4",
            rf: 12_331_000,
            if_khz: 1_653_000,
            divider: 5,
            reg22: 0xFF,
            reg23_or: 0x88,
            res_lsb: 306_111_111,
            n10: 3061,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 153_055_560,
            n_reg: 0x19,
            frac: [0x0E, 0x39],
        },
        Row {
            channel: "ND6",
            rf: 12_371_000,
            if_khz: 1_693_000,
            divider: 5,
            reg22: 0xFF,
            reg23_or: 0x88,
            res_lsb: 313_518_518,
            n10: 3135,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 156_759_260,
            n_reg: 0x1C,
            frac: [0xC2, 0x5E],
        },
        Row {
            channel: "ND8",
            rf: 12_411_000,
            if_khz: 1_733_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 256_740_740,
            n10: 2567,
            r: 3,
            p: 4,
            reg03: 2,
            reg1a: 1,
            k: 192_555_560,
            n_reg: 0x40,
            frac: [0x8E, 0x39],
        },
        Row {
            channel: "ND10",
            rf: 12_451_000,
            if_khz: 1_773_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 262_666_666,
            n10: 2627,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 131_333_330,
            n_reg: 0x03,
            frac: [0x55, 0x55],
        },
        Row {
            channel: "ND12",
            rf: 12_491_000,
            if_khz: 1_813_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 268_592_592,
            n10: 2686,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 134_296_300,
            n_reg: 0x06,
            frac: [0x4B, 0xDA],
        },
        Row {
            channel: "ND14",
            rf: 12_531_000,
            if_khz: 1_853_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 274_518_518,
            n10: 2745,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 137_259_260,
            n_reg: 0x09,
            frac: [0x42, 0x5E],
        },
        Row {
            channel: "ND16",
            rf: 12_571_000,
            if_khz: 1_893_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 280_444_444,
            n10: 2804,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 140_222_220,
            n_reg: 0x0C,
            frac: [0x38, 0xE3],
        },
        Row {
            channel: "ND18",
            rf: 12_611_000,
            if_khz: 1_933_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 286_370_370,
            n10: 2864,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 143_185_190,
            n_reg: 0x0F,
            frac: [0x2F, 0x68],
        },
        Row {
            channel: "ND20",
            rf: 12_651_000,
            if_khz: 1_973_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 292_296_296,
            n10: 2923,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 146_148_150,
            n_reg: 0x12,
            frac: [0x25, 0xED],
        },
        Row {
            channel: "ND22",
            rf: 12_691_000,
            if_khz: 2_013_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 298_222_222,
            n10: 2982,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 149_111_110,
            n_reg: 0x15,
            frac: [0x1C, 0x71],
        },
        Row {
            channel: "ND24",
            rf: 12_731_000,
            if_khz: 2_053_000,
            divider: 4,
            reg22: 0xFF,
            reg23_or: 0x08,
            res_lsb: 304_148_148,
            n10: 3041,
            r: 1,
            p: 2,
            reg03: 0,
            reg1a: 0,
            k: 152_074_070,
            n_reg: 0x18,
            frac: [0x12, 0xF6],
        },
    ];

    fn ratio_rp(ratio: Ratio) -> (u8, u8) {
        match ratio {
            Ratio::R1P2 => (1, 2),
            Ratio::R2P2 => (2, 2),
            Ratio::R3P2 => (3, 2),
            Ratio::R3P4 => (3, 4),
        }
    }

    /// Entries `(limit, code)` of the LPF table.
    const LPF_TABLE: [(u32, u8); 37] = [
        (2850, 0x13),
        (3900, 0x18),
        (4850, 0x22),
        (5800, 0x25),
        (6650, 0x31),
        (7700, 0x33),
        (8900, 0x41),
        (9900, 0x43),
        (10_700, 0x51),
        (11_400, 0x52),
        (12_500, 0x60),
        (13_500, 0x62),
        (14_550, 0x70),
        (15_800, 0x72),
        (16_800, 0x73),
        (17_750, 0x74),
        (18_350, 0x3D),
        (19_200, 0x75),
        (20_200, 0x76),
        (21_000, 0x5A),
        (22_450, 0x77),
        (23_900, 0x5B),
        (25_000, 0x6A),
        (26_500, 0x79),
        (27_900, 0x6B),
        (28_750, 0x7A),
        (29_700, 0x4E),
        (30_700, 0x5D),
        (31_450, 0x6C),
        (33_950, 0x7B),
        (39_000, 0x7C),
        (42_500, 0x7D),
        (43_950, 0x6E),
        (47_350, 0x5F),
        (51_700, 0x7E),
        (57_750, 0x6F),
        (100_000, 0x7F),
    ];

    #[test]
    fn table_has_all_transponders() {
        for row in &ROWS {
            // IF = RF - 10678000 kHz.
            assert_eq!(row.rf - 10_678_000, row.if_khz, "{}", row.channel);
        }
    }

    #[test]
    fn pll_values_match_every_transponder() {
        for row in ROWS {
            let ch = &row.channel;
            let lo = lo_config(row.if_khz);
            assert_eq!(lo.divider, row.divider, "{ch}");
            assert_eq!(lo.reg22, row.reg22, "{ch}");
            assert_eq!(lo.reg23_or, row.reg23_or, "{ch}");

            let res = res_lsb(lo.divider, row.if_khz);
            assert_eq!(res, row.res_lsb, "{ch}");
            assert_eq!(ratio_n10(res), row.n10, "{ch}");
            let ratio = select_ratio(res).unwrap();
            assert_eq!(ratio_rp(ratio), (row.r, row.p), "{ch}");
            assert_eq!(ratio_k(ratio, res), row.k, "{ch}");

            let plan = plan(row.if_khz).unwrap();
            assert_eq!(plan.lo, lo, "{ch}");
            assert_eq!(plan.ratio, ratio, "{ch}");
            assert_eq!(plan.ratio.reg03_field(), row.reg03, "{ch}");
            assert_eq!(u8::from(plan.ratio.predivider_4()), row.reg1a, "{ch}");
            assert_eq!(plan.n_reg, row.n_reg, "{ch}");
            assert_eq!(plan.frac.to_be_bytes(), row.frac, "{ch}");
        }
    }

    #[test]
    fn registers_after_tune_match_every_transponder() {
        for row in ROWS {
            let ch = &row.channel;
            let mut fake = initialised();
            let s = fake.regs;
            tune(&mut fake, row.if_khz, Lna::Least).unwrap();
            let r = fake.regs;

            // PLL registers.
            assert_eq!(r[0x22], row.reg22, "{ch}");
            assert_eq!(r[0x23], (s[0x23] & 0x53) | row.reg23_or, "{ch}");
            assert_eq!(r[0x25], s[0x25], "{ch}");
            assert_eq!(r[0x03], (s[0x03] & 0x3F) | (row.reg03 << 6), "{ch}");
            assert_eq!(r[0x1A], (s[0x1A] & 0xDF) | (row.reg1a << 5), "{ch}");
            assert_eq!(r[0x1E], row.n_reg, "{ch}");
            assert_eq!([r[0x1F], r[0x20]], row.frac, "{ch}");
            // The calibration start bits are cleared again.
            assert_eq!(r[0x13] & 0x10, 0, "{ch}");
            assert_eq!(r[0x12] & 0x80, 0, "{ch}");
            // Low-pass filter: 0x0A[3:0] = 0xA, 0x0B[7:1] = 0x7C.
            // Gains: 0x06 = &0x08 | 0x01, 0x07 = &0xAC | 0x02, 0x09 = &0x03 | 0xB0,
            // 0x0A[7:5] = 3, 0x0C[7:4] = 0.
            assert_eq!(r[0x0A], (s[0x0A] & 0x10) | 0x6A, "{ch}");
            assert_eq!(r[0x0B], (s[0x0B] & 0x01) | 0xF8, "{ch}");
            assert_eq!(r[0x06], (s[0x06] & 0x08) | 0x01, "{ch}");
            assert_eq!(r[0x07], (s[0x07] & 0xAC) | 0x02, "{ch}");
            assert_eq!(r[0x09], (s[0x09] & 0x03) | 0xB0, "{ch}");
            assert_eq!(r[0x0C] >> 4, 0, "{ch}");
        }
    }

    #[test]
    fn lpf_codes_follow_the_formula() {
        let table = LPF_TABLE;
        assert_eq!(table.len(), 37);

        let (symbol_rate, roll_off, offset): (u32, u32, u32) = (28_860, 135, 5_000);
        let bw = ((symbol_rate * roll_off) / 2 + offset * 100) / 72 + 2000;
        assert_eq!(bw, 36_000);
        let reg0a = match bw {
            45_000.. => 0x8,
            35_000.. => 0xA,
            25_000.. => 0xC,
            _ => 0xF,
        };
        let e = bw - 2000;
        let reg0b = table
            .iter()
            .find(|(limit, _)| e < *limit)
            .map_or(table[36].1, |(_, code)| *code);

        assert_eq!(LPF_0A_CODE, reg0a);
        assert_eq!(LPF_0B_CODE, reg0b);
    }

    #[test]
    fn gain_registers_follow_the_register_logic() {
        // The chosen gains.
        let (g0, g1, g2, g3) = (0u8, 0u8, 0u8, 0u8);
        let (lna, lpt, h2, h3, h4, ampout) = (4u8, 0u8, 5u8, 4u8, 3u8, 0u8);

        // The register logic of the gains.
        let reg07_or = (u8::from(g1 == 1) * 0x40)
            | (u8::from(g2 != 1) * 0x02)
            | (u8::from(g3 == 1) * 0x10)
            | (u8::from(g0 == 2 || g0 == 3));
        let lna_bits = match lna {
            4 => 0x01,
            0..=3 => 0x80 | ((lna & 3) << 4) | u8::from(lna == 3),
            _ => 0x80,
        };
        let reg06_or = lna_bits | ((lpt & 3) << 1);
        let reg09_or = (h2 << 5) | (h3 << 2);

        assert_eq!(g0 & 1, 0);
        assert_eq!(GAIN_07_OR, reg07_or);
        assert_eq!(Lna::Least.reg06_or(), reg06_or);
        // LNA code 0 with the same LPT.
        assert_eq!(Lna::Full.reg06_or(), 0x80 | ((lpt & 3) << 1));
        assert_eq!(GAIN_09_OR, reg09_or);
        assert_eq!(GAIN_0A_CODE, h4);
        assert_eq!(GAIN_0C_CODE, ampout);
    }

    #[test]
    fn sleep_switches_the_pll_off_and_leaves_the_rf_input_and_loop_through_on() {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        let awake = fake.regs;
        fake.ops.clear();
        sleep(&mut fake).unwrap();
        assert_eq!(
            fake.ops,
            [
                R(0x1B),
                W(0x1B, awake[0x1B] & 0x7F),
                R(0x18),
                W(0x18, awake[0x18] & 0x7F),
                R(0x1A),
                W(0x1A, awake[0x1A] & 0xBF),
            ]
        );
        // The RF input stage and the loop-through output stay on.
        assert_eq!(fake.regs[0x02], awake[0x02]);
        assert_eq!(fake.regs[0x06] & 0x08, 0x08);
    }

    #[test]
    fn wake_repeats_the_pll_power_up_of_init() {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        let awake = fake.regs;
        sleep(&mut fake).unwrap();
        let asleep = fake.regs;
        fake.ops.clear();
        wake(&mut fake).unwrap();
        // The PLL power-on and VCO calibration exactly as init runs them, from the same
        // registers.
        let mut steps = Fake {
            regs: asleep,
            ..Fake::new()
        };
        pll_power_on(&mut steps).unwrap();
        vco_power_on(&mut steps).unwrap();
        assert_eq!(fake.ops, steps.ops);
        for reg in [0x18, 0x1A, 0x1B] {
            assert_eq!(fake.regs[reg], awake[reg], "{reg:#04x}");
        }
    }

    #[test]
    fn wake_reports_a_calibration_that_does_not_finish() {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        sleep(&mut fake).unwrap();
        fake.done11_at = None;
        assert_eq!(wake(&mut fake), Err(TdaError::Calibration));
    }

    #[test]
    fn set_lna_rewrites_register_06_alone() {
        let mut fake = Fake::new();
        tune(&mut fake, 1_318_000, Lna::Least).unwrap();
        assert_eq!(fake.regs[0x06] & 0xB7, 0x01);
        fake.ops.clear();
        set_lna(&mut fake, Lna::Full).unwrap();
        assert_eq!(
            fake.ops,
            [R(0x06), W(0x06, (fake.regs[0x06] & 0x48) | 0x80)]
        );
        assert_eq!(fake.regs[0x06] & 0xB7, 0x80);
        set_lna(&mut fake, Lna::Least).unwrap();
        assert_eq!(fake.regs[0x06] & 0xB7, 0x01);
    }

    // ---- sequences --------------------------------------------------------------------------

    #[test]
    fn init_writes_the_expected_values_in_order() {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        assert_eq!(fake.ops, INIT_OPS);
        assert_eq!(fake.sleeps(), 0);
    }

    #[test]
    fn init_clears_the_pll_mode_bit_the_ratio_limits_assume() {
        let mut fake = Fake::new();
        fake.regs[0x1B] |= 0x01;
        fake.regs[0x21] &= !0x40;
        init(&mut fake).unwrap();
        // 0x1B[0] = 0 and 0x21[6] = 1 select RATIO_MIN_N10..=RATIO_MAX_N10.
        assert_eq!(fake.regs[0x1B] & 0x01, 0);
        assert_eq!(fake.regs[0x21] & 0x40, 0x40);
    }

    #[test]
    fn init_leaves_the_pll_alone() {
        // No default tune is done: no ratio, no integer or fraction, no VCO channel change (which
        // would poll 0x15).
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        for reg in [0x03, 0x1E, 0x1F, 0x20] {
            assert!(!fake.ops.iter().any(|op| matches!(op, W(r, _) if *r == reg)));
        }
        assert!(!fake.ops.contains(&R(0x15)));
    }

    #[test]
    fn init_reads_before_every_read_modify_write() {
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        // A write of a register that also has a read in the log is preceded by exactly that read.
        for (i, op) in fake.ops.iter().enumerate() {
            if let W(reg, _) = op {
                let rmw = i > 0 && fake.ops[i - 1] == R(*reg);
                let plain = matches!(reg, 0x22 | 0x0C | 0x19);
                assert!(rmw != plain, "write {i} to {reg:#04x}");
            }
        }
    }

    #[test]
    fn tune_writes_the_expected_values_in_order() {
        let mut fake = initialised();
        tune(&mut fake, 1_049_480, Lna::Least).unwrap();
        assert_eq!(fake.ops, TUNE_BS1_OPS);

        fake.ops.clear();
        tune(&mut fake, 1_733_000, Lna::Least).unwrap();
        assert_eq!(fake.ops, TUNE_ND8_OPS);
    }

    // ---- frequency range --------------------------------------------------------------------

    #[test]
    fn frequencies_outside_the_range_are_refused_without_bus_access() {
        for f in [0, 1, IF_MIN_KHZ - 1, IF_MAX_KHZ + 1, 10_678_000, u32::MAX] {
            let mut fake = initialised();
            assert_eq!(
                tune(&mut fake, f, Lna::Least),
                Err(TdaError::Frequency),
                "{f}"
            );
            assert!(fake.ops.is_empty(), "{f}");
        }
    }

    #[test]
    fn the_range_limits_are_accepted() {
        for f in [IF_MIN_KHZ, IF_MAX_KHZ] {
            let mut fake = initialised();
            assert_eq!(tune(&mut fake, f, Lna::Least), Ok(()), "{f}");
        }
    }

    #[test]
    fn every_frequency_of_the_range_has_a_pll_setting() {
        // The reference fraction: frac * 65536 / 10^6 in 16 doublings with scaling by 10.
        let reference_frac = |frac: u64| {
            let (mut c, mut d) = (frac, 1_000_000);
            for i in 0..16 {
                c *= 2;
                if c > 0x0FFF_FFFF && i != 15 {
                    c /= 10;
                    d /= 10;
                }
            }
            c / d
        };

        let mut largest_n10 = 0;
        for f in IF_MIN_KHZ..=IF_MAX_KHZ {
            let p = plan(f).unwrap_or_else(|| panic!("{f}"));
            let res = res_lsb(p.lo.divider, f);
            largest_n10 = largest_n10.max(ratio_n10(res));
            let k = ratio_k(p.ratio, res);
            assert_eq!(u64::from(p.frac), reference_frac(k % 1_000_000), "{f}");
            assert_eq!(u64::from(p.n_reg) + 128, k / 1_000_000, "{f}");
        }
        // The R/P = 1/4 and 3/4 fallbacks for 10 * N / 2 above the upper limit need
        // n10 > 2 * RATIO_MAX_N10.
        assert!(largest_n10 <= 2 * RATIO_MAX_N10);
    }

    // ---- calibration ------------------------------------------------------------------------

    fn polls(fake: &Fake, reg: u8) -> usize {
        fake.ops.iter().filter(|op| **op == R(reg)).count()
    }

    #[test]
    fn power_on_calibration_waits_between_polls() {
        for done in [1, 2, 7, CAL_LIMIT_MS] {
            let mut fake = Fake::new();
            fake.done11_at = Some(done);
            init(&mut fake).unwrap();
            assert_eq!(fake.sleeps(), done);
            assert_eq!(polls(&fake, 0x11), done as usize + 1);

            // Every read of the done bit but the first follows a sleep.
            let poll_ops: Vec<Op> = fake
                .ops
                .iter()
                .copied()
                .filter(|op| matches!(op, R(0x11) | Sleep(_)))
                .collect();
            assert_eq!(poll_ops.len(), 2 * done as usize + 1);
            for pair in poll_ops.chunks(2) {
                assert_eq!(pair[0], R(0x11));
                assert!(pair.len() == 1 || pair[1] == Sleep(CAL_POLL_MS));
            }
        }
    }

    #[test]
    fn power_on_calibration_that_never_finishes_fails() {
        for done in [Some(CAL_LIMIT_MS + 1), Some(1000), None] {
            let mut fake = Fake::new();
            fake.done11_at = done;
            assert_eq!(init(&mut fake), Err(TdaError::Calibration));
            assert_eq!(fake.sleeps(), CAL_LIMIT_MS);
            assert_eq!(fake.now_ms, CAL_LIMIT_MS);
            assert_eq!(polls(&fake, 0x11), (CAL_LIMIT_MS + 1) as usize);
            // Nothing after the failed wait: no result copied, no loop-through.
            assert_eq!(fake.ops.last(), Some(&R(0x11)));
        }
    }

    #[test]
    fn power_on_calibration_copies_the_result() {
        // 0x10[3:0] to 0x0F[3:0].
        let mut fake = Fake::new();
        init(&mut fake).unwrap();
        // The code is read after the init table and the two start writes of 0x10[5].
        let code = ((reset(0x10) & 0x2F) | 0x90 | 0x20) & 0x0F;
        let started = (reset(0x0F) & 0x9F) | 0x80 | 0x20;
        assert_eq!(fake.regs[0x0F], (((started & 0xF0) | code) | 0x40) & 0xDF);
    }

    #[test]
    fn channel_calibration_waits_between_polls() {
        for done in [1, 2, 7, CAL_LIMIT_MS] {
            let mut fake = initialised();
            fake.done15_at = Some(done);
            tune(&mut fake, 1_318_000, Lna::Least).unwrap();
            assert_eq!(fake.sleeps(), done);
            assert_eq!(polls(&fake, 0x15), done as usize + 1);

            let poll_ops: Vec<Op> = fake
                .ops
                .iter()
                .copied()
                .filter(|op| matches!(op, R(0x15) | Sleep(_)))
                .collect();
            for pair in poll_ops.chunks(2) {
                assert_eq!(pair[0], R(0x15));
                assert!(pair.len() == 1 || pair[1] == Sleep(CAL_POLL_MS));
            }
            // The tune went on to the filter and the gains.
            assert_eq!(fake.ops.last(), Some(&W(0x0C, fake.regs[0x0C])));
        }
    }

    #[test]
    fn channel_calibration_that_never_finishes_fails_and_cleans_up() {
        for done in [Some(CAL_LIMIT_MS + 1), Some(1000), None] {
            let mut fake = initialised();
            fake.done15_at = done;
            let before = fake.regs;
            let mut finished = initialised();
            tune(&mut finished, 1_318_000, Lna::Least).unwrap();
            assert_eq!(
                tune(&mut fake, 1_318_000, Lna::Least),
                Err(TdaError::Calibration)
            );
            assert_eq!(fake.sleeps(), CAL_LIMIT_MS);
            assert_eq!(polls(&fake, 0x15), (CAL_LIMIT_MS + 1) as usize);

            // The calibration registers end as after a successful calibration (leaving 0x13[4]
            // and 0x12[7] set would be a mistake), and neither the filter nor the gains were
            // touched.
            assert_eq!(fake.regs[0x13], finished.regs[0x13]);
            assert_eq!(fake.regs[0x12], finished.regs[0x12]);
            assert_eq!(fake.regs[0x13] & 0x10, 0);
            assert_eq!(fake.regs[0x12] & 0x80, 0);
            for reg in [0x0A, 0x0B, 0x06, 0x07, 0x09, 0x0C] {
                assert_eq!(fake.regs[reg], before[reg], "{reg:#04x}");
            }
            // The PLL itself was programmed.
            assert_eq!(fake.regs[0x22], 0xFF);
        }
    }

    #[test]
    fn calibration_time_is_bounded() {
        let mut fake = Fake::new();
        fake.done11_at = None;
        fake.done15_at = None;
        assert_eq!(init(&mut fake), Err(TdaError::Calibration));
        let mut total = fake.now_ms;

        let mut fake = initialised();
        fake.done15_at = None;
        assert_eq!(
            tune(&mut fake, 1_318_000, Lna::Least),
            Err(TdaError::Calibration)
        );
        total += fake.now_ms;
        assert_eq!(total, 2 * CAL_LIMIT_MS);
    }

    // ---- bus errors -------------------------------------------------------------------------

    #[test]
    fn every_bus_error_of_init_is_reported() {
        let mut clean = Fake::new();
        init(&mut clean).unwrap();
        assert_eq!(clean.calls, clean.ops.len());

        for n in 0..clean.calls {
            let mut fake = Fake::failing_at(n);
            assert_eq!(init(&mut fake), Err(TdaError::Bus(n)), "call {n}");
            // Nothing is done after the failure.
            assert_eq!(fake.calls, n + 1, "call {n}");
        }
    }

    #[test]
    fn every_bus_error_of_tune_is_reported() {
        let mut clean = initialised();
        tune(&mut clean, 1_318_000, Lna::Least).unwrap();
        let init_calls = {
            let mut f = Fake::new();
            init(&mut f).unwrap();
            f.calls
        };
        let tune_calls = clean.calls - init_calls;
        assert!(tune_calls > 40);

        for n in 0..tune_calls {
            let mut fake = Fake::new();
            init(&mut fake).unwrap();
            fake.fail_at = Some(init_calls + n);
            assert_eq!(
                tune(&mut fake, 1_318_000, Lna::Least),
                Err(TdaError::Bus(init_calls + n)),
                "call {n}"
            );
            assert_eq!(fake.calls, init_calls + n + 1, "call {n}");
        }
    }

    #[test]
    fn a_bus_error_while_polling_is_not_a_calibration_error() {
        // No sleep comes before the first poll, so the position of its read in the log is also
        // its call number.
        let mut probe = Fake::new();
        probe.done11_at = None;
        let _ = init(&mut probe);
        let first_poll = probe.ops.iter().position(|op| *op == R(0x11)).unwrap();

        let mut fake = Fake::failing_at(first_poll + 3);
        fake.done11_at = None;
        assert_eq!(init(&mut fake), Err(TdaError::Bus(first_poll + 3)));
    }

    #[test]
    fn a_bus_error_while_cleaning_up_is_reported() {
        let mut probe = initialised();
        probe.done15_at = None;
        let base = probe.calls;
        assert_eq!(
            tune(&mut probe, 1_318_000, Lna::Least),
            Err(TdaError::Calibration)
        );
        let last = probe.calls - 1;

        let mut fake = initialised();
        fake.done15_at = None;
        fake.fail_at = Some(last);
        assert_eq!(
            tune(&mut fake, 1_318_000, Lna::Least),
            Err(TdaError::Bus(last))
        );
        assert!(last > base);
    }
}
