// SPDX-License-Identifier: GPL-2.0

//! Newport Media NM120 terrestrial tuner (8-bit I2C address 0xCE behind the demodulator gateway).
//!
//! The register sequences are the revealed ones. Only what the board reaches is implemented:
//! standard 6 (digital 6 MHz), output mode 2 (digital low IF of 4 MHz), no frequency offset, no
//! spectrum inversion and none of the override fields, and sleep and wake. The NM130/NM131 parts
//! and analogue standards are not.
//!
//! The wire format (the gateway prefix and the register payloads) belongs to the caller; this
//! module works at register level through [`NmiBus`]. Every bus error is returned at once;
//! latching the first error and silently refusing all later accesses would be a mistake.

/// Register access to the tuner and a delay, supplied by the caller.
///
/// Register reads are read-modify-write inputs; nothing here assumes a reset value.
pub(crate) trait NmiBus {
    /// Error of a failed bus transaction.
    type Error;

    /// Reads the 8-bit RF register `reg` (16-bit address 0x0000..0x00FF).
    fn read8(&mut self, reg: u8) -> Result<u8, Self::Error>;

    /// Writes the 8-bit RF register `reg`.
    fn write8(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Reads the 32-bit baseband register at `addr` (0x0100..0x03FC).
    fn read32(&mut self, addr: u16) -> Result<u32, Self::Error>;

    /// Writes the 32-bit baseband register at `addr`.
    fn write32(&mut self, addr: u16, value: u32) -> Result<(), Self::Error>;

    /// Sleeps for `ms` milliseconds.
    fn sleep_ms(&mut self, ms: u32);
}

/// Why an NM120 operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NmiError<E> {
    /// A bus transaction failed.
    Bus(E),
    /// The chip ID register holds this value, which is not an NM120.
    UnsupportedChip(u32),
    /// The RF frequency is outside [`RF_MIN_HZ`]..=[`RF_MAX_HZ`].
    Frequency,
}

/// Lowest accepted RF frequency: the lower limit of the ISDB-T range the driver declares. The
/// worked examples start at 93.143 MHz; the tables would take lower frequencies, but nothing
/// shows that the chip tunes there.
pub(crate) const RF_MIN_HZ: u32 = 93_000_000;

/// Highest accepted RF frequency: the upper limit of the ISDB-T range the driver declares. The
/// arithmetic of [`Pll::new`] is proven up to this value plus the retry step.
pub(crate) const RF_MAX_HZ: u32 = 768_000_000;

/// Chip ID register (32-bit).
const REG_CHIP_ID: u16 = 0x3FC;

/// RF gain register (8-bit) and its value for gain level 0, the chosen gain. The library calls it
/// the loop-through gain, but on the card bit 4 is a gain stage in the tuner's own path: clearing
/// it costs 6 to 7 dB of C/N. No other bit, nor the reset value 0xFF, made a measurable
/// difference; 0xFB is kept as the reference value.
const REG_RF_GAIN: u8 = 0x0A;
const RF_GAIN: u8 = 0xFB;

/// LDO register (8-bit); bit 7 is the LDO bypass, which this board leaves off.
const REG_LDO: u8 = 0x36;
const LDO_BYPASS: u8 = 0x80;

/// Baseband control register (32-bit): sleep clears bits 0 and 5 and sets bit 4, wake does the
/// opposite.
const REG_BB_CTRL: u16 = 0x104;
const BB_RUN: u32 = 0x21;
const BB_HOLD: u32 = 0x10;

/// What sleep writes to the 8-bit registers it saves: analogue gains off, RF gain register 0xF4
/// with the revealed level 3, and the LDO at 0x78 (bypass bit clear).
const SLEEP_ANALOGUE_GAIN: u8 = 0x00;
const SLEEP_RF_GAIN: u8 = 0xF7;
const SLEEP_LDO: u8 = 0x78;

/// RF defaults written at init, in order.
const RF_DEFAULTS: [(u8, u8); 30] = [
    (0x06, 0x48),
    (0x07, 0x40),
    (0x0A, 0xEB),
    (0x0B, 0x11),
    (0x0C, 0x10),
    (0x0D, 0x88),
    (0x10, 0x04),
    (0x11, 0x30),
    (0x12, 0x30),
    (0x15, 0xAA),
    (0x16, 0x03),
    (0x17, 0x80),
    (0x18, 0x67),
    (0x19, 0xD4),
    (0x1A, 0x44),
    (0x1C, 0x10),
    (0x1D, 0xEE),
    (0x1E, 0x99),
    (0x21, 0xC5),
    (0x22, 0x91),
    (0x24, 0x01),
    (0x2B, 0x91),
    (0x2D, 0x01),
    (0x2F, 0x80),
    (0x31, 0x00),
    (0x33, 0x00),
    (0x38, 0x00),
    (0x39, 0x2F),
    (0x3A, 0x00),
    (0x3B, 0x00),
];

/// Baseband defaults written at init.
const BB_DEFAULTS: [(u16, u32); 2] = [(0x164, 0x0000_0800), (0x1C0, 0x2D8C_19C7)];

/// NM120 specific RF registers written at init.
const NM120_TABLE: [(u8, u8); 9] = [
    (0x0E, 0x45),
    (0x1B, 0x0E),
    (0x23, 0xFF),
    (0x26, 0x82),
    (0x28, 0x00),
    (0x30, 0xDF),
    (0x32, 0xDF),
    (0x34, 0x68),
    (0x35, 0x18),
];

/// Crystal frequency in kHz (24 MHz).
const XO_KHZ: u32 = 24_000;

/// Reference selection register: bits 1:0 = 2 doubles and = 3 halves the reference.
const REG_REF_SEL: u8 = 0x21;

/// The kHz added when the fractional part is nonzero.
const RETRY_STEP_HZ: u32 = 1000;

/// Intermediate frequency of output mode 2 (digital low IF), Hz.
const IF_HZ: u32 = 4_000_000;

/// RF band: highest frequency in Hz (inclusive), register 0x1D bits 7..5 and LO divider
/// multiplier M. Above the last entry: band 1, M = 1.
const BANDS: [(u32, u8, u32); 6] = [
    (53_000_000, 7, 12),
    (79_000_000, 6, 9),
    (118_000_000, 5, 6),
    (214_000_000, 4, 4),
    (237_000_000, 3, 3),
    (434_000_000, 2, 2),
];

/// Tracking filter: lower edge in Hz and registers 0x08, 0x09. Entry `i` applies to
/// `F[i] < fr <= F[i + 1]`; anything else gets the last entry's zeros.
const FILTERS: [(u32, u8, u8); 9] = [
    (45_000_000, 0xA7, 0x3A),
    (55_000_000, 0x97, 0x39),
    (65_000_000, 0x64, 0x36),
    (75_000_000, 0x53, 0x35),
    (85_000_000, 0x52, 0x35),
    (95_000_000, 0x41, 0x34),
    (105_000_000, 0x40, 0x34),
    (115_000_000, 0x40, 0x34),
    (125_000_000, 0x00, 0x00),
];

/// Everything the RF part derives from the frequency.
struct Pll {
    /// RF frequency actually programmed, Hz.
    fr: u32,
    /// Band, 1..=7 (register 0x1D bits 7..5).
    band: u8,
    /// Registers 0x08 and 0x09.
    filter: (u8, u8),
    /// Integer part of the feedback divider.
    n: u32,
    /// 19-bit fractional part.
    frac: u32,
    /// Baseband clock divider, 16..=31.
    d: u32,
    /// Sample clock offset in 1/512 kHz relative to 216 MHz; positive over the accepted range.
    clk: u32,
}

impl Pll {
    /// Computes the synthesiser setting for `fr` Hz with the reference at `xo_khz`.
    ///
    /// Range proof for `RF_MIN_HZ <= fr <= RF_MAX_HZ + RETRY_STEP_HZ` and `xo_khz` in
    /// 12 000..=48 000. The divider product is `v = M * 8 * (fr / 1000)`:
    /// - `v` is at most 6 944 000 (band 2 at 434 MHz; band 1 reaches 6 144 008), so `v << 9` is
    ///   below 2^32 and equals the product mod 2^32.
    /// - `v` is at least 3 472 000 in band 1 (above 434 MHz), 3 792 000 in band 2, 5 136 000 in
    ///   band 3, 3 776 000 in band 4 and 4 464 000 in band 5 (from 93 MHz), all above the
    ///   3 456 000 at which `d` would be clamped to 16 with `v / d` below 216 000. Where `d` is
    ///   `v / 216 000` or the upper clamp 31, `v / d >= 216 000`, so `clk` is not negative; it
    ///   stays below 7 000 000.
    /// - `r < xo_khz` gives `(2^31 / xo_khz) * r < 2^31`.
    fn new(fr: u32, xo_khz: u32) -> Self {
        let (band, m) = BANDS
            .iter()
            .find(|&&(upper, _, _)| fr <= upper)
            .map_or((1, 1), |&(_, band, m)| (band, m));
        let filter = FILTERS
            .windows(2)
            .find(|w| w[0].0 < fr && fr <= w[1].0)
            .map_or((0, 0), |w| (w[0].1, w[0].2));

        let v = m * 8 * (fr / 1000);
        let n = v / xo_khz;
        let r = v % xo_khz;
        let frac = (((0x8000_0000 / xo_khz) * r) >> 12) & 0x7_FFFF;

        let d = (v / 216_000).clamp(16, 31);
        let clk = ((v << 9) / d).wrapping_sub(110_592_000);

        Pll {
            fr,
            band,
            filter,
            n,
            frac,
            d,
            clk,
        }
    }
}

/// State of one NM120 that the tune path needs between calls.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Nmi {
    /// Last value written to register 0x05, `None` until the first write. Starting from an assumed
    /// 0x87 that was never read from the chip would mean a computed 0x87 is never written; here
    /// the first tune always writes.
    reg05: Option<u8>,
    /// The "first tune" register blocks have been written. They run whenever the standard or the
    /// mode differs from the last tune (both unset after init); this driver only ever uses one
    /// standard and one mode, so that is a flag.
    baseband_ready: bool,
    /// What [`Nmi::sleep`] overwrote, while the tuner is stopped.
    asleep: Option<Saved>,
}

/// Registers that sleep overwrites and wake restores.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Saved {
    reg09: u8,
    reg0a: u8,
    reg34: u8,
    reg35: u8,
    reg36: u8,
}

/// Sets and clears bits of the 8-bit register `reg` read through the bus.
fn modify8<B: NmiBus>(bus: &mut B, reg: u8, set: u8, clear: u8) -> Result<(), NmiError<B::Error>> {
    let value = bus.read8(reg).map_err(NmiError::Bus)?;
    write8(bus, reg, (value & !clear) | set)
}

/// Applies `f` to the 32-bit register `addr` read through the bus and writes the result back.
fn modify32<B: NmiBus>(
    bus: &mut B,
    addr: u16,
    f: impl FnOnce(u32) -> u32,
) -> Result<u32, NmiError<B::Error>> {
    let value = f(bus.read32(addr).map_err(NmiError::Bus)?);
    bus.write32(addr, value).map_err(NmiError::Bus)?;
    Ok(value)
}

fn write8<B: NmiBus>(bus: &mut B, reg: u8, value: u8) -> Result<(), NmiError<B::Error>> {
    bus.write8(reg, value).map_err(NmiError::Bus)
}

fn write32<B: NmiBus>(bus: &mut B, addr: u16, value: u32) -> Result<(), NmiError<B::Error>> {
    bus.write32(addr, value).map_err(NmiError::Bus)
}

/// Whether `id` is an NM120. The NM130 (`id & 0xFFFF00` = 0x013000 or 0x813000) and NM131
/// (`id & 0xFFF00` = 0x13100) patterns exist but are not supported here.
fn is_nm120(id: u32) -> bool {
    id & 0xF_FF00 == 0x1_2000
}

impl Nmi {
    /// Identifies the chip and brings it to the state of the revealed init sequence, with RF gain
    /// register 0x0A = 0xFB.
    ///
    /// The sequence also reads registers 0x00, 0x34 and 0x35 into shadows and the chip ID
    /// a second time for a log line. Neither is done here: [`Nmi::sleep`] reads what it
    /// saves when it runs and never writes 0x00, and the driver has no such log line.
    pub(crate) fn init<B: NmiBus>(bus: &mut B) -> Result<Self, NmiError<B::Error>> {
        let id = bus.read32(REG_CHIP_ID).map_err(NmiError::Bus)?;
        if !is_nm120(id) {
            return Err(NmiError::UnsupportedChip(id));
        }

        for &(reg, value) in &RF_DEFAULTS {
            write8(bus, reg, value)?;
        }

        // The LDO bypass is forced to the board configuration (0 = not bypassed) on every write of
        // this register (as revealed); the NM120 table below does not touch it
        // again.
        let ldo = bus.read8(REG_LDO).map_err(NmiError::Bus)?;
        write8(bus, REG_LDO, ldo & !LDO_BYPASS)?;

        for &(addr, value) in &BB_DEFAULTS {
            write32(bus, addr, value)?;
        }
        for &(reg, value) in &NM120_TABLE {
            write8(bus, reg, value)?;
        }

        write8(bus, REG_RF_GAIN, RF_GAIN)?;

        Ok(Nmi {
            reg05: None,
            baseband_ready: false,
            asleep: None,
        })
    }

    /// Stops the tuner: the sleep sequence without the parts that act on the RF front end. The
    /// revealed sequence starts with a tune to 474 MHz in another standard, which the next tune
    /// overwrites in full, and it writes register 0x00, register 0x05 and bit 6 of register 0x07.
    /// Those three are left out because they disturb the other NM120, which shares the antenna
    /// feed: measured step by step with the stopped tuner locked to a channel, as it is when its
    /// frontend closes, clearing 0x07 bit 6 was followed by losses of sync of the other tuner in 25
    /// of 40 rounds (none in the control windows) and kept them coming until it was set again; the
    /// writes of 0x00 and 0x05 drew every such loss in an earlier measurement, and once the other
    /// tuner lost packets within 4 ms of a full sleep. Without them, every step stayed within one
    /// round of its control window, the other tuner lost packets no more often while this one was
    /// opened and closed every 4 s than while it stayed asleep or awake, and the stopped tuner
    /// still delivers nothing (its demodulator sees no signal).
    ///
    /// A tuner that is asleep already is left as it is: sleeping again would save the values of
    /// the sleep state. The values are saved before the first write, so that a wake restores them
    /// even after a sleep that failed part-way.
    pub(crate) fn sleep<B: NmiBus>(&mut self, bus: &mut B) -> Result<(), NmiError<B::Error>> {
        if self.asleep.is_some() {
            return Ok(());
        }
        let mut read = |reg| bus.read8(reg).map_err(NmiError::Bus);
        let saved = Saved {
            reg09: read(0x09)?,
            reg0a: read(REG_RF_GAIN)?,
            reg34: read(0x34)?,
            reg35: read(0x35)?,
            reg36: read(REG_LDO)?,
        };
        self.asleep = Some(saved);
        modify32(bus, REG_BB_CTRL, |v| (v & !BB_RUN) | BB_HOLD)?;
        write8(bus, 0x09, saved.reg09 & !0x10)?;
        modify8(bus, 0x0B, 0, 0x10)?;
        write8(bus, 0x34, SLEEP_ANALOGUE_GAIN)?;
        write8(bus, 0x35, SLEEP_ANALOGUE_GAIN)?;
        write8(bus, REG_RF_GAIN, SLEEP_RF_GAIN)?;
        write8(bus, REG_LDO, SLEEP_LDO)
    }

    /// Restarts a stopped tuner: the wake sequence for the registers [`Nmi::sleep`] writes, and bit
    /// 4 of register 0x09, which the revealed wake leaves cleared. A tuner that is awake is left as
    /// it is; after a failed wake, the next one tries again. The next tune programs the frequency.
    pub(crate) fn wake<B: NmiBus>(&mut self, bus: &mut B) -> Result<(), NmiError<B::Error>> {
        let Some(saved) = self.asleep else {
            return Ok(());
        };
        modify32(bus, REG_BB_CTRL, |v| (v & !BB_HOLD) | BB_RUN)?;
        modify8(bus, 0x0B, 0x10, 0)?;
        write8(bus, 0x34, saved.reg34)?;
        write8(bus, 0x35, saved.reg35)?;
        write8(bus, REG_RF_GAIN, saved.reg0a)?;
        write8(bus, REG_LDO, saved.reg36 & !LDO_BYPASS)?;
        modify8(bus, 0x09, saved.reg09 & 0x10, 0)?;
        self.asleep = None;
        Ok(())
    }

    /// Tunes to `rf_hz` for ISDB-T (standard 6, mode 2, IF 4 MHz).
    ///
    /// Returns `Frequency` before touching the bus if `rf_hz` is outside
    /// [`RF_MIN_HZ`]..=[`RF_MAX_HZ`]. After a bus error the chip is in an undefined intermediate
    /// state; the next tune rewrites everything the first tune after init does.
    ///
    /// This code does not wait for or check a PLL lock here; the caller waits and judges lock on
    /// the demodulator.
    pub(crate) fn tune<B: NmiBus>(
        &mut self,
        bus: &mut B,
        rf_hz: u32,
    ) -> Result<(), NmiError<B::Error>> {
        if !(RF_MIN_HZ..=RF_MAX_HZ).contains(&rf_hz) {
            return Err(NmiError::Frequency);
        }
        let clk = self.tune_rf(bus, rf_hz)?;
        self.tune_baseband(bus, rf_hz, clk)?;
        self.baseband_ready = true;
        Ok(())
    }

    /// Returns the sample clock offset the baseband part needs.
    fn tune_rf<B: NmiBus>(&mut self, bus: &mut B, rf_hz: u32) -> Result<u32, NmiError<B::Error>> {
        // Read once. Reading it in every pass and applying the doubling or halving again on the
        // retry pass would be a mistake.
        let xo = match bus.read8(REG_REF_SEL).map_err(NmiError::Bus)? & 3 {
            2 => XO_KHZ * 2,
            3 => XO_KHZ / 2,
            _ => XO_KHZ,
        };

        // 1 kHz above the requested frequency is programmed whenever the fractional part is
        // nonzero, so almost every tune is retried. This may be an inverted spur test, but what was
        // meant cannot be told: the same condition picks integer or fractional mode in register
        // 0x1B on the other parts, which argues for a deliberate polarity. This is kept as
        // revealed; on the card the exact frequency and the retry gave the same C/N and lock time.
        // Only the final pass is written to the chip: registers 0x05, 0x08 and 0x09 of the first
        // pass would be overwritten by the retry pass.
        let mut pll = Pll::new(rf_hz, xo);
        if pll.frac != 0 {
            pll = Pll::new(rf_hz + RETRY_STEP_HZ, xo);
        }
        let fr = pll.fr;

        // Register 0x05: 0x05 only in 120.1..120.4 MHz, 0x85 elsewhere (digital standard).
        let reg05 = if (120_100_000..=120_400_000).contains(&fr) {
            0x05
        } else {
            0x85
        };
        if self.reg05 != Some(reg05) {
            write8(bus, 0x05, reg05)?;
            self.reg05 = Some(reg05);
        }

        write8(bus, 0x08, pll.filter.0)?;
        write8(bus, 0x09, pll.filter.1)?;

        // Every value is masked to the byte it is written to. `d << 4` keeps only the low
        // nibble of `d` (d is 16..=31, so bit 4 of d is dropped, as by a byte write).
        write8(bus, 0x01, ((pll.n >> 1) & 0xFF) as u8)?;
        write8(bus, 0x02, (((pll.frac << 1) | (pll.n & 1)) & 0xFF) as u8)?;
        write8(bus, 0x03, ((pll.frac >> 7) & 0xFF) as u8)?;
        write8(
            bus,
            0x04,
            (((pll.frac >> 15) & 0x0F) | ((pll.d & 0x0F) << 4)) as u8,
        )?;

        let reg1d = bus.read8(0x1D).map_err(NmiError::Bus)?;
        write8(bus, 0x1D, (reg1d & 0x1F) | (pll.band << 5))?;

        // Register 0x1B is not touched on the NM120.
        if fr >= 300_000_000 {
            write8(bus, 0x25, 0xF4)?;
            write8(bus, 0x27, 0xEF)?;
            write8(bus, 0x29, 0x4F)?;
            write8(bus, 0x2E, 0x34)?;
        } else {
            write8(bus, 0x25, 0x78)?;
            write8(bus, 0x27, 0x7F)?;
            write8(bus, 0x29, 0x7F)?;
            write8(bus, 0x2E, 0x12)?;
        }
        // Both values have bit 7 clear, as the LDO bypass setting requires.
        write8(bus, REG_LDO, if fr >= 155_000_000 { 0x7C } else { 0x54 })?;
        write8(
            bus,
            0x37,
            if (155_000_000..300_000_000).contains(&fr) {
                0x9C
            } else {
                0x84
            },
        )?;

        Ok(pll.clk)
    }

    /// `rf_hz` is the requested frequency without the retry step.
    fn tune_baseband<B: NmiBus>(
        &self,
        bus: &mut B,
        rf_hz: u32,
        clk: u32,
    ) -> Result<(), NmiError<B::Error>> {
        let first = !self.baseband_ready;
        // `clk / 6750 <= 636 288`, so `q` neither overflows nor is zero.
        let q = clk / 6750 + 0x4000;

        if first {
            write32(bus, 0x130, 0x200)?;
        }
        write32(bus, 0x164, if rf_hz >= 300_000_000 { 0x500 } else { 0x600 })?;
        if first {
            modify32(bus, 0x234, |v| (v & 0xF1FF_FFFF) | 0x0800_0000)?;
            write32(bus, 0x20C, 0x400)?;
            // 0x280 is written back unchanged in mode 2 (as revealed).
            modify32(bus, 0x280, |v| v)?;
            modify32(bus, 0x238, |v| v & 0xFFFF_C000)?;
            modify32(bus, 0x32C, |v| v & 0xBFFF_BFFF)?;
        }

        // `(2 * 0x1ED09) << 14` is 0xF684_8000 and fits.
        write32(bus, 0x230, (0xF684_8000 / q) | 0x8_0000)?;
        write32(bus, 0x250, 0)?;
        // Picking 0x27C from the previous mode would be a mistake; in mode 2 it is always 0x1010.
        write32(bus, 0x27C, 0x1010)?;
        // The DDFS table entry of standard 6 is 0.
        write32(bus, 0x1BC, 0)?;

        if first {
            write32(bus, 0x1C8, 0x8079)?;
            write32(bus, 0x1CC, 0x8079)?;
            write32(bus, 0x1D0, 0x8079)?;
            write32(bus, 0x110, 0x4C52)?;
            write32(bus, 0x26C, 2)?;
            write32(bus, 0x270, 6)?;
        }

        // 4 MHz IF NCO relative to the sample clock; the divisor is at least 6750.
        let nco = 2 * (((IF_HZ / 1000) << 16) / ((clk >> 14) + 6750));
        modify32(bus, 0x21C, |v| {
            (((v & 0xFFC0_0000) | nco) & 0xCFBF_FFFF) | 0x0800_0000
        })?;
        modify32(bus, 0x234, |v| v & 0xCFC0_0000)?;

        if first {
            modify32(bus, 0x12C, |v| (v & 0xFFFF_9F7F) | 0x1000)?;
            write32(bus, 0x138, 0x5B18)?;
            write32(bus, 0x148, 0x1F_F082)?;
            modify32(bus, 0x200, |v| (v & 0xFFFF_C000) | 0x8800)?;
        }

        // The subtraction wraps by design: 2^32 - 0x4E058000 = 13824 * 216000.
        let reg210 =
            ((864u32.wrapping_mul(clk >> 5).wrapping_sub(0x4E05_8000) / 216_000) & 0xFE00_FFF3) | 3;
        write32(bus, 0x210, reg210)?;

        // Register 0x104: digital standard, bits 14..11 = 1, bit 15 = 1, bits 19..16 from the
        // clock, bit 21 (mode 2), bit 23 (NM120).
        let clk_bits = if clk > 0x38_3FFC {
            2
        } else if clk > 0x1C_1FF8 {
            1
        } else {
            0
        };
        let ctrl = modify32(bus, 0x104, |v| {
            (v & 0x9400_07D1)
                | 0x1000_0011
                | (1 << 11)
                | (1 << 15)
                | (clk_bits << 16)
                | (1 << 21)
                | (1 << 23)
        })?;
        let ctrl = (ctrl & !0x10) | 0x2000_0020;
        write32(bus, 0x104, ctrl)?;
        bus.sleep_ms(1);
        if bus.read32(0x328).map_err(NmiError::Bus)? != 0 {
            write32(bus, 0x104, ctrl & !0x20)?;
            write32(bus, 0x104, ctrl)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::HashMap;
    use std::vec;
    use std::vec::Vec;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Op {
        R8(u8),
        W8(u8, u8),
        R32(u16),
        W32(u16, u32),
        Sleep(u32),
    }
    use Op::*;

    /// Error of a failed access: the index of the access.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Fail(usize);

    /// Assumed reset value of every 8-bit register (the card has not been read).
    const RESET8: u8 = 0x00;
    /// Assumed reset value of every 32-bit register except the chip ID and the status register
    /// 0x328 (see `inited`): all ones, so that the masks of the read-modify-write registers show in
    /// the written values.
    const RESET32: u32 = 0xFFFF_FFFF;

    /// Register-file bus that logs every access. Register reads and writes are numbered from 0
    /// in the order of the calls; the one numbered `fail_at` fails with its number and changes
    /// nothing. Sleeps are logged but not numbered.
    struct Fake {
        reg8: [u8; 256],
        reg32: HashMap<u16, u32>,
        log: Vec<Op>,
        accesses: usize,
        fail_at: Option<usize>,
    }

    impl Fake {
        fn new(chip_id: u32) -> Self {
            let mut reg32 = HashMap::new();
            reg32.insert(REG_CHIP_ID, chip_id);
            Fake {
                reg8: [RESET8; 256],
                reg32,
                log: Vec::new(),
                accesses: 0,
                fail_at: None,
            }
        }

        fn next(&mut self) -> Result<(), Fail> {
            let n = self.accesses;
            self.accesses += 1;
            if self.fail_at == Some(n) {
                Err(Fail(n))
            } else {
                Ok(())
            }
        }

        fn reg32(&self, addr: u16) -> u32 {
            self.reg32.get(&addr).copied().unwrap_or(RESET32)
        }
    }

    impl NmiBus for Fake {
        type Error = Fail;

        fn read8(&mut self, reg: u8) -> Result<u8, Fail> {
            self.log.push(R8(reg));
            self.next()?;
            Ok(self.reg8[usize::from(reg)])
        }

        fn write8(&mut self, reg: u8, value: u8) -> Result<(), Fail> {
            self.log.push(W8(reg, value));
            self.next()?;
            self.reg8[usize::from(reg)] = value;
            Ok(())
        }

        fn read32(&mut self, addr: u16) -> Result<u32, Fail> {
            self.log.push(R32(addr));
            self.next()?;
            Ok(self.reg32(addr))
        }

        fn write32(&mut self, addr: u16, value: u32) -> Result<(), Fail> {
            self.log.push(W32(addr, value));
            self.next()?;
            self.reg32.insert(addr, value);
            Ok(())
        }

        fn sleep_ms(&mut self, ms: u32) {
            self.log.push(Sleep(ms));
        }
    }

    /// Bus without a log: every 8-bit read returns `byte`, every 32-bit read all ones.
    struct Sink {
        byte: u8,
    }

    impl NmiBus for Sink {
        type Error = ();

        fn read8(&mut self, _reg: u8) -> Result<u8, ()> {
            Ok(self.byte)
        }

        fn write8(&mut self, _reg: u8, _value: u8) -> Result<(), ()> {
            Ok(())
        }

        fn read32(&mut self, _addr: u16) -> Result<u32, ()> {
            Ok(RESET32)
        }

        fn write32(&mut self, _addr: u16, _value: u32) -> Result<(), ()> {
            Ok(())
        }

        fn sleep_ms(&mut self, _ms: u32) {}
    }

    /// The log of `init` on an NM120 whose register 0x36 reads `ldo`: the init sequence in order.
    fn init_log(ldo: u8) -> Vec<Op> {
        let mut log = vec![R32(0x3FC)];
        for (reg, v) in [
            (0x06, 0x48),
            (0x07, 0x40),
            (0x0A, 0xEB),
            (0x0B, 0x11),
            (0x0C, 0x10),
            (0x0D, 0x88),
            (0x10, 0x04),
            (0x11, 0x30),
            (0x12, 0x30),
            (0x15, 0xAA),
            (0x16, 0x03),
            (0x17, 0x80),
            (0x18, 0x67),
            (0x19, 0xD4),
            (0x1A, 0x44),
            (0x1C, 0x10),
            (0x1D, 0xEE),
            (0x1E, 0x99),
            (0x21, 0xC5),
            (0x22, 0x91),
            (0x24, 0x01),
            (0x2B, 0x91),
            (0x2D, 0x01),
            (0x2F, 0x80),
            (0x31, 0x00),
            (0x33, 0x00),
            (0x38, 0x00),
            (0x39, 0x2F),
            (0x3A, 0x00),
            (0x3B, 0x00),
        ] {
            log.push(W8(reg, v));
        }
        log.push(R8(0x36));
        log.push(W8(0x36, ldo & 0x7F));
        log.push(W32(0x164, 0x800));
        log.push(W32(0x1C0, 0x2D8C_19C7));
        for (reg, v) in [
            (0x0E, 0x45),
            (0x1B, 0x0E),
            (0x23, 0xFF),
            (0x26, 0x82),
            (0x28, 0x00),
            (0x30, 0xDF),
            (0x32, 0xDF),
            (0x34, 0x68),
            (0x35, 0x18),
        ] {
            log.push(W8(reg, v));
        }
        log.push(W8(0x0A, 0xFB));
        log
    }

    /// An initialised tuner on a fresh fake with an empty log. The status register 0x328 is
    /// assumed to read 0 (no restart pulse needed).
    fn inited() -> (Nmi, Fake) {
        let mut bus = Fake::new(0x0001_2034);
        bus.reg32.insert(0x328, 0);
        let nmi = Nmi::init(&mut bus).unwrap();
        bus.log.clear();
        bus.accesses = 0;
        (nmi, bus)
    }

    /// The register writes (8-bit) to `reg` in `log`.
    fn writes8(log: &[Op], reg: u8) -> Vec<u8> {
        log.iter()
            .filter_map(|op| match *op {
                W8(r, v) if r == reg => Some(v),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn nm120_ids_are_accepted() {
        // Bits 31..20 and 7..0 do not take part in the test.
        for id in [0x1_2000, 0x1_20FF, 0x0001_2034, 0xFFF1_2000, 0xABC1_2055] {
            let mut bus = Fake::new(id);
            assert!(Nmi::init(&mut bus).is_ok(), "{id:#x}");
        }
    }

    #[test]
    fn other_ids_are_rejected_without_a_write() {
        // NM131, NM130 (both patterns), near misses of NM120, zero, all ones, a random value.
        for id in [
            0x1_3100,
            0x1_3155,
            0x01_3000,
            0x01_30FF,
            0x81_3000,
            0x81_3042,
            0x1_1F00,
            0x1_2100,
            0x1_3200,
            0,
            0xFFFF_FFFF,
            0xDEAD_BEEF,
        ] {
            let mut bus = Fake::new(id);
            assert_eq!(
                Nmi::init(&mut bus).err(),
                Some(NmiError::UnsupportedChip(id)),
                "{id:#x}"
            );
            assert_eq!(bus.log, vec![R32(0x3FC)], "{id:#x}");
        }
    }

    #[test]
    fn init_writes_the_init_sequence() {
        let mut bus = Fake::new(0x0001_2034);
        // The LDO bypass bit set at reset must be cleared from the value read.
        bus.reg8[0x36] = 0xFF;
        assert!(Nmi::init(&mut bus).is_ok());
        assert_eq!(bus.log, init_log(0xFF));
        assert_eq!(bus.log.len(), 1 + 30 + 2 + 2 + 9 + 1);
        assert_eq!(bus.reg8[0x0A], 0xFB);
    }

    #[test]
    fn init_stops_at_the_first_bus_error() {
        let expected = init_log(0);
        for n in 0..expected.len() {
            let mut bus = Fake::new(0x0001_2034);
            bus.fail_at = Some(n);
            assert_eq!(Nmi::init(&mut bus).err(), Some(NmiError::Bus(Fail(n))));
            // The failed access is the last one and nothing follows it.
            assert_eq!(bus.log, expected[..=n]);
        }
    }

    /// One row of the worked examples.
    struct Example {
        rf_hz: u32,
        /// Registers 0x08 and 0x09.
        filter: (u8, u8),
        /// Registers 0x01..0x04.
        pll: [u8; 4],
        /// Or-value of register 0x1D (the band in bits 7..5).
        reg1d_or: u8,
        /// Registers 0x25, 0x27, 0x29, 0x2E.
        band: [u8; 4],
        reg36: u8,
        reg37: u8,
        reg164: u32,
        reg230: u32,
        /// Or-value of 0x21C (and-mask 0xCF800000).
        reg21c: u32,
        reg210: u32,
        /// Or-value of 0x104 (and-mask 0x940007D1).
        reg104: u32,
    }

    const EXAMPLES: [Example; 4] = [
        Example {
            rf_hz: 93_143_000,
            filter: (0x52, 0x35),
            pll: [0x5D, 0xA4, 0x9B, 0x44],
            reg1d_or: 0xA0,
            band: [0x78, 0x7F, 0x7F, 0x12],
            reg36: 0x54,
            reg37: 0x84,
            reg164: 0x600,
            reg230: 0xBB8CE,
            reg21c: 0x0801_2532,
            reg210: 0x37E3,
            reg104: 0x10A2_8811,
        },
        Example {
            rf_hz: 473_143_000,
            filter: (0x00, 0x00),
            pll: [0x4E, 0x43, 0x6F, 0x1B],
            reg1d_or: 0x20,
            band: [0xF4, 0xEF, 0x4F, 0x34],
            reg36: 0x7C,
            reg37: 0x84,
            reg164: 0x500,
            reg230: 0xBBCA4,
            reg21c: 0x0801_2656,
            reg210: 0x37A3,
            reg104: 0x10A1_8811,
        },
        Example {
            rf_hz: 557_143_000,
            filter: (0x00, 0x00),
            pll: [0x5C, 0x43, 0x6F, 0x4B],
            reg1d_or: 0x20,
            band: [0xF4, 0xEF, 0x4F, 0x34],
            reg36: 0x7C,
            reg37: 0x84,
            reg164: 0x500,
            reg230: 0xBBBBC,
            reg21c: 0x0801_2614,
            reg210: 0x37B3,
            reg104: 0x10A1_8811,
        },
        Example {
            rf_hz: 767_143_000,
            filter: (0x00, 0x00),
            pll: [0x7F, 0x43, 0x6F, 0xCB],
            reg1d_or: 0x20,
            band: [0xF4, 0xEF, 0x4F, 0x34],
            reg36: 0x7C,
            reg37: 0x84,
            reg164: 0x500,
            reg230: 0xBCBC6,
            reg21c: 0x0801_2B04,
            reg210: 0x36C3,
            reg104: 0x10A0_8811,
        },
    ];

    /// The accesses to register 0x104 for the value `v` computed first, then the second write,
    /// the 1 ms wait, the status read and, if `pulse`, the restart pulse.
    fn ctrl_ops(v: u32, pulse: bool) -> Vec<Op> {
        let v2 = (v & !0x10) | 0x2000_0020;
        let mut l = vec![
            R32(0x104),
            W32(0x104, v),
            W32(0x104, v2),
            Sleep(1),
            R32(0x328),
        ];
        if pulse {
            l.extend([W32(0x104, v2 & !0x20), W32(0x104, v2)]);
        }
        l
    }

    /// The accesses of the first tune after `inited()` on a bus whose baseband registers read
    /// `RESET32`. Registers 0x1D and 0x21 hold what init wrote.
    fn first_tune_log(e: &Example, pulse: bool) -> Vec<Op> {
        let first_234 = (RESET32 & 0xF1FF_FFFF) | 0x0800_0000;
        let mut l = vec![
            R8(0x21),
            W8(0x05, 0x85),
            W8(0x08, e.filter.0),
            W8(0x09, e.filter.1),
            W8(0x01, e.pll[0]),
            W8(0x02, e.pll[1]),
            W8(0x03, e.pll[2]),
            W8(0x04, e.pll[3]),
            R8(0x1D),
            W8(0x1D, (0xEE & 0x1F) | e.reg1d_or),
            W8(0x25, e.band[0]),
            W8(0x27, e.band[1]),
            W8(0x29, e.band[2]),
            W8(0x2E, e.band[3]),
            W8(0x36, e.reg36),
            W8(0x37, e.reg37),
            W32(0x130, 0x200),
            W32(0x164, e.reg164),
            R32(0x234),
            W32(0x234, first_234),
            W32(0x20C, 0x400),
            R32(0x280),
            W32(0x280, RESET32),
            R32(0x238),
            W32(0x238, RESET32 & 0xFFFF_C000),
            R32(0x32C),
            W32(0x32C, RESET32 & 0xBFFF_BFFF),
            W32(0x230, e.reg230),
            W32(0x250, 0),
            W32(0x27C, 0x1010),
            W32(0x1BC, 0),
            W32(0x1C8, 0x8079),
            W32(0x1CC, 0x8079),
            W32(0x1D0, 0x8079),
            W32(0x110, 0x4C52),
            W32(0x26C, 2),
            W32(0x270, 6),
            R32(0x21C),
            W32(0x21C, (RESET32 & 0xCF80_0000) | e.reg21c),
            R32(0x234),
            W32(0x234, first_234 & 0xCFC0_0000),
            R32(0x12C),
            W32(0x12C, (RESET32 & 0xFFFF_9F7F) | 0x1000),
            W32(0x138, 0x5B18),
            W32(0x148, 0x1F_F082),
            R32(0x200),
            W32(0x200, (RESET32 & 0xFFFF_C000) | 0x8800),
            W32(0x210, e.reg210),
        ];
        l.extend(ctrl_ops((RESET32 & 0x9400_07D1) | e.reg104, pulse));
        l
    }

    #[test]
    fn first_tune_matches_the_worked_examples() {
        for e in &EXAMPLES {
            let (mut nmi, mut bus) = inited();
            nmi.tune(&mut bus, e.rf_hz).unwrap();
            assert_eq!(bus.log, first_tune_log(e, false), "{} Hz", e.rf_hz);
        }
    }

    #[test]
    fn restart_pulse_follows_a_nonzero_status() {
        let e = &EXAMPLES[1];
        let (mut nmi, mut bus) = inited();
        bus.reg32.insert(0x328, 1);
        nmi.tune(&mut bus, e.rf_hz).unwrap();
        assert_eq!(bus.log, first_tune_log(e, true));
        assert_eq!(bus.reg32(0x104) & 0x20, 0x20);
    }

    #[test]
    fn later_tunes_skip_the_first_tune_blocks() {
        let (first, second) = (&EXAMPLES[1], &EXAMPLES[2]);
        let (mut nmi, mut bus) = inited();
        nmi.tune(&mut bus, first.rf_hz).unwrap();
        bus.log.clear();
        nmi.tune(&mut bus, second.rf_hz).unwrap();

        // What the first tune left in the read-modify-write registers.
        let left_104 = (((RESET32 & 0x9400_07D1) | first.reg104) & !0x10) | 0x2000_0020;
        let left_21c = (RESET32 & 0xCF80_0000) | first.reg21c;
        let left_234 = ((RESET32 & 0xF1FF_FFFF) | 0x0800_0000) & 0xCFC0_0000;
        let left_1d = (0xEE & 0x1F) | first.reg1d_or;
        let mut expected = vec![
            R8(0x21),
            // Register 0x05 already holds 0x85.
            W8(0x08, second.filter.0),
            W8(0x09, second.filter.1),
            W8(0x01, second.pll[0]),
            W8(0x02, second.pll[1]),
            W8(0x03, second.pll[2]),
            W8(0x04, second.pll[3]),
            R8(0x1D),
            W8(0x1D, (left_1d & 0x1F) | second.reg1d_or),
            W8(0x25, second.band[0]),
            W8(0x27, second.band[1]),
            W8(0x29, second.band[2]),
            W8(0x2E, second.band[3]),
            W8(0x36, second.reg36),
            W8(0x37, second.reg37),
            W32(0x164, second.reg164),
            W32(0x230, second.reg230),
            W32(0x250, 0),
            W32(0x27C, 0x1010),
            W32(0x1BC, 0),
            R32(0x21C),
            W32(0x21C, (left_21c & 0xCF80_0000) | second.reg21c),
            R32(0x234),
            W32(0x234, left_234 & 0xCFC0_0000),
            W32(0x210, second.reg210),
        ];
        expected.extend(ctrl_ops((left_104 & 0x9400_07D1) | second.reg104, false));
        // The 0x104 read returns what the first tune left.
        assert_eq!(bus.log, expected);
    }

    #[test]
    fn register_05_follows_the_programmed_frequency() {
        // 120.1..120.4 MHz is judged on the frequency that is programmed, which is 1 kHz above
        // the request here (the fractional part is nonzero).
        let (mut nmi, mut bus) = inited();
        let mut seen = Vec::new();
        for rf in [
            473_143_000, // first tune: 0x85 is written
            500_000_000, // unchanged
            120_099_000, // programmed 120.100 MHz: 0x05
            120_200_000, // unchanged
            120_398_000, // programmed 120.399 MHz: unchanged
            120_399_000, // programmed 120.400 MHz: unchanged
            120_400_000, // programmed 120.401 MHz: 0x85
            120_400_000, // unchanged
        ] {
            bus.log.clear();
            nmi.tune(&mut bus, rf).unwrap();
            seen.push(writes8(&bus.log, 0x05));
        }
        assert_eq!(
            seen,
            [
                vec![0x85],
                vec![],
                vec![0x05],
                vec![],
                vec![],
                vec![],
                vec![0x85],
                vec![]
            ]
        );
    }

    #[test]
    fn a_zero_fractional_part_is_programmed_as_requested() {
        // 480 MHz: v = 3 840 000 kHz = 160 * 24 MHz exactly, so no retry;
        // 479 999 000 Hz is retried onto the same value.
        for rf in [480_000_000, 479_999_000] {
            let (mut nmi, mut bus) = inited();
            nmi.tune(&mut bus, rf).unwrap();
            let pll: Vec<u8> = [0x01, 0x02, 0x03, 0x04]
                .iter()
                .map(|&r| writes8(&bus.log, r)[0])
                .collect();
            // N = 160, frac = 0, D = 17.
            assert_eq!(pll, [0x50, 0x00, 0x00, 0x10], "{rf}");
        }
    }

    #[test]
    fn reference_selection_is_applied_once() {
        // Register 0x21 bits 1:0 = 2 doubles the reference: the same tune programs half the
        // integer part, also on the retry pass.
        let (mut nmi, mut bus) = inited();
        bus.reg8[0x21] = 0xC6;
        nmi.tune(&mut bus, 473_143_000).unwrap();
        assert_eq!(bus.log.iter().filter(|op| **op == R8(0x21)).count(), 1);
        // v = 3 785 152 kHz, xo = 48 MHz: N = 78, frac = (0x80000000 / 48000 * 41152) >> 12.
        let frac = (((0x8000_0000u32 / 48_000) * (3_785_152 % 48_000)) >> 12) & 0x7_FFFF;
        assert_eq!(writes8(&bus.log, 0x01), [78 >> 1]);
        assert_eq!(writes8(&bus.log, 0x02), [((frac << 1) & 0xFF) as u8]);
    }

    #[test]
    fn frequencies_outside_the_range_are_refused_before_the_bus() {
        for rf in [0, 92_999_999, 768_000_001, 1_000_000_000, u32::MAX] {
            let (mut nmi, mut bus) = inited();
            assert_eq!(nmi.tune(&mut bus, rf), Err(NmiError::Frequency), "{rf}");
            assert!(bus.log.is_empty(), "{rf}");
        }
        for rf in [93_000_000, 768_000_000] {
            let (mut nmi, mut bus) = inited();
            assert_eq!(nmi.tune(&mut bus, rf), Ok(()), "{rf}");
        }
    }

    #[test]
    fn tune_stops_at_the_first_bus_error() {
        let expected = first_tune_log(&EXAMPLES[1], false);
        let total = expected.iter().filter(|op| !matches!(op, Sleep(_))).count();
        for n in 0..total {
            let (mut nmi, mut bus) = inited();
            bus.fail_at = Some(n);
            assert_eq!(
                nmi.tune(&mut bus, EXAMPLES[1].rf_hz),
                Err(NmiError::Bus(Fail(n)))
            );
            // Everything up to the failed access, nothing after it.
            let mut want = expected.clone();
            let mut count = 0;
            let cut = want
                .iter()
                .position(|op| {
                    if !matches!(op, Sleep(_)) {
                        count += 1;
                    }
                    count == n + 1
                })
                .unwrap();
            want.truncate(cut + 1);
            assert_eq!(bus.log, want, "failure at access {n}");
        }
    }

    #[test]
    fn a_failed_tune_is_redone_in_full_by_the_next() {
        // Access 40 lies in the baseband part, after the first-tune blocks have started.
        let e = &EXAMPLES[1];
        let (mut nmi, mut bus) = inited();
        bus.fail_at = Some(40);
        assert_eq!(nmi.tune(&mut bus, e.rf_hz), Err(NmiError::Bus(Fail(40))));

        bus.fail_at = None;
        bus.log.clear();
        nmi.tune(&mut bus, e.rf_hz).unwrap();
        // Register 0x05 was written by the failed tune and is not repeated; the first-tune
        // blocks are.
        assert!(writes8(&bus.log, 0x05).is_empty());
        for op in [
            W32(0x130, 0x200),
            W32(0x20C, 0x400),
            W32(0x270, 6),
            W32(0x138, 0x5B18),
        ] {
            assert!(bus.log.contains(&op), "{op:?}");
        }
        assert_eq!(bus.log.last(), Some(&R32(0x328)));
    }

    #[test]
    fn a_failed_register_05_write_is_repeated() {
        let (mut nmi, mut bus) = inited();
        // Access 0 is the reference read, access 1 the register 0x05 write.
        bus.fail_at = Some(1);
        assert!(nmi.tune(&mut bus, 473_143_000).is_err());
        bus.fail_at = None;
        bus.log.clear();
        nmi.tune(&mut bus, 473_143_000).unwrap();
        assert_eq!(writes8(&bus.log, 0x05), [0x85]);
    }

    #[test]
    fn synthesiser_matches_the_worked_examples() {
        // (tuned Hz, band, V kHz, N, frac, D, clk) from the worked examples.
        for (fr, band, n, frac, d, clk) in [
            (93_144_000, 5, 186, 150_994, 20, 3_863_347),
            (473_144_000, 1, 157, 374_689, 17, 3_407_872),
            (557_144_000, 1, 185, 374_689, 20, 3_511_091),
            (767_144_000, 1, 255, 374_689, 28, 1_630_208),
        ] {
            let p = Pll::new(fr, XO_KHZ);
            assert_eq!(
                (p.band, p.n, p.frac, p.d, p.clk),
                (band, n, frac, d, clk),
                "{fr}"
            );
        }
    }

    #[test]
    fn bands_and_filters_follow_the_tables() {
        // Band edges are inclusive at the top.
        for (fr, band) in [
            (53_000_000, 7),
            (53_000_001, 6),
            (79_000_000, 6),
            (79_000_001, 5),
            (118_000_000, 5),
            (118_000_001, 4),
            (214_000_000, 4),
            (214_000_001, 3),
            (237_000_000, 3),
            (237_000_001, 2),
            (434_000_000, 2),
            (434_000_001, 1),
        ] {
            assert_eq!(Pll::new(fr, XO_KHZ).band, band, "{fr}");
        }
        for (fr, regs) in [
            (45_000_000, (0, 0)),
            (45_000_001, (0xA7, 0x3A)),
            (55_000_000, (0xA7, 0x3A)),
            (55_000_001, (0x97, 0x39)),
            (94_999_999, (0x52, 0x35)),
            (95_000_000, (0x52, 0x35)),
            (95_000_001, (0x41, 0x34)),
            (115_000_001, (0x40, 0x34)),
            (125_000_000, (0x40, 0x34)),
            (125_000_001, (0, 0)),
        ] {
            assert_eq!(Pll::new(fr, XO_KHZ).filter, regs, "{fr}");
        }
    }

    #[test]
    fn whole_range_stays_inside_the_proven_bounds() {
        // The proof in `Pll::new` claims: no wrap of the sample clock, D in 16..=31.
        let mut fr = RF_MIN_HZ;
        while fr <= RF_MAX_HZ + RETRY_STEP_HZ {
            for xo in [XO_KHZ / 2, XO_KHZ, XO_KHZ * 2] {
                let p = Pll::new(fr, xo);
                assert!(p.clk < 7_000_000, "{fr} {xo}");
                assert!((16..=31).contains(&p.d), "{fr}");
                assert!(p.frac < 1 << 19, "{fr}");
            }
            fr += 997;
        }
    }

    #[test]
    fn sleep_writes_the_reduced_sequence_and_wake_undoes_it() {
        let mut bus = Fake::new(0x0001_2034);
        let mut nmi = Nmi::init(&mut bus).unwrap();
        for (reg, value) in [(0x07, 0x40), (0x09, 0xFA), (0x0A, 0xFB), (0x0B, 0x11)] {
            bus.reg8[reg] = value;
        }
        for (reg, value) in [
            (0x34, 0x68),
            (0x35, 0x18),
            (0x36, 0x64),
            (0x00, 0x9F),
            (0x05, 0x87),
        ] {
            bus.reg8[reg] = value;
        }
        bus.reg32.insert(REG_BB_CTRL, 0x2247_202F);
        let awake = bus.reg8;
        bus.log.clear();
        nmi.sleep(&mut bus).unwrap();
        assert_eq!(
            bus.log,
            vec![
                R8(0x09),
                R8(0x0A),
                R8(0x34),
                R8(0x35),
                R8(0x36),
                R32(REG_BB_CTRL),
                W32(REG_BB_CTRL, 0x2247_201E),
                W8(0x09, 0xEA),
                R8(0x0B),
                W8(0x0B, 0x01),
                W8(0x34, 0x00),
                W8(0x35, 0x00),
                W8(0x0A, 0xF7),
                W8(0x36, 0x78),
            ]
        );
        // The RF front end (0x00, 0x05, 0x07 bit 6) is never touched.
        assert_eq!(
            (bus.reg8[0x00], bus.reg8[0x05], bus.reg8[0x07]),
            (0x9F, 0x87, 0x40)
        );
        // Sleeping again keeps the saved values of the awake tuner.
        bus.log.clear();
        nmi.sleep(&mut bus).unwrap();
        assert!(bus.log.is_empty());
        nmi.wake(&mut bus).unwrap();
        assert_eq!(bus.reg8, awake);
        assert_eq!(bus.reg32(REG_BB_CTRL), 0x2247_202F);
        // Waking again does nothing.
        bus.log.clear();
        nmi.wake(&mut bus).unwrap();
        assert!(bus.log.is_empty());
    }

    #[test]
    fn wake_retries_after_a_failure_and_a_failed_sleep_is_undone() {
        let mut bus = Fake::new(0x0001_2034);
        let mut nmi = Nmi::init(&mut bus).unwrap();
        bus.reg8[0x09] = 0xFA;
        bus.reg8[0x34] = 0x68;
        let awake = bus.reg8;
        // A sleep that fails after its reads still has the values to restore.
        bus.fail_at = Some(bus.accesses + 8);
        assert!(nmi.sleep(&mut bus).is_err());
        bus.fail_at = Some(bus.accesses + 2);
        assert!(nmi.wake(&mut bus).is_err());
        bus.fail_at = None;
        nmi.wake(&mut bus).unwrap();
        assert_eq!(bus.reg8, awake);
    }

    #[test]
    fn whole_range_tunes_without_panic() {
        for byte in [0xC5, 0xC6, 0xC7] {
            let mut bus = Sink { byte };
            let mut nmi = Nmi {
                reg05: None,
                baseband_ready: false,
                asleep: None,
            };
            let mut rf = RF_MIN_HZ;
            while rf <= RF_MAX_HZ {
                assert_eq!(nmi.tune(&mut bus, rf), Ok(()), "{rf}");
                rf += 997;
            }
            assert_eq!(nmi.tune(&mut bus, RF_MAX_HZ), Ok(()));
        }
    }
}
