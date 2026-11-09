// SPDX-License-Identifier: GPL-2.0

//! The TC90522 demodulator: register sequences, status decoding and C/N conversion.
//!
//! Data and pure functions only. Each demodulator core is one I2C slave; ISDB-T cores sit at even
//! and ISDB-S cores at odd positions of a chip. The caller performs the transactions, so the same
//! sequences serve every core, and it checks every write.
//!
//! The sequences are the revealed ones. Where one is known to be wrong, the mainline decoding
//! (`drivers/media/dvb-frontends/tc90522.c`, cited as `tc90522.c:function`) or an exact computation
//! takes its place, and the place says so.

/// Register write `(register, value)`.
pub(crate) type Write = (u8, u8);

// ---------------------------------------------------------------------------------------------
// Initialisation and tuning
// ---------------------------------------------------------------------------------------------

/// Initialisation of an ISDB-T core, including its `full` block (the last three writes), which
/// every init path writes.
///
/// Registers 0xB0..0xB8 are configuration here, although mainline reads 0xB0 and 0xB2..0xB7 as
/// status: what a write sets and what a read returns differ.
pub(crate) const INIT_T: [Write; 14] = [
    (0xB0, 0xA0),
    (0xB2, 0x3D),
    (0xB3, 0x25),
    (0xB4, 0x8B),
    (0xB5, 0x4B),
    (0xB6, 0x3F),
    (0xB7, 0xFF),
    (0xB8, 0xC0),
    (0x03, 0x00),
    (0x1D, 0x00),
    (0x1F, 0x00),
    // `full` block.
    (0x0E, 0x77),
    (0x0F, 0x13),
    (0x75, 0x02),
];

/// Initialisation of an ISDB-S core with the `full` block (the last three writes).
///
/// 0x17 = 0x00 is the wake value of mainline (`tc90522.c:wakeup_sat`), [`WAKE_S`].
pub(crate) const INIT_S: [Write; 9] = [
    (0x13, 0x00),
    (0x15, 0x00),
    (0x17, 0x00),
    (0x1C, 0x00),
    (0x1D, 0x00),
    (0x1F, 0x00),
    // `full` block.
    (0x07, 0x31),
    (0x08, 0x77),
    (0x04, 0x02),
];

/// Stops an ISDB-T core: 0x03 is 0x00 after init and while working, and bit 4 stops the core and
/// its TS output (measured on the card; mainline's pair 0x90 / 0x80 differs from it in bit 4 only).
/// [`WAKE_T`] restarts it, and it resumes the channel it was tuned to.
pub(crate) const SLEEP_T: Write = (0x03, 0x10);
pub(crate) const WAKE_T: Write = (0x03, 0x00);

/// Stops an ISDB-S core and its TS output, as `tc90522.c:sleep_sat` (measured on the card);
/// [`WAKE_S`] restarts it.
pub(crate) const SLEEP_S: Write = (0x17, 0x01);
pub(crate) const WAKE_S: Write = (0x17, 0x00);

/// ISDB-T writes before the tuner is programmed. 0x25 = 0x00 and 0x23 = 0x4D switch the IF AGC off,
/// as in `tc90522.c:tc90522_set_if_agc`.
pub(crate) const PRE_TUNE_T: [Write; 5] = [
    (0x01, 0x50),
    (0x47, 0x30),
    (0x25, 0x00),
    (0x20, 0x00),
    (0x23, 0x4D),
];

/// ISDB-T writes after the tuner settled. 0x23 = 0x4C switches the IF AGC back on; 0x25 stays 0x00
/// (0x40 brought nothing on the card). 0x71 = 0x01 enables layers A and B and excludes layer C
/// (`tc90522.c:tc90522t_set_layers`).
pub(crate) const POST_TUNE_T: [Write; 4] = [(0x23, 0x4C), (0x01, 0x50), (0x71, 0x01), (0x72, 0x24)];

/// ISDB-S writes before the tuner is programmed: IF AGC off (0x0A, 0x10, 0x11) and a reacquire
/// (0x03).
pub(crate) const PRE_TUNE_S: [Write; 4] = [(0x0A, 0x00), (0x10, 0xB0), (0x11, 0x02), (0x03, 0x01)];

/// ISDB-S writes after the tuner settled: IF AGC on. The closing 0x03 = 0x01 of that sequence is
/// [`REACQUIRE_S`] here because it has to follow the TSID, so that the demodulator acquires the
/// stream that was asked for.
pub(crate) const POST_TUNE_S: [Write; 3] = [(0x0A, 0xFF), (0x10, 0xB2), (0x11, 0x00)];

/// Restarts the ISDB-S acquisition; written after the TSID, as in `tc90522.c:tc90522_set_frontend`.
pub(crate) const REACQUIRE_S: Write = (0x03, 0x01);

/// ISDB-S register holding the high byte of the TSID.
pub(crate) const REG_TSID_HIGH: u8 = 0x8F;
/// ISDB-S register holding the low byte of the TSID.
pub(crate) const REG_TSID_LOW: u8 = 0x90;

/// The writes selecting `tsid` (`tc90522.c:tc90522s_set_tsid`). 0xFFFF means "any TS", a decision
/// of this driver.
pub(crate) const fn tsid_writes(tsid: u16) -> [Write; 2] {
    let [high, low] = tsid.to_be_bytes();
    [(REG_TSID_HIGH, high), (REG_TSID_LOW, low)]
}

/// ISDB-S register of the first byte of the TMCC TSID list: [`TSID_LIST_LEN`] bytes, the eight
/// big-endian TSIDs of the streams the transponder carries, read in one burst.
pub(crate) const REG_S_TSID_LIST: u8 = 0xCE;
/// Length of the TMCC TSID list in bytes.
pub(crate) const TSID_LIST_LEN: usize = 16;

/// Whether the TMCC TSID list `list` (read from [`REG_S_TSID_LIST`]) names `tsid`.
///
/// The demodulator locks on a transponder whatever TSID is selected, and for a TSID the transponder
/// does not carry it delivers the transponder's first TS (measured): only the list tells whether
/// the selected stream exists.
// `as_chunks` needs Rust 1.88, and the kernel supports older compilers.
#[allow(clippy::chunks_exact_to_as_chunks)]
pub(crate) fn tsid_listed(list: &[u8; TSID_LIST_LEN], tsid: u16) -> bool {
    list.chunks_exact(2)
        .any(|entry| u16::from_be_bytes([entry[0], entry[1]]) == tsid)
}

// ---------------------------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------------------------

/// `FE_HAS_SIGNAL` of `enum fe_status` (`include/uapi/linux/dvb/frontend.h`).
pub(crate) const FE_HAS_SIGNAL: u32 = 0x01;
/// `FE_HAS_CARRIER`.
pub(crate) const FE_HAS_CARRIER: u32 = 0x02;
/// `FE_HAS_VITERBI`.
pub(crate) const FE_HAS_VITERBI: u32 = 0x04;
/// `FE_HAS_SYNC`.
pub(crate) const FE_HAS_SYNC: u32 = 0x08;
/// `FE_HAS_LOCK`.
pub(crate) const FE_HAS_LOCK: u32 = 0x10;

/// ISDB-S status register, input of [`status_s`].
pub(crate) const REG_S_STATUS: u8 = 0xC3;
/// ISDB-S TMCC status register, input of [`status_s`].
pub(crate) const REG_S_TMCC: u8 = 0xC5;
/// ISDB-T lock register, input of [`status_t`].
pub(crate) const REG_T_LOCK: u8 = 0x96;
/// ISDB-T status register, input of [`status_t`].
pub(crate) const REG_T_STATUS: u8 = 0x80;
/// ISDB-T mode register, input of [`status_t`] (its low nibble is part of the lock condition).
pub(crate) const REG_T_MODE: u8 = 0xB0;

/// fe_status bits of an ISDB-S core from registers 0xC3 and 0xC5
/// (`tc90522.c:tc90522s_read_status`).
///
/// 0xC3 bit 7 set: input level under the minimum, nothing is reported. Otherwise `FE_HAS_SIGNAL`;
/// bits 6:5 set: no carrier, nothing more. Otherwise carrier, Viterbi and sync; bit 4 set: no lock.
/// Only with that bit clear, 0xC5 bits 1:0 (TMCC valid) decide `FE_HAS_LOCK`. Deriving the lock
/// from a BER bin would be a mistake: it reports failed register reads as locked.
///
/// Mainline reads 0xC5 only when 0xC3 shows no reason for "no lock", so a caller may pass any
/// `c5` otherwise.
pub(crate) const fn status_s(c3: u8, c5: u8) -> u32 {
    if c3 & 0x80 != 0 {
        return 0;
    }
    let mut status = FE_HAS_SIGNAL;
    if c3 & 0x60 != 0 {
        return status;
    }
    status |= FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC;
    if c3 & 0x10 == 0 && c5 & 0x03 != 0 {
        status |= FE_HAS_LOCK;
    }
    status
}

/// fe_status bits of an ISDB-T core from registers 0x96, 0x80 and 0xB0
/// (`tc90522.c:tc90522t_read_status`).
///
/// 0x96 bits 7:5 set: full lock, all five bits are reported. Otherwise 0x80 decides stepwise:
/// bits 7:4 set: no signal, nothing is reported; else `FE_HAS_SIGNAL | FE_HAS_CARRIER`; bits 3:2
/// set: no sync, nothing more; else sync and Viterbi; bit 1 set: no lock.
///
/// `FE_HAS_LOCK` additionally requires `(rb0 & 0x0F) >= 8`, a decision of this driver. It applies
/// on the full-lock path too: the test is on the same chip state, so a lock it denies must not be
/// reported. The other bits are mainline's alone.
///
/// Mainline reads 0x80 only without a full lock; a caller may pass any value for a register the
/// decoding does not reach.
pub(crate) const fn status_t(r96: u8, r80: u8, rb0: u8) -> u32 {
    let lock = if rb0 & 0x0F >= 8 { FE_HAS_LOCK } else { 0 };
    if r96 & 0xE0 != 0 {
        return FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | lock;
    }
    if r80 & 0xF0 != 0 {
        return 0;
    }
    let mut status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
    if r80 & 0x0C != 0 {
        return status;
    }
    status |= FE_HAS_SYNC | FE_HAS_VITERBI;
    if r80 & 0x02 == 0 {
        status |= lock;
    }
    status
}

// ---------------------------------------------------------------------------------------------
// C/N
// ---------------------------------------------------------------------------------------------

/// ISDB-T register of the first C/N byte. [`CN_T_LEN`] bytes (0x8B..0x8D, big endian) are read in
/// one burst: separate reads tear the value.
pub(crate) const REG_CN_T: u8 = 0x8B;
/// Number of bytes of the ISDB-T C/N value.
pub(crate) const CN_T_LEN: usize = 3;
/// ISDB-S register of the first C/N byte. [`CN_S_LEN`] bytes (0xBC, 0xBD, big endian) are read
/// in one burst.
pub(crate) const REG_CN_S: u8 = 0xBC;
/// Number of bytes of the ISDB-S C/N value.
pub(crate) const CN_S_LEN: usize = 2;

/// ISDB-T statistics latch: 0x00 is written here before every C/N and BER read and never otherwise
/// (as revealed); the meaning is unknown, so the write is kept.
pub(crate) const LATCH_T_STATS: Write = (0xBA, 0x00);

/// Integer dB of the first table entry.
const CN_FIRST_DB: i32 = 2;

/// Raw ISDB-T C/N values at 2, 3, ..., 30 dB. A larger raw value is a lower C/N.
const CN_TABLE_T: [u32; 29] = [
    9_778_432, 5_735_416, 3_903_295, 2_811_328, 2_089_200, 1_583_680, 1_216_799, 943_867, 737_184,
    578_602, 455_722, 359_802, 284_533, 225_191, 178_317, 141_219, 111_834, 88_556, 70_125, 55_546,
    44_026, 34_937, 27_773, 22_134, 17_696, 14_204, 11_455, 9_287, 7_573,
];

/// Raw ISDB-S C/N values at 2, 3, ..., 30 dB. A larger raw value is a lower C/N.
const CN_TABLE_S: [u32; 29] = [
    35_825, 34_437, 32_889, 31_162, 29_249, 27_174, 25_008, 22_846, 20_774, 18_842, 17_062, 15_430,
    13_934, 12_566, 11_321, 10_198, 9_199, 8_323, 7_568, 6_924, 6_379, 5_920, 5_532, 5_204, 4_925,
    4_685, 4_479, 4_300, 4_145,
];

/// ISDB-S raw values below this carry no measurement: the register holds the measurement plus an
/// offset of 3000, as `tc90522.c:tc90522s_get_frontend` subtracts it (`cndat >= 3000`) and the
/// tables confirm.
const CN_S_OFFSET: u16 = 3000;

/// C/N in 0.001 dB for the raw value `raw` of a decreasing table.
///
/// Between two entries the value is interpolated linearly in hundredths of a dB with exact integer
/// arithmetic, `100 * i + (Tab[i] - raw) * 100 / (Tab[i] - Tab[i + 1])`, rounded down and scaled to
/// milli-dB. Dividing by the truncated `(Tab[i] - Tab[i + 1]) / 100` would be a mistake: the
/// fraction can exceed 99 and the result is non-monotonic.
///
/// Outside the table the result is clamped to its range, 2.000 to 30.000 dB: the table says
/// nothing about weaker or stronger signals, and the value stays monotonic in `raw`.
fn cn_mdb(table: &[u32; 29], raw: u32) -> i32 {
    if raw >= table[0] {
        return CN_FIRST_DB * 1000;
    }
    let found = table.windows(2).enumerate().find_map(|(i, w)| match *w {
        [hi, lo] if raw > lo => Some((i, hi, lo)),
        _ => None,
    });
    let Some((i, hi, lo)) = found else {
        // Not above the 30 dB entry.
        return (CN_FIRST_DB + 28) * 1000;
    };
    // `lo < raw <= hi`: every earlier window had `raw <= its lo`, which is this window's `hi`,
    // and the first one has `raw < table[0]`. So the step is positive and the fraction is in
    // 0..100; `i` is below 28.
    let hundredths = u64::from(hi - raw) * 100 / u64::from(hi - lo);
    (CN_FIRST_DB + i as i32) * 1000 + hundredths as i32 * 10
}

/// C/N of an ISDB-S core in 0.001 dB from registers 0xBC:0xBD as read (big endian). `None` for
/// raw values below 3000, which are no measurement and not 0 dB. A
/// measurement weaker than 2 dB reads 2000, one stronger than 30 dB reads 30000.
///
/// The core must be locked, which the caller checks: reading the registers regardless and returning
/// garbage on failures would be a mistake.
pub(crate) fn cn_s_mdb(raw: u16) -> Option<i32> {
    if raw < CN_S_OFFSET {
        return None;
    }
    Some(cn_mdb(&CN_TABLE_S, u32::from(raw)))
}

/// C/N of an ISDB-T core in 0.001 dB from registers 0x8B:0x8C:0x8D as read (24 bits, big
/// endian). `None` for 0, which is no measurement and not the strongest
/// signal. A measurement weaker than 2 dB reads 2000, one stronger than 30 dB reads 30000.
///
/// The core must be locked, and [`LATCH_T_STATS`] written before the registers are read.
pub(crate) fn cn_t_mdb(raw: u32) -> Option<i32> {
    if raw == 0 {
        return None;
    }
    Some(cn_mdb(&CN_TABLE_T, raw))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_t_writes_the_sequence_with_the_full_block() {
        assert_eq!(
            INIT_T,
            [
                (0xB0, 0xA0),
                (0xB2, 0x3D),
                (0xB3, 0x25),
                (0xB4, 0x8B),
                (0xB5, 0x4B),
                (0xB6, 0x3F),
                (0xB7, 0xFF),
                (0xB8, 0xC0),
                (0x03, 0x00),
                (0x1D, 0x00),
                (0x1F, 0x00),
                (0x0E, 0x77),
                (0x0F, 0x13),
                (0x75, 0x02),
            ]
        );
    }

    #[test]
    fn init_s_writes_the_sequence_with_the_full_block() {
        assert_eq!(
            INIT_S,
            [
                (0x13, 0x00),
                (0x15, 0x00),
                (0x17, 0x00),
                (0x1C, 0x00),
                (0x1D, 0x00),
                (0x1F, 0x00),
                (0x07, 0x31),
                (0x08, 0x77),
                (0x04, 0x02),
            ]
        );
    }

    #[test]
    fn tune_sequences_are_the_revealed_ones() {
        assert_eq!(
            PRE_TUNE_T,
            [
                (0x01, 0x50),
                (0x47, 0x30),
                (0x25, 0x00),
                (0x20, 0x00),
                (0x23, 0x4D)
            ]
        );
        assert_eq!(
            POST_TUNE_T,
            [(0x23, 0x4C), (0x01, 0x50), (0x71, 0x01), (0x72, 0x24)]
        );
        assert_eq!(
            PRE_TUNE_S,
            [(0x0A, 0x00), (0x10, 0xB0), (0x11, 0x02), (0x03, 0x01)]
        );
        assert_eq!(POST_TUNE_S, [(0x0A, 0xFF), (0x10, 0xB2), (0x11, 0x00)]);
        assert_eq!(REACQUIRE_S, (0x03, 0x01));
    }

    #[test]
    fn tsid_is_written_high_byte_first() {
        assert_eq!(tsid_writes(0x1234), [(0x8F, 0x12), (0x90, 0x34)]);
        assert_eq!(tsid_writes(0x0001), [(0x8F, 0x00), (0x90, 0x01)]);
        assert_eq!(tsid_writes(0x4010), [(0x8F, 0x40), (0x90, 0x10)]);
        assert_eq!(tsid_writes(0xFFFF), [(0x8F, 0xFF), (0x90, 0xFF)]);
        assert_eq!(tsid_writes(0), [(0x8F, 0x00), (0x90, 0x00)]);
    }

    #[test]
    fn tsid_list_entries_are_big_endian_pairs() {
        // BS-15 as broadcast: 0x40F1, 0x40F2, 0x48F3, the rest unused.
        let mut list = [0; TSID_LIST_LEN];
        list[..6].copy_from_slice(&[0x40, 0xF1, 0x40, 0xF2, 0x48, 0xF3]);
        assert!(tsid_listed(&list, 0x40F1));
        assert!(tsid_listed(&list, 0x40F2));
        assert!(tsid_listed(&list, 0x48F3));
        assert!(!tsid_listed(&list, 0x40F0));
        // Only whole entries count: 0xF140 straddles two of them.
        assert!(!tsid_listed(&list, 0xF140));
        // The last entry counts.
        list[14..].copy_from_slice(&[0x47, 0x33]);
        assert!(tsid_listed(&list, 0x4733));
    }

    const SIGNAL: u32 = FE_HAS_SIGNAL;
    const CARRIER: u32 = SIGNAL | FE_HAS_CARRIER;
    const SYNCED: u32 = CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC;
    const LOCKED: u32 = SYNCED | FE_HAS_LOCK;

    #[test]
    fn fe_status_bits_are_the_uapi_values() {
        assert_eq!(FE_HAS_SIGNAL, 0x01);
        assert_eq!(FE_HAS_CARRIER, 0x02);
        assert_eq!(FE_HAS_VITERBI, 0x04);
        assert_eq!(FE_HAS_SYNC, 0x08);
        assert_eq!(FE_HAS_LOCK, 0x10);
    }

    #[test]
    fn status_s_follows_mainline() {
        // 0xC3 bit 7: under the minimum level, whatever the other bits say.
        assert_eq!(status_s(0x80, 0x03), 0);
        assert_eq!(status_s(0xFF, 0x03), 0);
        // Bits 6:5: no carrier.
        assert_eq!(status_s(0x20, 0x03), SIGNAL);
        assert_eq!(status_s(0x40, 0x03), SIGNAL);
        assert_eq!(status_s(0x70, 0x03), SIGNAL);
        // Bit 4: no lock.
        assert_eq!(status_s(0x10, 0x03), SYNCED);
        // Locked needs TMCC valid in 0xC5 bits 1:0.
        assert_eq!(status_s(0x00, 0x00), SYNCED);
        assert_eq!(status_s(0x00, 0xFC), SYNCED);
        assert_eq!(status_s(0x00, 0x01), LOCKED);
        assert_eq!(status_s(0x00, 0x02), LOCKED);
        assert_eq!(status_s(0x00, 0x03), LOCKED);
        // The low nibble of 0xC3 is not part of the decoding.
        assert_eq!(status_s(0x0F, 0x03), LOCKED);
    }

    #[test]
    fn status_s_is_hierarchical_over_all_inputs() {
        for c3 in 0..=u8::MAX {
            for c5 in 0..=u8::MAX {
                let status = status_s(c3, c5);
                if status & FE_HAS_LOCK != 0 {
                    assert_eq!(status, LOCKED);
                    assert_eq!(c3 & 0xF0, 0);
                    assert_ne!(c5 & 0x03, 0);
                } else {
                    assert!([0, SIGNAL, SYNCED].contains(&status));
                }
            }
        }
    }

    const MODE_OK: u8 = 0x08;

    #[test]
    fn status_t_full_lock_path() {
        // 0x96 bits 7:5 decide before 0x80 is looked at.
        for r96 in [0x20, 0x40, 0x80, 0xE0, 0xFF] {
            assert_eq!(status_t(r96, 0xFF, MODE_OK), LOCKED);
            assert_eq!(status_t(r96, 0x00, MODE_OK), LOCKED);
        }
        // Bits 4:0 of 0x96 are not part of the decoding.
        assert_eq!(status_t(0x1F, 0x00, MODE_OK), LOCKED);
        assert_eq!(status_t(0x1F, 0xF0, MODE_OK), 0);
    }

    #[test]
    fn status_t_steps_through_0x80() {
        // Bits 7:4: no signal.
        for r80 in [0x10, 0x20, 0x40, 0x80, 0xF0, 0xFF] {
            assert_eq!(status_t(0, r80, MODE_OK), 0);
        }
        // Bits 3:2: no sync.
        for r80 in [0x04, 0x08, 0x0C, 0x0E, 0x0F] {
            assert_eq!(status_t(0, r80, MODE_OK), CARRIER);
        }
        // Bit 1: no lock.
        assert_eq!(status_t(0, 0x02, MODE_OK), SYNCED);
        assert_eq!(status_t(0, 0x03, MODE_OK), SYNCED);
        // Bit 0 is not part of the decoding.
        assert_eq!(status_t(0, 0x00, MODE_OK), LOCKED);
        assert_eq!(status_t(0, 0x01, MODE_OK), LOCKED);
    }

    #[test]
    fn status_t_lock_needs_the_mode_nibble_to_be_at_least_8() {
        for rb0 in 0..=u8::MAX {
            let lock = rb0 & 0x0F >= 8;
            let expect = |full: u32| if lock { full } else { full & !FE_HAS_LOCK };
            assert_eq!(status_t(0x00, 0x00, rb0), expect(LOCKED), "{rb0:#04x}");
            assert_eq!(status_t(0xE0, 0x00, rb0), expect(LOCKED), "{rb0:#04x}");
            // The high nibble of 0xB0 (mode and guard interval) is not part of the condition.
            assert_eq!(status_t(0x00, 0x00, rb0), status_t(0x00, 0x00, rb0 & 0x0F));
            // The other bits do not depend on 0xB0.
            assert_eq!(status_t(0x00, 0x02, rb0), SYNCED);
            assert_eq!(status_t(0x00, 0x04, rb0), CARRIER);
            assert_eq!(status_t(0x00, 0x10, rb0), 0);
        }
        assert_eq!(status_t(0, 0, 0x07), SYNCED);
        assert_eq!(status_t(0, 0, 0x08), LOCKED);
        assert_eq!(status_t(0, 0, 0x09), LOCKED);
        assert_eq!(status_t(0, 0, 0x0F), LOCKED);
        assert_eq!(status_t(0, 0, 0x10), SYNCED);
        assert_eq!(status_t(0xE0, 0, 0x07), SYNCED);
    }

    #[test]
    fn status_t_lock_implies_every_other_bit() {
        for r96 in 0..=u8::MAX {
            for r80 in 0..=u8::MAX {
                for rb0 in [0x00, 0x07, 0x08, 0x0F] {
                    let status = status_t(r96, r80, rb0);
                    if status & FE_HAS_LOCK != 0 {
                        assert_eq!(status, LOCKED);
                        assert!(rb0 & 0x0F >= 8);
                    } else {
                        assert!([0, CARRIER, SYNCED].contains(&status));
                    }
                }
            }
        }
    }

    /// Raw value to the exact mdB value for every table entry, in dB.
    fn entries(table: &[u32; 29]) -> impl Iterator<Item = (i32, u32)> + '_ {
        (2..).zip(table.iter().copied())
    }

    #[test]
    fn tables_are_strictly_decreasing_and_span_2_to_30_db() {
        for table in [&CN_TABLE_S, &CN_TABLE_T] {
            assert_eq!(table.len(), 29);
            assert!(table.windows(2).all(|w| w[0] > w[1]));
        }
        assert_eq!(CN_TABLE_T[0], 9_778_432);
        assert_eq!(CN_TABLE_T[28], 7_573);
        assert_eq!(CN_TABLE_S[0], 35_825);
        assert_eq!(CN_TABLE_S[28], 4_145);
    }

    #[test]
    fn table_entries_are_whole_db() {
        for (db, raw) in entries(&CN_TABLE_S) {
            assert_eq!(cn_s_mdb(raw as u16), Some(db * 1000), "S {db} dB");
        }
        for (db, raw) in entries(&CN_TABLE_T) {
            assert_eq!(cn_t_mdb(raw), Some(db * 1000), "T {db} dB");
        }
    }

    #[test]
    fn values_between_entries_interpolate_linearly() {
        // S 10 dB = 20774, 11 dB = 18842: the step is 1932.
        assert_eq!(cn_s_mdb(20_774), Some(10_000));
        assert_eq!(cn_s_mdb(19_808), Some(10_500));
        assert_eq!(cn_s_mdb(18_842), Some(11_000));
        // One count above the 11 dB entry is 99/100 of the way: 1931 * 100 / 1932 = 99.9 -> 99.
        assert_eq!(cn_s_mdb(18_843), Some(10_990));
        // One count below the 10 dB entry: 1 * 100 / 1932 = 0.
        assert_eq!(cn_s_mdb(20_773), Some(10_000));
        // 20774 - 20 counts: 20 * 100 / 1932 = 1 hundredth.
        assert_eq!(cn_s_mdb(20_754), Some(10_010));
        // The last S step, 29 -> 30 dB (4300 -> 4145, step 155), where the truncated divisor
        // (155 / 100 = 1) would let the fraction reach 154.
        assert_eq!(cn_s_mdb(4_300), Some(29_000));
        assert_eq!(cn_s_mdb(4_299), Some(29_000));
        assert_eq!(cn_s_mdb(4_250), Some(29_320));
        assert_eq!(cn_s_mdb(4_146), Some(29_990));
        assert_eq!(cn_s_mdb(4_145), Some(30_000));
        // T 2 dB = 9778432, 3 dB = 5735416: the step is 4043016, halfway is 7756924.
        assert_eq!(cn_t_mdb(7_756_924), Some(2_500));
        assert_eq!(cn_t_mdb(5_735_417), Some(2_990));
        // T 29 dB = 9287, 30 dB = 7573: the step is 1714.
        assert_eq!(cn_t_mdb(8_430), Some(29_500));
        assert_eq!(cn_t_mdb(7_574), Some(29_990));
    }

    /// Every value of every segment of `table` stays within the whole dB below it and below the
    /// next entry, in steps of 10 mdB; the sweep covers all values of short segments.
    fn check_segments(table: &[u32; 29], convert: impl Fn(u32) -> i32) {
        for (i, w) in table.windows(2).enumerate() {
            let floor = (2 + i as i32) * 1000;
            let step = ((w[0] - w[1]) / 2000).max(1) as usize;
            for raw in (w[1] + 1..=w[0]).step_by(step) {
                let mdb = convert(raw);
                assert!((floor..floor + 1000).contains(&mdb), "{raw} -> {mdb}");
                assert_eq!(mdb % 10, 0, "{raw}");
            }
        }
    }

    #[test]
    fn interpolation_never_leaves_its_segment() {
        check_segments(&CN_TABLE_S, |raw| cn_s_mdb(raw as u16).unwrap());
        check_segments(&CN_TABLE_T, |raw| cn_t_mdb(raw).unwrap());
    }

    #[test]
    fn values_beyond_the_table_clamp_to_its_range() {
        assert_eq!(cn_s_mdb(35_826), Some(2_000));
        assert_eq!(cn_s_mdb(u16::MAX), Some(2_000));
        assert_eq!(cn_s_mdb(4_144), Some(30_000));
        assert_eq!(cn_s_mdb(3_000), Some(30_000));
        assert_eq!(cn_t_mdb(9_778_433), Some(2_000));
        assert_eq!(cn_t_mdb(0x00FF_FFFF), Some(2_000));
        assert_eq!(cn_t_mdb(u32::MAX), Some(2_000));
        assert_eq!(cn_t_mdb(7_572), Some(30_000));
        assert_eq!(cn_t_mdb(1), Some(30_000));
    }

    #[test]
    fn values_without_a_measurement_are_none() {
        assert_eq!(cn_s_mdb(0), None);
        assert_eq!(cn_s_mdb(2_999), None);
        assert!(cn_s_mdb(3_000).is_some());
        assert_eq!(cn_t_mdb(0), None);
        assert!(cn_t_mdb(1).is_some());
    }

    #[test]
    fn cn_s_is_monotonic_over_the_whole_input_range() {
        let mut previous = i32::MAX;
        for raw in 3_000..=u16::MAX {
            let mdb = cn_s_mdb(raw).unwrap();
            assert!(mdb <= previous, "{raw}");
            assert!((2_000..=30_000).contains(&mdb), "{raw}");
            previous = mdb;
        }
    }

    #[test]
    fn cn_t_is_monotonic_over_the_whole_input_range() {
        // Every value up to the 3 dB entry, which covers all short segments, then every 4093rd
        // value up to the largest register value, and the bounds of the u32 range.
        let mut previous = i32::MAX;
        let mut check = |raw: u32| {
            let mdb = cn_t_mdb(raw).unwrap();
            assert!(mdb <= previous, "{raw}");
            assert!((2_000..=30_000).contains(&mdb), "{raw}");
            previous = mdb;
        };
        (1..=CN_TABLE_T[1]).for_each(&mut check);
        (CN_TABLE_T[1]..0x0100_0000)
            .step_by(4093)
            .for_each(&mut check);
        check(0x00FF_FFFF);
        check(0x0100_0000);
        check(u32::MAX - 1);
        check(u32::MAX);
    }
}
