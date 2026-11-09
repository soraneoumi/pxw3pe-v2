// SPDX-License-Identifier: GPL-2.0

//! Answer-to-Reset parsing (ISO/IEC 7816-3 section 8) and the T=1 timing it implies.
//!
//! The parser is incremental: the UART layer feeds the bytes received so far and learns how many
//! more the ATR announces, so it never has to guess the ATR length.

/// Longest possible ATR: TS plus 32 further characters (ISO/IEC 7816-3 8.2.1).
pub(crate) const MAX_LEN: usize = 33;

/// Reasons an ATR is rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AtrError {
    /// TS is neither the direct (0x3B) nor the inverse (0x3F) convention.
    BadTs(u8),
    /// The interface bytes announce more than [`MAX_LEN`] characters.
    TooLong,
    /// TCK does not make the exclusive-or of T0..TCK zero.
    BadTck,
    /// A field holds a value that ISO/IEC 7816-3 reserves for future use.
    Reserved,
    /// The card is valid but not usable by this reader (see [`Atr::t1_params`]).
    Unsupported(&'static str),
}

/// Result of scanning a (possibly partial) ATR.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Scan {
    /// The ATR is complete and consists of exactly this many bytes.
    Complete(usize),
    /// At least this many further bytes are needed before the length is known.
    NeedMore(usize),
}

const TA: u8 = 0x10;
const TB: u8 = 0x20;
const TC: u8 = 0x40;
const TD: u8 = 0x80;

/// Determines how long the ATR starting at `bytes[0]` is, without validating TCK.
pub(crate) fn scan(bytes: &[u8]) -> Result<Scan, AtrError> {
    let Some(&ts) = bytes.first() else {
        return Ok(Scan::NeedMore(1));
    };
    if ts != 0x3B && ts != 0x3F {
        return Err(AtrError::BadTs(ts));
    }
    let Some(&t0) = bytes.get(1) else {
        return Ok(Scan::NeedMore(1));
    };
    let historical = usize::from(t0 & 0x0F);
    // Index of the next indicator byte (T0 or a TDi) and the presence bits it carries.
    let mut pos = 1;
    let mut y = t0 & 0xF0;
    // TCK is present unless T=0 is the only protocol indicated (8.2.5).
    let mut tck = false;
    loop {
        let count = [TA, TB, TC, TD].iter().filter(|&&b| y & b != 0).count();
        let td_at = pos + count;
        if td_at >= MAX_LEN {
            return Err(AtrError::TooLong);
        }
        if y & TD == 0 {
            pos = td_at;
            break;
        }
        let Some(&td) = bytes.get(td_at) else {
            return Ok(Scan::NeedMore(td_at + 1 - bytes.len()));
        };
        if td & 0x0F != 0 {
            tck = true;
        }
        pos = td_at;
        y = td & 0xF0;
    }
    let len = pos + 1 + historical + usize::from(tck);
    if len > MAX_LEN {
        return Err(AtrError::TooLong);
    }
    Ok(match len.checked_sub(bytes.len()) {
        Some(0) | None => Scan::Complete(len),
        Some(n) => Scan::NeedMore(n),
    })
}

/// The interface characters of a complete, checksum-verified ATR.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Atr {
    /// TS was 0x3B.
    pub(crate) direct: bool,
    /// TA1, if present.
    pub(crate) ta1: Option<u8>,
    /// TC1 (extra guard time N), if present.
    pub(crate) tc1: Option<u8>,
    /// TA2 (specific mode byte), if present.
    pub(crate) ta2: Option<u8>,
    /// Bit `t` is set when protocol T=`t` is indicated in some TDi.
    pub(crate) protocols: u16,
    /// First TAi (i >= 3) following a TD indicating T=1: IFSC.
    pub(crate) t1_ta: Option<u8>,
    /// First TBi (i >= 3) following a TD indicating T=1: BWI and CWI.
    pub(crate) t1_tb: Option<u8>,
    /// First TCi (i >= 3) following a TD indicating T=1: error detection code.
    pub(crate) t1_tc: Option<u8>,
}

impl Atr {
    /// Parses a complete ATR (as delimited by [`scan`]) and verifies TCK.
    pub(crate) fn parse(bytes: &[u8]) -> Result<Self, AtrError> {
        match scan(bytes)? {
            Scan::Complete(len) if len == bytes.len() => {}
            _ => return Err(AtrError::TooLong),
        }
        let mut atr = Atr {
            direct: bytes[0] == 0x3B,
            ta1: None,
            tc1: None,
            ta2: None,
            protocols: 0,
            t1_ta: None,
            t1_tb: None,
            t1_tc: None,
        };
        let mut y = bytes[1] & 0xF0;
        let mut pos = 2;
        // Interface byte group number i of the TAi..TDi being read.
        let mut group = 1;
        // Protocol announced by the TD that introduced this group (T=0 for group 1).
        let mut proto = 0u8;
        let mut tck = false;
        loop {
            let mut take = |bit: u8| {
                if y & bit == 0 {
                    return None;
                }
                let b = bytes[pos];
                pos += 1;
                Some(b)
            };
            let (ta, tb, tc, td) = (take(TA), take(TB), take(TC), take(TD));
            match group {
                1 => {
                    atr.ta1 = ta;
                    atr.tc1 = tc;
                }
                2 => atr.ta2 = ta,
                _ if proto == 1 => {
                    atr.t1_ta = atr.t1_ta.or(ta);
                    atr.t1_tb = atr.t1_tb.or(tb);
                    atr.t1_tc = atr.t1_tc.or(tc);
                }
                _ => {}
            }
            let Some(td) = td else { break };
            proto = td & 0x0F;
            atr.protocols |= 1 << proto;
            if proto != 0 {
                tck = true;
            }
            y = td & 0xF0;
            group += 1;
        }
        if tck && bytes[1..].iter().fold(0u8, |acc, &b| acc ^ b) != 0 {
            return Err(AtrError::BadTck);
        }
        Ok(atr)
    }

    /// Validates the ATR against the ARIB STD-B25 4.3.2 card profile and returns the T=1
    /// parameters this reader operates with.
    ///
    /// The profile is the specific mode for T=1 announced by TA2, with the TA1 transmission
    /// factors applied directly after the ATR: no PPS exchange takes place, so a card that would
    /// require one (negotiable mode) is rejected instead of being driven with guessed parameters.
    pub(crate) fn t1_params(&self) -> Result<T1Params, AtrError> {
        if !self.direct {
            return Err(AtrError::Unsupported("inverse convention"));
        }
        let ta2 = self
            .ta2
            .ok_or(AtrError::Unsupported("negotiable mode (no TA2)"))?;
        if ta2 & 0x0F != 1 {
            return Err(AtrError::Unsupported("specific mode is not T=1"));
        }
        if ta2 & 0x10 != 0 {
            return Err(AtrError::Unsupported("implicit transmission parameters"));
        }
        let ta1 = self.ta1.unwrap_or(0x11);
        let f = fi(ta1 >> 4)?;
        let d = di(ta1 & 0x0F)?;
        let ifsc = self.t1_ta.unwrap_or(32);
        if ifsc == 0 || ifsc == 0xFF {
            return Err(AtrError::Reserved);
        }
        let tb = self.t1_tb.unwrap_or(0x4D);
        let bwi = tb >> 4;
        if bwi > 9 {
            return Err(AtrError::Reserved);
        }
        if self.t1_tc.unwrap_or(0) & 0x01 != 0 {
            return Err(AtrError::Unsupported("CRC error detection"));
        }
        Ok(T1Params {
            f,
            d,
            n: self.tc1.unwrap_or(0),
            ifsc,
            bwi,
            cwi: tb & 0x0F,
        })
    }
}

/// Clock rate conversion integer Fi (ISO/IEC 7816-3 table 7).
fn fi(code: u8) -> Result<u32, AtrError> {
    Ok(match code {
        0 | 1 => 372,
        2 => 558,
        3 => 744,
        4 => 1116,
        5 => 1488,
        6 => 1860,
        9 => 512,
        10 => 768,
        11 => 1024,
        12 => 1536,
        13 => 2048,
        _ => return Err(AtrError::Reserved),
    })
}

/// Baud rate adjustment integer Di (ISO/IEC 7816-3 table 8).
fn di(code: u8) -> Result<u32, AtrError> {
    Ok(match code {
        1 => 1,
        2 => 2,
        3 => 4,
        4 => 8,
        5 => 16,
        6 => 32,
        7 => 64,
        8 => 12,
        9 => 20,
        _ => return Err(AtrError::Reserved),
    })
}

/// Transmission parameters of a T=1 card in specific mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct T1Params {
    /// Clock rate conversion integer F.
    pub(crate) f: u32,
    /// Baud rate adjustment integer D.
    pub(crate) d: u32,
    /// Extra guard time N (TC1).
    pub(crate) n: u8,
    /// Information field size of the card.
    pub(crate) ifsc: u8,
    /// Block waiting time integer.
    pub(crate) bwi: u8,
    /// Character waiting time integer.
    pub(crate) cwi: u8,
}

/// Waiting times of a T=1 session in microseconds, rounded up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Timing {
    /// Block waiting time: maximum delay from our last character to the card's first one.
    pub(crate) bwt_us: u32,
    /// Character waiting time: maximum delay between two characters of one block.
    pub(crate) cwt_us: u32,
    /// Block guard time: minimum delay between characters of blocks in opposite directions.
    pub(crate) bgt_us: u32,
}

impl T1Params {
    /// Computes the waiting times for a card clocked at `clock_hz` (ISO/IEC 7816-3 11.4.3).
    pub(crate) fn timing(&self, clock_hz: u32) -> Timing {
        let clock = u64::from(clock_hz);
        // One etu is F / (D * f) seconds; work in picoseconds to keep the rounding in one place.
        let etu_ps = u64::from(self.f) * 1_000_000_000_000 / (u64::from(self.d) * clock);
        let us = |ps: u64| u32::try_from(ps.div_ceil(1_000_000)).unwrap_or(u32::MAX);
        let cwt = (11 + (1u64 << self.cwi)) * etu_ps;
        // The BWT term uses the fixed Fd = 372, not F, by definition. It is divided before the
        // shift: shifting first overflows u64 from BWI 6 on, and BWI goes up to 9.
        let bwt = 11 * etu_ps + ((960 * 372 * 1_000_000_000_000 / clock) << self.bwi);
        Timing {
            bwt_us: us(bwt),
            cwt_us: us(cwt),
            bgt_us: us(22 * etu_ps),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ATR of the B-CAS card, published in the B-CAS card briefing SB0100D and ARIB STD-B25.
    const BCAS: [u8; 13] = [
        0x3B, 0xF0, 0x12, 0x00, 0xFF, 0x91, 0x81, 0xB1, 0x7C, 0x45, 0x1F, 0x03, 0x99,
    ];

    #[test]
    fn bcas_length_is_found_incrementally() {
        assert_eq!(scan(&[]), Ok(Scan::NeedMore(1)));
        assert_eq!(scan(&BCAS[..1]), Ok(Scan::NeedMore(1)));
        // T0 announces TA1..TD1; TD1 itself is still missing.
        assert_eq!(scan(&BCAS[..2]), Ok(Scan::NeedMore(4)));
        assert_eq!(scan(&BCAS[..6]), Ok(Scan::NeedMore(2)));
        // Until TD3 (index 10) arrives, only the bytes up to it are known to be needed.
        for n in 8..11 {
            assert_eq!(scan(&BCAS[..n]), Ok(Scan::NeedMore(11 - n)), "prefix {n}");
        }
        for n in 11..13 {
            assert_eq!(scan(&BCAS[..n]), Ok(Scan::NeedMore(13 - n)), "prefix {n}");
        }
        assert_eq!(scan(&BCAS), Ok(Scan::Complete(13)));
    }

    #[test]
    fn bcas_parameters() {
        let atr = Atr::parse(&BCAS).unwrap();
        assert_eq!(atr.protocols, (1 << 1) | (1 << 15));
        let p = atr.t1_params().unwrap();
        assert_eq!(
            p,
            T1Params {
                f: 372,
                d: 2,
                n: 0xFF,
                ifsc: 0x7C,
                bwi: 4,
                cwi: 5,
            }
        );
        let t = p.timing(4_000_000);
        // etu = 372 / (2 * 4 MHz) = 46.5 us.
        assert_eq!(t.cwt_us, 2000); // 43 etu
        assert_eq!(t.bgt_us, 1023); // 22 etu
        assert_eq!(t.bwt_us, 1_428_992); // 11 etu + 16 * 960 * 372 / 4 MHz
    }

    #[test]
    fn the_longest_block_waiting_time_is_computed_without_overflow() {
        let p = T1Params {
            f: 372,
            d: 2,
            n: 0xFF,
            ifsc: 0x7C,
            bwi: 9,
            cwi: 5,
        };
        // 11 etu (511.5 us, rounded up) + 512 * 960 * 372 / 4 MHz = 512 + 45 711 360 us.
        assert_eq!(p.timing(4_000_000).bwt_us, 45_711_872);
    }

    #[test]
    fn checksum_and_reserved_values_are_rejected() {
        let mut bad = BCAS;
        bad[12] ^= 1;
        assert_eq!(Atr::parse(&bad), Err(AtrError::BadTck));
        assert_eq!(scan(&[0x3C]), Err(AtrError::BadTs(0x3C)));
        // Same ATR with TA1 = 0x72 (FI = 7 is reserved), TCK recomputed.
        let mut rfu = BCAS;
        rfu[2] = 0x72;
        rfu[12] = rfu[1..12].iter().fold(0, |a, &b| a ^ b);
        let atr = Atr::parse(&rfu).unwrap();
        assert_eq!(atr.t1_params(), Err(AtrError::Reserved));
    }

    #[test]
    fn negotiable_mode_is_not_driven_without_pps() {
        // T=1 card without TA2: 3B 80 01 TCK.
        let atr = [0x3B, 0x80, 0x01, 0x81];
        assert_eq!(scan(&atr), Ok(Scan::Complete(4)));
        let parsed = Atr::parse(&atr).unwrap();
        assert!(matches!(parsed.t1_params(), Err(AtrError::Unsupported(_))));
    }

    #[test]
    fn t0_only_atr_has_no_tck() {
        // 3B 02 + two historical bytes, T=0 implicit.
        assert_eq!(scan(&[0x3B, 0x02, 0x14, 0x50]), Ok(Scan::Complete(4)));
        assert!(Atr::parse(&[0x3B, 0x02, 0x14, 0x50]).is_ok());
    }

    #[test]
    fn oversized_atr_is_rejected() {
        // T0 announces 15 historical bytes, then each TD announces TA..TD again.
        let mut atr = [0u8; 40];
        atr[0] = 0x3B;
        atr[1] = 0x8F;
        atr[2..].fill(0xF1);
        assert_eq!(scan(&atr), Err(AtrError::TooLong));
    }
}
