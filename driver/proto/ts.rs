// SPDX-License-Identifier: GPL-2.0

//! Transport stream handling between the DMA buffers and the DVB demux: routing the packets of
//! a bridge port to their tuner, and removing the scrambling the ASIE5606 applies to every
//! packet, with the derivation of its mask from the seed the driver programs into the chip.

/// MPEG-TS packet size.
pub(crate) const PACKET: usize = 188;

/// Most packets in one DMA transfer: 100 ms of the port's full rate, all four tuners streaming
/// (`stream::TARGET_US`). The transfer length is 22 bits wide, so the hardware allows much more.
pub(crate) const MAX_PACKETS_PER_TRANSFER: usize = 4992;

/// Bytes of a DMA buffer: the longest transfer.
pub(crate) const TRANSFER: usize = PACKET * MAX_PACKETS_PER_TRANSFER;

/// Number of tuners of the board.
pub(crate) const TUNERS: usize = 4;

/// Bridge TS input port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Port {
    /// Carries the demodulator group at 7-bit base address 0x14.
    A,
    /// Carries the demodulator group at 7-bit base address 0x10.
    B,
}

/// Position of each tuner's demodulator core within its group of four: ISDB-S 0 and 1 on the odd
/// cores 1 and 3, ISDB-T 0 and 1 on the even cores 0 and 2.
const POSITION: [u8; TUNERS] = [1, 3, 0, 2];

/// Sync byte tag the demodulator puts on the packets of the core at each position. The tag follows
/// the position and no register write sets it, so it is inferred; a wrong table shows as packets of
/// one tuner arriving at another's demux. It cannot come from the bridge: the four cores share one
/// bridge input.
const TAG_AT_POSITION: [u8; 4] = [0xC7, 0x47, 0x07, 0x87];

/// 7-bit base addresses the demodulator group can have. The group at the first address is wired to
/// port B, the one at the second to port A.
pub(crate) const BASES: [u8; 2] = [0x10, 0x14];

/// Where the four tuners of the board are: the demodulator cores' addresses and the one bridge port
/// their streams arrive on, with the tags that tell them apart.
///
/// Tuner 0 and 1 are ISDB-S, 2 and 3 ISDB-T. This is the only place that knows the table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Routing {
    base: u8,
    port: Port,
}

impl Routing {
    /// Routing of the board whose demodulator group starts at the 7-bit address `base`; `None`
    /// for an address that is no group of the board.
    pub(crate) fn for_base(base: u8) -> Option<Routing> {
        let port = match BASES.iter().position(|&b| b == base)? {
            0 => Port::B,
            _ => Port::A,
        };
        Some(Routing { base, port })
    }

    /// 7-bit I2C address of the demodulator core of `tuner` (0..=3).
    pub(crate) fn demod_addr(&self, tuner: usize) -> u8 {
        self.base + POSITION[tuner]
    }

    /// Port carrying the stream of `tuner`. All tuners share one port; the other port is never
    /// started.
    pub(crate) fn port(&self, _tuner: usize) -> Port {
        self.port
    }

    /// Tuner (0..=3) a packet received on `port` belongs to, from its sync byte, which the
    /// demodulator replaces with a per-core tag; `None` for anything else, which is not a packet
    /// start.
    pub(crate) fn tuner(&self, port: Port, tag: u8) -> Option<usize> {
        if port != self.port {
            return None;
        }
        POSITION
            .iter()
            .position(|&pos| TAG_AT_POSITION[usize::from(pos)] == tag)
    }
}

/// Number of packets of `transfer` that start with a known tag of `port`.
// `as_chunks` needs Rust 1.88, and the kernel supports older compilers.
#[allow(clippy::chunks_exact_to_as_chunks)]
pub(crate) fn tagged_packets(routing: &Routing, port: Port, transfer: &[u8]) -> usize {
    transfer
        .chunks_exact(PACKET)
        .filter(|packet| routing.tuner(port, packet[0]).is_some())
        .count()
}

/// Fewest packets that [`in_sync`] judges.
pub(crate) const SYNC_SAMPLE: usize = 8;

/// Whether `packets` packets of a transfer, `tagged` of which start with a known tag, are packet
/// aligned: at least half of them do, or there are fewer than [`SYNC_SAMPLE`].
///
/// Misaligned data starts a "packet" with a tag byte by chance, about four times in 256 (4.9
/// packets of the shortest transfer), so one or a few tagged packets prove nothing. The opposite
/// rule, any untagged packet means lost sync, is too strict: a demodulator core that restarts can
/// send an untagged packet (measured), the bridge's treatment of other first bytes is only
/// inferred, and an occasional untagged packet must not restart the stream. Half separates the two
/// cases by a wide margin once there are enough packets: eight misaligned ones pass with a chance
/// of about 4e-6, eight aligned ones with one or two untagged among them always do. Fewer cannot be
/// told apart; the stream delivers a transfer's start early in pieces that can be that short, and
/// judges each piece together with the pieces of the same transfer before it, so a whole transfer
/// is always judged.
pub(crate) fn in_sync(tagged: usize, packets: usize) -> bool {
    packets < SYNC_SAMPLE || tagged * 2 >= packets
}

/// Scrambling scheme of the ASIE5606, selected by its version register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Scheme {
    /// Version 0x11: XOR mask only.
    Xor,
}

impl Scheme {
    /// Scheme for the version read from register 0x09 as `(raw & 0x3E) >> 1`; `None` for every
    /// other version.
    pub(crate) fn for_version(version: u8) -> Option<Scheme> {
        match version {
            0x11 => Some(Scheme::Xor),
            _ => None,
        }
    }
}

/// The seed the driver programs into the scrambler's registers 0x10..0x1F (`bringup::load_seed`)
/// and derives its mask from. The chip derives the mask it applies from whatever seed it is given,
/// so the value is arbitrary; this one is readable in a register dump.
pub(crate) const SEED: [u8; 16] = *b"pxw3pe V2.0 seed";

/// Fixed bytes of the 14-byte mask of KeyTransfer1 (revealed); bytes 4 and 10 are the parity bits
/// of the two seed halves.
const KT1_MASK: [u8; 14] = [
    0x60, 0x57, 0x79, 0x88, 0x00, 0x00, 0x02, 0x6F, 0x67, 0x40, 0x00, 0x00, 0xFA, 0xCD,
];

/// Byte permutation of KeyTransfer2 and the mask constant of its byte 6 (revealed). Its other
/// constant does not enter the XOR seed.
const KT2_PERMUTATION: [u8; 14] = [
    0x0B, 0x02, 0x01, 0x0D, 0x04, 0x07, 0x06, 0x05, 0x00, 0x0A, 0x03, 0x0C, 0x09, 0x08,
];
const KT2_CONSTANT: u8 = 0x02;

/// Bit `i` is bit 0 of `bytes[i]`.
fn parity_bits(bytes: &[u8]) -> u8 {
    bytes
        .iter()
        .enumerate()
        .fold(0, |acc, (i, &b)| acc | (b & 1) << i)
}

const MASK_112: u128 = (1 << 112) - 1;

fn rotate_right_112(x: u128, n: u32) -> u128 {
    (x >> n | x << (112 - n)) & MASK_112
}

fn from_bytes_14(b: &[u8; 14]) -> u128 {
    let mut le = [0u8; 16];
    le[..14].copy_from_slice(b);
    u128::from_le_bytes(le)
}

fn to_bytes_14(x: u128) -> [u8; 14] {
    let le = x.to_le_bytes();
    let mut out = [0u8; 14];
    out.copy_from_slice(&le[..14]);
    out
}

/// KeyTransfer1: the 16-byte seed to the 14-byte intermediate.
fn key_transfer1(seed: &[u8; 16]) -> [u8; 14] {
    // Bits 1..7 of each seed byte, byte 0 first, form a 112-bit string.
    let bits = seed
        .iter()
        .enumerate()
        .fold(0u128, |acc, (k, &b)| acc | u128::from(b >> 1) << (7 * k));
    // The string is packed into bytes starting at bit 56.
    let packed = rotate_right_112(bits, 56);
    let mut mask = KT1_MASK;
    mask[4] = parity_bits(&seed[..8]);
    mask[10] = parity_bits(&seed[8..]);
    // Output byte k is (x[k+7] << 5) | (x[k+6] >> 3), indices mod 14.
    to_bytes_14(rotate_right_112(packed ^ from_bytes_14(&mask), 51))
}

/// KeyTransfer2: the intermediate to the 4-byte XOR seed. It forms 14 bytes, the intermediate
/// permuted and XORed with a mask, and takes the XOR seed from seven of them (0, 1, 3, 6, 8, 9,
/// 12); the other seven do not enter it and are not computed.
fn key_transfer2(seed: &[u8; 16], k1: &[u8; 14]) -> [u8; 4] {
    let lo = parity_bits(&seed[..8]);
    let hi = parity_bits(&seed[8..]);
    let r = |k: usize, mask: u8| k1[usize::from(KT2_PERMUTATION[k])] ^ mask;
    let (r0, r1, r3, r6) = (r(0, hi), r(1, lo), r(3, hi), r(6, KT2_CONSTANT));
    let (r8, r9, r12) = (r(8, hi), r(9, hi), r(12, lo));
    [
        r8 << 4 | r9 >> 4,
        r9 << 4 | r12 >> 4,
        (r1 & 0xF0) | r3 >> 4,
        (r6 & 0xF0) | (r0 & 0x0F),
    ]
}

/// The 4-byte XOR seed the scrambler derives from `seed`.
pub(crate) fn xor_seed(seed: &[u8; 16]) -> [u8; 4] {
    key_transfer2(seed, &key_transfer1(seed))
}

/// Order in which the XOR seed bytes form the 8-byte mask (the revealed order).
const MASK_ORDER: [usize; 8] = [0, 0, 3, 1, 0, 2, 1, 2];

/// Removes the per-packet scrambling. The 4-byte header is never scrambled; the whole 184-byte
/// payload always is, independently of the packet's own scrambling control bits.
pub(crate) struct Descrambler {
    mask: [u8; 8],
}

impl Descrambler {
    /// Builds the descrambler for a chip programmed with `seed`.
    pub(crate) fn new(seed: &[u8; 16]) -> Self {
        let xor = xor_seed(seed);
        Descrambler {
            mask: MASK_ORDER.map(|i| xor[i]),
        }
    }

    /// Descrambles one packet in place.
    // `as_chunks_mut` would avoid the conversion below but needs Rust 1.88, and the kernel
    // supports older compilers.
    #[allow(clippy::chunks_exact_to_as_chunks)]
    pub(crate) fn packet(&self, packet: &mut [u8; PACKET]) {
        for chunk in packet[4..].chunks_exact_mut(8) {
            for (b, m) in chunk.iter_mut().zip(self.mask) {
                *b ^= m;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn routing(base: u8) -> Routing {
        Routing::for_base(base).expect("a base of the board")
    }

    #[test]
    fn routing_table_for_both_bases() {
        // Tuner, demodulator core, tag; the port follows the base.
        let table = [(0, 1, 0x47), (1, 3, 0x87), (2, 0, 0xC7), (3, 2, 0x07)];
        for (base, port) in [(0x10, Port::B), (0x14, Port::A)] {
            let r = routing(base);
            for (tuner, position, tag) in table {
                assert_eq!(r.demod_addr(tuner), base + position, "base {base:#x}");
                assert_eq!(r.port(tuner), port);
                assert_eq!(r.tuner(port, tag), Some(tuner));
            }
        }
    }

    #[test]
    fn only_the_wired_port_and_known_tags_route() {
        let b = routing(0x10);
        assert_eq!(b.tuner(Port::A, 0x47), None);
        let a = routing(0x14);
        assert_eq!(a.tuner(Port::B, 0xC7), None);
        for tag in 0..=0xFF {
            let known = [0x47, 0x87, 0xC7, 0x07].contains(&tag);
            assert_eq!(a.tuner(Port::A, tag).is_some(), known, "tag {tag:#x}");
        }
    }

    #[test]
    fn bases_outside_the_board_have_no_routing() {
        for base in [0x00, 0x0F, 0x11, 0x12, 0x13, 0x15, 0x18, 0xFF] {
            assert_eq!(Routing::for_base(base), None, "base {base:#x}");
        }
    }

    #[test]
    fn aligned_transfer_is_in_sync() {
        let r = routing(0x14);
        let mut transfer = [0u8; TRANSFER];
        for start in (0..TRANSFER).step_by(PACKET) {
            transfer[start] = 0x47;
        }
        assert_eq!(
            tagged_packets(&r, Port::A, &transfer),
            MAX_PACKETS_PER_TRANSFER
        );
        assert!(in_sync(MAX_PACKETS_PER_TRANSFER, MAX_PACKETS_PER_TRANSFER));
        // The tags of all four cores count.
        for (n, tag) in [0x87, 0xC7, 0x07].into_iter().enumerate() {
            transfer[PACKET * (5 + n)] = tag;
        }
        assert_eq!(
            tagged_packets(&r, Port::A, &transfer),
            MAX_PACKETS_PER_TRANSFER
        );
        // The other port is never started, so nothing on it is a packet start.
        assert_eq!(tagged_packets(&r, Port::B, &transfer), 0);
    }

    #[test]
    fn too_few_packets_are_not_judged() {
        for packets in 0..SYNC_SAMPLE {
            assert!(in_sync(0, packets), "{packets} packets");
        }
        assert!(!in_sync(0, SYNC_SAMPLE));
        assert!(!in_sync(SYNC_SAMPLE / 2 - 1, SYNC_SAMPLE));
        assert!(in_sync(SYNC_SAMPLE / 2, SYNC_SAMPLE));
        // A restarted core's untagged packet among the first few of a transfer.
        assert!(in_sync(SYNC_SAMPLE - 1, SYNC_SAMPLE));
    }

    #[test]
    fn sync_check_counts_each_of_the_four_tags() {
        let r = routing(0x10);
        let mut transfer = [0u8; TRANSFER];
        for (n, tag) in [0x47, 0x87, 0xC7, 0x07, 0x47, 0x07].into_iter().enumerate() {
            transfer[PACKET * n] = tag;
        }
        assert_eq!(tagged_packets(&r, Port::B, &transfer), 6);
        // Near misses of a tag do not count.
        transfer[PACKET * 6] = 0x46;
        transfer[PACKET * 7] = 0xC6;
        assert_eq!(tagged_packets(&r, Port::B, &transfer), 6);
    }

    #[test]
    fn misaligned_transfer_is_out_of_sync() {
        let r = routing(0x14);
        let mut stream = [0u8; TRANSFER + 1];
        for start in (1..=TRANSFER).step_by(PACKET) {
            stream[start] = 0x47;
        }
        // Read one byte early, no packet start carries its tag. Real misaligned data hits a tag
        // by chance in a few packets only.
        let transfer = &mut stream[..TRANSFER];
        assert_eq!(tagged_packets(&r, Port::A, transfer), 0);
        for n in [0, 1, 2, 3, 5] {
            transfer[PACKET * n] = 0xC7;
        }
        assert_eq!(tagged_packets(&r, Port::A, transfer), 5);
        assert!(!in_sync(5, 312));
        assert!(!in_sync(0, 312));
    }

    #[test]
    fn sync_needs_half_of_the_packets_tagged() {
        for packets in [312, 1000, MAX_PACKETS_PER_TRANSFER] {
            assert!(in_sync(packets / 2, packets));
            assert!(!in_sync(packets / 2 - 1, packets));
            // A few untagged packets (a core that is not locked, or inferred bridge behaviour)
            // do not restart the stream.
            assert!(in_sync(packets - 10, packets));
        }
    }

    /// Values computed with an independent derivation of the chip's key schedule (revealed).
    #[test]
    fn key_transfer_matches_the_verified_derivation() {
        let cases: [([u8; 16], [u8; 14], [u8; 4]); 3] = [
            (
                SEED,
                [
                    0xEC, 0x2A, 0xD7, 0x6D, 0x7E, 0x72, 0x77, 0x33, 0x9F, 0xEB, 0x4F, 0xE3, 0x37,
                    0xD9,
                ],
                [0xC3, 0xFC, 0xFA, 0x73],
            ),
            (
                [0; 16],
                [
                    0xE0, 0xED, 0x0C, 0x08, 0x00, 0x40, 0xBF, 0x19, 0xEC, 0x2A, 0x0F, 0x11, 0x00,
                    0x40,
                ],
                [0x00, 0xF2, 0x04, 0xB1],
            ),
            (
                core::array::from_fn(|i| (i as u8).wrapping_mul(0x9D) ^ 0x5A),
                [
                    0x4D, 0xD8, 0x8A, 0x51, 0x3A, 0x65, 0x82, 0x3B, 0x17, 0xCC, 0xC6, 0x56, 0x44,
                    0x73,
                ],
                [0x76, 0xC6, 0x2D, 0x8C],
            ),
        ];
        for (seed, k1, xor) in cases {
            assert_eq!(key_transfer1(&seed), k1);
            assert_eq!(xor_seed(&seed), xor);
        }
    }

    /// KeyTransfer1 written bit by bit, against the word-wise implementation.
    #[test]
    fn key_transfer1_matches_the_byte_formula() {
        fn reference(seed: &[u8; 16]) -> [u8; 14] {
            let bit = |n: usize| (seed[n / 7] >> (n % 7 + 1)) & 1;
            let pack = |start: usize| (0..8).fold(0, |acc, j| acc | bit(start + j) << j);
            let mut m = [0u8; 14];
            for n in 0..7 {
                m[n] = pack(56 + 8 * n);
                m[7 + n] = pack(8 * n);
            }
            let mut mask = KT1_MASK;
            mask[4] = parity_bits(&seed[..8]);
            mask[10] = parity_bits(&seed[8..]);
            let x: [u8; 14] = core::array::from_fn(|i| m[i] ^ mask[i]);
            core::array::from_fn(|k| x[(k + 7) % 14] << 5 | x[(k + 6) % 14] >> 3)
        }
        for seed in [
            SEED,
            [0xFF; 16],
            core::array::from_fn(|i| (i as u8).wrapping_mul(0x9D) ^ 0x5A),
            core::array::from_fn(|i| (0x80u8 >> (i % 8)) ^ (i as u8)),
        ] {
            assert_eq!(key_transfer1(&seed), reference(&seed));
        }
    }

    #[test]
    fn mask_expansion_and_payload_xor() {
        let d = Descrambler::new(&SEED);
        // xor_seed(&SEED) = C3 FC FA 73 in the order {0, 0, 3, 1, 0, 2, 1, 2}.
        assert_eq!(d.mask, [0xC3, 0xC3, 0x73, 0xFC, 0xC3, 0xFA, 0xFC, 0xFA]);
        let mut p = [0u8; PACKET];
        p[..4].copy_from_slice(&[0x47, 0x01, 0x02, 0x03]);
        d.packet(&mut p);
        assert_eq!(&p[..4], &[0x47, 0x01, 0x02, 0x03]);
        for n in 0..23 {
            assert_eq!(&p[4 + 8 * n..12 + 8 * n], &d.mask, "block {n}");
        }
    }

    #[test]
    fn descrambling_twice_restores_the_packet() {
        let d = Descrambler::new(&[0x5A; 16]);
        let mut p = [0u8; PACKET];
        for (i, b) in p.iter_mut().enumerate() {
            *b = i as u8;
        }
        let original = p;
        d.packet(&mut p);
        assert_ne!(p, original);
        d.packet(&mut p);
        assert_eq!(p, original);
    }

    #[test]
    fn only_version_0x11_has_a_scheme() {
        assert_eq!(Scheme::for_version(0x11), Some(Scheme::Xor));
        for v in (0..=0x1F).filter(|&v| v != 0x11) {
            assert_eq!(Scheme::for_version(v), None, "version {v:#x}");
        }
    }
}
