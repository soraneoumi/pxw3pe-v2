//! Mirror of the kernel interface in `driver/uapi/pxw3pe_card.h`; any change there must be made
//! here too. The test at the bottom checks this file against the header itself.

// The `_IOC` layout below (2 direction bits at 30, 14 size bits at 16) is the generic one. These
// architectures use 3 direction bits and 13 size bits, so the numbers would silently be wrong.
#[cfg(any(
    target_arch = "mips",
    target_arch = "mips64",
    target_arch = "powerpc",
    target_arch = "powerpc64",
    target_arch = "sparc",
    target_arch = "sparc64"
))]
compile_error!("this architecture uses a different _IOC encoding");

pub const ATR_MAX: usize = 33;
/// Smallest command: an APDU has at least CLA INS P1 P2 (ISO/IEC 7816-4).
pub const APDU_MIN: usize = 4;
/// Largest command or response one T=1 block carries (IFSD = 254); chaining is never used.
pub const APDU_MAX: usize = 254;

/// A card is inserted.
pub const PRESENT: u32 = 1 << 0;
/// The card has been reset and a protocol session is open. The handler does not need it: a
/// missing session is reported by the transmit call itself.
#[allow(dead_code)]
pub const ACTIVE: u32 = 1 << 1;
/// Since the previous status call, a session ended because its card was pulled.
pub const REMOVED: u32 = 1 << 2;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CardStatus {
    pub flags: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CardAtr {
    pub len: u32,
    pub atr: [u8; ATR_MAX],
    pub reserved: [u8; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CardTransfer {
    /// In: command length.
    pub command_len: u32,
    /// Out: response length, including SW1 SW2.
    pub response_len: u32,
    pub command: [u8; 256],
    pub response: [u8; 256],
}

impl CardAtr {
    pub const fn zeroed() -> Self {
        Self {
            len: 0,
            atr: [0; ATR_MAX],
            reserved: [0; 3],
        }
    }
}

impl CardTransfer {
    pub const fn zeroed() -> Self {
        Self {
            command_len: 0,
            response_len: 0,
            command: [0; 256],
            response: [0; 256],
        }
    }
}

macro_rules! assert_layout {
    ($ty:ty, size $size:expr $(, $field:ident @ $offset:expr)*) => {
        const _: () = assert!(core::mem::size_of::<$ty>() == $size);
        $(const _: () = assert!(core::mem::offset_of!($ty, $field) == $offset);)*
    };
}

assert_layout!(CardStatus, size 4, flags @ 0);
assert_layout!(CardAtr, size 40, len @ 0, atr @ 4, reserved @ 37);
assert_layout!(CardTransfer, size 520, command_len @ 0, response_len @ 4, command @ 8, response @ 264);

const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

/// `_IOC(dir, type, nr, size)` of `<asm-generic/ioctl.h>`.
const fn ioc(dir: u32, ty: u32, nr: u32, size: usize) -> u32 {
    // The size field is 14 bits wide; a larger argument cannot be encoded.
    assert!(size < 1 << 14);
    (dir << 30) | ((size as u32) << 16) | (ty << 8) | nr
}

const fn io(ty: u32, nr: u32) -> u32 {
    ioc(IOC_NONE, ty, nr, 0)
}

const fn ior<T>(ty: u32, nr: u32) -> u32 {
    ioc(IOC_READ, ty, nr, core::mem::size_of::<T>())
}

const fn iowr<T>(ty: u32, nr: u32) -> u32 {
    ioc(IOC_READ | IOC_WRITE, ty, nr, core::mem::size_of::<T>())
}

const IOC_TYPE: u32 = 0xBA;

pub const GET_STATUS: u32 = ior::<CardStatus>(IOC_TYPE, 0);
pub const RESET: u32 = ior::<CardAtr>(IOC_TYPE, 1);
pub const POWER_OFF: u32 = io(IOC_TYPE, 2);
pub const TRANSMIT: u32 = iowr::<CardTransfer>(IOC_TYPE, 3);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cprobe;

    #[test]
    fn ioctl_numbers() {
        assert_eq!(GET_STATUS, 0x8004_BA00);
        assert_eq!(RESET, 0x8028_BA01);
        assert_eq!(POWER_OFF, 0x0000_BA02);
        assert_eq!(TRANSMIT, 0xC208_BA03);
    }

    #[test]
    fn ioc_fields() {
        assert_eq!(ioc(IOC_READ, 0xBA, 7, 0x123), 0x8123_BA07);
        assert_eq!(ioc(IOC_WRITE, 0x01, 0xFF, 0), 0x4000_01FF);
        assert_eq!(
            ioc(IOC_READ | IOC_WRITE, 0xFF, 0, (1 << 14) - 1),
            0xFFFF_FF00
        );
    }

    /// Compares the numbers and layouts with what the C compiler makes of the header.
    #[test]
    fn mirror_matches_uapi_header() {
        let uapi = concat!(env!("CARGO_MANIFEST_DIR"), "/../../driver/uapi");
        let checks: &[(&str, i128)] = &[
            ("PXW3PE_CARD_GET_STATUS", GET_STATUS.into()),
            ("PXW3PE_CARD_RESET", RESET.into()),
            ("PXW3PE_CARD_POWER_OFF", POWER_OFF.into()),
            ("PXW3PE_CARD_TRANSMIT", TRANSMIT.into()),
            ("PXW3PE_CARD_ATR_MAX", ATR_MAX as i128),
            ("PXW3PE_CARD_APDU_MIN", APDU_MIN as i128),
            ("PXW3PE_CARD_APDU_MAX", APDU_MAX as i128),
            ("PXW3PE_CARD_PRESENT", PRESENT.into()),
            ("PXW3PE_CARD_ACTIVE", ACTIVE.into()),
            ("PXW3PE_CARD_REMOVED", REMOVED.into()),
            (
                "sizeof(struct pxw3pe_card_status)",
                core::mem::size_of::<CardStatus>() as i128,
            ),
            (
                "sizeof(struct pxw3pe_card_atr)",
                core::mem::size_of::<CardAtr>() as i128,
            ),
            (
                "offsetof(struct pxw3pe_card_atr, atr)",
                core::mem::offset_of!(CardAtr, atr) as i128,
            ),
            (
                "offsetof(struct pxw3pe_card_atr, reserved)",
                core::mem::offset_of!(CardAtr, reserved) as i128,
            ),
            (
                "sizeof(struct pxw3pe_card_transfer)",
                core::mem::size_of::<CardTransfer>() as i128,
            ),
            (
                "offsetof(struct pxw3pe_card_transfer, response_len)",
                core::mem::offset_of!(CardTransfer, response_len) as i128,
            ),
            (
                "offsetof(struct pxw3pe_card_transfer, command)",
                core::mem::offset_of!(CardTransfer, command) as i128,
            ),
            (
                "offsetof(struct pxw3pe_card_transfer, response)",
                core::mem::offset_of!(CardTransfer, response) as i128,
            ),
            (
                "sizeof(((struct pxw3pe_card_transfer *)0)->command)",
                core::mem::size_of::<[u8; 256]>() as i128,
            ),
        ];
        cprobe::assert_values("uapi", &["pxw3pe_card.h", "stddef.h"], &[uapi], checks);
    }
}
