//! The parts of the pcsc-lite IFD handler ABI (`ifdhandler.h`, `pcsclite.h`, `reader.h` and
//! `wintypes.h` in `/usr/include/PCSC`) that this handler uses.
//!
//! The names follow the C headers so that they can be compared with them directly; the test at
//! the bottom compiles a C program against the installed headers and checks every value here.

#![allow(non_camel_case_types, clippy::upper_case_acronyms)]

use core::ffi::{c_char, c_long, c_ulong};

/// `wintypes.h` defines `DWORD` as `unsigned long` everywhere but on macOS.
pub type DWORD = c_ulong;
pub type PDWORD = *mut DWORD;
pub type UCHAR = u8;
pub type PUCHAR = *mut UCHAR;
pub type LPSTR = *mut c_char;
/// `ifdhandler.h`: `typedef long RESPONSECODE`.
pub type RESPONSECODE = c_long;

// Lengths are converted between `DWORD` and `usize` with `as`; this makes those casts lossless.
const _: () = assert!(core::mem::size_of::<DWORD>() == core::mem::size_of::<usize>());

/// `SCARD_IO_HEADER`, passed by value to `IFDHTransmitToICC`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ScardIoHeader {
    /// Protocol number (0 for T=0, 1 for T=1), not the `SCARD_PROTOCOL_*` bit mask.
    pub protocol: DWORD,
    pub length: DWORD,
}

pub const MAX_ATR_SIZE: usize = 33;

pub const SCARD_PROTOCOL_T1: DWORD = 0x0002;
// The handler rejects everything but T=1 without naming the alternatives; the tests name them.
#[cfg(test)]
pub const SCARD_PROTOCOL_T0: DWORD = 0x0001;
#[cfg(test)]
pub const SCARD_PROTOCOL_RAW: DWORD = 0x0004;

pub const TAG_IFD_ATR: DWORD = 0x0303;
pub const TAG_IFD_SLOT_THREAD_SAFE: DWORD = 0x0FAC;
pub const TAG_IFD_THREAD_SAFE: DWORD = 0x0FAD;
pub const TAG_IFD_SLOTS_NUMBER: DWORD = 0x0FAE;
pub const TAG_IFD_SIMULTANEOUS_ACCESS: DWORD = 0x0FAF;
/// `reader.h`: `SCARD_ATTR_VALUE(SCARD_CLASS_ICC_STATE, 0x0303)`, the `SCardGetAttrib()` name of
/// the ATR, which pcscd passes through to `IFDHGetCapabilities`.
pub const SCARD_ATTR_ATR_STRING: DWORD = 0x0009_0303;

pub const IFD_POWER_UP: DWORD = 500;
pub const IFD_POWER_DOWN: DWORD = 501;
pub const IFD_RESET: DWORD = 502;

pub const IFD_NEGOTIATE_PTS1: UCHAR = 1;
#[cfg(test)]
pub const IFD_NEGOTIATE_PTS2: UCHAR = 2;
#[cfg(test)]
pub const IFD_NEGOTIATE_PTS3: UCHAR = 4;

pub const IFD_SUCCESS: RESPONSECODE = 0;
pub const IFD_ERROR_TAG: RESPONSECODE = 600;
pub const IFD_ERROR_VALUE_READ_ONLY: RESPONSECODE = 602;
pub const IFD_ERROR_PTS_FAILURE: RESPONSECODE = 605;
pub const IFD_ERROR_NOT_SUPPORTED: RESPONSECODE = 606;
pub const IFD_PROTOCOL_NOT_SUPPORTED: RESPONSECODE = 607;
pub const IFD_ERROR_POWER_ACTION: RESPONSECODE = 608;
pub const IFD_COMMUNICATION_ERROR: RESPONSECODE = 612;
pub const IFD_NOT_SUPPORTED: RESPONSECODE = 614;
pub const IFD_ICC_PRESENT: RESPONSECODE = 615;
pub const IFD_ICC_NOT_PRESENT: RESPONSECODE = 616;
pub const IFD_NO_SUCH_DEVICE: RESPONSECODE = 617;
pub const IFD_ERROR_INSUFFICIENT_BUFFER: RESPONSECODE = 618;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cprobe;

    /// Compares every declaration above with the installed pcsc-lite headers.
    #[test]
    fn declarations_match_installed_headers() {
        let checks: &[(&str, i128)] = &[
            ("sizeof(DWORD)", core::mem::size_of::<DWORD>() as i128),
            ("sizeof(UCHAR)", core::mem::size_of::<UCHAR>() as i128),
            (
                "sizeof(RESPONSECODE)",
                core::mem::size_of::<RESPONSECODE>() as i128,
            ),
            ("(DWORD)-1 > 0", i128::from(DWORD::MAX > 0)),
            ("(RESPONSECODE)-1 < 0", i128::from(RESPONSECODE::MIN < 0)),
            (
                "sizeof(SCARD_IO_HEADER)",
                core::mem::size_of::<ScardIoHeader>() as i128,
            ),
            (
                "offsetof(SCARD_IO_HEADER, Protocol)",
                core::mem::offset_of!(ScardIoHeader, protocol) as i128,
            ),
            (
                "offsetof(SCARD_IO_HEADER, Length)",
                core::mem::offset_of!(ScardIoHeader, length) as i128,
            ),
            ("MAX_ATR_SIZE", MAX_ATR_SIZE as i128),
            ("SCARD_PROTOCOL_T0", i128::from(SCARD_PROTOCOL_T0)),
            ("SCARD_PROTOCOL_T1", i128::from(SCARD_PROTOCOL_T1)),
            ("SCARD_PROTOCOL_RAW", i128::from(SCARD_PROTOCOL_RAW)),
            ("TAG_IFD_ATR", i128::from(TAG_IFD_ATR)),
            (
                "TAG_IFD_SLOT_THREAD_SAFE",
                i128::from(TAG_IFD_SLOT_THREAD_SAFE),
            ),
            ("TAG_IFD_THREAD_SAFE", i128::from(TAG_IFD_THREAD_SAFE)),
            ("TAG_IFD_SLOTS_NUMBER", i128::from(TAG_IFD_SLOTS_NUMBER)),
            (
                "TAG_IFD_SIMULTANEOUS_ACCESS",
                i128::from(TAG_IFD_SIMULTANEOUS_ACCESS),
            ),
            ("SCARD_ATTR_ATR_STRING", i128::from(SCARD_ATTR_ATR_STRING)),
            ("IFD_POWER_UP", i128::from(IFD_POWER_UP)),
            ("IFD_POWER_DOWN", i128::from(IFD_POWER_DOWN)),
            ("IFD_RESET", i128::from(IFD_RESET)),
            ("IFD_NEGOTIATE_PTS1", i128::from(IFD_NEGOTIATE_PTS1)),
            ("IFD_NEGOTIATE_PTS2", i128::from(IFD_NEGOTIATE_PTS2)),
            ("IFD_NEGOTIATE_PTS3", i128::from(IFD_NEGOTIATE_PTS3)),
            ("IFD_SUCCESS", IFD_SUCCESS as i128),
            ("IFD_ERROR_TAG", IFD_ERROR_TAG as i128),
            (
                "IFD_ERROR_VALUE_READ_ONLY",
                IFD_ERROR_VALUE_READ_ONLY as i128,
            ),
            ("IFD_ERROR_PTS_FAILURE", IFD_ERROR_PTS_FAILURE as i128),
            ("IFD_ERROR_NOT_SUPPORTED", IFD_ERROR_NOT_SUPPORTED as i128),
            (
                "IFD_PROTOCOL_NOT_SUPPORTED",
                IFD_PROTOCOL_NOT_SUPPORTED as i128,
            ),
            ("IFD_ERROR_POWER_ACTION", IFD_ERROR_POWER_ACTION as i128),
            ("IFD_COMMUNICATION_ERROR", IFD_COMMUNICATION_ERROR as i128),
            ("IFD_NOT_SUPPORTED", IFD_NOT_SUPPORTED as i128),
            ("IFD_ICC_PRESENT", IFD_ICC_PRESENT as i128),
            ("IFD_ICC_NOT_PRESENT", IFD_ICC_NOT_PRESENT as i128),
            ("IFD_NO_SUCH_DEVICE", IFD_NO_SUCH_DEVICE as i128),
            (
                "IFD_ERROR_INSUFFICIENT_BUFFER",
                IFD_ERROR_INSUFFICIENT_BUFFER as i128,
            ),
        ];
        cprobe::assert_values(
            "pcsc",
            &["ifdhandler.h", "reader.h", "stddef.h"],
            &["/usr/include/PCSC"],
            checks,
        );
    }
}
