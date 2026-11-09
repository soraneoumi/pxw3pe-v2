//! The pcsc-lite IFD Handler API v3 entry points.
//!
//! Each exported function forwards to a generic adapter that converts between C pointers and the
//! [`Handler`] API, so that the adapters can be tested against a fake card. The adapters are
//! where every pointer and length coming from pcscd is checked.
//!
//! # Panics
//!
//! No panic may unwind into pcscd. The handler code is written not to panic (lengths are checked
//! before every slice operation), and each entry point additionally runs inside
//! [`std::panic::catch_unwind`], so a panic that slips through anyway is reported as
//! IFD_COMMUNICATION_ERROR. Out-parameters that must be zero on failure are zeroed before any
//! other work, so they are zero in that case as well.

#![allow(non_snake_case)]

use crate::device::{Card, CharDevice};
use crate::handler::{Handler, IfdResult};
use crate::pcsc::{
    ScardIoHeader, DWORD, IFD_COMMUNICATION_ERROR, IFD_ERROR_INSUFFICIENT_BUFFER,
    IFD_ICC_NOT_PRESENT, IFD_ICC_PRESENT, IFD_NOT_SUPPORTED, IFD_NO_SUCH_DEVICE, IFD_SUCCESS,
    LPSTR, PDWORD, PUCHAR, RESPONSECODE, UCHAR,
};
use core::ffi::{c_char, CStr};
use std::ffi::OsStr;
use std::io;
use std::os::unix::ffi::OsStrExt;
use std::panic::{self, AssertUnwindSafe};
use std::path::Path;
use std::ptr;

/// The readers of this process. pcscd loads the library once for all readers configured with
/// it, so this table is shared by them.
static HANDLER: Handler<CharDevice> = Handler::new();

fn guard(f: impl FnOnce() -> RESPONSECODE) -> RESPONSECODE {
    // Asserting unwind safety is sound because all state shared between calls is behind mutexes,
    // which a panic while held poisons, and the handler reports a poisoned lock as an error
    // instead of using the state behind it.
    panic::catch_unwind(AssertUnwindSafe(f)).unwrap_or(IFD_COMMUNICATION_ERROR)
}

fn code(result: IfdResult<()>) -> RESPONSECODE {
    match result {
        Ok(()) => IFD_SUCCESS,
        Err(code) => code,
    }
}

/// The response code for a device node that cannot be opened.
fn open_error(err: &io::Error) -> RESPONSECODE {
    match err.raw_os_error() {
        // No node, or a node without a bound device: the reader is absent, which pcscd logs as
        // information instead of as a failure.
        Some(libc::ENOENT | libc::ENODEV | libc::ENXIO) => IFD_NO_SUCH_DEVICE,
        _ => IFD_COMMUNICATION_ERROR,
    }
}

/// Copies `src` to the C buffer `dst` of `capacity` bytes and stores the length copied in `*len`.
/// Returns false, writing nothing to `dst`, if `src` does not fit.
///
/// # Safety
///
/// `dst` must be valid for writes of `capacity` bytes and `len` valid for a write.
unsafe fn copy_out(src: &[u8], dst: PUCHAR, capacity: usize, len: PDWORD) -> bool {
    if src.len() > capacity {
        return false;
    }
    // SAFETY: `dst` holds `capacity >= src.len()` bytes, and Rust memory never overlaps a buffer
    // the caller owns. Raw copies are used instead of a slice because the buffer may be
    // uninitialised.
    unsafe {
        ptr::copy_nonoverlapping(src.as_ptr(), dst, src.len());
        len.write(src.len() as DWORD);
    }
    true
}

/// # Safety
///
/// `device_name` must be null or a NUL-terminated string.
unsafe fn create_channel_by_name<C: Card>(
    handler: &Handler<C>,
    lun: DWORD,
    device_name: *const c_char,
    open: impl FnOnce(&Path) -> io::Result<C>,
) -> RESPONSECODE {
    if device_name.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    // SAFETY: Guaranteed by the caller; the string is only borrowed for this call.
    let name = unsafe { CStr::from_ptr(device_name) };
    match open(Path::new(OsStr::from_bytes(name.to_bytes()))) {
        Ok(card) => code(handler.open(lun, card)),
        Err(err) => open_error(&err),
    }
}

/// # Safety
///
/// `length` must be null or valid for reads and writes, and `value` null or valid for writes of
/// `*length` bytes.
unsafe fn get_capabilities<C: Card>(
    handler: &Handler<C>,
    lun: DWORD,
    tag: DWORD,
    length: PDWORD,
    value: PUCHAR,
) -> RESPONSECODE {
    if length.is_null() || value.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    let bytes = match handler.capability(lun, tag) {
        Ok(bytes) => bytes,
        Err(code) => return code,
    };
    // SAFETY: Guaranteed by the caller.
    let capacity = unsafe { length.read() } as usize;
    // SAFETY: Guaranteed by the caller.
    if unsafe { copy_out(bytes.as_slice(), value, capacity, length) } {
        IFD_SUCCESS
    } else {
        // Tells the caller how much room the value needs, as SCardGetAttrib() does.
        // SAFETY: Guaranteed by the caller.
        unsafe { length.write(bytes.as_slice().len() as DWORD) };
        IFD_ERROR_INSUFFICIENT_BUFFER
    }
}

/// # Safety
///
/// `atr_length` must be null or valid for reads and writes, and `atr` null or valid for writes
/// of `*atr_length` bytes.
unsafe fn power_icc<C: Card>(
    handler: &Handler<C>,
    lun: DWORD,
    action: DWORD,
    atr: PUCHAR,
    atr_length: PDWORD,
) -> RESPONSECODE {
    if atr_length.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    // The length is in/out: pcscd passes the size of its ATR buffer.
    // SAFETY: Guaranteed by the caller.
    let capacity = unsafe { atr_length.read() } as usize;
    // Zero on every path but success, including a power-down, which has no ATR.
    // SAFETY: Guaranteed by the caller.
    unsafe { atr_length.write(0) };
    if atr.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    match handler.power(lun, action) {
        Ok(None) => IFD_SUCCESS,
        Ok(Some(new)) => {
            // SAFETY: Guaranteed by the caller.
            if unsafe { copy_out(new.as_slice(), atr, capacity, atr_length) } {
                IFD_SUCCESS
            } else {
                IFD_ERROR_INSUFFICIENT_BUFFER
            }
        }
        Err(code) => code,
    }
}

/// # Safety
///
/// `tx_buffer` must be null or valid for reads of `tx_length` bytes, `rx_length` null or valid
/// for reads and writes, and `rx_buffer` null or valid for writes of `*rx_length` bytes.
#[allow(clippy::too_many_arguments)]
unsafe fn transmit_to_icc<C: Card>(
    handler: &Handler<C>,
    lun: DWORD,
    send_pci: ScardIoHeader,
    tx_buffer: PUCHAR,
    tx_length: DWORD,
    rx_buffer: PUCHAR,
    rx_length: PDWORD,
) -> RESPONSECODE {
    if rx_length.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    // SAFETY: Guaranteed by the caller.
    let capacity = unsafe { rx_length.read() } as usize;
    // The IFD API wants RxLength zero on every error.
    // SAFETY: Guaranteed by the caller.
    unsafe { rx_length.write(0) };
    if tx_buffer.is_null() || rx_buffer.is_null() {
        return IFD_COMMUNICATION_ERROR;
    }
    // SAFETY: Guaranteed by the caller; pcscd's command buffer is initialised up to TxLength.
    let command = unsafe { std::slice::from_raw_parts(tx_buffer, tx_length as usize) };
    match handler.transmit(lun, send_pci.protocol, command) {
        Ok(response) => {
            // SAFETY: Guaranteed by the caller.
            if unsafe { copy_out(response.as_slice(), rx_buffer, capacity, rx_length) } {
                IFD_SUCCESS
            } else {
                // The card has already executed the command; its answer is lost.
                IFD_ERROR_INSUFFICIENT_BUFFER
            }
        }
        Err(code) => code,
    }
}

/// # Safety
///
/// `bytes_returned` must be null or valid for a write.
unsafe fn control<C: Card>(
    handler: &Handler<C>,
    lun: DWORD,
    bytes_returned: PDWORD,
) -> RESPONSECODE {
    if !bytes_returned.is_null() {
        // Nothing is ever returned; the IFD API wants zero on error.
        // SAFETY: Guaranteed by the caller.
        unsafe { bytes_returned.write(0) };
    }
    code(handler.control(lun))
}

fn icc_presence<C: Card>(handler: &Handler<C>, lun: DWORD) -> RESPONSECODE {
    match handler.presence(lun) {
        Ok(true) => IFD_ICC_PRESENT,
        Ok(false) => IFD_ICC_NOT_PRESENT,
        Err(code) => code,
    }
}

/// Opens the card reader at the path `DeviceName` (the `DEVICENAME` of `reader.conf`).
///
/// # Safety
///
/// `DeviceName` must be null or a NUL-terminated string.
#[no_mangle]
pub unsafe extern "C" fn IFDHCreateChannelByName(Lun: DWORD, DeviceName: LPSTR) -> RESPONSECODE {
    // SAFETY: Forwarded from the caller.
    guard(|| unsafe { create_channel_by_name(&HANDLER, Lun, DeviceName, CharDevice::open) })
}

/// Channel numbers (`CHANNELID`) are not supported: the reader is addressed by its device node,
/// so `reader.conf` must give a `DEVICENAME`, which makes pcscd call
/// [`IFDHCreateChannelByName`] instead.
#[no_mangle]
pub extern "C" fn IFDHCreateChannel(_Lun: DWORD, _Channel: DWORD) -> RESPONSECODE {
    IFD_NOT_SUPPORTED
}

#[no_mangle]
pub extern "C" fn IFDHCloseChannel(Lun: DWORD) -> RESPONSECODE {
    guard(|| code(HANDLER.close(Lun)))
}

/// # Safety
///
/// `Length` must be null or valid for reads and writes, and `Value` null or valid for writes of
/// `*Length` bytes.
#[no_mangle]
pub unsafe extern "C" fn IFDHGetCapabilities(
    Lun: DWORD,
    Tag: DWORD,
    Length: PDWORD,
    Value: PUCHAR,
) -> RESPONSECODE {
    // SAFETY: Forwarded from the caller.
    guard(|| unsafe { get_capabilities(&HANDLER, Lun, Tag, Length, Value) })
}

/// No tag is writable, so `Length` and `Value` are not read.
#[no_mangle]
pub extern "C" fn IFDHSetCapabilities(
    Lun: DWORD,
    Tag: DWORD,
    _Length: DWORD,
    _Value: PUCHAR,
) -> RESPONSECODE {
    guard(|| code(HANDLER.set_capability(Lun, Tag)))
}

/// See [`Handler::set_protocol`]. The PTS2 and PTS3 values are not read because requesting
/// either of them fails regardless of the value.
#[no_mangle]
pub extern "C" fn IFDHSetProtocolParameters(
    Lun: DWORD,
    Protocol: DWORD,
    Flags: UCHAR,
    PTS1: UCHAR,
    _PTS2: UCHAR,
    _PTS3: UCHAR,
) -> RESPONSECODE {
    guard(|| code(HANDLER.set_protocol(Lun, Protocol, Flags, PTS1)))
}

/// # Safety
///
/// `AtrLength` must be null or valid for reads and writes, and `Atr` null or valid for writes of
/// `*AtrLength` bytes.
#[no_mangle]
pub unsafe extern "C" fn IFDHPowerICC(
    Lun: DWORD,
    Action: DWORD,
    Atr: PUCHAR,
    AtrLength: PDWORD,
) -> RESPONSECODE {
    // SAFETY: Forwarded from the caller.
    guard(|| unsafe { power_icc(&HANDLER, Lun, Action, Atr, AtrLength) })
}

/// `RecvPci` is left alone: pcscd copies it back into the application's `SCARD_IO_REQUEST`,
/// whose `dwProtocol` is a `SCARD_PROTOCOL_*` mask, so storing the protocol number there would
/// corrupt it.
///
/// # Safety
///
/// `TxBuffer` must be null or valid for reads of `TxLength` bytes, `RxLength` null or valid for
/// reads and writes, and `RxBuffer` null or valid for writes of `*RxLength` bytes.
#[no_mangle]
pub unsafe extern "C" fn IFDHTransmitToICC(
    Lun: DWORD,
    SendPci: ScardIoHeader,
    TxBuffer: PUCHAR,
    TxLength: DWORD,
    RxBuffer: PUCHAR,
    RxLength: PDWORD,
    _RecvPci: *mut ScardIoHeader,
) -> RESPONSECODE {
    // SAFETY: Forwarded from the caller.
    guard(|| unsafe {
        transmit_to_icc(
            &HANDLER, Lun, SendPci, TxBuffer, TxLength, RxBuffer, RxLength,
        )
    })
}

/// # Safety
///
/// `pdwBytesReturned` must be null or valid for a write. The buffers are not accessed.
#[no_mangle]
pub unsafe extern "C" fn IFDHControl(
    Lun: DWORD,
    _dwControlCode: DWORD,
    _TxBuffer: PUCHAR,
    _TxLength: DWORD,
    _RxBuffer: PUCHAR,
    _RxLength: DWORD,
    pdwBytesReturned: PDWORD,
) -> RESPONSECODE {
    // SAFETY: Forwarded from the caller.
    guard(|| unsafe { control(&HANDLER, Lun, pdwBytesReturned) })
}

#[no_mangle]
pub extern "C" fn IFDHICCPresence(Lun: DWORD) -> RESPONSECODE {
    guard(|| icc_presence(&HANDLER, Lun))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake::{FakeCard, BCAS_ATR};
    use crate::pcsc::{
        IFD_ERROR_NOT_SUPPORTED, IFD_ERROR_POWER_ACTION, IFD_ERROR_TAG, IFD_POWER_DOWN,
        IFD_POWER_UP, MAX_ATR_SIZE, TAG_IFD_ATR, TAG_IFD_THREAD_SAFE,
    };

    const LUN: DWORD = 0;
    const T1: ScardIoHeader = ScardIoHeader {
        protocol: 1,
        length: 8,
    };
    const SENTINEL: u8 = 0xEE;

    fn open() -> (Handler<FakeCard>, FakeCard) {
        let card = FakeCard::default();
        let handler = Handler::new();
        handler.open(LUN, card.clone()).unwrap();
        (handler, card)
    }

    /// Calls `power_icc` with a buffer that announces `capacity` bytes but is larger, and
    /// returns the code, the reported length and the whole buffer.
    fn power(
        handler: &Handler<FakeCard>,
        action: DWORD,
        capacity: usize,
    ) -> (RESPONSECODE, DWORD, Vec<u8>) {
        let mut buf = vec![SENTINEL; MAX_ATR_SIZE + 8];
        let mut len = capacity as DWORD;
        // SAFETY: `buf` holds more than `capacity` bytes.
        let code = unsafe { power_icc(handler, LUN, action, buf.as_mut_ptr(), &mut len) };
        (code, len, buf)
    }

    fn transmit(
        handler: &Handler<FakeCard>,
        pci: ScardIoHeader,
        command: &[u8],
        capacity: usize,
    ) -> (RESPONSECODE, DWORD, Vec<u8>) {
        let mut buf = vec![SENTINEL; capacity + 8];
        let mut len = capacity as DWORD;
        let mut command = command.to_vec();
        // SAFETY: `command` holds its length in bytes and `buf` more than `capacity`.
        let code = unsafe {
            transmit_to_icc(
                handler,
                LUN,
                pci,
                command.as_mut_ptr(),
                command.len() as DWORD,
                buf.as_mut_ptr(),
                &mut len,
            )
        };
        (code, len, buf)
    }

    #[test]
    fn power_icc_writes_the_atr_within_the_buffer() {
        let (handler, _card) = open();
        let (code, len, buf) = power(&handler, IFD_POWER_UP, MAX_ATR_SIZE);
        assert_eq!(code, IFD_SUCCESS);
        assert_eq!(&buf[..len as usize], BCAS_ATR);
        assert!(buf[BCAS_ATR.len()..].iter().all(|&b| b == SENTINEL));

        // Exactly enough room.
        let (code, len, _) = power(&handler, IFD_POWER_UP, BCAS_ATR.len());
        assert_eq!((code, len), (IFD_SUCCESS, BCAS_ATR.len() as DWORD));

        // One byte short: nothing is written and the length is zero.
        let (code, len, buf) = power(&handler, IFD_POWER_UP, BCAS_ATR.len() - 1);
        assert_eq!((code, len), (IFD_ERROR_INSUFFICIENT_BUFFER, 0));
        assert!(buf.iter().all(|&b| b == SENTINEL));
    }

    #[test]
    fn power_icc_zeroes_the_length_unless_an_atr_is_returned() {
        let (handler, card) = open();
        let (code, len, buf) = power(&handler, IFD_POWER_DOWN, MAX_ATR_SIZE);
        assert_eq!((code, len), (IFD_SUCCESS, 0));
        assert!(buf.iter().all(|&b| b == SENTINEL));

        card.state().reset_error = Some(libc::EIO);
        let (code, len, _) = power(&handler, IFD_POWER_UP, MAX_ATR_SIZE);
        assert_eq!((code, len), (IFD_ERROR_POWER_ACTION, 0));

        let mut len = MAX_ATR_SIZE as DWORD;
        // SAFETY: A null ATR buffer is rejected before use.
        let code = unsafe { power_icc(&handler, LUN, IFD_POWER_UP, ptr::null_mut(), &mut len) };
        assert_eq!((code, len), (IFD_COMMUNICATION_ERROR, 0));
        // SAFETY: A null length is rejected before use.
        let code = unsafe {
            power_icc(
                &handler,
                LUN,
                IFD_POWER_UP,
                [0u8; 1].as_mut_ptr(),
                ptr::null_mut(),
            )
        };
        assert_eq!(code, IFD_COMMUNICATION_ERROR);
    }

    #[test]
    fn transmit_writes_the_response_within_the_buffer() {
        let (handler, card) = open();
        power(&handler, IFD_POWER_UP, MAX_ATR_SIZE);
        let response = [0x12, 0x34, 0x56, 0x90, 0x00];
        card.state().response = response.to_vec();

        let (code, len, buf) = transmit(&handler, T1, &[0x90, 0x30, 0, 0, 0], 258);
        assert_eq!((code, len), (IFD_SUCCESS, 5));
        assert_eq!(&buf[..5], response);
        assert!(buf[5..].iter().all(|&b| b == SENTINEL));

        let (code, len, _) = transmit(&handler, T1, &[0x90, 0x30, 0, 0, 0], 5);
        assert_eq!((code, len), (IFD_SUCCESS, 5));

        let (code, len, buf) = transmit(&handler, T1, &[0x90, 0x30, 0, 0, 0], 4);
        assert_eq!((code, len), (IFD_ERROR_INSUFFICIENT_BUFFER, 0));
        assert!(buf.iter().all(|&b| b == SENTINEL));
    }

    #[test]
    fn transmit_zeroes_the_length_on_error() {
        let (handler, card) = open();
        // No session.
        let (code, len, buf) = transmit(&handler, T1, &[0x90, 0x30, 0, 0, 0], 258);
        assert_eq!((code, len), (IFD_COMMUNICATION_ERROR, 0));
        assert!(buf.iter().all(|&b| b == SENTINEL));

        power(&handler, IFD_POWER_UP, MAX_ATR_SIZE);
        card.state().present = false;
        let (code, len, _) = transmit(&handler, T1, &[0x90, 0x30, 0, 0, 0], 258);
        assert_eq!((code, len), (IFD_ICC_NOT_PRESENT, 0));

        let t0 = ScardIoHeader {
            protocol: 0,
            length: 8,
        };
        let (code, len, _) = transmit(&handler, t0, &[0x90, 0x30, 0, 0, 0], 258);
        assert_eq!((code, len), (crate::pcsc::IFD_PROTOCOL_NOT_SUPPORTED, 0));

        let mut len = 258;
        let mut rx = [0u8; 258];
        // SAFETY: A null command is rejected before use.
        let code = unsafe {
            transmit_to_icc(
                &handler,
                LUN,
                T1,
                ptr::null_mut(),
                5,
                rx.as_mut_ptr(),
                &mut len,
            )
        };
        assert_eq!((code, len), (IFD_COMMUNICATION_ERROR, 0));
    }

    #[test]
    fn get_capabilities_respects_and_reports_the_length() {
        let (handler, _card) = open();
        power(&handler, IFD_POWER_UP, MAX_ATR_SIZE);
        let mut buf = [SENTINEL; MAX_ATR_SIZE + 8];
        let mut len = MAX_ATR_SIZE as DWORD;
        // SAFETY: `buf` holds more than `len` bytes.
        let code =
            unsafe { get_capabilities(&handler, LUN, TAG_IFD_ATR, &mut len, buf.as_mut_ptr()) };
        assert_eq!((code, len), (IFD_SUCCESS, BCAS_ATR.len() as DWORD));
        assert_eq!(&buf[..BCAS_ATR.len()], BCAS_ATR);
        assert!(buf[BCAS_ATR.len()..].iter().all(|&b| b == SENTINEL));

        let mut buf = [SENTINEL; MAX_ATR_SIZE];
        let mut len = 4;
        // SAFETY: As above.
        let code =
            unsafe { get_capabilities(&handler, LUN, TAG_IFD_ATR, &mut len, buf.as_mut_ptr()) };
        assert_eq!(
            (code, len),
            (IFD_ERROR_INSUFFICIENT_BUFFER, BCAS_ATR.len() as DWORD)
        );
        assert!(buf.iter().all(|&b| b == SENTINEL));

        let mut byte = [SENTINEL; 2];
        let mut len = 1;
        // SAFETY: As above.
        let code = unsafe {
            get_capabilities(
                &handler,
                LUN,
                TAG_IFD_THREAD_SAFE,
                &mut len,
                byte.as_mut_ptr(),
            )
        };
        assert_eq!((code, len, byte), (IFD_SUCCESS, 1, [1, SENTINEL]));

        // pcscd asks for TAG_IFD_POLLING_THREAD_WITH_TIMEOUT with a function pointer as buffer.
        let mut function = [SENTINEL; 8];
        let mut len = 8;
        // SAFETY: As above.
        let code =
            unsafe { get_capabilities(&handler, LUN, 0x0FB3, &mut len, function.as_mut_ptr()) };
        assert_eq!((code, len, function), (IFD_ERROR_TAG, 8, [SENTINEL; 8]));
    }

    #[test]
    fn control_returns_nothing() {
        let (handler, _card) = open();
        let mut returned = 77;
        // SAFETY: `returned` is a valid DWORD.
        let code = unsafe { control(&handler, LUN, &mut returned) };
        assert_eq!((code, returned), (IFD_ERROR_NOT_SUPPORTED, 0));
    }

    #[test]
    fn presence_codes() {
        let (handler, card) = open();
        assert_eq!(icc_presence(&handler, LUN), IFD_ICC_PRESENT);
        card.state().present = false;
        assert_eq!(icc_presence(&handler, LUN), IFD_ICC_NOT_PRESENT);
        card.state().gone = true;
        assert_eq!(icc_presence(&handler, LUN), IFD_NO_SUCH_DEVICE);
        assert_eq!(icc_presence(&handler, 5), IFD_COMMUNICATION_ERROR);
    }

    #[test]
    fn create_channel_by_name_opens_real_nodes() {
        let handler = Handler::<CharDevice>::new();
        let open = |name: &CStr| {
            // SAFETY: `name` is NUL-terminated.
            unsafe { create_channel_by_name(&handler, LUN, name.as_ptr(), CharDevice::open) }
        };
        assert_eq!(open(c"/dev/pxw3pe-card-nonexistent"), IFD_NO_SUCH_DEVICE);
        // A character device of another driver answers the status ioctl with ENOTTY.
        assert_eq!(open(c"/dev/null"), IFD_COMMUNICATION_ERROR);
        assert_eq!(icc_presence(&handler, LUN), IFD_COMMUNICATION_ERROR);
        // SAFETY: Null is rejected before use.
        let code = unsafe { create_channel_by_name(&handler, LUN, ptr::null(), CharDevice::open) };
        assert_eq!(code, IFD_COMMUNICATION_ERROR);
    }

    #[test]
    fn exported_entry_points_without_a_reader() {
        // A Lun pcscd never opened in this process.
        let lun = 0x0007_0000;
        // SAFETY: The name is NUL-terminated.
        let code = unsafe {
            IFDHCreateChannelByName(lun, c"/dev/pxw3pe-card-nonexistent".as_ptr().cast_mut())
        };
        assert_eq!(code, IFD_NO_SUCH_DEVICE);
        assert_eq!(IFDHCreateChannel(lun, 1), IFD_NOT_SUPPORTED);
        assert_eq!(IFDHICCPresence(lun), IFD_COMMUNICATION_ERROR);
        assert_eq!(IFDHCloseChannel(lun), IFD_COMMUNICATION_ERROR);
    }

    #[test]
    fn a_panic_becomes_an_error_code() {
        assert_eq!(guard(|| panic!("handler bug")), IFD_COMMUNICATION_ERROR);
        assert_eq!(guard(|| IFD_SUCCESS), IFD_SUCCESS);
    }
}
