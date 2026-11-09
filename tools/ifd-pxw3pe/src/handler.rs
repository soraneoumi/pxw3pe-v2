//! The IFD handler logic: which kernel call serves which IFD request, and what each outcome
//! means to pcscd. It is independent of the C ABI (see `ffi`) and of the device node (see
//! [`Card`]).
//!
//! # Concurrency
//!
//! Readers are kept in a table keyed by Lun. The table lock is held only to look a reader up,
//! never across a kernel call; each reader has its own lock, held for the whole of every call on
//! it. Calls on different readers therefore run in parallel and calls on one reader are
//! serialised, which is what `TAG_IFD_THREAD_SAFE = 1` promises pcscd.

use crate::device::Card;
use crate::pcsc::{
    DWORD, IFD_COMMUNICATION_ERROR, IFD_ERROR_NOT_SUPPORTED, IFD_ERROR_POWER_ACTION,
    IFD_ERROR_PTS_FAILURE, IFD_ERROR_TAG, IFD_ERROR_VALUE_READ_ONLY, IFD_ICC_NOT_PRESENT,
    IFD_NEGOTIATE_PTS1, IFD_NOT_SUPPORTED, IFD_NO_SUCH_DEVICE, IFD_POWER_DOWN, IFD_POWER_UP,
    IFD_PROTOCOL_NOT_SUPPORTED, IFD_RESET, MAX_ATR_SIZE, RESPONSECODE, SCARD_ATTR_ATR_STRING,
    SCARD_PROTOCOL_T1, TAG_IFD_ATR, TAG_IFD_SIMULTANEOUS_ACCESS, TAG_IFD_SLOTS_NUMBER,
    TAG_IFD_SLOT_THREAD_SAFE, TAG_IFD_THREAD_SAFE, UCHAR,
};
use crate::uapi::{self, CardTransfer};
use std::collections::btree_map::{BTreeMap, Entry};
use std::io;
use std::sync::{Arc, Mutex};

/// Readers one loaded handler serves: one per `/dev/pxw3pe-cardN` node the kernel driver can
/// create. pcscd gives the readers configured with this library distinct Luns only while this
/// count allows it; with 1, a second reader would get the first one's Lun.
pub const MAX_READERS: u8 = 8;

/// `SCARD_IO_HEADER.Protocol` of T=1. It is a protocol number, unlike the `SCARD_PROTOCOL_*` bit
/// masks used everywhere else: pcscd converts `SCARD_PROTOCOL_T1` to 1 before the call.
const PCI_PROTOCOL_T1: DWORD = 1;

/// TA1 when the ATR has none: Fi = 372, Di = 1 (ISO/IEC 7816-3 8.3).
const DEFAULT_TA1: u8 = 0x11;

/// Outcome of an IFD request; the error is the response code pcscd receives.
pub type IfdResult<T> = Result<T, RESPONSECODE>;

/// A byte string of at most `N` bytes, held inline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bytes<const N: usize> {
    buf: [u8; N],
    /// Invariant: `len <= N`.
    len: usize,
}

impl<const N: usize> Bytes<N> {
    const EMPTY: Self = Self {
        buf: [0; N],
        len: 0,
    };

    fn byte(byte: u8) -> Self {
        const { assert!(N > 0) };
        let mut buf = [0; N];
        buf[0] = byte;
        Self { buf, len: 1 }
    }

    fn new(src: &[u8]) -> Option<Self> {
        let mut buf = [0; N];
        buf.get_mut(..src.len())?.copy_from_slice(src);
        Some(Self {
            buf,
            len: src.len(),
        })
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

pub type Atr = Bytes<MAX_ATR_SIZE>;
pub type Response = Bytes<{ uapi::APDU_MAX }>;

const _: () = assert!(uapi::ATR_MAX == MAX_ATR_SIZE);

/// The TA1 in effect for `atr`, or `None` if T0 announces a TA1 that the ATR does not contain.
fn ta1(atr: &[u8]) -> Option<u8> {
    let t0 = *atr.get(1)?;
    if t0 & 0x10 == 0 {
        Some(DEFAULT_TA1)
    } else {
        atr.get(2).copied()
    }
}

/// What a status call says after a card operation failed in a way that is ambiguous on its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Slot {
    Present,
    Absent,
    DeviceGone,
    /// The status call failed for another reason.
    Unknown,
}

fn is_errno(err: &io::Error, errno: i32) -> bool {
    err.raw_os_error() == Some(errno)
}

/// The response code for a failed status call. There, ENODEV can only mean that the board is
/// gone: a missing card is a flag in the result, not an error.
fn status_error(err: &io::Error) -> RESPONSECODE {
    if is_errno(err, libc::ENODEV) {
        IFD_NO_SUCH_DEVICE
    } else {
        IFD_COMMUNICATION_ERROR
    }
}

struct Reader<C> {
    card: C,
    /// ATR of the last successful reset; `None` before the first one and once the card is
    /// powered down, fails to reset or is seen to be absent or replaced. A session the kernel
    /// ended after a transmission error keeps it: the card is still the same.
    atr: Option<Atr>,
    /// The kernel reported that a session ended with the card's removal, and pcscd's presence
    /// poll has not been told yet.
    removal_pending: bool,
}

impl<C: Card> Reader<C> {
    fn present(&mut self) -> io::Result<bool> {
        let flags = self.card.status()?;
        // The kernel reports a removal once, to whichever call asks first; it is kept for the
        // presence poll, which is what tells pcscd.
        if flags & uapi::REMOVED != 0 {
            self.removal_pending = true;
            self.atr = None;
        }
        let present = flags & uapi::PRESENT != 0;
        if !present {
            self.atr = None;
        }
        Ok(present)
    }

    /// Serves pcscd's presence poll. A card that was pulled and put back (or replaced) between two
    /// polls is reported absent once: pcscd only notices a removal as a change of presence, and
    /// without one it would keep the card's old state and handles.
    fn presence(&mut self) -> io::Result<bool> {
        let present = self.present()?;
        // Taken whatever the answer: an absent card reports the removal itself.
        let pending = std::mem::take(&mut self.removal_pending);
        Ok(present && !pending)
    }

    /// Asks the kernel what a failed card operation meant: it reports both "no card" and "device
    /// gone" as ENODEV, and a session that ended with the card's removal as ENOTCONN.
    fn slot(&mut self) -> Slot {
        match self.present() {
            Ok(true) => Slot::Present,
            Ok(false) => Slot::Absent,
            Err(err) if is_errno(&err, libc::ENODEV) => Slot::DeviceGone,
            Err(_) => Slot::Unknown,
        }
    }

    /// The response code for a failed reset or power-off.
    fn power_error(&mut self, err: &io::Error) -> RESPONSECODE {
        if is_errno(err, libc::ENODEV) && self.slot() == Slot::DeviceGone {
            IFD_NO_SUCH_DEVICE
        } else {
            IFD_ERROR_POWER_ACTION
        }
    }

    fn reset(&mut self) -> IfdResult<Atr> {
        self.atr = None;
        let raw = self.card.reset().map_err(|err| self.power_error(&err))?;
        // The UAPI promises at most ATR_MAX bytes; TS and T0 are mandatory (ISO/IEC 7816-3 8.2).
        let atr = raw
            .atr
            .get(..raw.len as usize)
            .filter(|atr| atr.len() >= 2)
            .and_then(Atr::new)
            .ok_or(IFD_COMMUNICATION_ERROR)?;
        self.atr = Some(atr);
        Ok(atr)
    }

    fn power_off(&mut self) -> IfdResult<()> {
        self.atr = None;
        self.card.power_off().map_err(|err| self.power_error(&err))
    }

    fn transmit(&mut self, command: &[u8]) -> IfdResult<Response> {
        // A command without the CLA INS P1 P2 header is no APDU, and an over-long one does not fit
        // in a block; the kernel could only reject them as well.
        if !(uapi::APDU_MIN..=uapi::APDU_MAX).contains(&command.len()) {
            return Err(IFD_COMMUNICATION_ERROR);
        }
        let mut transfer = CardTransfer::zeroed();
        transfer.command[..command.len()].copy_from_slice(command);
        transfer.command_len = command.len() as u32;
        if let Err(err) = self.card.transmit(&mut transfer) {
            return Err(self.transmit_error(&err));
        }
        // The response includes SW1 SW2 and fits in one block; anything else breaks the UAPI.
        transfer
            .response
            .get(..transfer.response_len as usize)
            .filter(|response| response.len() >= 2)
            .and_then(Response::new)
            .ok_or(IFD_COMMUNICATION_ERROR)
    }

    /// The response code for a failed `PXW3PE_CARD_TRANSMIT`, by the errno values its UAPI
    /// documents.
    fn transmit_error(&mut self, err: &io::Error) -> RESPONSECODE {
        match err.raw_os_error() {
            // EMSGSIZE: longer than the card's IFSC (ARIB STD-B25 excludes chaining).
            // EIO: the kernel lost the T=1 session and ended it; the card stays present, and the
            // application has to reset it (SCardReconnect) before the next command.
            Some(libc::EMSGSIZE | libc::EIO) => IFD_COMMUNICATION_ERROR,
            // No session. If the card is gone, the session ended with its removal: pcscd turns
            // IFD_ICC_NOT_PRESENT into SCARD_E_NO_SMARTCARD and marks the reader empty, which is
            // the truth. If the card is still there, it was powered down or lost its session
            // earlier; claiming it absent would be false, so this is a communication error.
            Some(libc::ENOTCONN) => match self.slot() {
                Slot::Absent => IFD_ICC_NOT_PRESENT,
                Slot::DeviceGone => IFD_NO_SUCH_DEVICE,
                Slot::Present | Slot::Unknown => IFD_COMMUNICATION_ERROR,
            },
            // No card, or the board is gone. A card that is present now was reinserted after the
            // exchange failed and has not been reset. Reporting it absent here would be wrong:
            // pcscd would mark the reader empty, and its presence poll, seeing the card, would
            // not mark it full again. The removal reaches pcscd through that poll instead: the
            // kernel reported REMOVED to the status call `slot` made, and `present` kept it.
            Some(libc::ENODEV) => match self.slot() {
                Slot::DeviceGone => IFD_NO_SUCH_DEVICE,
                Slot::Unknown => IFD_COMMUNICATION_ERROR,
                Slot::Absent => IFD_ICC_NOT_PRESENT,
                Slot::Present => {
                    // The ATR belonged to the card that was pulled.
                    self.atr = None;
                    IFD_COMMUNICATION_ERROR
                }
            },
            // Everything else is a communication error. EINVAL is a command that is no APDU,
            // which `transmit` rules out before the call. EINTR means that this process was
            // killed while the call ran; nobody is left to report to, and if the exchange had
            // begun, the kernel ended the session.
            _ => IFD_COMMUNICATION_ERROR,
        }
    }
}

/// The readers of one loaded handler, keyed by Lun.
pub struct Handler<C> {
    readers: Mutex<BTreeMap<DWORD, Arc<Mutex<Reader<C>>>>>,
}

impl<C: Card> Handler<C> {
    pub const fn new() -> Self {
        Self {
            readers: Mutex::new(BTreeMap::new()),
        }
    }

    /// Registers `card` as the reader `lun`.
    pub fn open(&self, lun: DWORD, mut card: C) -> IfdResult<()> {
        // pcscd polls the presence as soon as the channel is open. Asking once now also rejects a
        // node that is not a PX-W3PE card reader (ENOTTY).
        card.status().map_err(|err| status_error(&err))?;
        let mut readers = self.readers.lock().map_err(|_| IFD_COMMUNICATION_ERROR)?;
        match readers.entry(lun) {
            Entry::Occupied(_) => Err(IFD_COMMUNICATION_ERROR),
            Entry::Vacant(entry) => {
                entry.insert(Arc::new(Mutex::new(Reader {
                    card,
                    atr: None,
                    removal_pending: false,
                })));
                Ok(())
            }
        }
    }

    /// Unregisters the reader `lun` and leaves its card unpowered, as the IFD API asks. The
    /// reader is gone afterwards even if the power-off fails.
    pub fn close(&self, lun: DWORD) -> IfdResult<()> {
        let reader = self
            .readers
            .lock()
            .map_err(|_| IFD_COMMUNICATION_ERROR)?
            .remove(&lun)
            .ok_or(IFD_COMMUNICATION_ERROR)?;
        // Waits for a call still running on another thread; the device node is closed when the
        // last reference to the reader goes away.
        let mut reader = reader.lock().map_err(|_| IFD_COMMUNICATION_ERROR)?;
        reader.power_off()
    }

    /// Runs `f` on the reader `lun` under its lock.
    ///
    /// A poisoned lock (a panic interrupted an earlier call) is reported instead of being used,
    /// since the ATR cache may no longer match the card.
    fn with<T>(&self, lun: DWORD, f: impl FnOnce(&mut Reader<C>) -> IfdResult<T>) -> IfdResult<T> {
        let reader = {
            let readers = self.readers.lock().map_err(|_| IFD_COMMUNICATION_ERROR)?;
            readers.get(&lun).cloned().ok_or(IFD_COMMUNICATION_ERROR)?
        };
        let mut reader = reader.lock().map_err(|_| IFD_COMMUNICATION_ERROR)?;
        f(&mut reader)
    }

    /// Whether a card is inserted.
    pub fn presence(&self, lun: DWORD) -> IfdResult<bool> {
        self.with(lun, |reader| {
            reader.presence().map_err(|err| status_error(&err))
        })
    }

    /// Serves `IFDHPowerICC`: the new ATR for a power-up or reset, `None` for a power-down.
    ///
    /// The kernel offers one way to start a session, a reset that activates the card from
    /// scratch, so a power-up and a warm reset are the same request to it.
    pub fn power(&self, lun: DWORD, action: DWORD) -> IfdResult<Option<Atr>> {
        self.with(lun, |reader| match action {
            IFD_POWER_UP | IFD_RESET => reader.reset().map(Some),
            IFD_POWER_DOWN => reader.power_off().map(|()| None),
            _ => Err(IFD_NOT_SUPPORTED),
        })
    }

    /// Serves `IFDHSetProtocolParameters`.
    ///
    /// The card's ATR selects the specific mode for T=1 (TA2) with the Fi/Di of TA1; PPS belongs
    /// to the negotiable mode only (ISO/IEC 7816-3 6.3.1). The kernel accepts no other card, uses
    /// those parameters right after the reset and never sends a PPS, so there is nothing to
    /// negotiate: a request is
    /// accepted only if it asks for what is already in effect, which is T=1 with at most PTS1
    /// equal to TA1. Anything else could only be satisfied by a PPS exchange, so it fails with
    /// IFD_ERROR_PTS_FAILURE and pcscd rejects the connection instead of using parameters the
    /// card is not running with. PTS2 and PTS3 have no counterpart in the ATR, so requesting
    /// either fails whatever its value. pcscd itself always passes no flags.
    pub fn set_protocol(
        &self,
        lun: DWORD,
        protocol: DWORD,
        flags: UCHAR,
        pts1: UCHAR,
    ) -> IfdResult<()> {
        self.with(lun, |reader| {
            if protocol != SCARD_PROTOCOL_T1 {
                return Err(IFD_PROTOCOL_NOT_SUPPORTED);
            }
            let atr = reader.atr.ok_or(IFD_COMMUNICATION_ERROR)?;
            if flags & !IFD_NEGOTIATE_PTS1 != 0 {
                return Err(IFD_ERROR_PTS_FAILURE);
            }
            if flags & IFD_NEGOTIATE_PTS1 != 0 && ta1(atr.as_slice()) != Some(pts1) {
                return Err(IFD_ERROR_PTS_FAILURE);
            }
            Ok(())
        })
    }

    /// Serves `IFDHTransmitToICC`; `protocol` is `SCARD_IO_HEADER.Protocol`.
    pub fn transmit(&self, lun: DWORD, protocol: DWORD, command: &[u8]) -> IfdResult<Response> {
        self.with(lun, |reader| {
            if protocol != PCI_PROTOCOL_T1 {
                return Err(IFD_PROTOCOL_NOT_SUPPORTED);
            }
            reader.transmit(command)
        })
    }

    /// Serves `IFDHGetCapabilities`.
    pub fn capability(&self, lun: DWORD, tag: DWORD) -> IfdResult<Bytes<MAX_ATR_SIZE>> {
        self.with(lun, |reader| {
            let byte = match tag {
                // An empty ATR while no session is open, as for a card without one.
                TAG_IFD_ATR | SCARD_ATTR_ATR_STRING => {
                    return Ok(reader.atr.unwrap_or(Atr::EMPTY));
                }
                TAG_IFD_SLOTS_NUMBER => 1,
                TAG_IFD_SIMULTANEOUS_ACCESS => MAX_READERS,
                // See the module documentation.
                TAG_IFD_THREAD_SAFE => 1,
                // There is a single slot, so pcscd never asks; the truthful answer is "no".
                TAG_IFD_SLOT_THREAD_SAFE => 0,
                _ => return Err(IFD_ERROR_TAG),
            };
            Ok(Bytes::byte(byte))
        })
    }

    /// Serves `IFDHSetCapabilities`: no tag is writable.
    pub fn set_capability(&self, lun: DWORD, tag: DWORD) -> IfdResult<()> {
        self.with(lun, |_| match tag {
            TAG_IFD_ATR
            | SCARD_ATTR_ATR_STRING
            | TAG_IFD_SLOTS_NUMBER
            | TAG_IFD_SIMULTANEOUS_ACCESS
            | TAG_IFD_THREAD_SAFE
            | TAG_IFD_SLOT_THREAD_SAFE => Err(IFD_ERROR_VALUE_READ_ONLY),
            _ => Err(IFD_ERROR_TAG),
        })
    }

    /// Serves `IFDHControl`: the reader has no functions beyond the card (no PIN pad, display or
    /// escape commands). pcscd reports IFD_ERROR_NOT_SUPPORTED as SCARD_E_UNSUPPORTED_FEATURE.
    pub fn control(&self, lun: DWORD) -> IfdResult<()> {
        self.with(lun, |_| Err(IFD_ERROR_NOT_SUPPORTED))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake::{FakeCard, BCAS_ATR};
    use crate::pcsc::{
        IFD_NEGOTIATE_PTS2, IFD_NEGOTIATE_PTS3, SCARD_PROTOCOL_RAW, SCARD_PROTOCOL_T0,
    };
    use std::sync::mpsc;
    use std::thread;
    use std::time::Duration;

    const LUN: DWORD = 0;
    const SELECT: &[u8] = &[0x90, 0x30, 0x00, 0x00, 0x00];

    fn open() -> (Handler<FakeCard>, FakeCard) {
        let card = FakeCard::default();
        let handler = Handler::new();
        handler.open(LUN, card.clone()).unwrap();
        (handler, card)
    }

    fn powered() -> (Handler<FakeCard>, FakeCard) {
        let (handler, card) = open();
        handler.power(LUN, IFD_POWER_UP).unwrap();
        (handler, card)
    }

    fn atr_capability(handler: &Handler<FakeCard>) -> Vec<u8> {
        handler
            .capability(LUN, TAG_IFD_ATR)
            .unwrap()
            .as_slice()
            .to_vec()
    }

    #[test]
    fn open_probes_status_and_rejects_bad_nodes() {
        let (handler, card) = open();
        assert_eq!(card.state().calls, ["status"]);
        // The same Lun twice.
        assert_eq!(
            handler.open(LUN, FakeCard::default()),
            Err(IFD_COMMUNICATION_ERROR)
        );

        let other = FakeCard::default();
        other.state().status_error = Some(libc::ENOTTY);
        assert_eq!(handler.open(1, other), Err(IFD_COMMUNICATION_ERROR));
        let gone = FakeCard::default();
        gone.state().gone = true;
        assert_eq!(handler.open(2, gone), Err(IFD_NO_SUCH_DEVICE));
        assert_eq!(handler.presence(1), Err(IFD_COMMUNICATION_ERROR));
    }

    #[test]
    fn unknown_lun_is_an_error_everywhere() {
        let (handler, _card) = open();
        let lun = 0x0001_0000;
        assert_eq!(handler.presence(lun), Err(IFD_COMMUNICATION_ERROR));
        assert_eq!(
            handler.power(lun, IFD_POWER_UP),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.transmit(lun, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.set_protocol(lun, SCARD_PROTOCOL_T1, 0, 0),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.capability(lun, TAG_IFD_ATR),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.set_capability(lun, TAG_IFD_ATR),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(handler.control(lun), Err(IFD_COMMUNICATION_ERROR));
        assert_eq!(handler.close(lun), Err(IFD_COMMUNICATION_ERROR));
    }

    #[test]
    fn presence_maps_status() {
        let (handler, card) = powered();
        assert_eq!(handler.presence(LUN), Ok(true));
        assert_eq!(atr_capability(&handler), BCAS_ATR);

        card.state().present = false;
        assert_eq!(handler.presence(LUN), Ok(false));
        // The ATR belonged to the card that was removed.
        assert_eq!(atr_capability(&handler), b"");

        card.state().gone = true;
        assert_eq!(handler.presence(LUN), Err(IFD_NO_SUCH_DEVICE));
        card.state().gone = false;
        card.state().status_error = Some(libc::EFAULT);
        assert_eq!(handler.presence(LUN), Err(IFD_COMMUNICATION_ERROR));
    }

    #[test]
    fn a_removal_the_kernel_noticed_is_reported_to_the_presence_poll_once() {
        let (handler, card) = powered();
        // Pulled and put back between two polls; the kernel ended the session.
        card.state().removed = true;
        card.state().session = false;
        assert_eq!(handler.presence(LUN), Ok(false));
        assert_eq!(atr_capability(&handler), b"");
        assert_eq!(handler.presence(LUN), Ok(true));
    }

    #[test]
    fn a_removal_reported_while_the_card_is_out_does_not_hide_its_return() {
        let (handler, card) = powered();
        {
            let mut s = card.state();
            s.removed = true;
            s.present = false;
        }
        assert_eq!(handler.presence(LUN), Ok(false));
        card.state().present = true;
        assert_eq!(handler.presence(LUN), Ok(true));
    }

    #[test]
    fn a_removal_seen_by_a_failed_transmit_still_reaches_the_presence_poll() {
        let (handler, card) = powered();
        {
            let mut s = card.state();
            s.removed = true;
            s.session = false;
        }
        // The card in the slot was not reset, so the command fails without claiming it absent.
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(handler.presence(LUN), Ok(false));
        assert_eq!(handler.presence(LUN), Ok(true));
    }

    #[test]
    fn power_up_and_reset_return_the_atr() {
        let (handler, card) = open();
        for action in [IFD_POWER_UP, IFD_RESET] {
            let atr = handler.power(LUN, action).unwrap().unwrap();
            assert_eq!(atr.as_slice(), BCAS_ATR);
            assert_eq!(atr_capability(&handler), BCAS_ATR);
        }
        assert_eq!(card.state().calls, ["status", "reset", "reset"]);
        assert!(card.state().session);
    }

    #[test]
    fn power_down_powers_off_and_forgets_the_atr() {
        let (handler, card) = powered();
        assert_eq!(handler.power(LUN, IFD_POWER_DOWN), Ok(None));
        assert_eq!(card.state().calls.last(), Some(&"power_off"));
        assert!(!card.state().session);
        assert_eq!(atr_capability(&handler), b"");

        card.state().gone = true;
        assert_eq!(handler.power(LUN, IFD_POWER_DOWN), Err(IFD_NO_SUCH_DEVICE));
    }

    #[test]
    fn unknown_power_action_is_not_supported() {
        let (handler, card) = open();
        assert_eq!(handler.power(LUN, 503), Err(IFD_NOT_SUPPORTED));
        assert_eq!(card.state().calls, ["status"]);
    }

    #[test]
    fn failed_reset() {
        let (handler, card) = powered();
        card.state().reset_error = Some(libc::EIO);
        assert_eq!(handler.power(LUN, IFD_RESET), Err(IFD_ERROR_POWER_ACTION));
        assert_eq!(atr_capability(&handler), b"");

        // A rejected ATR.
        card.state().reset_error = Some(libc::EPROTO);
        assert_eq!(
            handler.power(LUN, IFD_POWER_UP),
            Err(IFD_ERROR_POWER_ACTION)
        );

        // ENODEV with the board still there: the card is missing.
        card.state().reset_error = None;
        card.state().present = false;
        assert_eq!(
            handler.power(LUN, IFD_POWER_UP),
            Err(IFD_ERROR_POWER_ACTION)
        );
        assert_eq!(card.state().calls.last(), Some(&"status"));

        // ENODEV with the board gone.
        card.state().gone = true;
        assert_eq!(handler.power(LUN, IFD_POWER_UP), Err(IFD_NO_SUCH_DEVICE));
    }

    #[test]
    fn malformed_atr_from_the_kernel_is_rejected() {
        for len in [0, 1, 34, u32::MAX] {
            let (handler, card) = open();
            card.state().atr_len = Some(len);
            assert_eq!(
                handler.power(LUN, IFD_POWER_UP),
                Err(IFD_COMMUNICATION_ERROR),
                "{len}"
            );
            assert_eq!(atr_capability(&handler), b"");
        }
    }

    #[test]
    fn set_protocol_accepts_only_what_the_atr_specifies() {
        let (handler, card) = open();
        assert_eq!(
            handler.set_protocol(LUN, SCARD_PROTOCOL_T1, 0, 0),
            Err(IFD_COMMUNICATION_ERROR)
        );
        handler.power(LUN, IFD_POWER_UP).unwrap();

        // What pcscd sends.
        assert_eq!(handler.set_protocol(LUN, SCARD_PROTOCOL_T1, 0, 0), Ok(()));
        for protocol in [
            SCARD_PROTOCOL_T0,
            SCARD_PROTOCOL_RAW,
            SCARD_PROTOCOL_T0 | SCARD_PROTOCOL_T1,
        ] {
            assert_eq!(
                handler.set_protocol(LUN, protocol, 0, 0),
                Err(IFD_PROTOCOL_NOT_SUPPORTED)
            );
        }

        // The B-CAS ATR has TA1 = 0x12.
        assert_eq!(
            handler.set_protocol(LUN, SCARD_PROTOCOL_T1, IFD_NEGOTIATE_PTS1, 0x12),
            Ok(())
        );
        for pts1 in [0x11, 0x13, 0x96] {
            assert_eq!(
                handler.set_protocol(LUN, SCARD_PROTOCOL_T1, IFD_NEGOTIATE_PTS1, pts1),
                Err(IFD_ERROR_PTS_FAILURE)
            );
        }
        for flags in [
            IFD_NEGOTIATE_PTS2,
            IFD_NEGOTIATE_PTS3,
            IFD_NEGOTIATE_PTS1 | IFD_NEGOTIATE_PTS2,
            0x08,
            0x80,
        ] {
            assert_eq!(
                handler.set_protocol(LUN, SCARD_PROTOCOL_T1, flags, 0x12),
                Err(IFD_ERROR_PTS_FAILURE),
                "{flags:#x}"
            );
        }

        // Without TA1 the default Fi = 372, Di = 1 is in effect.
        card.state().atr = vec![0x3B, 0x80, 0x01, 0x81];
        handler.power(LUN, IFD_RESET).unwrap();
        assert_eq!(
            handler.set_protocol(LUN, SCARD_PROTOCOL_T1, IFD_NEGOTIATE_PTS1, 0x11),
            Ok(())
        );
        assert_eq!(
            handler.set_protocol(LUN, SCARD_PROTOCOL_T1, IFD_NEGOTIATE_PTS1, 0x12),
            Err(IFD_ERROR_PTS_FAILURE)
        );
        // No card call is made: there is nothing to negotiate.
        assert_eq!(card.state().calls, ["status", "reset", "reset"]);
    }

    #[test]
    fn ta1_of_a_truncated_atr() {
        assert_eq!(ta1(&[0x3B, 0x10]), None);
        assert_eq!(ta1(&[0x3B, 0x10, 0x95]), Some(0x95));
        assert_eq!(ta1(&[0x3B, 0x00]), Some(DEFAULT_TA1));
        assert_eq!(ta1(&[0x3B]), None);
    }

    #[test]
    fn transmit_exchanges_one_apdu() {
        let (handler, card) = powered();
        card.state().response = vec![0x00, 0x21, 0x90, 0x00];
        let response = handler.transmit(LUN, 1, SELECT).unwrap();
        assert_eq!(response.as_slice(), [0x00, 0x21, 0x90, 0x00]);
        assert_eq!(card.state().commands, [SELECT]);

        // Longer than the card's IFSC: the kernel refuses it and keeps the session.
        let longest = [0xA5; uapi::APDU_MAX];
        assert_eq!(
            handler.transmit(LUN, 1, &longest),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert!(handler.transmit(LUN, 1, SELECT).is_ok());

        // A card with the largest IFSC takes the longest command the UAPI carries.
        card.state().ifsc = uapi::APDU_MAX;
        handler.transmit(LUN, 1, &longest).unwrap();
        assert_eq!(card.state().commands.last().unwrap(), &longest);
    }

    #[test]
    fn transmit_rejects_other_protocols_and_bad_lengths_without_a_call() {
        let (handler, card) = powered();
        // T=0 and RAW as pcscd encodes them in SCARD_IO_HEADER.
        assert_eq!(
            handler.transmit(LUN, 0, SELECT),
            Err(IFD_PROTOCOL_NOT_SUPPORTED)
        );
        assert_eq!(
            handler.transmit(LUN, SCARD_PROTOCOL_RAW, SELECT),
            Err(IFD_PROTOCOL_NOT_SUPPORTED)
        );
        assert_eq!(handler.transmit(LUN, 1, &[]), Err(IFD_COMMUNICATION_ERROR));
        assert_eq!(
            handler.transmit(LUN, 1, &SELECT[..uapi::APDU_MIN - 1]),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.transmit(LUN, 1, &[0; uapi::APDU_MAX + 1]),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert!(card.state().commands.is_empty());
    }

    #[test]
    fn transmit_errno_mapping() {
        let cases: &[(i32, RESPONSECODE)] = &[
            (libc::EMSGSIZE, IFD_COMMUNICATION_ERROR),
            (libc::EIO, IFD_COMMUNICATION_ERROR),
            (libc::EINTR, IFD_COMMUNICATION_ERROR),
            (libc::EFAULT, IFD_COMMUNICATION_ERROR),
            (libc::EINVAL, IFD_COMMUNICATION_ERROR),
        ];
        for &(errno, expected) in cases {
            let (handler, card) = powered();
            card.state().transmit_error = Some(errno);
            assert_eq!(
                handler.transmit(LUN, 1, SELECT),
                Err(expected),
                "errno {errno}"
            );
        }
    }

    #[test]
    fn transmit_without_session() {
        // Card present but powered down: not an absent card.
        let (handler, card) = open();
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        // After EIO the session is gone until the next reset.
        handler.power(LUN, IFD_POWER_UP).unwrap();
        card.state().transmit_error = Some(libc::EIO);
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(card.state().calls.last(), Some(&"status"));
        handler.power(LUN, IFD_RESET).unwrap();
        assert!(handler.transmit(LUN, 1, SELECT).is_ok());

        // The card was pulled while idle and the status poll ended its session (ENOTCONN).
        card.state().present = false;
        assert_eq!(handler.presence(LUN), Ok(false));
        assert_eq!(handler.transmit(LUN, 1, SELECT), Err(IFD_ICC_NOT_PRESENT));

        // ENOTCONN, then the board turns out to be gone.
        let (handler, card) = open();
        card.state().status_error = Some(libc::ENODEV);
        assert_eq!(handler.transmit(LUN, 1, SELECT), Err(IFD_NO_SUCH_DEVICE));
        card.state().status_error = Some(libc::EFAULT);
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
    }

    #[test]
    fn transmit_enodev_tells_card_removal_from_device_loss() {
        // Pulled during the exchange: the kernel reports ENODEV, the status says absent.
        let (handler, card) = powered();
        card.state().present = false;
        assert_eq!(handler.transmit(LUN, 1, SELECT), Err(IFD_ICC_NOT_PRESENT));
        assert_eq!(atr_capability(&handler), b"");

        // Pulled and reinserted before the status call: the card is there, but not reset.
        let (handler, card) = powered();
        card.state().transmit_error = Some(libc::ENODEV);
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
        assert_eq!(atr_capability(&handler), b"");
        handler.power(LUN, IFD_RESET).unwrap();
        assert!(handler.transmit(LUN, 1, SELECT).is_ok());

        // The board went away.
        let (handler, card) = powered();
        card.state().gone = true;
        assert_eq!(handler.transmit(LUN, 1, SELECT), Err(IFD_NO_SUCH_DEVICE));

        // The status call fails otherwise: nothing can be concluded.
        let (handler, card) = powered();
        card.state().transmit_error = Some(libc::ENODEV);
        card.state().status_error = Some(libc::EFAULT);
        assert_eq!(
            handler.transmit(LUN, 1, SELECT),
            Err(IFD_COMMUNICATION_ERROR)
        );
    }

    #[test]
    fn malformed_response_from_the_kernel_is_rejected() {
        for len in [0, 1, uapi::APDU_MAX as u32 + 1, 256, 257, u32::MAX] {
            let (handler, card) = powered();
            card.state().response_len = Some(len);
            assert_eq!(
                handler.transmit(LUN, 1, SELECT),
                Err(IFD_COMMUNICATION_ERROR),
                "{len}"
            );
        }
        let (handler, card) = powered();
        card.state().response = vec![0x5A; uapi::APDU_MAX];
        assert_eq!(
            handler.transmit(LUN, 1, SELECT).unwrap().as_slice(),
            [0x5A; uapi::APDU_MAX]
        );
    }

    #[test]
    fn capabilities() {
        let (handler, _card) = open();
        let byte = |tag| handler.capability(LUN, tag).map(|v| v.as_slice().to_vec());
        assert_eq!(byte(TAG_IFD_SLOTS_NUMBER), Ok(vec![1]));
        assert_eq!(byte(TAG_IFD_SIMULTANEOUS_ACCESS), Ok(vec![MAX_READERS]));
        assert_eq!(byte(TAG_IFD_THREAD_SAFE), Ok(vec![1]));
        assert_eq!(byte(TAG_IFD_SLOT_THREAD_SAFE), Ok(vec![0]));
        assert_eq!(byte(TAG_IFD_ATR), Ok(vec![]));
        // TAG_IFD_POLLING_THREAD_WITH_TIMEOUT: pcscd then polls the presence itself.
        assert_eq!(byte(0x0FB3), Err(IFD_ERROR_TAG));
        assert_eq!(byte(0x0FB4), Err(IFD_ERROR_TAG));

        handler.power(LUN, IFD_POWER_UP).unwrap();
        assert_eq!(byte(TAG_IFD_ATR), Ok(BCAS_ATR.to_vec()));
        assert_eq!(byte(SCARD_ATTR_ATR_STRING), Ok(BCAS_ATR.to_vec()));

        assert_eq!(
            handler.set_capability(LUN, TAG_IFD_ATR),
            Err(IFD_ERROR_VALUE_READ_ONLY)
        );
        assert_eq!(handler.set_capability(LUN, 0x0FB4), Err(IFD_ERROR_TAG));
        assert_eq!(handler.control(LUN), Err(IFD_ERROR_NOT_SUPPORTED));
    }

    #[test]
    fn close_powers_off_and_forgets_the_reader() {
        let (handler, card) = powered();
        assert_eq!(handler.close(LUN), Ok(()));
        assert_eq!(card.state().calls.last(), Some(&"power_off"));
        assert_eq!(handler.presence(LUN), Err(IFD_COMMUNICATION_ERROR));
        // The Lun can be opened again.
        handler.open(LUN, FakeCard::default()).unwrap();

        let gone = FakeCard::default();
        handler.open(1, gone.clone()).unwrap();
        gone.state().gone = true;
        assert_eq!(handler.close(1), Err(IFD_NO_SUCH_DEVICE));
        assert_eq!(handler.presence(1), Err(IFD_COMMUNICATION_ERROR));
    }

    /// A call blocked in the kernel on one reader must not hold up another reader; this is what
    /// TAG_IFD_THREAD_SAFE = 1 claims.
    #[test]
    fn readers_do_not_block_each_other() {
        let handler = Arc::new(Handler::new());
        let slow = FakeCard::default();
        let other = FakeCard::default();
        handler.open(0, slow.clone()).unwrap();
        handler.open(0x0001_0000, other.clone()).unwrap();
        handler.power(0, IFD_POWER_UP).unwrap();

        let (entered_tx, entered) = mpsc::channel();
        let (release, release_rx) = mpsc::channel();
        slow.state().gate = Some((entered_tx, release_rx));
        let blocked = {
            let handler = Arc::clone(&handler);
            thread::spawn(move || handler.transmit(0, 1, SELECT))
        };
        entered.recv_timeout(Duration::from_secs(10)).unwrap();

        let (done_tx, done) = mpsc::channel();
        let concurrent = {
            let handler = Arc::clone(&handler);
            thread::spawn(move || {
                let result = (
                    handler.presence(0x0001_0000),
                    handler.power(0x0001_0000, IFD_POWER_UP).is_ok(),
                );
                done_tx.send(result).unwrap();
            })
        };
        assert_eq!(
            done.recv_timeout(Duration::from_secs(10)),
            Ok((Ok(true), true))
        );
        concurrent.join().unwrap();

        release.send(()).unwrap();
        assert!(blocked.join().unwrap().is_ok());
    }
}
