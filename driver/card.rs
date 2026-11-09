// SPDX-License-Identifier: GPL-2.0

//! Built-in smart-card reader, exposed as `/dev/pxw3pe-cardN` (see `uapi/pxw3pe_card.h`).
//!
//! Every operation runs in the caller's context under the per-card mutex. Card removal during an
//! operation is noticed by the UART layer on every wait; between operations a delayed work of the
//! glue (`chrdev_glue.c`) samples the detect switch every 100 ms, and the status call samples it
//! directly.
//!
//! The device node is registered through `chrdev`, which gives it the module as owner so that an
//! open file keeps the module loaded.

use crate::proto::{
    atr,
    t1::{self, Session},
    uart::{self, Bus, Fault, Uart},
};
use crate::{
    chrdev::{self, pxw3pe_fatal_signal_pending, pxw3pe_mutex_lock_killable},
    hw::Board,
};
use core::{
    ffi::{c_long, c_uint, c_ulong, c_void},
    sync::atomic::{AtomicBool, Ordering},
};
use kernel::{
    bindings, c_str,
    error::to_result,
    ioctl::{_IO, _IOR, _IOWR},
    new_mutex,
    prelude::*,
    str::CStr,
    sync::{Arc, ArcBorrow, Mutex, MutexGuard},
    time::{delay::fsleep, Delta, Instant, Monotonic},
    transmute::{AsBytes, FromBytes},
    types::ForeignOwnable,
    uaccess::{UserPtr, UserSlice},
};

/// Card access through the board registers, with a monotonic microsecond clock.
struct BoardBus {
    board: Arc<Board>,
    epoch: Instant<Monotonic>,
    /// The reader's `gone` flag, see [`Card`].
    gone: Arc<AtomicBool>,
}

impl Bus for BoardBus {
    fn read32(&self, offset: usize) -> Result<u32, Fault> {
        self.board.read32(offset).ok_or(Fault::DeviceGone)
    }

    fn write32(&self, value: u32, offset: usize) -> Result<(), Fault> {
        self.board.write32(value, offset).ok_or(Fault::DeviceGone)
    }

    fn update_gpio(&self, set: u32, clear: u32) -> Result<(), Fault> {
        self.board.update_gpio(set, clear).ok_or(Fault::DeviceGone)
    }

    fn sleep_us(&self, us: u32) {
        fsleep(Delta::from_micros(i64::from(us)));
    }

    fn now_us(&self) -> u64 {
        // The epoch lies in the past, so the difference is never negative.
        (Instant::<Monotonic>::now() - self.epoch).as_micros_ceil() as u64
    }

    fn check_abort(&self) -> Result<(), Fault> {
        if self.gone.load(Ordering::Acquire) {
            return Err(Fault::DeviceGone);
        }
        // Only a fatal signal ends an operation: any other one leaves the session usable, and the
        // caller sees it when the operation returns.
        //
        // SAFETY: The function only reads the pending signals of the current task.
        if unsafe { pxw3pe_fatal_signal_pending() } {
            return Err(Fault::Interrupted);
        }
        Ok(())
    }
}

// Mirrors of `uapi/pxw3pe_card.h`.

const IOC: u32 = 0xBA;
const PRESENT: u32 = 1 << 0;
const ACTIVE: u32 = 1 << 1;
const REMOVED: u32 = 1 << 2;
/// Smallest APDU: CLA INS P1 P2 (ISO/IEC 7816-4).
const APDU_MIN: usize = 4;

#[repr(C)]
#[derive(Default)]
struct StatusArg {
    flags: u32,
}

#[repr(C)]
struct AtrArg {
    len: u32,
    atr: [u8; atr::MAX_LEN],
    reserved: [u8; 3],
}

#[repr(C)]
struct TransferArg {
    command_len: u32,
    response_len: u32,
    command: [u8; 256],
    response: [u8; 256],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct StatsArg {
    edc_errors: u32,
    other_errors: u32,
    retransmissions: u32,
    resynchs: u32,
    wtx: u32,
    resets: u32,
    failed_resets: u32,
    reserved: u32,
}

const _: () = assert!(core::mem::size_of::<StatusArg>() == 4);
const _: () = assert!(core::mem::size_of::<AtrArg>() == 40);
const _: () = assert!(core::mem::size_of::<TransferArg>() == 520);
const _: () = assert!(core::mem::size_of::<StatsArg>() == 32);

// SAFETY: These are `repr(C)` structs of integers and byte arrays without padding (checked by the
// size assertions above), so every bit pattern is valid and no byte is uninitialised.
unsafe impl FromBytes for TransferArg {}
// SAFETY: See above.
unsafe impl AsBytes for TransferArg {}
// SAFETY: See above.
unsafe impl AsBytes for StatusArg {}
// SAFETY: See above.
unsafe impl AsBytes for AtrArg {}
// SAFETY: See above.
unsafe impl AsBytes for StatsArg {}

const GET_STATUS: u32 = _IOR::<StatusArg>(IOC, 0);
const RESET: u32 = _IOR::<AtrArg>(IOC, 1);
const POWER_OFF: u32 = _IO(IOC, 2);
const TRANSMIT: u32 = _IOWR::<TransferArg>(IOC, 3);
const GET_STATS: u32 = _IOR::<StatsArg>(IOC, 4);

struct State {
    uart: Uart<BoardBus>,
    session: Option<Session>,
    atr: [u8; atr::MAX_LEN],
    atr_len: usize,
    /// Counters of sessions that have ended.
    totals: StatsArg,
    /// A session ended because the card was pulled, and no status call has reported it yet.
    removal_unreported: bool,
    /// `Card::shutdown` has held the card in reset for good.
    shut_down: bool,
}

impl State {
    /// Ends the current session, if any, folding its counters into the totals.
    fn end_session(&mut self) {
        if let Some(session) = self.session.take() {
            let s = session.stats();
            let t = &mut self.totals;
            t.edc_errors = t.edc_errors.wrapping_add(s.edc_errors);
            t.other_errors = t.other_errors.wrapping_add(s.other_errors);
            t.retransmissions = t.retransmissions.wrapping_add(s.retransmissions);
            t.resynchs = t.resynchs.wrapping_add(s.resynchs);
            t.wtx = t.wtx.wrapping_add(s.wtx);
        }
    }

    /// Ends the current session and holds the card in reset, so that nothing stays powered up
    /// with a session the driver no longer tracks. The card is gone or unusable already, so a
    /// failure to reset the UART changes nothing for the caller.
    fn abandon_session(&mut self) {
        self.end_session();
        let _ = self.uart.deactivate();
    }

    /// Ends the session of a card that was pulled. The next status call reports the removal even
    /// if the card is back by then: the card in the slot may be another one, and the session
    /// user space believes in is gone either way.
    fn end_removed_session(&mut self) {
        self.abandon_session();
        self.removal_unreported = true;
    }

    /// Whether the card of the open session has been pulled, by the rule of `uart::REMOVAL_US`;
    /// ends the session if so. False without a session.
    fn check_removal(&mut self) -> Result<bool, Fault> {
        if self.session.is_none() {
            return Ok(false);
        }
        let removed = self.uart.card_removed()?;
        if removed {
            self.end_removed_session();
        }
        Ok(removed)
    }

    fn stats(&self) -> StatsArg {
        let mut out = self.totals;
        if let Some(session) = &self.session {
            let s = session.stats();
            out.edc_errors = out.edc_errors.wrapping_add(s.edc_errors);
            out.other_errors = out.other_errors.wrapping_add(s.other_errors);
            out.retransmissions = out.retransmissions.wrapping_add(s.retransmissions);
            out.resynchs = out.resynchs.wrapping_add(s.resynchs);
            out.wtx = out.wtx.wrapping_add(s.wtx);
        }
        out
    }
}

/// Builds an error for an errno value that `kernel::error::code` does not name.
fn errno(code: u32) -> Error {
    // Errno values are small positive constants.
    Error::from_errno(-(code as i32))
}

fn fault_to_error(fault: Fault) -> Error {
    match fault {
        Fault::DeviceGone | Fault::NoCard => ENODEV,
        Fault::Interrupted => EINTR,
        Fault::TxStuck | Fault::RxStuck | Fault::Mute => EIO,
        Fault::Atr(_) => errno(bindings::EPROTO),
    }
}

/// The card reader of one board.
#[pin_data]
pub(crate) struct Card {
    #[pin]
    state: Mutex<State>,
    /// Set when the board is going away. Open files keep the card alive after the device node is
    /// gone, and a late `RESET` must not power the card up again. It lives outside the mutex so
    /// that setting it ends the operation that holds the mutex; the bus of the UART shares it.
    gone: Arc<AtomicBool>,
}

impl Card {
    fn new(board: Arc<Board>) -> Result<Arc<Self>> {
        let gone = Arc::new(AtomicBool::new(false), GFP_KERNEL)?;
        let bus = BoardBus {
            board,
            epoch: Instant::now(),
            gone: gone.clone(),
        };
        Arc::pin_init(
            pin_init!(Card {
                state <- new_mutex!(State {
                    uart: Uart::new(bus),
                    session: None,
                    atr: [0; atr::MAX_LEN],
                    atr_len: 0,
                    totals: StatsArg::default(),
                    removal_unreported: false,
                    shut_down: false,
                }),
                gone,
            }),
            GFP_KERNEL,
        )
    }

    /// Takes the state lock, giving up with `EINTR` if the caller is killed while it waits: a
    /// card operation may run for a long time, and nothing else can end the wait.
    fn lock_killable(&self) -> Result<MutexGuard<'_, State>> {
        let mutex = core::ptr::from_ref(&self.state)
            .cast::<bindings::mutex>()
            .cast_mut();
        // SAFETY: `Lock` is `#[repr(C)]` and its first field is `state: Opaque<B::State>`, which
        // is `#[repr(transparent)]` over the `struct mutex` of `MutexBackend`
        // (rust/kernel/sync/lock.rs, `Lock`; `Lock::from_raw` relies on the same layout), so
        // `mutex` points to the initialised mutex of `self.state`, which lives as long as `self`.
        to_result(unsafe { pxw3pe_mutex_lock_killable(mutex) })?;
        // SAFETY: The call succeeded, so this thread owns the mutex. The guard releases it with
        // `mutex_unlock`, as the guards made by `Mutex::lock` do.
        Ok(unsafe { MutexGuard::new(&self.state, ()) })
    }

    /// Locks the state for an operation, which a reader that is gone refuses with `ENODEV`.
    fn lock_live(&self) -> Result<MutexGuard<'_, State>> {
        let state = self.lock_killable()?;
        if self.gone.load(Ordering::Acquire) {
            return Err(ENODEV);
        }
        Ok(state)
    }

    fn status(&self) -> Result<u32> {
        let mut state = self.lock_live()?;
        state.check_removal().map_err(fault_to_error)?;
        // A card that has been absent for less than the removal time keeps its session, and so
        // counts as present: reporting it absent would make user space end the session itself.
        // Without a session the card is already held in reset, so polling an empty slot does not
        // touch the UART.
        let present =
            state.session.is_some() || state.uart.card_present().map_err(fault_to_error)?;
        let removed = core::mem::take(&mut state.removal_unreported);
        let active = state.session.is_some();
        Ok(if present { PRESENT } else { 0 }
            | if active { ACTIVE } else { 0 }
            | if removed { REMOVED } else { 0 })
    }

    fn reset(&self) -> Result<AtrArg> {
        let mut state = self.lock_live()?;
        state.end_session();
        state.totals.resets = state.totals.resets.wrapping_add(1);
        let result = state.uart.activate().and_then(|a| {
            let mut session = Session::new(&a.params, a.params.timing(uart::CARD_CLOCK_HZ));
            match session.negotiate_ifsd(&mut state.uart) {
                Ok(()) => Ok((a, session)),
                Err(t1::Error::Io(fault)) => Err(fault),
                // IFS negotiation cannot be too long, so every other failure means a mute card.
                Err(_) => Err(Fault::Mute),
            }
        });
        let (activation, session) = match result {
            Ok(ok) => ok,
            Err(fault) => {
                state.totals.failed_resets = state.totals.failed_resets.wrapping_add(1);
                // Leave the card in reset rather than half-initialised.
                let _ = state.uart.deactivate();
                return Err(fault_to_error(fault));
            }
        };
        state.atr = activation.atr;
        state.atr_len = activation.atr_len;
        state.session = Some(session);
        Ok(AtrArg {
            len: activation.atr_len as u32,
            atr: activation.atr,
            reserved: [0; 3],
        })
    }

    fn power_off(&self) -> Result {
        let mut state = self.lock_live()?;
        state.end_session();
        state.uart.deactivate().map_err(fault_to_error)
    }

    /// Marks the card as gone and holds it in reset; every later operation fails with `ENODEV`.
    /// Only the first call touches the card.
    fn shutdown(&self) {
        // Before the lock is taken: an operation in progress then aborts at its next step
        // instead of running on, and releases the lock.
        self.gone.store(true, Ordering::Release);
        let mut state = self.state.lock();
        if !state.shut_down {
            state.shut_down = true;
            state.abandon_session();
        }
    }

    /// Samples the detect switch while a session is open, so that a card pulled between two
    /// operations ends its session even if it is back in the slot when the next one starts.
    /// Runs every 100 ms from the glue's delayed work.
    fn poll(&self) {
        // An operation in progress samples the switch itself on every wait, by the same rule;
        // waiting for it here would only delay the next sample.
        let Some(mut state) = self.state.try_lock() else {
            return;
        };
        if self.gone.load(Ordering::Acquire) {
            return;
        }
        // Fails only once the board is gone; `shutdown` ends the session then.
        let _ = state.check_removal();
    }

    fn transmit(&self, arg: &mut TransferArg) -> Result {
        let len = arg.command_len as usize;
        if len < APDU_MIN {
            return Err(EINVAL);
        }
        if len > t1::MAX_INF {
            return Err(errno(bindings::EMSGSIZE));
        }
        let mut guard = self.lock_live()?;
        let state = &mut *guard;
        let session = state
            .session
            .as_mut()
            .ok_or_else(|| errno(bindings::ENOTCONN))?;
        match session.transceive(&mut state.uart, &arg.command[..len], &mut arg.response) {
            Ok(n) => {
                arg.response_len = n as u32;
                Ok(())
            }
            Err(t1::Error::CommandTooLong) => Err(errno(bindings::EMSGSIZE)),
            Err(t1::Error::Io(Fault::NoCard)) => {
                state.end_removed_session();
                Err(ENODEV)
            }
            Err(err) => {
                // The session is gone; the card must be reset before it can be used again.
                state.abandon_session();
                Err(match err {
                    t1::Error::Io(fault) => fault_to_error(fault),
                    _ => EIO,
                })
            }
        }
    }

    fn stats(&self) -> Result<StatsArg> {
        Ok(self.lock_live()?.stats())
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> Result {
        let user = UserSlice::new(UserPtr::from_addr(arg), kernel::ioctl::_IOC_SIZE(cmd));
        match cmd {
            GET_STATUS => user.writer().write(&StatusArg {
                flags: self.status()?,
            })?,
            RESET => user.writer().write(&self.reset()?)?,
            POWER_OFF => self.power_off()?,
            TRANSMIT => {
                let (mut reader, mut writer) = user.reader_writer();
                let mut transfer: TransferArg = reader.read()?;
                self.transmit(&mut transfer)?;
                writer.write(&transfer)?;
            }
            GET_STATS => user.writer().write(&self.stats()?)?,
            _ => return Err(ENOTTY),
        }
        Ok(())
    }
}

static OPS: chrdev::Ops = chrdev::Ops {
    open: open_cb,
    release: release_cb,
    ioctl: ioctl_cb,
    poll: Some(poll_cb),
};

/// # Safety
///
/// `ctx` must be the context given to `chrdev::Registration::new`, and the registration must
/// still exist for `'a`.
unsafe fn registered_card<'a>(ctx: *mut c_void) -> ArcBorrow<'a, Card> {
    // SAFETY: `CardDevice::new` passes `Arc::as_ptr` of its `Card`, which it keeps alive until
    // after the unregistration; the caller guarantees the registration exists.
    unsafe { ArcBorrow::from_raw(ctx.cast::<Card>()) }
}

/// # Safety
///
/// Called by the glue with the registration context, while the device is registered.
unsafe extern "C" fn open_cb(ctx: *mut c_void) -> *mut c_void {
    // SAFETY: Guaranteed by the caller. `misc_open` calls this with `misc_mtx` held, and
    // `misc_deregister`, which the unregistration calls first, takes the same mutex, so the
    // registration exists for the whole call.
    let card = Arc::from(unsafe { registered_card(ctx) });
    // The file owns a reference, so it keeps the card, and through it the board, alive even if
    // the PCI device is unbound while the file is open.
    card.into_foreign()
}

/// # Safety
///
/// Called by the glue with a value returned by `open_cb`, once.
unsafe extern "C" fn release_cb(file: *mut c_void) {
    // SAFETY: Guaranteed by the caller, and no `ioctl_cb` call on the file is running any more:
    // the VFS releases a file after its last use.
    drop(unsafe { <Arc<Card> as ForeignOwnable>::from_foreign(file) });
}

/// # Safety
///
/// Called by the glue with a value returned by `open_cb` that `release_cb` has not been called
/// with, and with `arg` as passed by user space.
unsafe extern "C" fn ioctl_cb(file: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: Guaranteed by the caller; the file is released only after its last ioctl returned.
    let card = unsafe { <Arc<Card> as ForeignOwnable>::borrow(file) };
    match card.ioctl(cmd, arg as usize) {
        Ok(()) => 0,
        Err(e) => c_long::from(e.to_errno()),
    }
}

/// # Safety
///
/// Called by the glue with the registration context, while the device is registered.
unsafe extern "C" fn poll_cb(ctx: *mut c_void) {
    // SAFETY: Guaranteed by the caller. The unregistration disables the work that calls
    // this and waits for a run in progress before it returns.
    unsafe { registered_card(ctx) }.poll();
}

/// Registered character device of one reader.
pub(crate) struct CardDevice {
    // Dropped before `card`, which `Drop` shuts down first: no `open_cb` or `poll_cb` runs once
    // the registration has gone.
    _registration: chrdev::Registration,
    card: Arc<Card>,
}

/// Device node names, by board number.
const NAMES: [&CStr; chrdev::MAX_BOARDS] = [
    c_str!("pxw3pe-card0"),
    c_str!("pxw3pe-card1"),
    c_str!("pxw3pe-card2"),
    c_str!("pxw3pe-card3"),
    c_str!("pxw3pe-card4"),
    c_str!("pxw3pe-card5"),
    c_str!("pxw3pe-card6"),
    c_str!("pxw3pe-card7"),
];

impl CardDevice {
    /// Creates the reader of `board` and registers its device node, named after `slot`, which
    /// has to outlive the device.
    pub(crate) fn new(board: Arc<Board>, slot: &chrdev::Slot) -> Result<Pin<KBox<Self>>> {
        let card = Card::new(board)?;
        let ctx = Arc::as_ptr(&card).cast_mut().cast::<c_void>();
        // SAFETY: The callbacks of `OPS` take `ctx` as a `Card` registration context and what
        // `open_cb` returns as a file, and `ctx` points into `card`, which the `CardDevice`
        // keeps alive until after the unregistration.
        let registration = unsafe { chrdev::Registration::new(NAMES[slot.index()], &OPS, ctx)? };
        // If the allocation fails, the registration is dropped and thereby unregistered.
        Ok(KBox::pin(
            CardDevice {
                _registration: registration,
                card,
            },
            GFP_KERNEL,
        )?)
    }

    /// Holds the card in reset and refuses every later operation. An operation that is running
    /// aborts at its next step. May be called more than once.
    pub(crate) fn shutdown(&self) {
        self.card.shutdown();
    }
}

impl Drop for CardDevice {
    fn drop(&mut self) {
        // Unregistering does not wait for open files, which keep the card alive: they must find
        // it shut down, and it must not stay powered up with a stale session. The registration
        // goes with the fields, afterwards.
        self.shutdown();
    }
}
