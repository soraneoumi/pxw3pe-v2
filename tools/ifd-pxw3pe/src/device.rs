//! Access to one `/dev/pxw3pe-cardN` node.
//!
//! The IFD logic only sees the [`Card`] trait, so that it can be tested against a fake card.

use crate::uapi::{self, CardAtr, CardStatus, CardTransfer};
use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::Path;

/// The operations of the kernel interface, one method per ioctl. Errors carry the errno the
/// kernel reported.
///
/// Every method takes `&mut self` because the IFD logic serialises the calls on one card;
/// the kernel would serialise them anyway, but holding the lock across the call also keeps the
/// cached ATR consistent with the session it belongs to.
pub trait Card {
    /// `PXW3PE_CARD_GET_STATUS`: the `PRESENT`/`ACTIVE` flags.
    fn status(&mut self) -> io::Result<u32>;
    /// `PXW3PE_CARD_RESET`: resets the card, opening a new session, and returns its ATR.
    fn reset(&mut self) -> io::Result<CardAtr>;
    /// `PXW3PE_CARD_POWER_OFF`: holds the card in reset, ending the session.
    fn power_off(&mut self) -> io::Result<()>;
    /// `PXW3PE_CARD_TRANSMIT`: exchanges one APDU in place.
    fn transmit(&mut self, transfer: &mut CardTransfer) -> io::Result<()>;
}

/// An open card device node.
pub struct CharDevice {
    file: File,
}

impl CharDevice {
    pub fn open(path: &Path) -> io::Result<Self> {
        // Read-write because commands are sent to the card; std adds O_CLOEXEC so that the
        // descriptor does not leak into processes pcscd spawns.
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        Ok(Self { file })
    }

    /// Issues `request` with `arg` as its argument.
    ///
    /// # Safety
    ///
    /// `arg` must be null for an `_IO` request, or point to a live, writable value of exactly the
    /// type whose size `request` encodes.
    unsafe fn ioctl(&self, request: u32, arg: *mut libc::c_void) -> io::Result<()> {
        // SAFETY: The descriptor is open for the lifetime of `self.file`; the caller guarantees
        // that `arg` matches `request`, so the kernel reads and writes only memory we own.
        // The `as _` converts to the request parameter type of the C library, which is
        // `unsigned long` on glibc and `int` on musl; the kernel only uses the low 32 bits.
        let rc = unsafe { libc::ioctl(self.file.as_raw_fd(), request as _, arg) };
        if rc < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
}

impl Card for CharDevice {
    fn status(&mut self) -> io::Result<u32> {
        let mut status = CardStatus::default();
        // SAFETY: GET_STATUS is `_IOR` of `struct pxw3pe_card_status`, which `CardStatus` mirrors.
        unsafe { self.ioctl(uapi::GET_STATUS, (&raw mut status).cast()) }?;
        Ok(status.flags)
    }

    fn reset(&mut self) -> io::Result<CardAtr> {
        let mut atr = CardAtr::zeroed();
        // SAFETY: RESET is `_IOR` of `struct pxw3pe_card_atr`, which `CardAtr` mirrors.
        unsafe { self.ioctl(uapi::RESET, (&raw mut atr).cast()) }?;
        Ok(atr)
    }

    fn power_off(&mut self) -> io::Result<()> {
        // SAFETY: POWER_OFF is `_IO`: it takes no argument.
        unsafe { self.ioctl(uapi::POWER_OFF, core::ptr::null_mut()) }
    }

    fn transmit(&mut self, transfer: &mut CardTransfer) -> io::Result<()> {
        // SAFETY: TRANSMIT is `_IOWR` of `struct pxw3pe_card_transfer`, which `CardTransfer`
        // mirrors; the exclusive borrow keeps it alive and unaliased during the call.
        unsafe { self.ioctl(uapi::TRANSMIT, core::ptr::from_mut(transfer).cast()) }
    }
}
