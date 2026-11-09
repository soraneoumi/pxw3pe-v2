// SPDX-License-Identifier: GPL-2.0

//! Rust side of `chrdev_glue.h`: character devices whose open files keep the module loaded, and
//! the board numbers that name them.
//!
//! The declarations below mirror the C header; any change there must be made here too.

use core::{
    ffi::{c_int, c_long, c_uint, c_ulong, c_void},
    mem::{offset_of, size_of},
    ptr::NonNull,
    sync::atomic::{AtomicU32, Ordering},
};
use kernel::{bindings, error::from_err_ptr, ffi::c_char, prelude::*, str::CStr};

/// `struct pxw3pe_chrdev_ops`. See `chrdev_glue.h` for when each callback runs and with which
/// argument.
#[repr(C)]
pub(crate) struct Ops {
    pub(crate) open: unsafe extern "C" fn(*mut c_void) -> *mut c_void,
    pub(crate) release: unsafe extern "C" fn(*mut c_void),
    pub(crate) ioctl: unsafe extern "C" fn(*mut c_void, c_uint, c_ulong) -> c_long,
    /// `None` is the C side's NULL: the device has no periodic work.
    pub(crate) poll: Option<unsafe extern "C" fn(*mut c_void)>,
}

// The layout `chrdev_glue.c` asserts for the C structure, so a structure changed on one side only
// fails the build; change both together. `Option` of a function pointer has the pointer's layout,
// with `None` as null.
const PTR: usize = size_of::<*const c_void>();
const _: () = {
    assert!(size_of::<Ops>() == 4 * PTR);
    assert!(offset_of!(Ops, release) == PTR);
    assert!(offset_of!(Ops, ioctl) == 2 * PTR);
    assert!(offset_of!(Ops, poll) == 3 * PTR);
};

/// Opaque `struct pxw3pe_chrdev`.
#[repr(C)]
struct RawDev {
    _opaque: [u8; 0],
}

extern "C" {
    fn pxw3pe_chrdev_register(
        name: *const c_char,
        ops: *const Ops,
        ctx: *mut c_void,
    ) -> *mut RawDev;
    fn pxw3pe_chrdev_unregister(dev: *mut RawDev);
    pub(crate) fn pxw3pe_mutex_lock_killable(lock: *mut bindings::mutex) -> c_int;
    pub(crate) fn pxw3pe_fatal_signal_pending() -> bool;
    pub(crate) fn pxw3pe_capable_rawio() -> bool;
}

/// A registered device node; dropping it unregisters the node. Files that are still open stay
/// usable through `release` and `ioctl`, with what `open` returned for them.
pub(crate) struct Registration {
    raw: NonNull<RawDev>,
}

// SAFETY: The raw handle is only used for unregistration, which may happen on any thread.
unsafe impl Send for Registration {}
// SAFETY: `&Registration` gives no access to the handle.
unsafe impl Sync for Registration {}

impl Registration {
    /// Registers `/dev/<name>`.
    ///
    /// # Safety
    ///
    /// The callbacks of `ops` must accept `ctx` as their registration context (`open`, `poll`)
    /// and what `open` returns as their file (`release`, `ioctl`), and `ctx` must stay valid
    /// until the registration has been dropped.
    pub(crate) unsafe fn new(
        name: &'static CStr,
        ops: &'static Ops,
        ctx: *mut c_void,
    ) -> Result<Self> {
        // SAFETY: `ops` is static, the name is a static string, and the caller keeps `ctx`
        // valid for as long as the C side may use it.
        let raw = from_err_ptr(unsafe { pxw3pe_chrdev_register(name.as_char_ptr(), ops, ctx) })?;
        // SAFETY: `pxw3pe_chrdev_register` returns an error pointer or a valid one, never null.
        let raw = unsafe { NonNull::new_unchecked(raw) };
        Ok(Registration { raw })
    }
}

impl Drop for Registration {
    fn drop(&mut self) {
        // SAFETY: `raw` came from a successful registration and is unregistered exactly once.
        // Afterwards no `open` or `poll` callback runs.
        unsafe { pxw3pe_chrdev_unregister(self.raw.as_ptr()) };
    }
}

/// Number of boards the device names provide for.
pub(crate) const MAX_BOARDS: usize = 8;

/// Bit `i` is set while board number `i` is in use.
static SLOTS: AtomicU32 = AtomicU32::new(0);

/// The number of a bound board, which its device nodes carry (`pxw3pe-card<N>`,
/// `pxw3pe-lnb<N>`), so that the nodes of one board have the same number. Released on drop;
/// it has to outlive the registrations that use it, since a name may be reused only once the
/// registration that owns it has gone.
pub(crate) struct Slot(usize);

impl Slot {
    /// Claims the lowest free number; `EBUSY` when all [`MAX_BOARDS`] are taken.
    pub(crate) fn claim() -> Result<Self> {
        let mut used = SLOTS.load(Ordering::Relaxed);
        loop {
            let slot = (!used).trailing_zeros() as usize;
            if slot >= MAX_BOARDS {
                return Err(EBUSY);
            }
            match SLOTS.compare_exchange(
                used,
                used | 1 << slot,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => return Ok(Slot(slot)),
                Err(now) => used = now,
            }
        }
    }

    pub(crate) fn index(&self) -> usize {
        self.0
    }
}

impl Drop for Slot {
    fn drop(&mut self) {
        SLOTS.fetch_and(!(1 << self.0), Ordering::AcqRel);
    }
}
