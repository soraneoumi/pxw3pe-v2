// SPDX-License-Identifier: GPL-2.0

//! Rust side of `board_glue.h`: what the PCI device and system shutdown need that the kernel
//! crate's abstractions of the targeted kernels do not provide.
//!
//! The declarations below mirror the C header; any change there must be made here too.

use core::{ffi::c_void, pin::Pin, ptr::NonNull};
use kernel::{
    bindings,
    device::{self, Core},
    error::to_result,
    ffi::{c_int, c_uint},
    pci,
    prelude::*,
    sync::{aref::ARef, Arc},
};

/// Opaque `struct pxw3pe_reboot`.
#[repr(C)]
struct RawReboot {
    _opaque: [u8; 0],
}

/// Opaque `struct pxw3pe_poller`.
#[repr(C)]
struct RawPoller {
    _opaque: [u8; 0],
}

/// Opaque `struct pxw3pe_irqlock`.
#[repr(C)]
struct RawIrqLock {
    _opaque: [u8; 0],
}

/// Opaque `struct pxw3pe_latency`.
#[repr(C)]
struct RawLatency {
    _opaque: [u8; 0],
}

extern "C" {
    // `struct device *`, passed through opaquely.
    fn pxw3pe_pci_clear_master(dev: *mut c_void);
    fn pxw3pe_pci_disable_device(dev: *mut c_void);
    fn pxw3pe_mb();
    fn pxw3pe_irqlock_new() -> *mut RawIrqLock;
    fn pxw3pe_irqlock_free(lock: *mut RawIrqLock);
    fn pxw3pe_irqlock_lock(lock: *mut RawIrqLock) -> kernel::ffi::c_ulong;
    fn pxw3pe_irqlock_unlock(lock: *mut RawIrqLock, flags: kernel::ffi::c_ulong);
    fn pxw3pe_reboot_new() -> *mut RawReboot;
    fn pxw3pe_reboot_register(
        reboot: *mut RawReboot,
        prepare: unsafe extern "C" fn(*mut c_void),
        ctx: *mut c_void,
    ) -> kernel::ffi::c_int;
    fn pxw3pe_reboot_unregister(reboot: *mut RawReboot);
    fn pxw3pe_reboot_free(reboot: *mut RawReboot);
    fn pxw3pe_poller_new(
        f: unsafe extern "C" fn(*mut c_void),
        ctx: *mut c_void,
        period_ms: c_uint,
    ) -> *mut RawPoller;
    fn pxw3pe_poller_start(poller: *mut RawPoller);
    fn pxw3pe_poller_free(poller: *mut RawPoller);
    fn pxw3pe_latency_new() -> *mut RawLatency;
    fn pxw3pe_latency_hold(latency: *mut RawLatency, us: c_int);
    fn pxw3pe_latency_release(latency: *mut RawLatency);
    fn pxw3pe_latency_free(latency: *mut RawLatency);
}

/// The `struct device *` of `dev`, for the C glue. The kernel crate keeps its raw accessor
/// private; `device::Device` is `#[repr(transparent)]` over `Opaque<bindings::device>` (plus a
/// zero-sized marker), so the reference points at the C structure.
pub(crate) fn raw_device<Ctx: device::DeviceContext>(dev: &device::Device<Ctx>) -> *mut c_void {
    core::ptr::from_ref(dev)
        .cast::<bindings::device>()
        .cast_mut()
        .cast::<c_void>()
}

/// Full memory barrier, `mb()`: orders the CPU's earlier reads of coherent DMA memory before its
/// later MMIO writes. `smp_mb()` is only a compiler barrier on a kernel built without SMP, and
/// `writel()` orders only the writes before it (`Documentation/memory-barriers.txt`, "KERNEL I/O
/// BARRIER EFFECTS", `writeX()` rule 3).
#[inline(always)]
pub(crate) fn mb() {
    // SAFETY: `mb()` is safe to call.
    unsafe { pxw3pe_mb() };
}

/// A spinlock that is taken with interrupts disabled, so that a hard interrupt handler can take
/// it too.
pub(crate) struct IrqLock(NonNull<RawIrqLock>);

// SAFETY: The lock is a `spinlock_t`, which any thread and the interrupt handler may take.
unsafe impl Send for IrqLock {}
// SAFETY: See above.
unsafe impl Sync for IrqLock {}

impl IrqLock {
    pub(crate) fn new() -> Result<Self> {
        // SAFETY: No preconditions.
        let raw = unsafe { pxw3pe_irqlock_new() };
        NonNull::new(raw).map(IrqLock).ok_or(ENOMEM)
    }

    /// Runs `f` with the lock held and local interrupts disabled. `f` must not sleep.
    pub(crate) fn with<R>(&self, f: impl FnOnce() -> R) -> R {
        // SAFETY: The lock is live; it is released below with the flags this returns.
        let flags = unsafe { pxw3pe_irqlock_lock(self.0.as_ptr()) };
        let result = f();
        // SAFETY: Taken above by this thread.
        unsafe { pxw3pe_irqlock_unlock(self.0.as_ptr(), flags) };
        result
    }
}

impl Drop for IrqLock {
    fn drop(&mut self) {
        // SAFETY: The lock came from `pxw3pe_irqlock_new`, is not held (`with` releases it before
        // returning), and is freed exactly once.
        unsafe { pxw3pe_irqlock_free(self.0.as_ptr()) };
    }
}

/// A limit on how long the CPUs may take to leave their idle states, which keeps the memory
/// path they share awake too. Allocated not held; dropping it lifts the limit.
pub(crate) struct LatencyLimit(NonNull<RawLatency>);

// SAFETY: The request is only used through `&mut self`, so it may move between threads but is
// never used by two at once.
unsafe impl Send for LatencyLimit {}

impl LatencyLimit {
    pub(crate) fn new() -> Result<Self> {
        // SAFETY: No preconditions.
        let raw = unsafe { pxw3pe_latency_new() };
        NonNull::new(raw).map(LatencyLimit).ok_or(ENOMEM)
    }

    /// Limits the idle exit latency of every CPU to `us` microseconds until [`Self::release`].
    /// Must not be held already.
    pub(crate) fn hold(&mut self, us: u32) {
        // SAFETY: The request is live, and the caller guarantees that it is not held.
        unsafe { pxw3pe_latency_hold(self.0.as_ptr(), us as c_int) };
    }

    /// Lifts the limit; does nothing if it is not held.
    pub(crate) fn release(&mut self) {
        // SAFETY: The request is live.
        unsafe { pxw3pe_latency_release(self.0.as_ptr()) };
    }
}

impl Drop for LatencyLimit {
    fn drop(&mut self) {
        // SAFETY: The request came from `pxw3pe_latency_new` and is freed exactly once.
        unsafe { pxw3pe_latency_free(self.0.as_ptr()) };
    }
}

/// A PCI device with its memory resources enabled. Dropping it stops the device from mastering
/// the bus and disables it.
///
/// The device's interrupt has to be freed before the device is disabled, because disabling
/// releases the interrupt routing (`pcibios_disable_device()`, `arch/x86/pci/common.c`
/// 6.18.59:703, calls `acpi_pci_irq_disable()`, `drivers/acpi/pci_irq.c`:478, which unregisters
/// the GSI of the legacy interrupt); so in the driver state, whose fields drop in declaration
/// order, the guard is the last field. Register access stays possible after the disable, which
/// clears bus mastering but leaves memory decoding on (`do_pci_disable_device()`,
/// `drivers/pci/pci.c` 6.18.59:2191): a probe whose interrupt request fails drops the guard with
/// the rest of the state before its power guard switches the chips off through BAR0.
pub(crate) struct PciEnabled {
    dev: ARef<device::Device>,
}

impl PciEnabled {
    /// Enables the memory resources of `pdev`.
    pub(crate) fn new(pdev: &pci::Device<Core>) -> Result<Self> {
        pdev.enable_device_mem()?;
        Ok(PciEnabled {
            dev: pdev.as_ref().into(),
        })
    }

    /// The PCI device, for log messages.
    pub(crate) fn device(&self) -> &device::Device {
        &self.dev
    }

    /// Stops the device from mastering the bus. Idempotent.
    pub(crate) fn clear_master(&self) {
        // SAFETY: `dev` is the device embedded in the `struct pci_dev` that `new` enabled, and
        // the reference held by `self` keeps it alive. `pci_clear_master()` (`drivers/pci/pci.c`,
        // 6.18.59:4188) only updates the command register and has no other precondition.
        unsafe { pxw3pe_pci_clear_master(raw_device(&self.dev)) };
    }
}

impl Drop for PciEnabled {
    fn drop(&mut self) {
        // `pci_disable_device()` clears bus mastering too, but only when the last enable is
        // undone; the explicit call keeps the guarantee independent of that count.
        self.clear_master();
        // SAFETY: `new` enabled the device exactly once and this is the only disable
        // (`pci_disable_device()` counts enables, `drivers/pci/pci.c` 6.18.59:2227). The driver
        // state keeps this guard as its last field, so the interrupt has been freed (see the
        // type documentation).
        unsafe { pxw3pe_pci_disable_device(raw_device(&self.dev)) };
    }
}

/// State that is told when the system is about to restart, halt or power off.
pub(crate) trait Reboot: Sync {
    /// Called in process context, where it may sleep, before the devices are shut down.
    fn prepare(&self);
}

/// # Safety
///
/// `ctx` must point to a live `T` for the duration of the call.
unsafe extern "C" fn prepare_cb<T: Reboot>(ctx: *mut c_void) {
    // SAFETY: Guaranteed by the caller.
    unsafe { &*ctx.cast::<T>() }.prepare();
}

/// A reboot notifier. It is allocated unregistered, so that it can be a member of the state that
/// its callback works on, and registered once that state is complete.
pub(crate) struct RebootHook(NonNull<RawReboot>);

// SAFETY: The block is only changed by `register` and `unregister`, which are unsafe and require
// their callers not to run them concurrently; the notifier chain does not touch the handle's flag.
unsafe impl Send for RebootHook {}
// SAFETY: See above.
unsafe impl Sync for RebootHook {}

impl RebootHook {
    /// Allocates an unregistered notifier.
    pub(crate) fn new() -> Result<Self> {
        // SAFETY: No preconditions.
        let raw = unsafe { pxw3pe_reboot_new() };
        NonNull::new(raw).map(RebootHook).ok_or(ENOMEM)
    }

    /// Starts calling [`Reboot::prepare`] on `state`.
    ///
    /// # Safety
    ///
    /// `state` must outlive the registration: [`RebootHook::unregister`] must have returned, or
    /// the hook been dropped, before `state` is freed. Must not run concurrently with itself or
    /// [`RebootHook::unregister`].
    pub(crate) unsafe fn register<T: Reboot>(&self, state: Pin<&T>) -> Result {
        let ctx = core::ptr::from_ref(state.get_ref())
            .cast_mut()
            .cast::<c_void>();
        // SAFETY: The block is live and not registered yet (the driver registers once), the
        // caller rules out concurrent updates, `prepare_cb::<T>` accepts `ctx`, and the caller
        // keeps `state` alive and in place (it is pinned) until after the unregistration.
        to_result(unsafe { pxw3pe_reboot_register(self.0.as_ptr(), prepare_cb::<T>, ctx) })
    }

    /// Stops the notifications, waiting for a callback that is running. Does nothing if the hook
    /// is not registered.
    ///
    /// # Safety
    ///
    /// Must not run concurrently with itself or [`RebootHook::register`] (they update the block's
    /// registration flag unsynchronised), and not from the callback, which would deadlock.
    pub(crate) unsafe fn unregister(&self) {
        // SAFETY: The block is live, and the caller rules out concurrent updates of it.
        unsafe { pxw3pe_reboot_unregister(self.0.as_ptr()) };
    }
}

impl Drop for RebootHook {
    fn drop(&mut self) {
        // SAFETY: the block came from `pxw3pe_reboot_new` and is freed exactly once.
        unsafe { pxw3pe_reboot_free(self.0.as_ptr()) };
    }
}

/// State that is polled periodically.
pub(crate) trait Poll: Send + Sync {
    /// Called in process context, where it may sleep, from a workqueue. Calls do not overlap.
    fn poll(&self);
}

/// # Safety
///
/// `ctx` must point to a live `T` for the duration of the call.
unsafe extern "C" fn poll_cb<T: Poll>(ctx: *mut c_void) {
    // SAFETY: Guaranteed by the caller.
    unsafe { &*ctx.cast::<T>() }.poll();
}

/// A periodic call of [`Poll::poll`] on a state it shares with the rest of the driver. It is
/// allocated unstarted, so that it can be a member of the state its callback works on, and
/// started once that state is complete. Dropping it stops the calls.
///
/// The poller owns a reference to the state, which the C side holds as its context: the state
/// outlives every call without lifetime rules for the user, and the reference is only released
/// after the C side has stopped calling.
pub(crate) struct Poller<T: Poll> {
    raw: NonNull<RawPoller>,
    /// The pointer `Arc::into_raw` made of the owned reference; the C context.
    ctx: *const T,
}

// SAFETY: The handle is only used through `&self` calls that the C side serialises with the
// workqueue, and the context is an `Arc<T>` with `T: Send + Sync`.
unsafe impl<T: Poll> Send for Poller<T> {}
// SAFETY: See above.
unsafe impl<T: Poll> Sync for Poller<T> {}

impl<T: Poll> Poller<T> {
    /// Creates an unstarted poller that will call `ctx.poll()` every `period_ms`.
    pub(crate) fn new(ctx: Arc<T>, period_ms: u32) -> Result<Self> {
        let ctx = Arc::into_raw(ctx);
        // SAFETY: `poll_cb::<T>` accepts `ctx`, which stays valid until `drop` releases it, after
        // the poller has been freed.
        let raw = unsafe { pxw3pe_poller_new(poll_cb::<T>, ctx.cast_mut().cast(), period_ms) };
        match NonNull::new(raw) {
            Some(raw) => Ok(Poller { raw, ctx }),
            None => {
                // SAFETY: `ctx` came from `Arc::into_raw` above, and no poller exists that could
                // use it.
                drop(unsafe { Arc::from_raw(ctx) });
                Err(ENOMEM)
            }
        }
    }

    /// Starts the calls; the first is one period from now. Starting a started poller does
    /// nothing.
    pub(crate) fn start(&self) {
        // SAFETY: The poller is live until `drop`.
        unsafe { pxw3pe_poller_start(self.raw.as_ptr()) };
    }
}

impl<T: Poll> Drop for Poller<T> {
    fn drop(&mut self) {
        // SAFETY: The poller came from `pxw3pe_poller_new` and is freed exactly once. It is not
        // called from the callback: `Poll::poll` has no access to the poller.
        unsafe { pxw3pe_poller_free(self.raw.as_ptr()) };
        // SAFETY: `ctx` came from `Arc::into_raw` in `new` and is released exactly once, after the
        // C side has stopped using it.
        drop(unsafe { Arc::from_raw(self.ctx) });
    }
}
