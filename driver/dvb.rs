// SPDX-License-Identifier: GPL-2.0

//! Rust side of `dvb_glue.h`: one DVB adapter (frontend, demux, dvr) per tuner.
//!
//! The declarations below mirror the C header; any change there must be made here too.

use crate::platform;
use core::{
    ffi::{c_int, c_void},
    mem::{offset_of, size_of},
    ptr::NonNull,
};
use kernel::{device, error::from_err_ptr, ffi::c_char, prelude::*, str::CStr, sync::Arc};

/// Tuning request (`struct pxw3pe_tune`).
#[repr(C)]
struct RawTune {
    frequency: u32,
    stream_id: u32,
}

/// Signal report (`struct pxw3pe_signal`).
#[repr(C)]
#[derive(Default)]
pub(crate) struct Signal {
    /// `enum fe_status` bits.
    pub(crate) status: u32,
    /// C/N in 0.001 dB, meaningful when `cnr_valid` is set.
    pub(crate) cnr_mdb: i32,
    /// Whether `cnr_mdb` holds a measurement.
    pub(crate) cnr_valid: u8,
}

/// `struct pxw3pe_dvb_ops`.
#[repr(C)]
struct RawOps {
    init: unsafe extern "C" fn(*mut c_void) -> c_int,
    sleep: unsafe extern "C" fn(*mut c_void) -> c_int,
    tune: unsafe extern "C" fn(*mut c_void, *const RawTune) -> c_int,
    read_signal: unsafe extern "C" fn(*mut c_void, *mut Signal) -> c_int,
    set_lnb: unsafe extern "C" fn(*mut c_void, bool) -> c_int,
    start_streaming: unsafe extern "C" fn(*mut c_void, *mut RawDvb) -> c_int,
    stop_streaming: unsafe extern "C" fn(*mut c_void),
}

/// `struct pxw3pe_dvb_info`.
#[repr(C)]
struct RawInfo {
    name: *const c_char,
    satellite: bool,
    frequency_min_hz: u32,
    frequency_max_hz: u32,
    frequency_stepsize_hz: u32,
}

// The layout `dvb_glue.c` asserts for the C structures, so a structure changed on one side only
// fails the build; change both together.
const PTR: usize = size_of::<*const c_void>();
const _: () = {
    assert!(size_of::<RawTune>() == 8);
    assert!(offset_of!(RawTune, stream_id) == 4);
    assert!(size_of::<Signal>() == 12);
    assert!(offset_of!(Signal, cnr_mdb) == 4);
    assert!(offset_of!(Signal, cnr_valid) == 8);
    assert!(size_of::<RawOps>() == 7 * PTR);
    assert!(offset_of!(RawOps, sleep) == PTR);
    assert!(offset_of!(RawOps, tune) == 2 * PTR);
    assert!(offset_of!(RawOps, read_signal) == 3 * PTR);
    assert!(offset_of!(RawOps, set_lnb) == 4 * PTR);
    assert!(offset_of!(RawOps, start_streaming) == 5 * PTR);
    assert!(offset_of!(RawOps, stop_streaming) == 6 * PTR);
    assert!(size_of::<RawInfo>() == PTR + 16);
    assert!(offset_of!(RawInfo, satellite) == PTR);
    assert!(offset_of!(RawInfo, frequency_min_hz) == PTR + 4);
    assert!(offset_of!(RawInfo, frequency_max_hz) == PTR + 8);
    assert!(offset_of!(RawInfo, frequency_stepsize_hz) == PTR + 12);
};

/// Opaque `struct pxw3pe_dvb`.
#[repr(C)]
struct RawDvb {
    _opaque: [u8; 0],
}

extern "C" {
    // `struct device *` and `struct module *`, passed through opaquely.
    fn pxw3pe_dvb_register(
        dev: *mut c_void,
        owner: *mut c_void,
        info: *const RawInfo,
        ops: *const RawOps,
        ctx: *mut c_void,
    ) -> *mut RawDvb;
    fn pxw3pe_dvb_unregister(dvb: *mut RawDvb);
    fn pxw3pe_dvb_feed(dvb: *mut RawDvb, packets: *const u8, count: usize);
    fn pxw3pe_dvb_adapter_num(dvb: *const RawDvb) -> c_int;
}

/// Callbacks of one adapter. All run in process context and may sleep.
pub(crate) trait Frontend: Send + Sync {
    /// The frontend device was opened.
    fn init(&self) -> Result;
    /// The frontend device has been idle since its last close.
    fn sleep(&self) -> Result;
    /// Tunes. `frequency` is in Hz for ISDB-T and kHz (IF) for ISDB-S, as in the DVB API.
    fn tune(&self, frequency: u32, stream_id: u32) -> Result;
    /// Reports the current signal state.
    fn read_signal(&self) -> Result<Signal>;
    /// Switches the LNB supply (satellite adapters only).
    fn set_lnb(&self, on: bool) -> Result;
    /// The first feed of the demux started; `feed` hands packets to it. The handle stays valid
    /// until [`Frontend::stop_streaming`] has returned.
    fn start_streaming(&self, feed: Feed) -> Result;
    /// The last feed of the demux stopped.
    fn stop_streaming(&self);
}

fn to_c(result: Result) -> c_int {
    match result {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// # Safety
///
/// `ctx` must point to a live `T` (guaranteed by [`Adapter`] until unregistration).
unsafe fn frontend<'a, T: Frontend>(ctx: *mut c_void) -> &'a T {
    // SAFETY: Guaranteed by the caller.
    unsafe { &*ctx.cast::<T>() }
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered.
unsafe extern "C" fn init_cb<T: Frontend>(ctx: *mut c_void) -> c_int {
    // SAFETY: The glue passes the context given at registration while the adapter is registered.
    to_c(unsafe { frontend::<T>(ctx) }.init())
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered.
unsafe extern "C" fn sleep_cb<T: Frontend>(ctx: *mut c_void) -> c_int {
    // SAFETY: As in `init_cb`.
    to_c(unsafe { frontend::<T>(ctx) }.sleep())
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered,
/// and with `tune` pointing to an object of the glue that is valid for the call.
unsafe extern "C" fn tune_cb<T: Frontend>(ctx: *mut c_void, tune: *const RawTune) -> c_int {
    // SAFETY: As in `init_cb`; `tune` points to a stack object of the glue for this call.
    let (fe, tune) = unsafe { (frontend::<T>(ctx), &*tune) };
    to_c(fe.tune(tune.frequency, tune.stream_id))
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered,
/// and with `out` pointing to an object of the glue that is valid for the call.
unsafe extern "C" fn read_signal_cb<T: Frontend>(ctx: *mut c_void, out: *mut Signal) -> c_int {
    // SAFETY: As in `init_cb`.
    match unsafe { frontend::<T>(ctx) }.read_signal() {
        Ok(signal) => {
            // SAFETY: `out` points to a zero-initialised object of the glue for this call.
            unsafe { out.write(signal) };
            0
        }
        Err(e) => e.to_errno(),
    }
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered.
unsafe extern "C" fn set_lnb_cb<T: Frontend>(ctx: *mut c_void, on: bool) -> c_int {
    // SAFETY: As in `init_cb`.
    to_c(unsafe { frontend::<T>(ctx) }.set_lnb(on))
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered,
/// and with `dvb` being the adapter it was registered as.
unsafe extern "C" fn start_cb<T: Frontend>(ctx: *mut c_void, dvb: *mut RawDvb) -> c_int {
    // SAFETY: The glue passes its own adapter, which is never null (it is allocated by
    // `pxw3pe_dvb_register` and `feed->demux->priv` points to it).
    let feed = Feed(unsafe { NonNull::new_unchecked(dvb) });
    // SAFETY: As in `init_cb`.
    to_c(unsafe { frontend::<T>(ctx) }.start_streaming(feed))
}

/// # Safety
///
/// Called by the glue with the context passed at registration, while the adapter is registered.
unsafe extern "C" fn stop_cb<T: Frontend>(ctx: *mut c_void) {
    // SAFETY: As in `init_cb`.
    unsafe { frontend::<T>(ctx) }.stop_streaming()
}

/// Static description of an adapter.
pub(crate) struct Info {
    /// Adapter and frontend name; the DVB core keeps the pointer, hence `'static`.
    pub(crate) name: &'static CStr,
    /// ISDB-S (true) or ISDB-T.
    pub(crate) satellite: bool,
    /// Lowest tunable frequency in Hz.
    pub(crate) frequency_min_hz: u32,
    /// Highest tunable frequency in Hz.
    pub(crate) frequency_max_hz: u32,
    /// Tuning step in Hz.
    pub(crate) frequency_stepsize_hz: u32,
}

/// Handle through which the stream path hands packets to an adapter's demux.
#[derive(Clone, Copy)]
pub(crate) struct Feed(NonNull<RawDvb>);

// SAFETY: `pxw3pe_dvb_feed` only calls `dvb_dmx_swfilter_packets`, which takes the demux lock and
// may be called from any thread. A feed is handed out by `start_streaming` and dropped by its
// users when `stop_streaming` is called, which happens before the adapter is unregistered
// (`pxw3pe_dvb_unregister` waits for the demux and dvr files to close, which stops their feeds).
unsafe impl Send for Feed {}
// SAFETY: See above.
unsafe impl Sync for Feed {}

impl Feed {
    /// Delivers whole 188-byte packets.
    pub(crate) fn packets(&self, data: &[u8]) {
        // SAFETY: The adapter is registered (see the `Send` impl) and `data` is valid for reads.
        unsafe { pxw3pe_dvb_feed(self.0.as_ptr(), data.as_ptr(), data.len() / 188) };
    }
}

/// A registered DVB adapter. Dropping it unregisters the adapter: that waits for its demux and
/// dvr files to be closed and for callbacks still running, after which the glue calls no driver
/// callback any more (a frontend file that stays open gets ENODEV), and only then is the context
/// released.
pub(crate) struct Adapter<T: Frontend> {
    raw: NonNull<RawDvb>,
    _ops: KBox<RawOps>,
    _ctx: Arc<T>,
}

// SAFETY: The raw handle is only used for unregistration, which may happen on any thread.
unsafe impl<T: Frontend> Send for Adapter<T> {}
// SAFETY: `&Adapter` only exposes the adapter number.
unsafe impl<T: Frontend> Sync for Adapter<T> {}

impl<T: Frontend + 'static> Adapter<T> {
    /// Registers an adapter whose callbacks go to `ctx`.
    pub(crate) fn register(
        dev: &device::Device<device::Bound>,
        module: &'static ThisModule,
        info: &Info,
        ctx: Arc<T>,
    ) -> Result<Self> {
        let ops = KBox::new(
            RawOps {
                init: init_cb::<T>,
                sleep: sleep_cb::<T>,
                tune: tune_cb::<T>,
                read_signal: read_signal_cb::<T>,
                set_lnb: set_lnb_cb::<T>,
                start_streaming: start_cb::<T>,
                stop_streaming: stop_cb::<T>,
            },
            GFP_KERNEL,
        )?;
        let raw_info = RawInfo {
            name: info.name.as_char_ptr(),
            satellite: info.satellite,
            frequency_min_hz: info.frequency_min_hz,
            frequency_max_hz: info.frequency_max_hz,
            frequency_stepsize_hz: info.frequency_stepsize_hz,
        };
        let ctx_ptr = Arc::as_ptr(&ctx).cast_mut().cast::<c_void>();
        let raw_dev = platform::raw_device(dev);
        // SAFETY: All pointers are valid for the call; `ops` and `ctx` are kept alive by the
        // returned value until after `pxw3pe_dvb_unregister`.
        let raw = from_err_ptr(unsafe {
            pxw3pe_dvb_register(
                raw_dev,
                module.as_ptr().cast::<c_void>(),
                &raw_info,
                &*ops,
                ctx_ptr,
            )
        })?;
        Ok(Adapter {
            raw: NonNull::new(raw).ok_or(ENOMEM)?,
            _ops: ops,
            _ctx: ctx,
        })
    }

    /// Number of the DVB adapter.
    pub(crate) fn number(&self) -> i32 {
        // SAFETY: The adapter is registered while `self` lives.
        unsafe { pxw3pe_dvb_adapter_num(self.raw.as_ptr()) }
    }
}

impl<T: Frontend> Drop for Adapter<T> {
    fn drop(&mut self) {
        // SAFETY: `raw` came from a successful registration and is unregistered exactly once.
        unsafe { pxw3pe_dvb_unregister(self.raw.as_ptr()) };
    }
}
