// SPDX-License-Identifier: GPL-2.0

//! The 15 V LNB supply: the requests of the two satellite frontends, the short protection, and the
//! status device `/dev/pxw3pe-lnbN` (`uapi/pxw3pe_lnb.h`).
//!
//! The supply is switched only through the frontends (`FE_SET_VOLTAGE`); the device node reports
//! its state. An open file owns a reference to [`Lnb`], which reaches only the board registers,
//! so a file left open across driver removal keeps nothing else alive and finds the supply
//! disabled.
//!
//! The node also gives raw access to the hardware for testing: bridge registers, I2C registers
//! and tuner registers behind a demodulator gateway (`uapi/pxw3pe_lnb.h`). It is the board-wide
//! node that belongs to no frontend. These calls take the I2C bus as the driver does, so they
//! never split a gateway access, but they can come between two accesses of a driver sequence,
//! and nothing the driver knows about the chips is updated by them.

use crate::{
    bringup, chrdev,
    hw::{Board, BAR0_SIZE, GATE_MAX},
    platform,
    proto::i2c,
};
use core::ffi::{c_long, c_uint, c_ulong, c_void};
use kernel::{
    c_str,
    ioctl::{_IOR, _IOWR},
    new_mutex,
    prelude::*,
    str::CStr,
    sync::{Arc, ArcBorrow, Mutex},
    transmute::{AsBytes, FromBytes},
    types::ForeignOwnable,
    uaccess::{UserPtr, UserSlice},
};

/// Period of the supply check, in milliseconds.
pub(crate) const POLL_MS: u32 = 100;

/// Time after switch-on during which the supply's power-good bit is not judged. The settle time
/// of the supply is not known; judging the first sample would trip on the switch-on transient. To
/// measure on the card.
const SETTLE_US: u64 = 200_000;

/// Consecutive samples without power good that count as a short. One sample can be a glitch at
/// switch-on, so a single one is not judged a short.
const BAD_SAMPLES: u8 = 2;

/// State of the supply.
struct State {
    /// Requests of the two satellite tuners; the supply is on while either wants it.
    wanted: [bool; 2],
    /// Whether the supply is on, `None` when unknown: a switch that failed part-way (an I2C
    /// timeout) may have left it in either state. Unknown differs from every request, so the
    /// next one switches again.
    on: Option<bool>,
    /// The supply was switched off for good (driver removal, reboot); it cannot be requested
    /// any more.
    disabled: bool,
    /// The supply was switched off because of a short. It stays off until both requests have
    /// been withdrawn.
    fault: bool,
    /// `Board::now_us` of the last switch-on.
    on_since_us: u64,
    /// Consecutive samples without power good.
    bad_samples: u8,
}

/// The supply of one board.
///
/// Lock order: the LNB mutex, then the I2C bus; never together with a demodulator core's mutex.
#[pin_data]
pub(crate) struct Lnb {
    board: Arc<Board>,
    #[pin]
    state: Mutex<State>,
}

impl Lnb {
    /// Creates the bookkeeping; the supply must already be switched off
    /// (`bringup::lnb_switch`), which is what it starts from.
    pub(crate) fn new(board: Arc<Board>) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Lnb {
                board,
                state <- new_mutex!(State {
                    wanted: [false; 2],
                    on: Some(false),
                    disabled: false,
                    fault: false,
                    on_since_us: 0,
                    bad_samples: 0,
                }),
            }),
            GFP_KERNEL,
        )
    }

    /// Writes the supply state. `state` is the locked state; it records `None` for the duration
    /// of the write, which stays if the write fails.
    fn apply(&self, state: &mut State, on: bool) -> Result {
        state.on = None;
        bringup::lnb_switch(&self.board, on)?;
        state.on = Some(on);
        if on {
            state.on_since_us = self.board.now_us();
            state.bad_samples = 0;
        }
        Ok(())
    }

    /// Switches the supply off for good (driver removal, reboot). The switch is made whatever
    /// the bookkeeping says: after a failed switch the supply may be on.
    pub(crate) fn off(&self) {
        let mut state = self.state.lock();
        state.disabled = true;
        state.wanted = [false; 2];
        if self.apply(&mut state, false).is_err() {
            pr_warn!("LNB supply may still be on\n");
        }
    }

    /// Refuses every later request and status call without touching the hardware. For the
    /// status device, which may outlive the board: whoever tears the board down switches the
    /// supply off (`off`, or probe's cleanup when probe fails before the driver state exists),
    /// and this may run after the board has been powered down.
    fn retire(&self) {
        let mut state = self.state.lock();
        state.disabled = true;
        state.wanted = [false; 2];
    }

    /// Records the request of satellite tuner `satellite` (0 or 1) and brings the supply in line
    /// with the requests of both.
    pub(crate) fn request(&self, satellite: usize, on: bool) -> Result {
        let mut state = self.state.lock();
        if state.disabled {
            return Err(ENODEV);
        }
        state.wanted[satellite] = on;
        if !state.wanted.iter().any(|&w| w) {
            // Both requests are withdrawn, which also ends a fault.
            state.fault = false;
            if state.on != Some(false) {
                self.apply(&mut state, false)?;
            }
            return Ok(());
        }
        if state.fault {
            // The supply stays off. Withdrawing one of two requests is no request.
            return if on { Err(EIO) } else { Ok(()) };
        }
        if state.on != Some(true) {
            self.apply(&mut state, true)?;
        }
        Ok(())
    }

    /// One check of the supply, every [`POLL_MS`].
    ///
    /// A switch that failed leaves the supply in an unknown state, and nothing else would switch
    /// it again (a close that failed to switch it off is not repeated), so every check first
    /// brings the supply in line with what it should be: off once disabled, after a short, or
    /// without a request; on otherwise. A supply that has then been on for the settle time and
    /// reports no power good in [`BAD_SAMPLES`] consecutive samples is switched off, and stays
    /// off until both requests have been withdrawn.
    fn check(&self) {
        let mut state = self.state.lock();
        let target = !state.disabled && !state.fault && state.wanted.iter().any(|&w| w);
        if state.on != Some(target) {
            let _ = self.apply(&mut state, target);
            return;
        }
        if !target || self.board.now_us().saturating_sub(state.on_since_us) < SETTLE_US {
            return;
        }
        // A failed read counts as neither a good nor a bad sample.
        match bringup::lnb_status(&self.board) {
            Err(_) => {}
            Ok(true) => state.bad_samples = 0,
            Ok(false) => {
                state.bad_samples += 1;
                if state.bad_samples >= BAD_SAMPLES {
                    state.fault = true;
                    pr_err!("LNB supply reports no power good, switched off\n");
                    if self.apply(&mut state, false).is_err() {
                        pr_warn!("LNB supply may still be on\n");
                    }
                }
            }
        }
    }

    /// The flags of `GET_STATUS`. The power-good bit is read now, and only while the supply is
    /// on: it means nothing otherwise.
    fn status(&self) -> Result<u32> {
        let state = self.state.lock();
        if state.disabled {
            return Err(ENODEV);
        }
        let mut flags = match state.on {
            Some(true) if bringup::lnb_status(&self.board)? => ON | POWER_GOOD,
            Some(true) => ON,
            Some(false) => 0,
            None => UNKNOWN,
        };
        if state.fault {
            flags |= FAULT;
        }
        if state.wanted[0] {
            flags |= REQUEST_S0;
        }
        if state.wanted[1] {
            flags |= REQUEST_S1;
        }
        Ok(flags)
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> Result {
        let user = UserSlice::new(UserPtr::from_addr(arg), kernel::ioctl::_IOC_SIZE(cmd));
        match cmd {
            GET_STATUS => user.writer().write(&StatusArg {
                flags: self.status()?,
            }),
            RAW_REG => self.raw(user, Self::raw_reg),
            RAW_I2C => self.raw(user, Self::raw_i2c),
            RAW_GATE => self.raw(user, Self::raw_gate),
            _ => Err(ENOTTY),
        }
    }

    /// Runs one of the raw access calls: checks the caller's privilege and that the board is
    /// still there, then reads the argument, applies `op` and writes the argument back.
    fn raw<T: FromBytes + AsBytes>(
        &self,
        user: UserSlice,
        op: fn(&Self, &mut T) -> Result,
    ) -> Result {
        // SAFETY: A plain query of the current task's credentials.
        if !unsafe { chrdev::pxw3pe_capable_rawio() } {
            return Err(EPERM);
        }
        // After `retire` the board may be powered down; the register and bus accesses would fail
        // on their own once the device is unbound, but not during the power-down itself.
        if self.state.lock().disabled {
            return Err(ENODEV);
        }
        let (mut reader, mut writer) = user.reader_writer();
        let mut arg: T = reader.read()?;
        op(self, &mut arg)?;
        writer.write(&arg)
    }

    fn raw_reg(&self, a: &mut RegArg) -> Result {
        let offset = a.offset as usize;
        if offset % 4 != 0 || offset + 4 > BAR0_SIZE {
            return Err(EINVAL);
        }
        match a.write {
            0 => a.value = self.board.read32(offset).ok_or(ENODEV)?,
            1 => self.board.write32(a.value, offset).ok_or(ENODEV)?,
            _ => return Err(EINVAL),
        }
        Ok(())
    }

    fn raw_i2c(&self, a: &mut I2cArg) -> Result {
        let addr = seven_bit(a.addr)?;
        let reg = u8::try_from(a.reg).map_err(|_| EINVAL)?;
        let len = data_len(a.len, 1)?;
        let mut bus = self.board.bus();
        let result = match a.write {
            0 => bus.read_reg_rs(addr << 1, reg, &mut a.data[..len]),
            1 => bus.write_reg(addr << 1, reg, &a.data[..len]),
            _ => return Err(EINVAL),
        };
        a.status = bus_status(result)?;
        Ok(())
    }

    fn raw_gate(&self, a: &mut GateArg) -> Result {
        let demod = seven_bit(a.demod)?;
        let tuner = u8::try_from(a.tuner).map_err(|_| EINVAL)?;
        // The read address is the write address with bit 0 set.
        if tuner & 1 != 0 {
            return Err(EINVAL);
        }
        let len = data_len(a.len, 1)?;
        let read_len = data_len(a.read_len, 0)?;
        let payload = a.data;
        let mut bus = self.board.bus();
        let result = if read_len == 0 {
            bus.gate_write(demod << 1, tuner, &payload[..len])
        } else {
            bus.gate_read(demod << 1, tuner, &payload[..len], &mut a.data[..read_len])
        };
        a.status = bus_status(result)?;
        Ok(())
    }
}

/// The 7-bit I2C address `addr`.
fn seven_bit(addr: u32) -> Result<u8> {
    u8::try_from(addr).ok().filter(|&a| a <= 0x7F).ok_or(EINVAL)
}

/// The data length `len` of a raw call, at least `min` and at most [`RAW_DATA`].
fn data_len(len: u32, min: usize) -> Result<usize> {
    usize::try_from(len)
        .ok()
        .filter(|l| (min..=RAW_DATA).contains(l))
        .ok_or(EINVAL)
}

/// The `status` of a raw I2C call: 0, or the positive errno of a transaction the bus refused. A
/// device that does not answer is a result of the call, not a failure of it; an unbound board is.
fn bus_status(result: Result<(), i2c::Error>) -> Result<u32> {
    let code = match result {
        Ok(()) => return Ok(0),
        Err(i2c::Error::Nack) => EIO,
        Err(i2c::Error::Timeout) => ETIMEDOUT,
        Err(i2c::Error::Gone) => return Err(ENODEV),
        // The lengths are checked before the bus is taken.
        Err(i2c::Error::TooLong | i2c::Error::Invalid) => return Err(EINVAL),
    };
    Ok(code.to_errno().unsigned_abs())
}

impl platform::Poll for Lnb {
    fn poll(&self) {
        self.check();
    }
}

// Mirrors of `uapi/pxw3pe_lnb.h`.

const IOC: u32 = 0xBA;
const ON: u32 = 1 << 0;
const UNKNOWN: u32 = 1 << 1;
const POWER_GOOD: u32 = 1 << 2;
const FAULT: u32 = 1 << 3;
const REQUEST_S0: u32 = 1 << 4;
const REQUEST_S1: u32 = 1 << 5;

#[repr(C)]
struct StatusArg {
    flags: u32,
}

const _: () = assert!(core::mem::size_of::<StatusArg>() == 4);

// SAFETY: A `repr(C)` struct of one integer: no padding, every byte initialised.
unsafe impl AsBytes for StatusArg {}

const GET_STATUS: u32 = _IOR::<StatusArg>(IOC, 0x10);

/// Data bytes of a raw I2C or gateway call.
const RAW_DATA: usize = 16;
const _: () = assert!(RAW_DATA <= GATE_MAX);

#[repr(C)]
struct RegArg {
    offset: u32,
    value: u32,
    write: u32,
}

#[repr(C)]
struct I2cArg {
    addr: u32,
    reg: u32,
    len: u32,
    write: u32,
    status: u32,
    data: [u8; RAW_DATA],
}

#[repr(C)]
struct GateArg {
    demod: u32,
    tuner: u32,
    len: u32,
    read_len: u32,
    status: u32,
    data: [u8; RAW_DATA],
}

const _: () = assert!(core::mem::size_of::<RegArg>() == 12);
const _: () = assert!(core::mem::size_of::<I2cArg>() == 36);
const _: () = assert!(core::mem::size_of::<GateArg>() == 36);

// SAFETY: `repr(C)` structs of `u32` fields followed by a byte array whose length is a multiple
// of 4: no padding, and every bit pattern is valid.
unsafe impl AsBytes for RegArg {}
// SAFETY: As above.
unsafe impl FromBytes for RegArg {}
// SAFETY: As above.
unsafe impl AsBytes for I2cArg {}
// SAFETY: As above.
unsafe impl FromBytes for I2cArg {}
// SAFETY: As above.
unsafe impl AsBytes for GateArg {}
// SAFETY: As above.
unsafe impl FromBytes for GateArg {}

const RAW_REG: u32 = _IOWR::<RegArg>(IOC, 0x80);
const RAW_I2C: u32 = _IOWR::<I2cArg>(IOC, 0x83);
const RAW_GATE: u32 = _IOWR::<GateArg>(IOC, 0x84);

static OPS: chrdev::Ops = chrdev::Ops {
    open: open_cb,
    release: release_cb,
    ioctl: ioctl_cb,
    poll: None,
};

/// # Safety
///
/// Called by the glue with the registration context, while the device is registered.
unsafe extern "C" fn open_cb(ctx: *mut c_void) -> *mut c_void {
    // SAFETY: `LnbDevice::new` passes `Arc::as_ptr` of its `Lnb`, which it keeps alive until
    // after the unregistration. `misc_open` calls this with `misc_mtx` held, and
    // `misc_deregister`, which the unregistration calls first, takes the same mutex, so the
    // registration exists for the whole call.
    let lnb = Arc::from(unsafe { ArcBorrow::<Lnb>::from_raw(ctx.cast::<Lnb>()) });
    // The file owns a reference, so it keeps the state, and through it the board, alive even if
    // the PCI device is unbound while the file is open.
    lnb.into_foreign()
}

/// # Safety
///
/// Called by the glue with a value returned by `open_cb`, once.
unsafe extern "C" fn release_cb(file: *mut c_void) {
    // SAFETY: Guaranteed by the caller, and no `ioctl_cb` call on the file is running any more:
    // the VFS releases a file after its last use.
    drop(unsafe { <Arc<Lnb> as ForeignOwnable>::from_foreign(file) });
}

/// # Safety
///
/// Called by the glue with a value returned by `open_cb` that `release_cb` has not been called
/// with, and with `arg` as passed by user space.
unsafe extern "C" fn ioctl_cb(file: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: Guaranteed by the caller; the file is released only after its last ioctl returned.
    let lnb = unsafe { <Arc<Lnb> as ForeignOwnable>::borrow(file) };
    match lnb.ioctl(cmd, arg as usize) {
        Ok(()) => 0,
        Err(e) => c_long::from(e.to_errno()),
    }
}

/// Device node names, by board number.
const NAMES: [&CStr; chrdev::MAX_BOARDS] = [
    c_str!("pxw3pe-lnb0"),
    c_str!("pxw3pe-lnb1"),
    c_str!("pxw3pe-lnb2"),
    c_str!("pxw3pe-lnb3"),
    c_str!("pxw3pe-lnb4"),
    c_str!("pxw3pe-lnb5"),
    c_str!("pxw3pe-lnb6"),
    c_str!("pxw3pe-lnb7"),
];

/// Registered status device of one supply.
pub(crate) struct LnbDevice {
    // Dropped before `lnb`: no `open_cb` runs once the registration has gone.
    _registration: chrdev::Registration,
    lnb: Arc<Lnb>,
}

impl LnbDevice {
    /// Registers the status device of `lnb`, named after `slot`, which has to outlive the
    /// device.
    pub(crate) fn new(lnb: Arc<Lnb>, slot: &chrdev::Slot) -> Result<Self> {
        let ctx = Arc::as_ptr(&lnb).cast_mut().cast::<c_void>();
        // SAFETY: The callbacks of `OPS` take `ctx` as an `Lnb` registration context and what
        // `open_cb` returns as a file, and `ctx` points into `lnb`, which the `LnbDevice` keeps
        // alive until after the unregistration.
        let registration = unsafe { chrdev::Registration::new(NAMES[slot.index()], &OPS, ctx)? };
        Ok(LnbDevice {
            _registration: registration,
            lnb,
        })
    }
}

impl Drop for LnbDevice {
    fn drop(&mut self) {
        // Open files outlive the registration and must find the supply gone, also when probe
        // fails before `off` could be called. The registration goes with the fields, afterwards.
        self.lnb.retire();
    }
}
