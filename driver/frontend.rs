// SPDX-License-Identifier: GPL-2.0

//! The four tuners as DVB frontends: tuning, signal reports, the satellite frontends' LNB
//! requests (`lnb.rs`), and stream start/stop.

use crate::{
    dvb::{self, Feed, Signal},
    hw::{sleep_ms, Board},
    lnb::Lnb,
    proto::{
        i2c,
        isdb::{self, Hw},
        nmi::{self, Nmi},
        tc90522,
        tda2014x::Lna,
        ts::Routing,
    },
    stream::Stream,
};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use kernel::{
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
};

/// The TSID that makes the demodulator output any TS.
const ANY_TS: u16 = 0xFFFF;

/// The TSID user space asked for through DTV_STREAM_ID, if any.
///
/// The DVB API defines 1..=65535 as ISDB stream IDs and any other value as "no filter". 0xFFFF
/// is excluded as well: the demodulator takes it as "output any TS", which is no selection.
fn requested_tsid(stream_id: u32) -> Option<u16> {
    u16::try_from(stream_id)
        .ok()
        .filter(|&id| id != 0 && id != ANY_TS)
}

/// Chip-level access for the tuning logic. Every operation takes the I2C bus for itself only, so
/// that the sleeps in the tuning sequences do not block the other tuners.
struct BoardHw<'a> {
    board: &'a Board,
}

/// The 8-bit write address of the 7-bit address `addr`.
fn write_addr(addr: u8) -> u8 {
    addr << 1
}

impl Hw for BoardHw<'_> {
    type Error = i2c::Error;

    fn demod_write(&mut self, addr: u8, reg: u8, value: u8) -> Result<(), i2c::Error> {
        self.board.bus().write_reg(write_addr(addr), reg, &[value])
    }

    fn demod_read(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), i2c::Error> {
        self.board.bus().read_reg_rs(write_addr(addr), reg, buf)
    }

    fn gate_write(&mut self, addr: u8, tuner: u8, data: &[u8]) -> Result<(), i2c::Error> {
        self.board.bus().gate_write(write_addr(addr), tuner, data)
    }

    fn gate_read(
        &mut self,
        addr: u8,
        tuner: u8,
        sub: &[u8],
        buf: &mut [u8],
    ) -> Result<(), i2c::Error> {
        self.board
            .bus()
            .gate_read(write_addr(addr), tuner, sub, buf)
    }

    fn sleep_ms(&mut self, ms: u32) {
        sleep_ms(ms);
    }
}

/// Waits up to [`LNA_LOCK_WAIT_MS`] for the satellite core at `addr` to lock and
/// [`LNA_CN_SETTLE_MS`] more for its estimate to settle, then averages [`LNA_CN_SAMPLES`] C/N
/// readings [`LNA_CN_GAP_MS`] apart, in 0.001 dB; `None` without lock. The lock and the C/N are
/// the transponder's, whether or not it carries the TSID that was asked for.
fn cn_after_lock(hw: &mut BoardHw<'_>, addr: u8) -> Result<Option<i32>, isdb::Error<i2c::Error>> {
    let mut waited = 0;
    while isdb::status_s(hw, addr, None)?.fe_status & tc90522::FE_HAS_LOCK == 0 {
        if waited >= LNA_LOCK_WAIT_MS {
            return Ok(None);
        }
        sleep_ms(LNA_LOCK_POLL_MS);
        waited += LNA_LOCK_POLL_MS;
    }
    sleep_ms(LNA_CN_SETTLE_MS);
    let (mut sum, mut n) = (0i64, 0i64);
    for _ in 0..LNA_CN_SAMPLES {
        sleep_ms(LNA_CN_GAP_MS);
        if let Some(cn) = isdb::status_s(hw, addr, None)?.cnr_mdb {
            sum += i64::from(cn);
            n += 1;
        }
    }
    Ok((n > 0).then(|| (sum / n) as i32))
}

/// Finds the demodulator group of the board, and returns its 7-bit base address.
pub(crate) fn probe(board: &Board) -> Result<u8, isdb::ProbeError> {
    isdb::probe(&mut BoardHw { board })
}

fn error_code(e: isdb::Error<i2c::Error>) -> Error {
    match e {
        isdb::Error::Bus(_) | isdb::Error::Calibration => EIO,
        isdb::Error::UnsupportedChip(_) => ENODEV,
        isdb::Error::Frequency => EINVAL,
    }
}

/// Why a tuner could not be initialised.
pub(crate) struct InitError {
    /// The tuner (0..=3) whose core or tuner chip failed.
    pub(crate) tuner: usize,
    pub(crate) cause: isdb::Error<i2c::Error>,
}

impl InitError {
    /// The error to return to the kernel.
    pub(crate) fn errno(&self) -> Error {
        error_code(self.cause)
    }
}

/// How long the first lock of a satellite tuner is waited for before an LNA code counts as not
/// locking, in milliseconds; a BS transponder locks within 300 ms of the tune here.
const LNA_LOCK_WAIT_MS: u32 = 600;
const LNA_LOCK_POLL_MS: u32 = 20;
/// Time between lock and the first C/N sample: the demodulator's C/N estimate climbs for about
/// 150 ms after it locks (measured: 15.6 to 16.2 dB in that time, and from 7 dB on a re-lock),
/// so earlier samples read low, the code measured after a switch most of all.
const LNA_CN_SETTLE_MS: u32 = 150;
/// C/N samples averaged per LNA code, and their spacing. The estimate updates about every
/// 36 ms (measured), so samples closer than that repeat a value.
const LNA_CN_SAMPLES: u32 = 3;
const LNA_CN_GAP_MS: u32 = 40;
/// How much better, in 0.001 dB, `Lna::Full` has to measure to be chosen over `Lna::Least`:
/// the two differ only on a strong input, where `Least` wins by 1 to 2 dB, and are within the
/// sample spread of each other otherwise, so a small difference is noise and `Least` the choice
/// that cannot lose much.
const LNA_FULL_MARGIN_MDB: i32 = 300;

/// State of the two satellite tuners beyond the bus. They share one antenna input, so the LNA
/// choice is one for both, and switching the LNA gain of one tuner takes the other's carrier
/// away for up to 150 ms (measured on the card), so the comparison only runs while the other
/// frontend is closed.
struct SatState {
    /// The LNA code in use once a tune has let the two codes be compared (`choose_lna`); `None`
    /// until then, when a tune uses `Lna::Least`. Cleared when the last satellite frontend is
    /// closed, so that the next session measures the input it finds: the level drifts over hours on
    /// a shared antenna system.
    lna: Option<Lna>,
    /// Which satellite frontends are open (`Tuner::init` until `Tuner::sleep`).
    open: [bool; 2],
    /// The TSID each satellite tuner was last tuned to, `None` for any TS; the status reports lock
    /// only while the transponder carries it (`isdb::status_s`).
    tsid: [Option<u16>; 2],
}

/// What a tune leaves behind for the stream: how long the demodulator must have reported lock
/// before its packets are delivered (`Tuner::read_signal`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Tuned {
    pub(crate) settle_us: u64,
}

/// How long after the first lock report the packets are still dropped. The demodulator loses
/// single packets while it finishes acquiring, for up to 0.6 s after a terrestrial tune, and
/// for about a second after a change of the satellite LNA gain (measured on the card); the
/// status poll that sees the lock runs every 200 ms.
const LOCK_SETTLE_US: u64 = 500_000;
const LOCK_SETTLE_AFTER_GAIN_SWITCH_US: u64 = 1_000_000;

/// Per-card state shared by the tuners.
///
/// Lock order: a core's mutex, or the LNB mutex (`lnb::Lnb`), and then the I2C bus; never two core
/// mutexes, and never a core mutex inside the LNB mutex or the other way round.
#[pin_data]
pub(crate) struct Core {
    board: Arc<Board>,
    stream: Arc<Stream>,
    routing: Routing,
    /// The mutex of a demodulator core serialises everything done to the core and the tuner
    /// behind it. The satellite cores share one, with the LNA choice of their TDA2014x; each
    /// terrestrial core owns the state of its NM120, which exists once `init_chips` has run.
    #[pin]
    sat: Mutex<SatState>,
    #[pin]
    ter0: Mutex<Option<Nmi>>,
    #[pin]
    ter1: Mutex<Option<Nmi>>,
    /// The supply the satellite frontends request.
    lnb: Arc<Lnb>,
}

impl Core {
    /// Creates the shared state; the card must already be powered.
    pub(crate) fn new(
        board: Arc<Board>,
        stream: Arc<Stream>,
        routing: Routing,
        lnb: Arc<Lnb>,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Core {
                board,
                stream,
                routing,
                sat <- new_mutex!(SatState {
                    lna: None,
                    open: [false; 2],
                    tsid: [None; 2],
                }),
                ter0 <- new_mutex!(None),
                ter1 <- new_mutex!(None),
                lnb,
            }),
            GFP_KERNEL,
        )
    }

    fn hw(&self) -> BoardHw<'_> {
        BoardHw { board: &self.board }
    }

    /// The mutex of terrestrial tuner `tuner` (2 or 3).
    fn terrestrial(&self, tuner: usize) -> &Mutex<Option<Nmi>> {
        if tuner == 2 {
            &self.ter0
        } else {
            &self.ter1
        }
    }

    /// Initialises the demodulator cores and tuners of the whole card, each core followed by its
    /// tuner and every step checked. Each core and its tuner are then stopped until the frontend is
    /// opened.
    pub(crate) fn init_chips(&self) -> Result<(), InitError> {
        // In the order of the cores, as revealed.
        for tuner in [2, 0, 3, 1] {
            let addr = self.routing.demod_addr(tuner);
            let mut hw = self.hw();
            let result = if tuner < 2 {
                let _core = self.sat.lock();
                isdb::init_s(&mut hw, addr).and_then(|()| isdb::sleep_s(&mut hw, addr))
            } else {
                let mut state = self.terrestrial(tuner).lock();
                isdb::init_t(&mut hw, addr)
                    .and_then(|nmi| isdb::sleep_t(&mut hw, state.insert(nmi), addr))
            };
            result.map_err(|cause| InitError { tuner, cause })?;
        }
        Ok(())
    }

    /// Stops (`asleep`) or restarts the demodulator core of `tuner` and the tuner behind it. A
    /// stopped core sends no TS, so a closed frontend takes nothing from the port's rate and
    /// slack, and the chips of a closed frontend do not run.
    fn set_asleep(&self, tuner: usize, asleep: bool) -> Result {
        let addr = self.routing.demod_addr(tuner);
        let mut hw = self.hw();
        let result = if tuner < 2 {
            let _core = self.sat.lock();
            if asleep {
                isdb::sleep_s(&mut hw, addr)
            } else {
                isdb::wake_s(&mut hw, addr)
            }
        } else {
            let mut state = self.terrestrial(tuner).lock();
            let nmi = state.as_mut().ok_or(ENODEV)?;
            if asleep {
                isdb::sleep_t(&mut hw, nmi, addr)
            } else {
                isdb::wake_t(&mut hw, nmi, addr)
            }
        };
        result.map_err(error_code)
    }

    /// Tunes `tuner`; `frequency` and `stream_id` are those of the DVB property cache.
    fn tune(&self, tuner: usize, frequency: u32, stream_id: u32) -> Result<Tuned> {
        let addr = self.routing.demod_addr(tuner);
        let mut settle_us = LOCK_SETTLE_US;
        if tuner < 2 {
            let tsid = requested_tsid(stream_id);
            let mut state = self.sat.lock();
            state.tsid[tuner] = tsid;
            let lna = state.lna.unwrap_or(Lna::Least);
            isdb::tune_s(&mut self.hw(), addr, frequency, tsid.unwrap_or(ANY_TS), lna)
                .map_err(error_code)?;
            // The comparison switches this tuner's LNA gain twice, and each switch costs the
            // other tuner its carrier; it waits for a session that has the input to itself.
            if state.lna.is_none() && !state.open[tuner ^ 1] {
                let (chosen, switched) = self.choose_lna(tuner, addr).map_err(error_code)?;
                state.lna = chosen;
                if switched {
                    settle_us = LOCK_SETTLE_AFTER_GAIN_SWITCH_US;
                }
            }
        } else {
            let mut state = self.terrestrial(tuner).lock();
            let nmi = state.as_mut().ok_or(ENODEV)?;
            isdb::tune_t(&mut self.hw(), nmi, addr, frequency).map_err(error_code)?;
        }
        Ok(Tuned { settle_us })
    }

    /// Compares the two LNA codes on the signal just tuned with `Lna::Least` and leaves the one
    /// with the better C/N in the chip: `Full` if it locks where `Least` does not or measures
    /// better by [`LNA_FULL_MARGIN_MDB`], `Least` otherwise. `Least` is measured first, right after
    /// the tune's own lock, then `Full` after a switch; each gets the lock wait and the C/N samples
    /// of the `LNA_*` constants. Returns the choice (`None`, with `Least` back in place, if neither
    /// code locked: the next tune tries again) and whether the chip's gain was switched after the
    /// last measurement, which costs the demodulator about a second of sporadic packet loss
    /// (measured). Switching down to `Least` is the slower direction: the demodulator took more
    /// than the lock wait to lock again on the card, and its first readings after that were far too
    /// low, which is why `Least` is never measured after a switch. Runs in the DVB frontend thread
    /// with the core's mutex held, like a tune.
    fn choose_lna(
        &self,
        tuner: usize,
        addr: u8,
    ) -> Result<(Option<Lna>, bool), isdb::Error<i2c::Error>> {
        let mut hw = self.hw();
        let least = cn_after_lock(&mut hw, addr)?;
        isdb::set_lna_s(&mut hw, addr, Lna::Full)?;
        let full = cn_after_lock(&mut hw, addr)?;
        let chosen = match (least, full) {
            (None, None) => {
                isdb::set_lna_s(&mut hw, addr, Lna::Least)?;
                return Ok((None, true));
            }
            (None, Some(_)) => Lna::Full,
            (Some(_), None) => Lna::Least,
            (Some(l), Some(f)) if f >= l.saturating_add(LNA_FULL_MARGIN_MDB) => Lna::Full,
            _ => Lna::Least,
        };
        // The chip holds `Full` at this point.
        let switched = chosen != Lna::Full;
        if switched {
            isdb::set_lna_s(&mut hw, addr, Lna::Least)?;
        }
        pr_info!(
            "satellite tuner {}: C/N with LNA Least {:?} mdB, with Full {:?} mdB; using {:?}\n",
            tuner,
            least,
            full,
            chosen
        );
        Ok((Some(chosen), switched))
    }

    /// Records that the frontend of satellite tuner `tuner` is open.
    fn open_satellite(&self, tuner: usize) {
        self.sat.lock().open[tuner] = true;
    }

    /// Records that the frontend of satellite tuner `tuner` is closed; once both are, the LNA
    /// choice is forgotten and the next tune compares again.
    fn close_satellite(&self, tuner: usize) {
        let mut state = self.sat.lock();
        state.open[tuner] = false;
        if !state.open.iter().any(|&open| open) {
            state.lna = None;
        }
    }

    fn status(&self, tuner: usize) -> Result<isdb::Status> {
        let addr = self.routing.demod_addr(tuner);
        if tuner < 2 {
            let state = self.sat.lock();
            isdb::status_s(&mut self.hw(), addr, state.tsid[tuner]).map_err(error_code)
        } else {
            let _core = self.terrestrial(tuner).lock();
            isdb::status_t(&mut self.hw(), addr).map_err(error_code)
        }
    }
}

/// One tuner (0, 1: ISDB-S; 2, 3: ISDB-T) as a DVB frontend.
pub(crate) struct Tuner {
    index: usize,
    core: Arc<Core>,
    /// The last tune succeeded. The DVB core neither retries a failed tune nor remembers that it
    /// failed, so without this the next status polls would report the lock of whatever the
    /// hardware still receives, which is not what user space asked for.
    tuned: AtomicBool,
    /// `Board::now_us` of the first status poll since the last tune that saw the demodulator
    /// locked, 0 while it has not been; the stream stays muted until `settle_us` after it.
    locked_since_us: AtomicU64,
    /// Settle time the last tune asked for (`Tuned::settle_us`).
    settle_us: AtomicU64,
}

impl Tuner {
    /// Creates tuner `index` (0..=3).
    pub(crate) fn new(index: usize, core: Arc<Core>) -> Result<Arc<Self>> {
        Ok(Arc::new(
            Tuner {
                index,
                core,
                tuned: AtomicBool::new(false),
                locked_since_us: AtomicU64::new(0),
                settle_us: AtomicU64::new(LOCK_SETTLE_US),
            },
            GFP_KERNEL,
        )?)
    }

    fn satellite(&self) -> bool {
        self.index < 2
    }

    /// Adapter description; the frequency limits are those of the channel tables, as revealed.
    pub(crate) fn info(index: usize) -> dvb::Info {
        const NAMES: [&kernel::str::CStr; 4] = [
            kernel::c_str!("PX-W3PE V2 ISDB-S 0"),
            kernel::c_str!("PX-W3PE V2 ISDB-S 1"),
            kernel::c_str!("PX-W3PE V2 ISDB-T 0"),
            kernel::c_str!("PX-W3PE V2 ISDB-T 1"),
        ];
        if index < 2 {
            dvb::Info {
                name: NAMES[index],
                satellite: true,
                frequency_min_hz: isdb::S_IF_MIN_KHZ * 1000,
                frequency_max_hz: isdb::S_IF_MAX_KHZ * 1000,
                frequency_stepsize_hz: 1_000,
            }
        } else {
            dvb::Info {
                name: NAMES[index],
                satellite: false,
                frequency_min_hz: nmi::RF_MIN_HZ,
                frequency_max_hz: nmi::RF_MAX_HZ,
                frequency_stepsize_hz: 142_857,
            }
        }
    }
}

impl dvb::Frontend for Tuner {
    fn init(&self) -> Result {
        self.core.set_asleep(self.index, false)?;
        // A restarted core reacquires the channel it was tuned to, and what it sends meanwhile
        // has packets missing: it is not delivered before the demodulator has settled, as after
        // a tune. Without a tune since the load nothing is delivered before the first one.
        self.core.stream.set_muted(self.index, true);
        self.locked_since_us.store(0, Ordering::Relaxed);
        if self.satellite() {
            self.core.open_satellite(self.index);
        }
        Ok(())
    }

    // Closing stops the core, ends the LNB request and the session's LNA choice. The request is
    // withdrawn even if stopping the core failed.
    fn sleep(&self) -> Result {
        let stopped = self.core.set_asleep(self.index, true);
        if self.satellite() {
            self.core.close_satellite(self.index);
            stopped.and(self.core.lnb.request(self.index, false))
        } else {
            stopped
        }
    }

    fn tune(&self, frequency: u32, stream_id: u32) -> Result {
        self.tuned.store(false, Ordering::Relaxed);
        // Nothing the demodulator emits before it has settled on the new signal is delivered;
        // `read_signal` lifts this. A tune that fails leaves the stream muted: there is no
        // signal to deliver.
        self.core.stream.set_muted(self.index, true);
        self.locked_since_us.store(0, Ordering::Relaxed);
        let result = self.core.tune(self.index, frequency, stream_id);
        self.tuned.store(result.is_ok(), Ordering::Relaxed);
        let tuned = result?;
        self.settle_us.store(tuned.settle_us, Ordering::Relaxed);
        Ok(())
    }

    fn read_signal(&self) -> Result<Signal> {
        self.core.stream.recover(self.index);
        if !self.tuned.load(Ordering::Relaxed) {
            return Ok(Signal::default());
        }
        let status = self.core.status(self.index)?;
        if status.fe_status & tc90522::FE_HAS_LOCK == 0 {
            // The settle time runs from the start of an unbroken lock.
            self.locked_since_us.store(0, Ordering::Relaxed);
        } else {
            let now = self.core.board.now_us();
            match self.locked_since_us.load(Ordering::Relaxed) {
                0 => self.locked_since_us.store(now, Ordering::Relaxed),
                since if now >= since + self.settle_us.load(Ordering::Relaxed) => {
                    self.core.stream.set_muted(self.index, false);
                }
                _ => {}
            }
        }
        let mut signal = Signal {
            status: status.fe_status,
            ..Signal::default()
        };
        if let Some(cnr) = status.cnr_mdb {
            signal.cnr_mdb = cnr;
            signal.cnr_valid = 1;
        }
        Ok(signal)
    }

    fn set_lnb(&self, on: bool) -> Result {
        if !self.satellite() {
            return Err(EINVAL);
        }
        self.core.lnb.request(self.index, on)
    }

    fn start_streaming(&self, feed: Feed) -> Result {
        self.core.stream.start(self.index, feed)
    }

    fn stop_streaming(&self) {
        self.core.stream.stop(self.index);
    }
}
