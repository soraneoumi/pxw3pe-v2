// SPDX-License-Identifier: GPL-2.0

//! Register access to the ASV5220 bridge shared by all parts of the driver: BAR0, the GPIO
//! registers and the I2C bus.

use crate::proto::i2c::{self, Engine};
use kernel::{
    c_str,
    device::Core,
    devres::Devres,
    new_mutex, new_spinlock, pci,
    prelude::*,
    sync::{lock::mutex::MutexGuard, Arc, Mutex, SpinLock},
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

/// BAR0 extent covering every register the driver uses; the last is the card UART's 0xC9C
/// (`proto::uart`). Mapping fails on a BAR that is shorter. A memory BAR's size is a power of two,
/// and registers above 0x800 mean at least 0x1000, so this asks for no more than every board has.
pub(crate) const BAR0_SIZE: usize = 0xCA0;

/// BAR0 of the bridge.
pub(crate) type Bar0 = pci::Bar<BAR0_SIZE>;

/// Bridge GPIO register. Its bits belong to different functions (card reset, card detect, tuner
/// and demodulator reset and power), so every read-modify-write of it is done under one lock.
pub(crate) const GPIO: usize = 0x890;

/// Second GPIO output register (virtual GPIO bits 1..7 in its low byte).
const GPIO_OUT: usize = 0x894;

/// Per-card shared state: the mapped registers and the locks that order access to them.
#[pin_data]
pub(crate) struct Board {
    #[pin]
    bar: Devres<Bar0>,
    /// Orders read-modify-writes of [`GPIO`] and `GPIO_OUT`.
    #[pin]
    gpio: SpinLock<()>,
    /// The I2C bus: one transaction sequence at a time.
    #[pin]
    i2c: Mutex<()>,
    epoch: Instant<Monotonic>,
}

impl Board {
    /// Maps BAR0 of `pdev`.
    pub(crate) fn new(pdev: &pci::Device<Core>) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Board {
                bar <- pdev.iomap_region_sized::<BAR0_SIZE>(0, c_str!("pxw3pe")),
                gpio <- new_spinlock!(()),
                i2c <- new_mutex!(()),
                epoch: Instant::now(),
            }),
            GFP_KERNEL,
        )
    }

    /// Reads a register; `None` once the device has been unbound.
    pub(crate) fn read32(&self, offset: usize) -> Option<u32> {
        self.bar
            .try_access_with(|bar| bar.try_read32(offset).ok())
            .flatten()
    }

    /// Writes a register; `None` once the device has been unbound.
    pub(crate) fn write32(&self, value: u32, offset: usize) -> Option<()> {
        self.bar
            .try_access_with(|bar| bar.try_write32(value, offset).ok())
            .flatten()
    }

    fn write16(&self, value: u16, offset: usize) -> Option<()> {
        self.bar
            .try_access_with(|bar| bar.try_write16(value, offset).ok())
            .flatten()
    }

    /// Sets and clears bits of [`GPIO`] atomically with respect to the rest of the driver.
    pub(crate) fn update_gpio(&self, set: u32, clear: u32) -> Option<()> {
        let _guard = self.gpio.lock();
        let value = self.read32(GPIO)?;
        self.write32((value & !clear) | set, GPIO)
    }

    /// `Set_GPIO(value, mask)`: virtual bit 0 is [`GPIO`] bit 2, bits 1..7 are the low byte of
    /// `GPIO_OUT` (where bit 0 mirrors virtual bit 0 too). The two read-modify-writes are one
    /// atomic update, since doing them without a lock is a pitfall.
    pub(crate) fn set_gpio(&self, value: u8, mask: u8) -> Option<()> {
        let _guard = self.gpio.lock();
        if mask & 1 != 0 {
            let v = self.read32(GPIO)?;
            let bit = if value & 1 != 0 { 1 << 2 } else { 0 };
            self.write32((v & !(1 << 2)) | bit, GPIO)?;
        }
        let v = self.read32(GPIO_OUT)?;
        let (value, mask) = (u32::from(value), u32::from(mask));
        self.write32((v & !mask) | (value & mask), GPIO_OUT)
    }

    /// `Set_GPIO_Ex(value, mask)`: virtual bits 0..4 are [`GPIO`] bits 3..7.
    pub(crate) fn set_gpio_ex(&self, value: u8, mask: u8) -> Option<()> {
        let (value, mask) = (u32::from(value) << 3, (u32::from(mask) << 3) & 0xF8);
        self.update_gpio(value & mask, mask & !value)
    }

    /// Takes the I2C bus for a sequence of transactions.
    pub(crate) fn bus(&self) -> Bus<'_> {
        let guard = self.i2c.lock();
        let engine = Engine::new(BoardRegs(self), &i2c::MAP);
        Bus {
            _guard: guard,
            engine,
        }
    }

    /// Monotonic time since the board was mapped, in microseconds.
    pub(crate) fn now_us(&self) -> u64 {
        // The epoch lies in the past, so the difference is never negative.
        (Instant::<Monotonic>::now() - self.epoch).as_micros_ceil() as u64
    }
}

/// Sleeps uninterruptibly: a pending signal must not cut reset pulses and settling times short.
pub(crate) fn sleep_ms(ms: u32) {
    fsleep(Delta::from_millis(i64::from(ms)));
}

/// Register access of the I2C engine.
struct BoardRegs<'a>(&'a Board);

impl i2c::Regs for BoardRegs<'_> {
    fn read32(&self, offset: usize) -> Result<u32, i2c::Error> {
        self.0.read32(offset).ok_or(i2c::Error::Gone)
    }

    fn write32(&self, value: u32, offset: usize) -> Result<(), i2c::Error> {
        self.0.write32(value, offset).ok_or(i2c::Error::Gone)
    }

    fn write16(&self, value: u16, offset: usize) -> Result<(), i2c::Error> {
        self.0.write16(value, offset).ok_or(i2c::Error::Gone)
    }

    // All I2C traffic runs in process context, so even the engine's short settling waits may
    // sleep; `fsleep` never returns early.
    fn delay_us(&self, us: u32) {
        fsleep(Delta::from_micros(i64::from(us)));
    }

    fn sleep_us(&self, us: u32) {
        fsleep(Delta::from_micros(i64::from(us)));
    }

    fn now_us(&self) -> u64 {
        self.0.now_us()
    }
}

/// The demodulator cores' gateway register: what is written behind it goes to the tuner.
const GATEWAY: u8 = 0xFE;

/// Longest payload [`Bus::gate_write`] and [`Bus::gate_read`] forward to a tuner; the driver's own
/// accesses need at most 6 bytes (an NM120 32-bit write).
pub(crate) const GATE_MAX: usize = 16;

/// The I2C bus, held for one sequence of transactions. Gateway reads and read-modify-writes are
/// done within one `Bus`, so no other transaction can come between their parts.
pub(crate) struct Bus<'a> {
    _guard: MutexGuard<'a, ()>,
    engine: Engine<BoardRegs<'a>>,
}

impl Bus<'_> {
    /// `S addr reg data... P` (write mode 0).
    pub(crate) fn write_reg(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<(), i2c::Error> {
        self.engine.write(addr, Some(reg), data, true)
    }

    /// `S addr reg data...` without STOP (write mode 4).
    pub(crate) fn write_reg_nostop(
        &mut self,
        addr: u8,
        reg: u8,
        data: &[u8],
    ) -> Result<(), i2c::Error> {
        self.engine.write(addr, Some(reg), data, false)
    }

    /// `S addr data... P` (write mode 2).
    pub(crate) fn write_raw(&mut self, addr: u8, data: &[u8]) -> Result<(), i2c::Error> {
        self.engine.write(addr, None, data, true)
    }

    /// `S addr+1 data... P` (read mode 2); a repeated start after a write without STOP.
    pub(crate) fn read_raw(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), i2c::Error> {
        self.engine.read(addr, None, buf)
    }

    /// Like [`Bus::read_reg_rs`], but never resets the engine (for probing whether it answers).
    pub(crate) fn probe_reg(
        &mut self,
        addr: u8,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), i2c::Error> {
        self.engine.probe_read(addr, Some(reg), buf)
    }

    /// `S addr reg Sr addr+1 data... P` (read mode 0).
    pub(crate) fn read_reg_rs(
        &mut self,
        addr: u8,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), i2c::Error> {
        self.engine.read(addr, Some(reg), buf)
    }

    /// `S addr FE tuner data... P`: writes `data` to the tuner at the 8-bit address `tuner` behind
    /// the gateway of the demodulator core at the 8-bit write address `addr`.
    pub(crate) fn gate_write(
        &mut self,
        addr: u8,
        tuner: u8,
        data: &[u8],
    ) -> Result<(), i2c::Error> {
        let mut msg = [0u8; 2 + GATE_MAX];
        let msg = msg.get_mut(..2 + data.len()).ok_or(i2c::Error::TooLong)?;
        msg[0] = GATEWAY;
        msg[1] = tuner;
        msg[2..].copy_from_slice(data);
        self.write_raw(addr, msg)
    }

    /// `S addr FE tuner sub... P`, then `S addr FE tuner|1 Sr addr+1 buf... P`: a read of the
    /// tuner at `tuner` behind the gateway of the core at `addr`. Both parts run in this bus hold:
    /// releasing the bus after the write without STOP would let another transaction be forwarded
    /// to the tuner.
    pub(crate) fn gate_read(
        &mut self,
        addr: u8,
        tuner: u8,
        sub: &[u8],
        buf: &mut [u8],
    ) -> Result<(), i2c::Error> {
        let mut msg = [0u8; 1 + GATE_MAX];
        let msg = msg.get_mut(..1 + sub.len()).ok_or(i2c::Error::TooLong)?;
        msg[0] = tuner;
        msg[1..].copy_from_slice(sub);
        self.write_reg(addr, GATEWAY, msg)?;
        self.write_reg_nostop(addr, GATEWAY, &[tuner | 1])?;
        self.read_raw(addr, buf)
    }

    /// Atomic read-modify-write of one register through mode 0 accesses, for the ASIE5606 and
    /// demodulator bit updates.
    pub(crate) fn update_reg(
        &mut self,
        addr: u8,
        reg: u8,
        value: u8,
        mask: u8,
    ) -> Result<(), i2c::Error> {
        let mut v = [0u8];
        self.read_reg_rs(addr, reg, &mut v)?;
        self.write_reg(addr, reg, &[(v[0] & !mask) | (value & mask)])
    }
}
