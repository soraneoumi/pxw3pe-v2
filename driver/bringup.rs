// SPDX-License-Identifier: GPL-2.0

//! Bridge and chip bring-up, LNB supply and power-down.
//!
//! The register values are revealed truth; the order differs where the order they come in was
//! self-contradictory, as noted at each step.

use crate::{
    hw::{sleep_ms, Board},
    proto::{
        i2c,
        ts::{Scheme, SEED},
    },
};
use kernel::prelude::*;

/// Address of the ASIE5606 GPIO and status registers (reset lines, LNB supply). It is fixed
/// (`TLIB_SetGPIO_ASIE5606B5`, the LNB monitor) and independent of where the scrambler function
/// answered (as revealed).
const CONTROL_ADDR: u8 = 0x4A;

/// Scrambler addresses probed before a reset, and after one, in this order (as revealed in
/// `bBCardInit`).
const SCRAMBLER_ADDRS: [u8; 2] = [0x5A, 0x4A];
const SCRAMBLER_ADDRS_AFTER_RESET: [u8; 2] = [0x5C, 0x4C];

/// ASIE5606 registers.
const CTL_TS_OUTPUT: u8 = 0x05;
const CTL_VERSION: u8 = 0x09;
const CTL_GPIO: u8 = 0x0B;
const CTL_STATUS: u8 = 0x0C;
const CTL_SEED: u8 = 0x10;
const CTL_SHUTDOWN: u8 = 0x80;

/// Register 0x0B bits 1 and 2, pulsed at power-up: two reset lines of the chips behind the
/// bridge.
const GPIO_RESET_LINES: [u8; 2] = [0x02, 0x04];
/// Register 0x0B bit 5: the LNB supply, 1 = on.
const GPIO_LNB: u8 = 0x20;
/// Register 0x0C bit 0: the LNB supply is good, 0 = short or fault.
const STATUS_LNB_GOOD: u8 = 0x01;

/// Time every power-up step is given.
const POWER_STEP_MS: u32 = 100;

/// Register 0x05 bits: TS clock inversion at the scrambler input (set by the bridge's PCIe
/// initialisation, as revealed), and "key set".
const TS_CLK_INV_IN: u8 = 0x20;
const KEY_SET: u8 = 0x80;

/// Interrupt mask and status registers.
const IRQ_STATUS: usize = 0x808;
const IRQ_CLEAR: usize = 0x80C;
const IRQ_DISABLE: usize = 0x818;

fn bus_error(_: i2c::Error) -> Error {
    EIO
}

fn present(v: Option<()>) -> Result {
    v.ok_or(ENODEV)
}

fn write(board: &Board, value: u32, offset: usize) -> Result {
    present(board.write32(value, offset))
}

/// Masks and clears every interrupt. The line is shared and no handler exists yet, so a device
/// left asserting an interrupt (a previous driver instance, a warm reboot) must be silenced
/// before anything else.
pub(crate) fn mask_interrupts(board: &Board) -> Result {
    write(board, 0xFFFF, IRQ_DISABLE)?;
    let stale = board.read32(IRQ_STATUS).ok_or(ENODEV)?;
    write(board, stale, IRQ_CLEAR)
}

/// `Hardware_Init` and `RegisterInitialize`. Runs before anything else uses the GPIO registers.
/// Register names: 0x880 is the port mode multiplexer (bits 2:0 and 5:3 = port A and B mode, 0 =
/// plain DTV; bits 6 and 7 = the ports' AES enables), 0x900/0x904 are the infrared receiver's
/// control and counters, 0xA00/0xB40 the TS control of port A/B, 0x888 the peripheral control and
/// 0x88C/0x890/0x894 the three GPIO registers.
pub(crate) fn init_bridge(board: &Board) -> Result {
    let v = board.read32(0x880).ok_or(ENODEV)?;
    // The AES enables are kept here and the register is overwritten 22 ms later anyway (as
    // revealed); both writes are kept since what the intermediate state does is unknown.
    write(board, v & 0xC0, 0x880)?;
    // The infrared receiver set-up, as revealed; the board has no receiver, and the writes are kept
    // because their absence has not been tried on a card.
    write(board, 0x0032_00C8, 0x904)?;
    sleep_ms(2);
    write(board, 0x90, 0x900)?;
    sleep_ms(20);
    for (offset, value) in [
        (0x880, 0x0001_0000),
        (0xA00, 0x80),
        (0xB40, 0x80),
        (0x888, 0),
        // Writing 0x894 = 0xCF and 0x890 = 0x1004 here and correcting them right after with
        // `Set_GPIO(0x08, 0xFF)`, `Set_GPIO_Ex(0, 2)` and `Set_GPIO_Ex(1, 1)` would briefly release
        // the demodulator reset, so the final values are written directly: GPIO3 high and the other
        // virtual GPIOs low; GPIO0 low (reset asserted), GPIOEx0 high (chip power off), GPIOEx1
        // low, and bit 12 set (as revealed).
        (0x894, 0x08),
        (0x88C, 0x8000),
        (0x890, 0x1008),
        (0x900, 0x90),
        (0x904, 0x0032_00C8),
    ] {
        write(board, value, offset)?;
    }
    Ok(())
}

/// Checks that the I2C engine answers at the ASV5220's register map, with a read of the EEPROM at
/// 0xA0 (the revealed probe). A NACK proves the engine is there as well as an ACK does, so taking
/// it as "wrong map" is a pitfall; only a timeout fails. The probe never pulses the engine reset.
///
/// An alternate map (I2C at 0xA80, DMA at 0xD00/0xD40) exists for the same bridge. In the
/// bridge's register layout those are the AES block and the DMA ports C and D, whose completion
/// interrupts are bits 4..7 while that map keeps bits 0..3 for them: the alternate map is another
/// bridge variant, driven with a known inconsistency, so this driver refuses it instead of
/// guessing.
pub(crate) fn probe_i2c_engine(board: &Board) -> Result {
    let mut b = [0u8];
    match board.bus().probe_reg(0xA0, 0, &mut b) {
        Ok(()) | Err(i2c::Error::Nack) => Ok(()),
        Err(i2c::Error::Gone) => Err(ENODEV),
        Err(e) => Err(bus_error(e)),
    }
}

/// Powers the chips behind the bridge and takes them out of reset: GPIO0 = 1, 0, 1; GPIOEx0 = 1, 0
/// (the power switch of demodulator and tuners); register 0x0B bit 1 (the demodulator reset, which
/// it needs after power-up to reach its reset state) and then bit 2 (no effect on this board) of
/// the ASIE5606 each = 1, 0, 1; every step is given 100 ms (the revealed timing).
///
/// Register 0x0B is written with the other bits kept, in one bus hold, so the LNB supply bit is
/// not disturbed; its state at power-up is unknown, which is why probe switches it off afterwards.
pub(crate) fn power_on(board: &Board) -> Result {
    for v in [1, 0, 1] {
        present(board.set_gpio(v, 1))?;
        sleep_ms(POWER_STEP_MS);
    }
    for v in [1, 0] {
        present(board.set_gpio_ex(v, 1))?;
        sleep_ms(POWER_STEP_MS);
    }
    for bit in GPIO_RESET_LINES {
        for value in [bit, 0, bit] {
            board
                .bus()
                .update_reg(CONTROL_ADDR, CTL_GPIO, value, bit)
                .map_err(bus_error)?;
            sleep_ms(POWER_STEP_MS);
        }
    }
    Ok(())
}

/// Power-down: the demodulator reset is asserted first and kept asserted, then the chip power is
/// cut, the mirror image of power-on.
pub(crate) fn power_off(board: &Board) {
    sleep_ms(10);
    let _ = board.set_gpio(0, 1);
    sleep_ms(10);
    let _ = board.set_gpio_ex(1, 1);
}

/// The version of the scrambler function at `addr` (register 0x09 as `(raw & 0x3E) >> 1`), if it
/// answers.
fn version_at(board: &Board, addr: u8) -> Option<u8> {
    let mut raw = [0u8];
    board.bus().read_reg_rs(addr, CTL_VERSION, &mut raw).ok()?;
    Some((raw[0] & 0x3E) >> 1)
}

/// The first of `addrs` whose scrambler function has a supported version. A version without a
/// descrambling path is logged: it tells a wrong chip from a missing one.
fn supported_at(board: &Board, addrs: [u8; 2]) -> Option<u8> {
    for addr in addrs {
        let Some(version) = version_at(board, addr) else {
            continue;
        };
        if Scheme::for_version(version).is_some() {
            return Some(addr);
        }
        pr_err!(
            "scrambler at {:#04x} reports version {:#04x}; only 0x11 (XOR only) is supported\n",
            addr,
            version
        );
    }
    None
}

/// Finds the scrambler function of the ASIE5606 and checks its version (`bBCardInit`).
///
/// Only the XOR-only version 0x11 is accepted. Accepting any version up to 0x11 after the reset is
/// a pitfall, as is rejecting a supported version at the first address, which sends a chip of that
/// version through the reset only to miss it afterwards.
pub(crate) fn find_scrambler(board: &Board) -> Result<u8> {
    if let Some(addr) = supported_at(board, SCRAMBLER_ADDRS) {
        return Ok(addr);
    }
    // Nothing usable answered: reset the chip on GPIO7 (as revealed), then look at the addresses it
    // uses after a reset.
    for (v, ms) in [(0x80, 25), (0x00, 25), (0x80, 50)] {
        present(board.set_gpio(v, 0x80))?;
        sleep_ms(ms);
    }
    supported_at(board, SCRAMBLER_ADDRS_AFTER_RESET).ok_or(ENODEV)
}

/// Programs the scrambler seed ([`SEED`], which the descrambler derives its mask from) and
/// enables the TS output.
///
/// Writing the seed while streams run, or skipping it for some tuners, is a pitfall; here it
/// happens once, before any DMA is armed.
pub(crate) fn load_seed(board: &Board, addr: u8) -> Result {
    let mut bus = board.bus();
    bus.write_reg(addr, CTL_TS_OUTPUT, &[TS_CLK_INV_IN])
        .map_err(bus_error)?;
    for (i, &b) in SEED.iter().enumerate() {
        bus.write_reg(addr, CTL_SEED + i as u8, &[b])
            .map_err(bus_error)?;
    }
    bus.write_reg(addr, CTL_TS_OUTPUT, &[KEY_SET | TS_CLK_INV_IN])
        .map_err(bus_error)
}

/// `DTV_UnInit` and `bBCardUninit`: closes the scrambler.
pub(crate) fn shutdown_control_chip(board: &Board, addr: u8) {
    let mut bus = board.bus();
    let _ = bus.write_reg(addr, CTL_SHUTDOWN, &[0]);
    let _ = bus.write_reg(addr, 0x04, &[0]);
    let _ = bus.write_reg(addr, 0x00, &[0]);
    drop(bus);
    sleep_ms(50);
}

/// Switches the LNB supply: register 0x0B bit 5, one read-modify-write. A set bit is 15 V on the
/// antenna connector, and other boards on this bridge wire the register differently, so this only
/// runs on a board identified by its subsystem ID.
pub(crate) fn lnb_switch(board: &Board, on: bool) -> Result {
    let value = if on { GPIO_LNB } else { 0 };
    board
        .bus()
        .update_reg(CONTROL_ADDR, CTL_GPIO, value, GPIO_LNB)
        .map_err(bus_error)
}

/// Whether the supply reports power good: register 0x0C bit 0. Only meaningful while the supply is
/// on.
pub(crate) fn lnb_status(board: &Board) -> Result<bool> {
    let mut status = [0u8];
    board
        .bus()
        .read_reg_rs(CONTROL_ADDR, CTL_STATUS, &mut status)
        .map_err(bus_error)?;
    Ok(status[0] & STATUS_LNB_GOOD != 0)
}
