// SPDX-License-Identifier: GPL-2.0

//! Hardware-independent protocol logic.
//!
//! Nothing in here touches the device or the kernel API, so the same files are also built by the
//! host test crate in `host-tests/`.

pub(crate) mod atr;
pub(crate) mod i2c;
pub(crate) mod isdb;
pub(crate) mod nmi;
pub(crate) mod t1;
pub(crate) mod tc90522;
pub(crate) mod tda2014x;
pub(crate) mod ts;
pub(crate) mod uart;
