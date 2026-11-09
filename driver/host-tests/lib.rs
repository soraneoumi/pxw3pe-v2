// SPDX-License-Identifier: GPL-2.0

//! Builds the driver's `proto` module for the host so that its unit tests can run with
//! `cargo test`. The kernel module uses every item; here only the tests do.

#![no_std]
#![allow(dead_code)]

#[path = "../proto/mod.rs"]
mod proto;
