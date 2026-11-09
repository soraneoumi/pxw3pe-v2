//! pcsc-lite IFD handler (reader driver) for the B-CAS card reader built into the PX-W3PE.
//!
//! The kernel driver exposes each reader as `/dev/pxw3pe-cardN` and runs the T=1 protocol
//! itself (see `driver/uapi/pxw3pe_card.h`), so this handler maps the IFD Handler API v3 onto
//! four ioctls: status, reset, power-off and APDU exchange. pcscd loads it for the readers listed
//! in `/etc/reader.conf.d`.

#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(not(target_os = "linux"))]
compile_error!("the PX-W3PE card interface and the DWORD layout used here are Linux-specific");

#[cfg(test)]
mod cprobe;
mod device;
#[cfg(test)]
mod fake;
mod ffi;
mod handler;
mod pcsc;
mod uapi;
