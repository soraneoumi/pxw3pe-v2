//! A fake card reader for the tests, following the kernel driver (`driver/card.rs`): EINVAL for a
//! command shorter than an APDU, ENODEV when the board is gone or the card is missing, ENOTCONN
//! without a session, EMSGSIZE for a command longer than the IFSC, a session that ends when the
//! card is seen to be absent or an exchange fails for any other reason, and REMOVED reported by
//! one status call after a test set `removed`.

use crate::device::Card;
use crate::uapi::{self, CardAtr, CardTransfer};
use std::io;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex, MutexGuard};

/// The ATR of a B-CAS card: specific mode T=1 (TA2 = 0x81), TA1 = 0x12, IFSC 124.
pub const BCAS_ATR: &[u8] = &[
    0x3B, 0xF0, 0x12, 0x00, 0xFF, 0x91, 0x81, 0xB1, 0x7C, 0x45, 0x1F, 0x03, 0x99,
];

pub struct State {
    pub present: bool,
    pub gone: bool,
    pub session: bool,
    /// A session ended with the card's removal; the next status call reports REMOVED.
    pub removed: bool,
    pub atr: Vec<u8>,
    /// Overrides the ATR length the kernel reports.
    pub atr_len: Option<u32>,
    pub reset_error: Option<i32>,
    /// The card's IFSC (TA3 of `BCAS_ATR`).
    pub ifsc: usize,
    /// Makes the next exchange fail with this errno; it ends the session unless it is EMSGSIZE.
    pub transmit_error: Option<i32>,
    pub response: Vec<u8>,
    /// Overrides the response length the kernel reports.
    pub response_len: Option<u32>,
    pub status_error: Option<i32>,
    /// Every call, in order.
    pub calls: Vec<&'static str>,
    pub commands: Vec<Vec<u8>>,
    /// If set, a transmit signals `entered` and then waits on `release`.
    pub gate: Option<(Sender<()>, Receiver<()>)>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            present: true,
            gone: false,
            session: false,
            removed: false,
            atr: BCAS_ATR.to_vec(),
            atr_len: None,
            reset_error: None,
            ifsc: 0x7C,
            transmit_error: None,
            response: vec![0x90, 0x00],
            response_len: None,
            status_error: None,
            calls: Vec::new(),
            commands: Vec::new(),
            gate: None,
        }
    }
}

/// The card, plus a handle through which a test changes and inspects it after handing it over.
#[derive(Clone, Default)]
pub struct FakeCard(Arc<Mutex<State>>);

impl FakeCard {
    pub fn state(&self) -> MutexGuard<'_, State> {
        self.0.lock().unwrap()
    }
}

fn errno(errno: i32) -> io::Error {
    io::Error::from_raw_os_error(errno)
}

impl Card for FakeCard {
    fn status(&mut self) -> io::Result<u32> {
        let mut s = self.state();
        s.calls.push("status");
        if let Some(e) = s.status_error {
            return Err(errno(e));
        }
        if s.gone {
            return Err(errno(libc::ENODEV));
        }
        if !s.present {
            s.session = false;
        }
        let removed = std::mem::take(&mut s.removed);
        Ok(if s.present { uapi::PRESENT } else { 0 }
            | if s.session { uapi::ACTIVE } else { 0 }
            | if removed { uapi::REMOVED } else { 0 })
    }

    fn reset(&mut self) -> io::Result<CardAtr> {
        let mut s = self.state();
        s.calls.push("reset");
        s.session = false;
        if s.gone || !s.present {
            return Err(errno(libc::ENODEV));
        }
        if let Some(e) = s.reset_error {
            return Err(errno(e));
        }
        s.session = true;
        let mut atr = CardAtr::zeroed();
        atr.atr[..s.atr.len()].copy_from_slice(&s.atr);
        atr.len = s.atr_len.unwrap_or(s.atr.len() as u32);
        Ok(atr)
    }

    fn power_off(&mut self) -> io::Result<()> {
        let mut s = self.state();
        s.calls.push("power_off");
        s.session = false;
        if s.gone {
            return Err(errno(libc::ENODEV));
        }
        Ok(())
    }

    fn transmit(&mut self, transfer: &mut CardTransfer) -> io::Result<()> {
        let gate = {
            let mut s = self.state();
            s.calls.push("transmit");
            let command = transfer.command[..transfer.command_len as usize].to_vec();
            s.commands.push(command);
            s.gate.take()
        };
        if let Some((entered, release)) = gate {
            entered.send(()).unwrap();
            release.recv().unwrap();
        }
        let mut s = self.state();
        // The order of the checks in `Card::transmit` and `Session::transceive`.
        let len = transfer.command_len as usize;
        if len < uapi::APDU_MIN {
            return Err(errno(libc::EINVAL));
        }
        if len > uapi::APDU_MAX {
            return Err(errno(libc::EMSGSIZE));
        }
        if s.gone {
            return Err(errno(libc::ENODEV));
        }
        if !s.session {
            return Err(errno(libc::ENOTCONN));
        }
        if len > s.ifsc {
            return Err(errno(libc::EMSGSIZE));
        }
        let failure = s
            .transmit_error
            .take()
            .or(if s.present { None } else { Some(libc::ENODEV) });
        if let Some(e) = failure {
            if e != libc::EMSGSIZE {
                s.session = false;
            }
            return Err(errno(e));
        }
        transfer.response[..s.response.len()].copy_from_slice(&s.response);
        transfer.response_len = s.response_len.unwrap_or(s.response.len() as u32);
        Ok(())
    }
}
