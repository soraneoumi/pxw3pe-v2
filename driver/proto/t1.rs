// SPDX-License-Identifier: GPL-2.0

//! T=1 block transmission protocol, interface-device side (ISO/IEC 7816-3 clause 11), in the
//! profile of ARIB STD-B25 4.3.2: the node address is always zero and chaining is never used.
//!
//! The engine is written against [`Transport`] so that it carries no hardware knowledge. All
//! waiting times come from the ATR ([`super::atr::Timing`]); none is a poll count.
//!
//! Error recovery follows ISO/IEC 7816-3 11.6.3 and is bounded at every level: a faulty exchange
//! is repaired by up to [`MAX_RETRANSMIT`] retransmissions, then by resynchronisation (at most
//! [`MAX_RESYNCH`] S(RESYNCH request) blocks), after which [`Error::SessionLost`] tells the caller
//! that only a card reset can help. The whole of it, waiting time extensions included, must also
//! fit in [`EXCHANGE_LIMIT_US`]. Nothing that fails validation is ever passed upwards.

use super::atr::{T1Params, Timing};

/// Our information field size, announced to the card before the first I-block as ARIB
/// STD-B25 requires (it is also re-announced after every resynchronisation).
pub(crate) const IFSD: u8 = 254;

/// Largest INF a block may carry in either direction.
pub(crate) const MAX_INF: usize = 254;

/// Prologue (NAD, PCB, LEN) plus INF plus LRC.
const MAX_BLOCK: usize = 3 + MAX_INF + 1;

/// Retransmissions of a request (or of our R-block) before the exchange is resynchronised.
pub(crate) const MAX_RETRANSMIT: u8 = 2;

/// S(WTX request) blocks accepted from the card within one exchange attempt. A card that keeps
/// asking for more time never answers, so the request is treated as a protocol fault beyond this.
const MAX_WTX: u8 = 8;

/// Wall-clock time one [`Session::transceive`] or [`Session::negotiate_ifsd`] call may take in
/// total, every wait included: the resynchronisation round, and the waiting time extensions,
/// each of which may grant up to 255 times the BWT. The counts above bound the number of blocks,
/// not the time, and the card chooses both the multipliers and, through the ATR, the BWT, so
/// without this a faulty or hostile card could hold the reader for hours.
///
/// B-CAS commands are answered within 420 ms, and the first after a reset within 520 ms (the
/// B-CAS card specification, SB0100D). The limit lies far above that, and above the 12 BWT (about
/// 17 s at the B-CAS BWT) that the longest recovery of a silent card takes: three waits each for
/// the I-block, the RESYNCH, the IFS announcement and the repeated I-block. It therefore cuts off
/// waits that no working card needs, never a recovery.
pub(crate) const EXCHANGE_LIMIT_US: u64 = 30_000_000;

/// S(RESYNCH request) attempts before the session is given up.
pub(crate) const MAX_RESYNCH: u8 = 3;

/// Failure to receive a character from the transport.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum RecvError<E> {
    /// No character arrived within the given time.
    Timeout,
    /// The transport itself failed.
    Io(E),
}

/// Character transport to the card: a half-duplex UART with a receive FIFO.
pub(crate) trait Transport {
    /// Transport failure (hardware error, card removed, ...).
    type Error;

    /// Discards every character received but not yet read.
    fn purge(&mut self) -> Result<(), Self::Error>;

    /// Monotonic time in microseconds.
    fn now_us(&self) -> u64;

    /// Transmits `frame`, starting no earlier than `guard_us` after the last character received.
    fn send(&mut self, frame: &[u8], guard_us: u32) -> Result<(), Self::Error>;

    /// Returns the next received character, waiting at most `timeout_us` for it to start.
    fn recv(&mut self, timeout_us: u32) -> Result<u8, RecvError<Self::Error>>;
}

/// Failure of a T=1 exchange.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Error<E> {
    /// The transport failed; the session state is unknown and the card must be reset.
    Io(E),
    /// The command does not fit in one block of the card's IFSC; ARIB STD-B25 excludes chaining.
    /// Nothing was sent, and the session is unchanged.
    CommandTooLong,
    /// The response is longer than the buffer supplied for it. The exchange itself succeeded.
    ResponseTooLong,
    /// Resynchronisation failed, or after it the command no longer fits the IFSC it reset; the
    /// card must be reset before the next exchange.
    SessionLost,
    /// The exchange did not end within [`EXCHANGE_LIMIT_US`]; the card must be reset before the
    /// next exchange.
    TimeLimit,
}

/// Counters for diagnosing an unreliable card contact.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Stats {
    /// Blocks received with a wrong LRC.
    pub(crate) edc_errors: u32,
    /// Blocks that timed out or broke the protocol rules.
    pub(crate) other_errors: u32,
    /// Requests or R-blocks transmitted again.
    pub(crate) retransmissions: u32,
    /// Successful resynchronisations.
    pub(crate) resynchs: u32,
    /// Waiting time extensions granted to the card.
    pub(crate) wtx: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SKind {
    Resynch = 0,
    Ifs = 1,
    Abort = 2,
    Wtx = 3,
}

impl SKind {
    fn from_bits(bits: u8) -> Option<Self> {
        Some(match bits {
            0 => Self::Resynch,
            1 => Self::Ifs,
            2 => Self::Abort,
            3 => Self::Wtx,
            _ => return None,
        })
    }

    /// Length of the INF the block carries (one parameter byte for IFS and WTX).
    fn inf_len(self) -> usize {
        match self {
            Self::Ifs | Self::Wtx => 1,
            Self::Resynch | Self::Abort => 0,
        }
    }
}

/// A received block whose INF, if any, sits in the session's receive buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Block {
    I {
        ns: bool,
        more: bool,
        len: usize,
    },
    R {
        nr: bool,
    },
    S {
        response: bool,
        kind: SKind,
        param: Option<u8>,
    },
}

/// Why a block was not accepted; selects the error code of the R-block sent in reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fault {
    Edc = 1,
    Other = 2,
}

/// What an exchange is waiting for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Request {
    /// An I-block answering ours.
    Information,
    /// The response to our S-request of this kind, carrying `param`.
    Supervisory(SKind, Option<u8>),
}

fn lrc(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, &b| acc ^ b)
}

/// State of one T=1 session, from the ATR until the card is reset or removed.
pub(crate) struct Session {
    /// N(S) of our next I-block.
    ns: bool,
    /// N(S) expected in the card's next I-block.
    nr: bool,
    ifsc: u8,
    initial_ifsc: u8,
    timing: Timing,
    stats: Stats,
    /// Transport time at which the running exchange must have ended.
    deadline_us: u64,
    tx: [u8; MAX_BLOCK],
    tx_len: usize,
    rx: [u8; MAX_BLOCK],
}

impl Session {
    /// Starts a session with the parameters announced in the ATR.
    pub(crate) fn new(params: &T1Params, timing: Timing) -> Self {
        Session {
            ns: false,
            nr: false,
            ifsc: params.ifsc,
            initial_ifsc: params.ifsc,
            timing,
            stats: Stats::default(),
            deadline_us: 0,
            tx: [0; MAX_BLOCK],
            tx_len: 0,
            rx: [0; MAX_BLOCK],
        }
    }

    /// Diagnostic counters accumulated since the session started.
    pub(crate) fn stats(&self) -> Stats {
        self.stats
    }

    /// Current information field size of the card.
    #[cfg(test)]
    pub(crate) fn ifsc(&self) -> u8 {
        self.ifsc
    }

    /// Announces [`IFSD`] to the card. Must precede the first [`Session::transceive`].
    ///
    /// Like [`Session::transceive`], it fails with [`Error::TimeLimit`] after
    /// [`EXCHANGE_LIMIT_US`].
    pub(crate) fn negotiate_ifsd<T: Transport>(
        &mut self,
        t: &mut T,
    ) -> Result<(), Error<T::Error>> {
        self.exchange(t, Request::Supervisory(SKind::Ifs, Some(IFSD)), &mut [])
            .map(|_| ())
    }

    /// Sends `command` in one I-block and stores the card's answer in `response`.
    ///
    /// Returns the length of the answer. On [`Error::SessionLost`], [`Error::TimeLimit`] or
    /// [`Error::Io`] the card must be reset and a new session started.
    pub(crate) fn transceive<T: Transport>(
        &mut self,
        t: &mut T,
        command: &[u8],
        response: &mut [u8],
    ) -> Result<usize, Error<T::Error>> {
        if command.len() > usize::from(self.ifsc) {
            return Err(Error::CommandTooLong);
        }
        self.load_i(command);
        self.exchange(t, Request::Information, response)
    }

    fn load(&mut self, pcb: u8, inf: &[u8]) {
        self.tx[0] = 0;
        self.tx[1] = pcb;
        // `inf` is at most MAX_INF long: commands are checked against IFSC, the rest are S-blocks.
        self.tx[2] = inf.len() as u8;
        self.tx[3..3 + inf.len()].copy_from_slice(inf);
        let end = 3 + inf.len();
        self.tx[end] = lrc(&self.tx[..end]);
        self.tx_len = end + 1;
    }

    fn load_i(&mut self, inf: &[u8]) {
        self.load(if self.ns { 0x40 } else { 0x00 }, inf);
    }

    fn load_s(&mut self, response: bool, kind: SKind, param: Option<u8>) {
        let pcb = 0xC0 | if response { 0x20 } else { 0 } | kind as u8;
        match param {
            Some(p) => self.load(pcb, &[p]),
            None => self.load(pcb, &[]),
        }
    }

    fn transmit<T: Transport>(&mut self, t: &mut T) -> Result<(), Error<T::Error>> {
        t.send(&self.tx[..self.tx_len], self.timing.bgt_us)
            .map_err(Error::Io)
    }

    /// Sends a block that is not the pending request, keeping the request available for
    /// retransmission.
    fn send_aside<T: Transport>(
        &mut self,
        t: &mut T,
        pcb: u8,
        param: Option<u8>,
    ) -> Result<(), Error<T::Error>> {
        let mut frame = [0u8; 5];
        frame[1] = pcb;
        let len = match param {
            Some(p) => {
                frame[2] = 1;
                frame[3] = p;
                4
            }
            None => 3,
        };
        frame[len] = lrc(&frame[..len]);
        t.send(&frame[..=len], self.timing.bgt_us)
            .map_err(Error::Io)
    }

    /// Runs one request/response exchange including all error recovery.
    fn exchange<T: Transport>(
        &mut self,
        t: &mut T,
        request: Request,
        response: &mut [u8],
    ) -> Result<usize, Error<T::Error>> {
        self.deadline_us = t.now_us().saturating_add(EXCHANGE_LIMIT_US);
        if let Request::Supervisory(kind, param) = request {
            self.load_s(false, kind, param);
        }
        // Resynchronising rewrites the transmit buffer, but the request must then be sent again
        // unchanged (as a fresh I-block with N(S) = 0), so keep its INF aside.
        let mut inf = [0u8; MAX_INF];
        let inf_len = usize::from(self.tx[2]);
        inf[..inf_len].copy_from_slice(&self.tx[3..3 + inf_len]);

        if let Some(len) = self.exchange_once(t, request, response, MAX_RETRANSMIT)? {
            return Ok(len);
        }
        self.resynchronise(t)?;
        match request {
            // IFSC fell back to the ATR value, which may be smaller than the one the card set, so
            // the command can no longer be repeated. The card may have executed it already, so
            // this is not `CommandTooLong`, which promises that nothing was sent.
            Request::Information if inf_len > usize::from(self.ifsc) => {
                return Err(Error::SessionLost);
            }
            Request::Information => self.load_i(&inf[..inf_len]),
            Request::Supervisory(kind, param) => self.load_s(false, kind, param),
        }
        // One resynchronisation per exchange: a card that fails again needs a reset.
        self.exchange_once(t, request, response, MAX_RETRANSMIT)?
            .ok_or(Error::SessionLost)
    }

    /// Transmits the loaded request and waits for its answer.
    ///
    /// Returns `Ok(None)` when `limit` retransmissions did not produce a valid answer.
    fn exchange_once<T: Transport>(
        &mut self,
        t: &mut T,
        request: Request,
        response: &mut [u8],
        limit: u8,
    ) -> Result<Option<usize>, Error<T::Error>> {
        self.transmit(t)?;
        let mut retransmissions = 0u8;
        let mut wtx_requests = 0u8;
        // Waiting time multiplier granted by S(WTX); it applies to the next block only.
        let mut wtx = 1u32;
        loop {
            let wait = self.timing.bwt_us.saturating_mul(wtx);
            wtx = 1;
            let fault = match self.receive(t, wait)? {
                Err(fault) => fault,
                Ok(block) => match (request, block) {
                    (
                        Request::Information,
                        Block::I {
                            ns,
                            more: false,
                            len,
                        },
                    ) if ns == self.nr => {
                        self.ns = !self.ns;
                        self.nr = !self.nr;
                        let out = response.get_mut(..len).ok_or(Error::ResponseTooLong)?;
                        out.copy_from_slice(&self.rx[3..3 + len]);
                        return Ok(Some(len));
                    }
                    // The card did not receive our I-block correctly and asks for it again.
                    (Request::Information, Block::R { nr }) if nr == self.ns => {
                        if retransmissions == limit {
                            return Ok(None);
                        }
                        retransmissions += 1;
                        self.stats.retransmissions += 1;
                        self.transmit(t)?;
                        continue;
                    }
                    (
                        Request::Supervisory(kind, param),
                        Block::S {
                            response: true,
                            kind: k,
                            param: p,
                        },
                    ) if k == kind && p == param => {
                        if kind == SKind::Resynch {
                            self.ns = false;
                            self.nr = false;
                            self.ifsc = self.initial_ifsc;
                        }
                        return Ok(Some(0));
                    }
                    (
                        _,
                        Block::S {
                            response: false,
                            kind: SKind::Wtx,
                            param: Some(m),
                        },
                    ) if m != 0 && wtx_requests < MAX_WTX => {
                        self.send_aside(t, 0xE0 | SKind::Wtx as u8, Some(m))?;
                        wtx_requests += 1;
                        self.stats.wtx += 1;
                        wtx = u32::from(m);
                        continue;
                    }
                    (
                        _,
                        Block::S {
                            response: false,
                            kind: SKind::Ifs,
                            param: Some(n),
                        },
                    ) if (1..=254).contains(&n) => {
                        self.send_aside(t, 0xE0 | SKind::Ifs as u8, Some(n))?;
                        self.ifsc = n;
                        continue;
                    }
                    // Anything else breaks the protocol: a wrong N(S), a chained block (excluded
                    // by ARIB STD-B25), a response to a request we did not send, an ABORT, which
                    // is only meaningful during chaining, or a WTX request beyond `MAX_WTX`.
                    _ => Fault::Other,
                },
            };
            match fault {
                Fault::Edc => self.stats.edc_errors += 1,
                Fault::Other => self.stats.other_errors += 1,
            }
            if retransmissions == limit {
                return Ok(None);
            }
            retransmissions += 1;
            self.stats.retransmissions += 1;
            match request {
                // Ask the card to repeat its last block; if it never received ours, it answers
                // with an R-block requesting it, handled above.
                Request::Information => {
                    let pcb = 0x80 | if self.nr { 0x10 } else { 0 } | fault as u8;
                    self.send_aside(t, pcb, None)?;
                }
                Request::Supervisory(..) => self.transmit(t)?,
            }
        }
    }

    /// Resynchronises the session and announces IFSD again, as ARIB STD-B25 requires.
    ///
    /// S(RESYNCH request) is sent at most [`MAX_RESYNCH`] times in total.
    fn resynchronise<T: Transport>(&mut self, t: &mut T) -> Result<(), Error<T::Error>> {
        let resynch = Request::Supervisory(SKind::Resynch, None);
        self.load_s(false, SKind::Resynch, None);
        self.exchange_once(t, resynch, &mut [], MAX_RESYNCH - 1)?
            .ok_or(Error::SessionLost)?;
        self.stats.resynchs += 1;
        let ifs = Request::Supervisory(SKind::Ifs, Some(IFSD));
        self.load_s(false, SKind::Ifs, Some(IFSD));
        self.exchange_once(t, ifs, &mut [], MAX_RETRANSMIT)?
            .ok_or(Error::SessionLost)?;
        Ok(())
    }

    /// Waits for a character, at most `wait_us` and never beyond the end of the exchange; `None`
    /// on timeout.
    ///
    /// A wait cut short by the deadline that expires is not the card's silence: it ends the
    /// exchange instead of being repaired by retransmissions, which would only wait longer.
    fn recv<T: Transport>(&self, t: &mut T, wait_us: u32) -> Result<Option<u8>, Error<T::Error>> {
        let remaining = self.deadline_us.saturating_sub(t.now_us());
        if remaining == 0 {
            return Err(Error::TimeLimit);
        }
        let cut = u64::from(wait_us) > remaining;
        // A cut wait is shorter than `wait_us`, so it fits in 32 bits.
        let wait_us = if cut { remaining as u32 } else { wait_us };
        match t.recv(wait_us) {
            Ok(b) => Ok(Some(b)),
            Err(RecvError::Timeout) if cut => Err(Error::TimeLimit),
            Err(RecvError::Timeout) => Ok(None),
            Err(RecvError::Io(e)) => Err(Error::Io(e)),
        }
    }

    /// Reads one character into `self.rx[i]`; `Ok(false)` on timeout.
    fn get<T: Transport>(
        &mut self,
        t: &mut T,
        i: usize,
        wait_us: u32,
    ) -> Result<bool, Error<T::Error>> {
        Ok(match self.recv(t, wait_us)? {
            Some(b) => {
                self.rx[i] = b;
                true
            }
            None => false,
        })
    }

    /// Receives one block into `self.rx`.
    ///
    /// The outer `Result` carries transport failures; the inner one tells a valid block from a
    /// faulty one. After a faulty block the line is drained until it has been quiet for one CWT,
    /// so that our reply cannot collide with the rest of a block the card is still sending.
    fn receive<T: Transport>(
        &mut self,
        t: &mut T,
        first_wait_us: u32,
    ) -> Result<Result<Block, Fault>, Error<T::Error>> {
        let cwt = self.timing.cwt_us;
        if !self.get(t, 0, first_wait_us)? {
            return Ok(Err(Fault::Other));
        }
        for i in 1..3 {
            if !self.get(t, i, cwt)? {
                return Ok(Err(Fault::Other));
            }
        }
        let len = usize::from(self.rx[2]);
        if len > MAX_INF {
            self.drain(t)?;
            return Ok(Err(Fault::Other));
        }
        for i in 3..=3 + len {
            if !self.get(t, i, cwt)? {
                return Ok(Err(Fault::Other));
            }
        }
        if lrc(&self.rx[..=3 + len]) != 0 {
            self.drain(t)?;
            return Ok(Err(Fault::Edc));
        }
        if self.rx[0] != 0 {
            return Ok(Err(Fault::Other));
        }
        Ok(decode(self.rx[1], &self.rx[3..3 + len]).ok_or(Fault::Other))
    }

    /// Waits until the line has been quiet for one CWT, then purges the transport.
    ///
    /// The transport knows no character time, so the wait is bounded by counting characters: no
    /// valid block is longer than [`MAX_BLOCK`], so after `MAX_BLOCK + 1` characters the card is
    /// not finishing a block but babbling. With every gap shorter than one CWT, the drain takes
    /// at most `(MAX_BLOCK + 1) * cwt_us + cwt_us`.
    fn drain<T: Transport>(&mut self, t: &mut T) -> Result<(), Error<T::Error>> {
        for _ in 0..=MAX_BLOCK {
            if self.recv(t, self.timing.cwt_us)?.is_none() {
                break;
            }
        }
        t.purge().map_err(Error::Io)
    }
}

/// Decodes a PCB with its INF; `None` for any encoding ISO/IEC 7816-3 does not define.
fn decode(pcb: u8, inf: &[u8]) -> Option<Block> {
    match pcb & 0xC0 {
        0x00 | 0x40 if pcb & 0x1F == 0 => Some(Block::I {
            ns: pcb & 0x40 != 0,
            more: pcb & 0x20 != 0,
            len: inf.len(),
        }),
        0x80 if pcb & 0x2C == 0 && pcb & 0x03 != 0x03 && inf.is_empty() => Some(Block::R {
            nr: pcb & 0x10 != 0,
        }),
        0xC0 => {
            let kind = SKind::from_bits(pcb & 0x1F)?;
            if inf.len() != kind.inf_len() {
                return None;
            }
            Some(Block::S {
                response: pcb & 0x20 != 0,
                kind,
                param: inf.first().copied(),
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use std::vec::Vec;

    const TIMING: Timing = Timing {
        bwt_us: 1_428_992,
        cwt_us: 2000,
        bgt_us: 1023,
    };

    const PARAMS: T1Params = T1Params {
        f: 372,
        d: 2,
        n: 0xFF,
        ifsc: 0x7C,
        bwi: 4,
        cwi: 5,
    };

    fn frame(pcb: u8, inf: &[u8]) -> Vec<u8> {
        let mut f = std::vec![0, pcb, inf.len() as u8];
        f.extend_from_slice(inf);
        f.push(lrc(&f));
        f
    }

    /// What the simulated card does with the next block it receives.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Act {
        /// Process it as ISO/IEC 7816-3 prescribes.
        Normal,
        /// Answer correctly, but with a corrupted LRC.
        CorruptAnswer,
        /// Pretend the block arrived with an LRC error.
        CorruptRequest,
        /// Do not answer at all.
        Silent,
        /// Ask for a waiting time extension of this multiplier before answering.
        Wtx(u8),
        /// Answer an I-block with the wrong N(S).
        WrongNs,
        /// Ask to change IFSC to this value before answering.
        Ifs(u8),
        /// Answer with an I-block that has the chaining bit set.
        Chained,
    }

    /// A transport on which characters never stop arriving.
    struct Babbler {
        received: usize,
        purges: usize,
    }

    impl Transport for Babbler {
        type Error = ();

        fn purge(&mut self) -> Result<(), ()> {
            self.purges += 1;
            Ok(())
        }

        fn now_us(&self) -> u64 {
            0
        }

        fn send(&mut self, _frame: &[u8], _guard_us: u32) -> Result<(), ()> {
            Ok(())
        }

        fn recv(&mut self, _timeout_us: u32) -> Result<u8, RecvError<()>> {
            self.received += 1;
            // Zero is the LEN of the first block, so it is four characters long with a bad LRC.
            Ok(if self.received == 3 {
                0
            } else {
                self.received as u8
            })
        }
    }

    /// A B-CAS-like card speaking the card side of T=1.
    struct Card {
        script: VecDeque<Act>,
        rx: VecDeque<u8>,
        /// N(S) of the card's next I-block.
        ns: bool,
        /// N(S) expected in the reader's next I-block.
        nr: bool,
        ifsd: u8,
        last: Vec<u8>,
        commands: Vec<Vec<u8>>,
        waits: Vec<u32>,
        guards: Vec<u32>,
        sent: Vec<Vec<u8>>,
        /// Pending answer to give after a WTX or IFS exchange.
        deferred: Option<Vec<u8>>,
        /// Answer every block with a request for this multiplier, after thinking this long.
        wtx_forever: Option<(u8, u64)>,
        /// Do not answer the reader's WTX response.
        silent_after_wtx: bool,
        /// Microseconds that have passed: the time the card thinks and the reader waits in vain.
        clock_us: u64,
    }

    impl Card {
        fn new(script: &[Act]) -> Self {
            Card {
                script: script.iter().copied().collect(),
                rx: VecDeque::new(),
                ns: false,
                nr: false,
                ifsd: 32,
                last: Vec::new(),
                commands: Vec::new(),
                waits: Vec::new(),
                guards: Vec::new(),
                sent: Vec::new(),
                deferred: None,
                wtx_forever: None,
                silent_after_wtx: false,
                clock_us: 0,
            }
        }

        fn queue(&mut self, f: Vec<u8>, corrupt: bool) {
            let mut f = f;
            if corrupt {
                *f.last_mut().unwrap() ^= 0x55;
            }
            self.rx.extend(f);
        }

        fn answer(&mut self, f: Vec<u8>, act: Act) {
            match act {
                Act::Wtx(m) => {
                    self.deferred = Some(f);
                    self.queue(frame(0xC3, &[m]), false);
                }
                Act::Ifs(n) => {
                    self.deferred = Some(f);
                    self.queue(frame(0xC1, &[n]), false);
                }
                _ => {
                    self.last = f.clone();
                    self.queue(f, act == Act::CorruptAnswer);
                }
            }
        }

        fn handle(&mut self, f: &[u8]) {
            if let Some((m, think_us)) = self.wtx_forever {
                assert_eq!(lrc(f), 0, "reader sent a bad LRC");
                self.clock_us += think_us;
                self.queue(frame(0xC3, &[m]), false);
                return;
            }
            let act = self.script.pop_front().unwrap_or(Act::Normal);
            if act == Act::Silent {
                return;
            }
            let (pcb, inf) = (f[1], &f[3..f.len() - 1]);
            assert_eq!(lrc(f), 0, "reader sent a bad LRC");
            if act == Act::CorruptRequest {
                let nr = if self.nr { 0x10 } else { 0 };
                self.queue(frame(0x81 | nr, &[]), false);
                return;
            }
            match pcb & 0xC0 {
                0x00 | 0x40 => {
                    let ns = pcb & 0x40 != 0;
                    if ns != self.nr {
                        // A repeated I-block: our answer was lost, send it again.
                        let last = self.last.clone();
                        self.queue(last, false);
                        return;
                    }
                    self.nr = !self.nr;
                    self.commands.push(inf.to_vec());
                    let mut answer: Vec<u8> = inf.iter().rev().copied().collect();
                    answer.extend_from_slice(&[0x90, 0x00]);
                    let mut pcb = if self.ns { 0x40 } else { 0x00 };
                    if act == Act::WrongNs {
                        pcb ^= 0x40;
                    } else {
                        self.ns = !self.ns;
                    }
                    if act == Act::Chained {
                        pcb |= 0x20;
                    }
                    self.answer(frame(pcb, &answer), act);
                }
                0x80 => {
                    // The reader asks for the block with N(S) = N(R) again.
                    let nr = pcb & 0x10 != 0;
                    let last_ns = !self.ns;
                    if nr == last_ns {
                        let last = self.last.clone();
                        self.queue(last, act == Act::CorruptAnswer);
                    } else {
                        let r = if self.nr { 0x10 } else { 0 };
                        self.queue(frame(0x82 | r, &[]), false);
                    }
                }
                _ => match pcb {
                    0xC0 => {
                        self.ns = false;
                        self.nr = false;
                        self.ifsd = 32;
                        self.answer(frame(0xE0, &[]), act);
                    }
                    0xC1 => {
                        self.ifsd = inf[0];
                        self.answer(frame(0xE1, inf), act);
                    }
                    0xE3 if self.silent_after_wtx => {}
                    0xE1 | 0xE3 => {
                        let deferred = self.deferred.take().expect("unsolicited S-response");
                        self.last = deferred.clone();
                        self.queue(deferred, false);
                    }
                    _ => panic!("unexpected PCB {pcb:#04x}"),
                },
            }
        }
    }

    impl Transport for Card {
        type Error = ();

        fn now_us(&self) -> u64 {
            self.clock_us
        }

        fn purge(&mut self) -> Result<(), ()> {
            self.rx.clear();
            Ok(())
        }

        fn send(&mut self, frame: &[u8], guard_us: u32) -> Result<(), ()> {
            self.guards.push(guard_us);
            self.sent.push(frame.to_vec());
            self.handle(frame);
            Ok(())
        }

        fn recv(&mut self, timeout_us: u32) -> Result<u8, RecvError<()>> {
            self.waits.push(timeout_us);
            self.rx.pop_front().ok_or_else(|| {
                self.clock_us += u64::from(timeout_us);
                RecvError::Timeout
            })
        }
    }

    fn session(card: &mut Card) -> Session {
        let mut s = Session::new(&PARAMS, TIMING);
        s.negotiate_ifsd(card).unwrap();
        assert_eq!(card.ifsd, IFSD);
        s
    }

    fn ecm(s: &mut Session, card: &mut Card, cmd: &[u8]) -> Result<Vec<u8>, Error<()>> {
        let mut out = [0u8; MAX_INF];
        let n = s.transceive(card, cmd, &mut out)?;
        Ok(out[..n].to_vec())
    }

    fn expected(cmd: &[u8]) -> Vec<u8> {
        let mut v: Vec<u8> = cmd.iter().rev().copied().collect();
        v.extend_from_slice(&[0x90, 0x00]);
        v
    }

    #[test]
    fn sequence_numbers_alternate() {
        let mut card = Card::new(&[]);
        let mut s = session(&mut card);
        for i in 0..5u8 {
            let cmd = [0x90, 0x34, 0x00, 0x00, i];
            assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        }
        let pcbs: Vec<u8> = card.sent[1..].iter().map(|f| f[1]).collect();
        assert_eq!(pcbs, [0x00, 0x40, 0x00, 0x40, 0x00]);
        assert_eq!(s.stats(), Stats::default());
        // Every block waits BGT, and the first character of an answer gets the full BWT.
        assert!(card.guards.iter().all(|&g| g == TIMING.bgt_us));
        assert_eq!(card.waits[0], TIMING.bwt_us);
    }

    #[test]
    fn corrupted_answer_is_requested_again() {
        let mut card = Card::new(&[Act::Normal, Act::CorruptAnswer]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x34, 0x00, 0x00, 0x01];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        // The reader sent R(N(R)=0, EDC error) and did not toggle its N(S).
        assert_eq!(card.sent[2], frame(0x81, &[]));
        assert_eq!(s.stats().edc_errors, 1);
        assert_eq!(card.commands.len(), 1);
        let next = [0x90, 0x34, 0x00, 0x00, 0x02];
        assert_eq!(ecm(&mut s, &mut card, &next).unwrap(), expected(&next));
        assert_eq!(card.sent.last().unwrap()[1], 0x40);
    }

    #[test]
    fn corrupted_command_is_retransmitted() {
        let mut card = Card::new(&[Act::Normal, Act::CorruptRequest]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x32, 0x00, 0x00, 0x00];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        assert_eq!(card.sent[1], card.sent[2]);
        assert_eq!(s.stats().retransmissions, 1);
    }

    #[test]
    fn waiting_time_extension_applies_to_one_block() {
        let mut card = Card::new(&[Act::Normal, Act::Wtx(3)]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x36, 0x00, 0x00, 0x00];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        assert_eq!(card.sent[2], frame(0xE3, &[3]));
        let first_waits: Vec<u32> = card
            .waits
            .iter()
            .copied()
            .filter(|&w| w != TIMING.cwt_us)
            .collect();
        assert_eq!(
            first_waits,
            [TIMING.bwt_us, TIMING.bwt_us, 3 * TIMING.bwt_us]
        );
        assert_eq!(s.stats().wtx, 1);
    }

    #[test]
    fn endless_waiting_time_extensions_lose_the_session() {
        let mut card = Card::new(&[]);
        let mut s = session(&mut card);
        card.wtx_forever = Some((1, 0));
        let sent_before = card.sent.len();
        assert_eq!(
            ecm(&mut s, &mut card, &[0x90, 0x34, 0x00, 0x00, 0x00]),
            Err(Error::SessionLost)
        );
        // Per attempt: the request, MAX_WTX answers, then MAX_RETRANSMIT repeats. Two attempts
        // run (the I-block, then RESYNCH), and no more.
        let per_attempt = 1 + usize::from(MAX_WTX) + usize::from(MAX_RETRANSMIT);
        assert_eq!(card.sent.len() - sent_before, 2 * per_attempt);
        assert_eq!(s.stats().wtx, 2 * u32::from(MAX_WTX));
    }

    #[test]
    fn long_waiting_time_extensions_end_within_the_limit() {
        let mut card = Card::new(&[]);
        let mut s = session(&mut card);
        // A card that thinks 5 s, which its last extension allowed, and asks for 255 BWT again.
        let think_us = 5_000_000;
        card.wtx_forever = Some((255, think_us));
        let start = card.clock_us;
        assert_eq!(
            ecm(&mut s, &mut card, &[0x90, 0x34, 0x00, 0x00, 0x00]),
            Err(Error::TimeLimit)
        );
        let spent = card.clock_us - start;
        // The deadline is only checked when the reader waits, so the last answer may overshoot.
        assert!(
            (EXCHANGE_LIMIT_US..EXCHANGE_LIMIT_US + think_us).contains(&spent),
            "{spent}"
        );
        // No retransmission or resynchronisation follows the expiry.
        assert_eq!(card.sent.len(), 1 + 1 + 5);
    }

    #[test]
    fn a_wait_is_cut_to_what_remains_of_the_limit() {
        let mut card = Card::new(&[Act::Normal, Act::Wtx(255)]);
        card.silent_after_wtx = true;
        let mut s = session(&mut card);
        let start = card.clock_us;
        assert_eq!(
            ecm(&mut s, &mut card, &[0x90, 0x34, 0x00, 0x00, 0x00]),
            Err(Error::TimeLimit)
        );
        // 255 BWT were granted, but only the limit is waited for, and then the exchange ends
        // without asking the card again.
        assert_eq!(card.waits.last(), Some(&(EXCHANGE_LIMIT_US as u32)));
        assert_eq!(card.clock_us - start, EXCHANGE_LIMIT_US);
        let pcbs: Vec<u8> = card.sent.iter().map(|f| f[1]).collect();
        assert_eq!(pcbs, [0xC1, 0x00, 0xE3]);
    }

    #[test]
    fn ifs_negotiation_is_limited_too() {
        let mut card = Card::new(&[Act::Silent; 8]);
        let timing = Timing {
            bwt_us: 100_000_000,
            ..TIMING
        };
        let mut s = Session::new(&PARAMS, timing);
        assert_eq!(s.negotiate_ifsd(&mut card), Err(Error::TimeLimit));
        assert_eq!(card.clock_us, EXCHANGE_LIMIT_US);
        assert_eq!(card.sent.len(), 1);
    }

    #[test]
    fn the_limit_starts_with_each_exchange() {
        let mut card = Card::new(&[]);
        let mut s = session(&mut card);
        // Time spent in earlier exchanges does not count against the next one.
        card.clock_us = 10 * EXCHANGE_LIMIT_US;
        let cmd = [0x90, 0x34, 0x00, 0x00, 0x01];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
    }

    #[test]
    fn drain_gives_up_on_a_card_that_never_stops() {
        let mut s = Session::new(&PARAMS, TIMING);
        s.deadline_us = u64::MAX;
        let mut line = Babbler {
            received: 0,
            purges: 0,
        };
        assert_eq!(s.drain(&mut line), Ok(()));
        assert_eq!((line.received, line.purges), (MAX_BLOCK + 1, 1));

        // A block with a bad LRC is reported as such even when the line then keeps running.
        line.received = 0;
        assert_eq!(s.receive(&mut line, TIMING.bwt_us), Ok(Err(Fault::Edc)));
        assert_eq!(line.received, 4 + MAX_BLOCK + 1);
    }

    #[test]
    fn card_may_change_ifsc() {
        let mut card = Card::new(&[Act::Normal, Act::Ifs(16)]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x30, 0x00, 0x00, 0x00];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        assert_eq!(s.ifsc(), 16);
        assert_eq!(
            ecm(&mut s, &mut card, &[0u8; 17]),
            Err(Error::CommandTooLong)
        );
    }

    #[test]
    fn wrong_sequence_number_and_chaining_are_rejected() {
        for act in [Act::WrongNs, Act::Chained] {
            let mut card = Card::new(&[Act::Normal, act]);
            let mut s = session(&mut card);
            let cmd = [0x90, 0x34, 0x00, 0x00, 0x07];
            // The faulty answer is never returned; the R-block makes the card resend its last
            // block, which is still faulty, until the session resynchronises and retries.
            let got = ecm(&mut s, &mut card, &cmd).unwrap();
            assert_eq!(got, expected(&cmd), "{act:?}");
            assert!(s.stats().other_errors >= 1, "{act:?}");
            assert_eq!(s.stats().resynchs, 1, "{act:?}");
        }
    }

    #[test]
    fn silent_card_is_resynchronised_then_retried() {
        let mut card = Card::new(&[Act::Normal, Act::Silent, Act::Silent, Act::Silent]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x34, 0x00, 0x00, 0x09];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        let st = s.stats();
        assert_eq!((st.retransmissions, st.resynchs), (2, 1));
        // After RESYNCH the reader re-announced IFSD and restarted with N(S) = 0.
        let pcbs: Vec<u8> = card.sent.iter().map(|f| f[1]).collect();
        assert_eq!(pcbs, [0xC1, 0x00, 0x82, 0x82, 0xC0, 0xC1, 0x00]);
        assert_eq!(card.ifsd, IFSD);
    }

    #[test]
    fn a_command_that_no_longer_fits_after_resynchronisation_loses_the_session() {
        let mut card = Card::new(&[Act::Normal, Act::Ifs(200)]);
        let mut s = session(&mut card);
        let cmd = [0x90, 0x30, 0x00, 0x00, 0x00];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
        assert_eq!(s.ifsc(), 200);
        // Longer than the ATR's IFSC, which RESYNCH falls back to.
        let long = [0u8; 150];
        card.script.extend([Act::Silent; 3]);
        assert_eq!(ecm(&mut s, &mut card, &long), Err(Error::SessionLost));
        assert_eq!(s.ifsc(), PARAMS.ifsc);
    }

    #[test]
    fn dead_card_loses_the_session() {
        let mut card = Card::new(&[Act::Normal; 1]);
        card.script.extend([Act::Silent; 64]);
        let mut s = session(&mut card);
        assert_eq!(
            ecm(&mut s, &mut card, &[0x90, 0x34, 0x00, 0x00, 0x00]),
            Err(Error::SessionLost)
        );
        // IFS, then the I-block and MAX_RETRANSMIT R-blocks, then MAX_RESYNCH RESYNCH requests.
        let pcbs: Vec<u8> = card.sent.iter().map(|f| f[1]).collect();
        assert_eq!(pcbs, [0xC1, 0x00, 0x82, 0x82, 0xC0, 0xC0, 0xC0]);
    }

    #[test]
    fn ifs_negotiation_survives_one_lost_answer() {
        let mut card = Card::new(&[Act::CorruptAnswer]);
        let mut s = Session::new(&PARAMS, TIMING);
        s.negotiate_ifsd(&mut card).unwrap();
        assert_eq!(card.sent.len(), 2);
        assert_eq!(card.sent[0], card.sent[1]);
    }

    #[test]
    fn oversized_response_is_reported_after_a_complete_exchange() {
        let mut card = Card::new(&[]);
        let mut s = session(&mut card);
        let mut small = [0u8; 4];
        assert_eq!(
            s.transceive(&mut card, &[1, 2, 3, 4, 5], &mut small),
            Err(Error::ResponseTooLong)
        );
        // The sequence numbers advanced, so the next exchange still works.
        let cmd = [9, 9];
        assert_eq!(ecm(&mut s, &mut card, &cmd).unwrap(), expected(&cmd));
    }

    #[test]
    fn pcb_decoding_rejects_undefined_encodings() {
        assert_eq!(decode(0x01, &[]), None);
        assert_eq!(decode(0x83, &[]), None);
        assert_eq!(decode(0xA0, &[]), None);
        assert_eq!(decode(0x80, &[1]), None);
        assert_eq!(decode(0xC4, &[]), None);
        assert_eq!(decode(0xC1, &[]), None);
        assert_eq!(decode(0xC3, &[1, 2]), None);
        assert_eq!(decode(0x90, &[]), Some(Block::R { nr: true }));
        assert_eq!(
            decode(0xE3, &[5]),
            Some(Block::S {
                response: true,
                kind: SKind::Wtx,
                param: Some(5)
            })
        );
    }
}
