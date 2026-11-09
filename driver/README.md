# pxw3pe: Linux driver for the PX-W3PE V2.0

Rust kernel module for the PX-W3PE V2.0 ISDB-S/ISDB-T PCIe tuner (ASICEN ASV5220 bridge, PCI
188b:5220, subsystem 0b06:0003; probe refuses any other board, also when bound through
`driver_override`, because other boards built on the same bridge have different circuitry behind
it). It provides four DVB adapters (two ISDB-S, two ISDB-T), the 15 V LNB supply with
short-circuit protection, and the built-in B-CAS card reader.

On a card (Linux 6.18.59, x86-64) the driver probes, all four tuners lock and deliver a clean,
descrambled TS at once, and the LNB supply drives a BS antenna; the card reader has only been
checked without a card.

## Kernel requirements

The module is built against a kernel tree with:

| Option | Why |
|---|---|
| `CONFIG_RUST=y` | the driver is written in Rust |
| `CONFIG_DVB_CORE` | DVB adapters |
| `CONFIG_PCI_MSI`, `CONFIG_HIGH_RES_TIMERS` | standard on x86-64 |

It was compile-checked against Linux 6.18.59 (x86-64, `LLVM=1`). The Rust compiler must be the
one the kernel was built with.

## Build and install

`KDIR` is the kernel's build tree, or its headers if they contain the compiled Rust crates
(`rust/*.rmeta` and `rust/*.so`). The headers package of mainline's `make bindeb-pkg` leaves them
out, and a build against it fails with "can't find crate for `core`".
`../kernel/rust-headers.patch` adds them, together with the kernel's `.clippy.toml`, and
`../kernel/build-kernel.sh <version>` builds and installs a kernel with that patch applied.

```sh
make KDIR=/lib/modules/$(uname -r)/build LLVM=1
sudo make -C /lib/modules/$(uname -r)/build M=$PWD modules_install
sudo depmod -a
```

### DKMS

`dkms.conf` builds the module for every kernel that has the options above and skips the others.
Copy only the sources, not build output, into the DKMS source directory:

```sh
sudo mkdir -p /usr/src/pxw3pe-0.1.0
sudo cp -r dkms.conf Kbuild Makefile *.rs *_glue.[ch] proto uapi \
    /usr/src/pxw3pe-0.1.0/
sudo dkms add pxw3pe/0.1.0
sudo dkms install pxw3pe/0.1.0 -k <kernel release>
```

DKMS builds against `/lib/modules/<kernel release>/build`, so it needs headers with the Rust
crates (see above), or `--kernelsourcedir` pointing at the build tree. It finds `rustc` through
`PATH`. rustup adds `~/.cargo/bin` to `PATH` only in login shells, so a DKMS run started from
cron, a systemd unit or plain `sudo` (which resets `PATH`) does not find it; with rustup, run
the commands above from a root login shell (`sudo -i`).

DKMS signs with its own key (`/var/lib/dkms/mok.key`) unless told otherwise, and a kernel that
does not trust that key taints itself on loading the module. For kernels from `build-kernel`,
`../kernel/dkms-kernel-key.conf`, installed as `/etc/dkms/framework.conf.d/kernel-key.conf`,
makes DKMS sign with the key the kernel was built with, taken from its build tree in
`/usr/src/linux-<version>`; the headers package does not carry that key.

The uAPI headers are `uapi/pxw3pe_card.h` (card reader) and `uapi/pxw3pe_lnb.h` (LNB supply
status).

## Descrambling

The card's scrambler chip (ASIE5606, version 0x11: XOR only, no DES) scrambles every TS packet
with a mask it derives from a 16-byte seed the driver writes into it. The seed is arbitrary; the
driver writes a fixed one at probe and derives the mask itself (`proto/ts.rs`), so nothing has to
be installed or generated. Probe fails, with the version in the log, if the scrambler does not
report version 0x11.

## Module parameters

| Parameter | Default | Meaning |
|---|---|---|
| `adapter_nr` | unset | DVB adapter numbers, in the order ISDB-S 0, ISDB-S 1, ISDB-T 0, ISDB-T 1 |

## Using the tuners

- ISDB-T: `DTV_FREQUENCY` in Hz (for example 473142857 for UHF 13). The tuner is programmed with
  the frequency rounded to the nearest kHz; 93 to 768 MHz are accepted.
- ISDB-S: `DTV_FREQUENCY` as the L-band IF in kHz (RF minus 10678000, for example 1318000 for
  BS-15; 1049480 to 2053000 are accepted). `DTV_STREAM_ID` is the TSID (1..0xFFFE), as the DVB
  API defines it, on BS and CS alike; any other value selects no stream and the demodulator
  outputs any TS. The TSID is written on every tune.
- A tune that fails is reported as no lock until the next tune.
- Lock and signal bits follow the mainline TC90522 decoding; ISDB-T lock additionally needs the
  mode check `(0xB0 & 0x0F) >= 8`. The C/N is reported with lock only.
- Packets flagged with a transport error are delivered (the demux flags them); null packets are
  dropped.
- All four streams arrive on one DMA port and are told apart by the sync byte tag of their
  demodulator core. A transfer in which fewer than half of the packets start with a known tag is
  out of packet sync: it is not delivered and the port restarts. Occasional untagged packets do
  not.
- The bridge loses packets when a DMA transfer completes before the previous one has been
  handed back, so the interrupt thread must run within one transfer time. The driver sizes
  each transfer to about 100 ms of what the port currently carries (312 to 4992 packets): a
  kernel that lets other work hold the interrupt's CPU for 10 to 20 ms, which an idle machine
  with Intel graphics does while it polls for displays, loses nothing then. Packets reach the
  demux up to 100 ms after they arrived. A host that stalls for longer than that still loses
  packets; `preempt=full` on the kernel command line removes most such stalls.
- While a port's DMA runs the driver also limits how long the CPUs may take to leave an idle
  state (20 µs, through the kernel's CPU latency QoS). The bridge keeps no packet in reserve,
  and on a host whose memory path sleeps with the CPUs a write delayed by a deeper idle state
  loses the packet that arrives meanwhile: a few hundred a second on an idle Core i3-9100
  machine. The machine draws a little more power while a tuner streams, and none otherwise.
- After a tune the packets are dropped until the demodulator has reported lock for 0.5 s, so
  that a recording does not start with the losses of signal acquisition (and of the LNA
  comparison of a satellite tuner's first tune).

## LNB supply

`FE_SET_VOLTAGE` with 13 or 18 V switches the supply on (the board only has 15 V), OFF switches
it off; it stays on while either satellite frontend asks for it, and closing a frontend withdraws
its request. The driver checks the supply's power-good bit every 100 ms, starting 200 ms after
switch-on. Two consecutive samples without power good (a short) switch the supply off and log an
error; it then stays off, and further voltage requests fail with `EIO`, until both satellite
frontends have withdrawn their requests (`FE_SET_VOLTAGE` OFF or close). A switch that fails on
the I2C bus is repeated by the next check, so the supply follows the requests even after an I2C
error; the request that saw the error gets `EIO`.

`/dev/pxw3pe-lnbN` (same N as the board's card reader) reports the supply's state with one
ioctl, `PXW3PE_LNB_GET_STATUS` (`uapi/pxw3pe_lnb.h`): whether it is on, the power-good bit (read
by the call, while on), whether it was switched off after a short, and which of the two
satellite frontends request it. It cannot switch the supply. The DVB property `DTV_VOLTAGE` does
not tell this: it holds the last value set through that property on that frontend only, not one
set with `FE_SET_VOLTAGE`, and nothing of the other frontend or a short.

## Removal, reboot and power-off

On unload the driver holds the card reader in reset, waits for the DVB devices to be closed,
switches the LNB supply off, stops the DMA and clears bus mastering before the DMA buffers are
freed, and powers the chips down. The kernel has no shutdown callback for Rust PCI drivers, so a
reboot notifier holds the card reader in reset, switches the LNB supply off and stops the DMA
before the system restarts, halts or powers off; afterwards `FE_SET_VOLTAGE` fails with `ENODEV`.
If the supply cannot be switched off (I2C failure), the driver logs "LNB supply may still be on".

## Limitations

- No suspend/resume: the Rust PCI bindings of the targeted kernels have no power-management
  callbacks, and the chips lose their state in suspend. Unload the module before suspending.
- Almost no gain control: the tuners run with fixed gain values. The one exception is the input LNA
  of the satellite tuners, whose two useful settings the driver compares by C/N on the first lock
  after a satellite frontend was opened while the other one is closed, keeping the better one for
  both tuners until both frontends are closed; that first tune takes about a second longer. The two
  tuners share the antenna input, and switching the LNA gain of one takes the other's carrier away
  for up to 150 ms, so the comparison never runs while the other tuner may be streaming.

## Card reader

`/dev/pxw3pe-card0` speaks APDUs; the driver runs T=1 itself. Use it through pcscd with the
IFD handler in `../tools/ifd-pxw3pe`.

An open node keeps the module loaded, so `rmmod pxw3pe` fails with "in use" while pcscd holds it.
To unload or reload the module, stop `pcscd.service` and `pcscd.socket` first, and start
`pcscd.socket` again afterwards (see `../tools/ifd-pxw3pe/INSTALL.md`). Unbinding the PCI device
does not wait for open files: they stay valid and fail with `ENODEV`.

A card pulled for 200 ms or more while a session is open ends the session, even when no
operation is running at that moment: the next `TRANSMIT` fails with `ENOTCONN`, and the next
`GET_STATUS` reports `PXW3PE_CARD_REMOVED` once. The card counts as pulled once two samples of
the detect switch at least 100 ms apart, with no sample finding it in between, read it absent, so
a single dropout shorter than 100 ms is taken for a bouncing contact. A card
operation that takes longer than 30 s in total is given up as lost (`EIO`), and a caller that is
killed while it waits for the card or for another call gets `EINTR` at once.

## Tests

The hardware-independent parts (`proto/`) have host unit tests:

```sh
cd host-tests && cargo test && cargo clippy --all-targets -- -D warnings
```
