# ifd-pxw3pe: pcscd driver for the PX-W3PE card reader

`libifd_pxw3pe.so` is a pcsc-lite IFD handler for the B-CAS card reader built
into the PX-W3PE. The kernel driver exposes each reader as
`/dev/pxw3pe-cardN` and runs the T=1 protocol itself; this library connects
those nodes to pcscd, so that PC/SC applications such as recisdb (through
libarib25) can use the card. Other readers are not affected.

pcscd treats it like a serial reader driver: it is not a USB bundle under
`/usr/lib/pcsc/drivers`, but a plain shared library that an entry in
`/etc/reader.conf.d` names together with its device node.

## Build

Requirements: a Rust toolchain (1.82 or newer) and, for the tests only, a C
compiler and the pcsc-lite headers.

```sh
sudo apt-get install pcscd libpcsclite-dev   # the -dev package is needed for the tests
cargo build --release
cargo test                                    # optional
```

The result is `target/release/libifd_pxw3pe.so`. Its only exported symbols
are the ten `IFDH*` functions:

```sh
nm -D --defined-only target/release/libifd_pxw3pe.so
```

## Install

```sh
sudo install -D -m 0644 target/release/libifd_pxw3pe.so \
    /usr/local/lib/pcsc/drivers/serial/libifd_pxw3pe.so
sudo install -D -m 0644 packaging/pxw3pe /etc/reader.conf.d/pxw3pe
```

If the library is installed elsewhere, change `LIBPATH` in
`/etc/reader.conf.d/pxw3pe` to match. With more than one board, give each
further `/dev/pxw3pe-cardN` a copy of that file of its own (for example
`/etc/reader.conf.d/pxw3pe-1` for `/dev/pxw3pe-card1`), as its comments
describe: pcscd ignores the rest of a file after an entry whose device node is
missing, so one file per board keeps the other boards usable.

### When pcscd picks up the reader

pcscd reads `/etc/reader.conf.d` only at start-up, and drops an entry whose
`DEVICENAME` does not exist then (together with any later entry in the same
file). So the kernel driver must be loaded (and
`/dev/pxw3pe-card0` present) before pcscd starts. Debian starts pcscd on
demand through `pcscd.socket` and stops it after 60 s without clients, so
usually nothing needs to be done; to apply a change at once:

```sh
sudo systemctl restart pcscd.service
```

pcscd does not re-read `reader.conf.d` on its own; once the board is gone it
keeps the reader listed as unavailable until it is restarted.

### Unloading and reloading the kernel driver

While pcscd has `/dev/pxw3pe-card0` open, the node keeps the kernel module in
use, and `rmmod pxw3pe` fails with "Module pxw3pe is in use". Stop pcscd first;
stopping `pcscd.socket` as well keeps a client from starting it again halfway:

```sh
sudo systemctl stop pcscd.service pcscd.socket
sudo rmmod pxw3pe
sudo modprobe pxw3pe
sudo systemctl start pcscd.socket
```

Starting only the socket is enough: pcscd reads `reader.conf.d` when the next
client starts it, by which time the node exists again. The same applies when
the board is unbound from its driver without unloading the module: the open
node stays valid but fails with `ENODEV`, and pcscd has to be restarted once the
board is bound again.

### Access for services (polkit)

Debian's pcscd asks polkit whether a client may use it. The shipped policy
(`/usr/share/polkit-1/actions/org.debian.pcsc-lite.policy`) allows only
processes in an active local session, so a recorder running as a system
service, from cron or over SSH is refused. Allow its user explicitly, for
example in `/etc/polkit-1/rules.d/50-pxw3pe.rules` (replace `recorder` with
the user the application runs as):

```js
polkit.addRule(function(action, subject) {
    if ((action.id == "org.debian.pcsc-lite.access_pcsc" ||
         action.id == "org.debian.pcsc-lite.access_card") &&
        subject.user == "recorder") {
        return polkit.Result.YES;
    }
});
```

## Verify

Install `pcsc_scan` and run it with a card inserted:

```sh
sudo apt-get install pcsc-tools
pcsc_scan
```

It should list `PX-W3PE Card Reader 00 00`, report the card as inserted
and show its ATR (a B-CAS card answers in T=1 specific mode, for example
`3B F0 12 00 FF 91 81 B1 7C 45 1F 03 99`). Pulling the card should be
reported within about half a second; pcscd polls the reader every 400 ms.
While a session is open, the kernel driver samples the card-detect switch every
100 ms itself and ends the session once two samples at least 100 ms apart,
with none finding the card in between, read it absent, so a card that is
pulled for a little more than 200 ms is noticed even if pcscd does not poll at
that moment. If it is back before pcscd's next
poll, the driver still reports it absent to that one poll, so pcscd sees the
card leave and come back, and applications get `SCARD_W_REMOVED_CARD` and
reconnect. A single pull shorter than 100 ms is taken for a bouncing contact;
one of 100 to 200 ms may or may not be noticed.

If the reader does not appear, run pcscd in the foreground with debug output:

```sh
sudo systemctl stop pcscd.service pcscd.socket
sudo pcscd --foreground --debug
# ... Ctrl-C, then:
sudo systemctl start pcscd.socket
```

Look for `Card ATR:`, and with more than one board configured also for
`Support 8 simultaneous readers` and `Driver is thread safe`. `Open Port 0x0 Failed (/dev/pxw3pe-card0)` means that the node
could not be opened or is not a PX-W3PE card reader, and
`Error with device /dev/pxw3pe-card0` that the node did not exist when pcscd
read the configuration.

## Behaviour

- Power-up and reset both reset the card through the kernel and return its
  ATR; power-down holds the card in reset. Closing the reader powers it down.
- Only T=1 is offered. The card runs in the specific mode its ATR announces,
  so no PPS is ever sent; a protocol request is accepted only if it matches
  what the ATR already specifies.
- After a transmission error the kernel ends the T=1 session; the application
  has to reset the card (`SCardReconnect` with `SCARD_RESET_CARD`) before the
  next command.
- `SCardControl` is not supported: the reader has no functions beyond the
  card.
