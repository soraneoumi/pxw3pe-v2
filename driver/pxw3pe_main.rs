// SPDX-License-Identifier: GPL-2.0

//! Driver for the PX-W3PE V2.0 ISDB-S/ISDB-T PCIe tuner card.

mod bringup;
mod card;
mod chrdev;
mod dvb;
mod frontend;
mod hw;
mod lnb;
mod platform;
mod proto;
mod stream;

use kernel::{
    bindings, c_str,
    device::Core,
    dma::{self, DmaMask},
    irq, new_mutex, pci,
    prelude::*,
    sync::{Arc, Mutex},
};

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <Pxw3pe as pci::Driver>::IdInfo,
    [(PX_W3PE, ())]
);

/// ASICEN ASV5220 with the PX-W3PE V2.0 subsystem ID 0B06:0003. Other boards built on the same
/// bridge (subsystem IDs 0B06:0001 and 0B06:0002) have different chips behind it and must
/// not bind.
///
/// The named PCI ID constants of the kernel crate come from `pci_ids.h`, which does not include
/// ASICEN, and `pci::DeviceId` has no constructor for subsystem IDs, so the entry is built
/// directly.
const PX_W3PE: pci::DeviceId = {
    let id = bindings::pci_device_id {
        vendor: VENDOR as u32,
        device: DEVICE as u32,
        subvendor: SUBSYSTEM_VENDOR as u32,
        subdevice: SUBSYSTEM as u32,
        class: 0,
        class_mask: 0,
        driver_data: 0,
        override_only: 0,
    };
    // SAFETY: `pci::DeviceId` is `#[repr(transparent)]` over `bindings::pci_device_id`, and any
    // field values form a valid ID entry; `driver_data` is overwritten by the table macro.
    unsafe { core::mem::transmute::<bindings::pci_device_id, pci::DeviceId>(id) }
};

/// PCI IDs of the bridge, and the first half of the subsystem ID of the supported board.
const VENDOR: u16 = 0x188B;
const DEVICE: u16 = 0x5220;
const SUBSYSTEM_VENDOR: u16 = 0x0B06;

/// PCI subsystem device of the supported board.
const SUBSYSTEM: u16 = 0x0003;

/// Driver state of one card. Fields are dropped in declaration order after [`PinnedDrop`] has
/// shut the hardware down: the interrupt is freed before the remaining users of the stream (the
/// DMA buffers go with the last reference to it, by which time bus mastering is off), and the
/// PCI enable guard comes last because disabling the device releases its interrupt routing,
/// which has to follow `free_irq` (see [`platform::PciEnabled`]).
#[pin_data(PinnedDrop)]
struct Pxw3pe {
    /// The DVB adapters; taken out first on removal so that streaming has ended before the
    /// hardware is powered down.
    #[pin]
    adapters: Mutex<KVec<dvb::Adapter<frontend::Tuner>>>,
    #[pin]
    irq: irq::ThreadedRegistration<Arc<stream::Stream>>,
    nodes: Nodes,
    /// Checks the LNB supply periodically; its callback works on `lnb`, which is dropped later
    /// (and finds the supply disabled by then).
    poller: platform::Poller<lnb::Lnb>,
    /// Delivers the packets of slow ports and reports input overflows; its callback works on
    /// `stream`, which `PinnedDrop` quiesces first, so that it finds no running port any more.
    stream_poller: platform::Poller<stream::Stream>,
    stream: Arc<stream::Stream>,
    core: Arc<frontend::Core>,
    lnb: Arc<lnb::Lnb>,
    board: Arc<hw::Board>,
    /// I2C address of the scrambler function.
    scrambler: u8,
    /// Registered once the fields above are complete, and unregistered on removal before they
    /// are torn down: its callback works on them.
    reboot: platform::RebootHook,
    /// Last field, see above.
    enable: platform::PciEnabled,
}

/// The character devices of a board. Fields are dropped in declaration order, so the number in
/// their names is released only after both registrations have gone, however the value goes.
struct Nodes {
    card: Pin<KBox<card::CardDevice>>,
    _lnb: lnb::LnbDevice,
    _slot: chrdev::Slot,
}

impl Pxw3pe {
    /// Stops the DMA of the card for good and reports a channel that did not stop.
    fn quiesce_stream(&self) {
        if self.stream.quiesce() == Err(stream::StopError::Timeout) {
            dev_err!(self.enable.device(), "a DMA channel did not stop\n");
        }
    }
}

impl platform::Reboot for Pxw3pe {
    /// The system restarts, halts or powers off: nothing resets the board's 15 V supply by
    /// itself, and the card should not be left powered up with a stale session or DMA armed.
    fn prepare(&self) {
        self.nodes.card.shutdown();
        self.lnb.off();
        self.quiesce_stream();
    }
}

impl pci::Driver for Pxw3pe {
    type IdInfo = ();

    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe(pdev: &pci::Device<Core>, _info: &Self::IdInfo) -> Result<Pin<KBox<Self>>> {
        let dev = pdev.as_ref();
        // `driver_override` and dynamic IDs bind this driver to any device, bypassing the ID
        // table (`pci_match_device()` in `drivers/pci/pci-driver.c`), and the power and GPIO
        // sequences below are those of this board only.
        if pdev.vendor_id().as_raw() != VENDOR
            || pdev.device_id() != DEVICE
            || pdev.subsystem_vendor_id() != SUBSYSTEM_VENDOR
            || pdev.subsystem_device_id() != SUBSYSTEM
        {
            dev_err!(
                dev,
                "unsupported board {:04x}:{:04x} (subsystem {:04x}:{:04x}); only {:04x}:{:04x} \
                 (subsystem {:04x}:{:04x}) is supported\n",
                pdev.vendor_id().as_raw(),
                pdev.device_id(),
                pdev.subsystem_vendor_id(),
                pdev.subsystem_device_id(),
                VENDOR,
                DEVICE,
                SUBSYSTEM_VENDOR,
                SUBSYSTEM
            );
            return Err(ENODEV);
        }
        // Undoes the enable, and clears bus mastering, on every failure from here on.
        let enable = platform::PciEnabled::new(pdev)?;
        // A previous owner may have left bus mastering on, with a channel running.
        enable.clear_master();
        let board = hw::Board::new(pdev)?;
        bringup::mask_interrupts(&board)?;
        bringup::init_bridge(&board)?;
        bringup::probe_i2c_engine(&board).inspect_err(|_| {
            dev_err!(
                dev,
                "I2C engine does not answer at the ASV5220 register map; other bridge variants \
                 are not supported\n"
            );
        })?;
        // A previous driver instance may have left DMA armed with stale addresses; it must be
        // stopped before the device may master the bus again.
        stream::shutdown(&board).map_err(|e| match e {
            stream::StopError::Gone => ENODEV,
            stream::StopError::Timeout => {
                dev_err!(
                    dev,
                    "a DMA channel is still running; not enabling bus mastering\n"
                );
                ETIMEDOUT
            }
        })?;
        pdev.set_master();
        // SAFETY: No DMA mapping or allocation exists for the device yet.
        unsafe { dma::Device::dma_set_mask_and_coherent(pdev, DmaMask::new::<32>())? };

        let mut power = PowerGuard {
            board: board.clone(),
            scrambler: None,
            armed: true,
        };
        bringup::power_on(&board)?;
        let scrambler = bringup::find_scrambler(&board).inspect_err(|_| {
            dev_err!(dev, "scrambler not found or of an unsupported version\n");
        })?;
        power.scrambler = Some(scrambler);
        // The state of the supply bit at power-up is unknown, and on this board a set bit is
        // 15 V on the antenna connector.
        bringup::lnb_switch(&board, false)?;
        let base = frontend::probe(&board).map_err(|e| {
            dev_err!(
                dev,
                "no single demodulator group found; cores answering (bit n = address 0x10 + n): \
                 {:#04x}\n",
                e.answered
            );
            ENODEV
        })?;
        let routing = proto::ts::Routing::for_base(base).ok_or(EINVAL)?;

        let stream = stream::Stream::new(
            dev,
            board.clone(),
            routing,
            proto::ts::Descrambler::new(&proto::ts::SEED),
        )?;
        let lnb = lnb::Lnb::new(board.clone())?;
        let core = frontend::Core::new(board.clone(), stream.clone(), routing, lnb.clone())?;
        core.init_chips().map_err(|e| {
            dev_err!(
                dev,
                "tuner {} failed to initialise: {:?}\n",
                e.tuner,
                e.cause
            );
            e.errno()
        })?;
        bringup::load_seed(&board, scrambler)?;

        let slot = chrdev::Slot::claim()?;
        let card = card::CardDevice::new(board.clone(), &slot)?;
        let lnb_device = lnb::LnbDevice::new(lnb.clone(), &slot)?;
        let nodes = Nodes {
            card,
            _lnb: lnb_device,
            _slot: slot,
        };
        let reboot = platform::RebootHook::new()?;
        let poller = platform::Poller::new(lnb.clone(), lnb::POLL_MS)?;
        let stream_poller = platform::Poller::new(stream.clone(), stream::POLL_MS)?;
        // The interrupt handler exists before any adapter can start a stream.
        let this = KBox::pin_init(
            try_pin_init!(Pxw3pe {
                adapters <- new_mutex!(KVec::new()),
                irq <- pdev.request_threaded_irq(
                    0,
                    irq::Flags::SHARED,
                    c_str!("pxw3pe"),
                    Ok::<_, Error>(stream.clone())
                )?,
                nodes,
                poller,
                stream_poller,
                stream,
                core,
                lnb,
                board: board.clone(),
                scrambler,
                reboot,
                enable,
            }),
            GFP_KERNEL,
        )?;
        // From here on, `PinnedDrop` powers the card down on any failure.
        power.armed = false;
        // Everything the callback uses (card, LNB, stream) is complete now, and a frontend that
        // could switch the LNB on does not exist yet, so no request escapes it.
        //
        // SAFETY: The state is pinned in its box, which is not freed before `PinnedDrop` has run,
        // and that unregisters the hook before any field is dropped. The only unregistration is
        // in `PinnedDrop`, which cannot run while probe still holds the state.
        unsafe { this.reboot.register(this.as_ref())? };
        // The callback only needs `lnb`, which is complete.
        this.poller.start();
        // The interrupt handler is registered, and no port runs before an adapter exists.
        this.stream_poller.start();
        for index in 0..4 {
            let tuner = frontend::Tuner::new(index, this.core.clone())?;
            let adapter =
                dvb::Adapter::register(dev, &THIS_MODULE, &frontend::Tuner::info(index), tuner)?;
            dev_info!(dev, "tuner {} is DVB adapter {}\n", index, adapter.number());
            this.adapters.lock().push(adapter, GFP_KERNEL)?;
        }
        Ok(this)
    }
}

/// Powers the card down if probe fails after powering it up.
struct PowerGuard {
    board: Arc<hw::Board>,
    scrambler: Option<u8>,
    armed: bool,
}

impl Drop for PowerGuard {
    fn drop(&mut self) {
        if self.armed {
            // The supply's state is unknown until probe has switched it off, and a previous
            // driver instance may have left it on. It is switched at the control chip's fixed
            // address, so this does not wait for the scrambler probe: the board was identified
            // by its subsystem ID before the guard was armed.
            if bringup::lnb_switch(&self.board, false).is_err() {
                pr_warn!("LNB supply may still be on\n");
            }
            if let Some(addr) = self.scrambler {
                bringup::shutdown_control_chip(&self.board, addr);
            }
            bringup::power_off(&self.board);
        }
    }
}

#[pinned_drop]
impl PinnedDrop for Pxw3pe {
    fn drop(self: Pin<&mut Self>) {
        // The reboot hook stays registered until the hardware is quiet: unregistering the adapters
        // waits for every demux and dvr file to close, and a reboot forced meanwhile must still
        // switch the LNB off. What the callback does is locked and may be repeated, so it may
        // run alongside the steps below.
        //
        // Before anything is powered down, and so that a card operation in progress ends.
        self.nodes.card.shutdown();
        // Waits until the adapters' demux and dvr files are closed; their streams stop then.
        self.adapters.lock().clear();
        self.lnb.off();
        // No adapter can start a stream any more, and the interrupt thread cannot re-arm a port
        // after this.
        self.quiesce_stream();
        // Waits for a callback that runs; none starts afterwards, so none sees the power-down
        // below or the fields go.
        //
        // SAFETY: The only call, in remove, after probe's registration has returned and not from
        // the callback.
        unsafe { self.reboot.unregister() };
        // The DMA buffers are freed after this, with the last reference to the stream; the
        // engine must not write into them any more, which cleared bus mastering ensures even
        // for a channel that did not stop above.
        self.enable.clear_master();
        bringup::shutdown_control_chip(&self.board, self.scrambler);
        bringup::power_off(&self.board);
    }
}

kernel::module_pci_driver! {
    type: Pxw3pe,
    name: "pxw3pe",
    authors: ["SoraneOumi"],
    description: "PX-W3PE V2.0 ISDB-S/T tuner",
    license: "GPL",
}
