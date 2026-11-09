/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernel facilities of the PCI device and of system shutdown that the Rust abstractions of the
 * targeted kernels do not provide. The Rust side mirrors these declarations in platform.rs; any
 * change here must be made there too.
 */
#ifndef PXW3PE_BOARD_GLUE_H
#define PXW3PE_BOARD_GLUE_H

struct device;
struct pxw3pe_irqlock;
struct pxw3pe_latency;
struct pxw3pe_poller;
struct pxw3pe_reboot;

/* Both take the device embedded in the struct pci_dev that the driver is bound to. */

/* Stops the device from mastering the bus. */
void pxw3pe_pci_clear_master(struct device *dev);
/*
 * Undoes one successful pci_enable_device_mem(). Must come after the device's interrupt has been
 * freed: on x86 this releases the legacy interrupt routing of the device.
 */
void pxw3pe_pci_disable_device(struct device *dev);

/*
 * Full memory barrier, ordering earlier CPU reads of coherent DMA memory before later MMIO
 * writes.
 */
void pxw3pe_mb(void);

/*
 * A spinlock that may be taken in hard interrupt context: taken with interrupts disabled, and
 * from the handler. The Rust spinlock of the targeted kernels cannot disable interrupts.
 */

/* Returns NULL on allocation failure. */
struct pxw3pe_irqlock *pxw3pe_irqlock_new(void);
void pxw3pe_irqlock_free(struct pxw3pe_irqlock *lock);
/* spin_lock_irqsave(); returns the flags for the unlock. */
unsigned long pxw3pe_irqlock_lock(struct pxw3pe_irqlock *lock);
void pxw3pe_irqlock_unlock(struct pxw3pe_irqlock *lock, unsigned long flags);

/*
 * Handle of a reboot notifier, which runs before the system restarts, halts or powers off.
 * Allocation and registration are separate so that the handle can be a member of the state the
 * callback works on.
 */

/* Returns NULL on allocation failure. */
struct pxw3pe_reboot *pxw3pe_reboot_new(void);
/*
 * Starts calling prepare(ctx) in process context, where it may sleep. ctx must stay valid until
 * pxw3pe_reboot_unregister() or pxw3pe_reboot_free() has returned.
 */
int pxw3pe_reboot_register(struct pxw3pe_reboot *reboot, void (*prepare)(void *ctx), void *ctx);
/*
 * Waits for a running prepare() to finish; none runs or starts after this returns. Does nothing
 * if the handle is not registered. Must not be called from prepare().
 */
void pxw3pe_reboot_unregister(struct pxw3pe_reboot *reboot);
/* Unregisters the handle if needed and frees it. */
void pxw3pe_reboot_free(struct pxw3pe_reboot *reboot);

/*
 * A periodic call of fn(ctx) from a workqueue, in process context, where it may sleep. Allocation
 * and start are separate so that the handle can be a member of the state fn works on.
 */

/* Not started; returns NULL on allocation failure. */
struct pxw3pe_poller *pxw3pe_poller_new(void (*fn)(void *ctx), void *ctx, unsigned int period_ms);
/*
 * Starts calling fn(ctx) repeatedly: the first call is period_ms from now, each later one
 * period_ms after the previous call has returned.
 */
void pxw3pe_poller_start(struct pxw3pe_poller *poller);
/*
 * Stops and frees. When it returns, fn is not running and will not run again, and ctx is no
 * longer used. Must not be called from fn.
 */
void pxw3pe_poller_free(struct pxw3pe_poller *poller);

/*
 * A limit on how long the CPUs may take to leave their idle states (the kernel's CPU latency
 * QoS), which keeps the memory path they share awake too. Allocation and hold are separate so
 * that the handle can be a member of the state of the DMA port it protects.
 */

/* Not held; returns NULL on allocation failure. */
struct pxw3pe_latency *pxw3pe_latency_new(void);
/* Limits the idle exit latency of every CPU to us microseconds. Must not be held already. */
void pxw3pe_latency_hold(struct pxw3pe_latency *latency, int us);
/* Lifts the limit. Does nothing if it is not held. */
void pxw3pe_latency_release(struct pxw3pe_latency *latency);
/* Lifts the limit if held and frees. */
void pxw3pe_latency_free(struct pxw3pe_latency *latency);

#endif
