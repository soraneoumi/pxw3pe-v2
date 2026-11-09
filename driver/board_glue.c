// SPDX-License-Identifier: GPL-2.0
/* PCI device and shutdown helpers for the Rust pxw3pe driver; see board_glue.h. */

#include <asm/barrier.h>
#include <linux/bug.h>
#include <linux/container_of.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "board_glue.h"

struct pxw3pe_irqlock {
	/* Whatever the Rust side runs under pxw3pe_irqlock_lock(); see board_glue.h. */
	spinlock_t lock;
};

struct pxw3pe_reboot {
	struct notifier_block nb;
	void (*prepare)(void *ctx);
	void *ctx;
	/* Only touched by register and unregister, which probe and remove call one at a time. */
	bool registered;
};

struct pxw3pe_latency {
	struct pm_qos_request req;
};

struct pxw3pe_poller {
	struct delayed_work work;
	void (*fn)(void *ctx);
	void *ctx;
	/* In jiffies. */
	unsigned long period;
};

void pxw3pe_pci_clear_master(struct device *dev)
{
	pci_clear_master(to_pci_dev(dev));
}

void pxw3pe_pci_disable_device(struct device *dev)
{
	pci_disable_device(to_pci_dev(dev));
}

void pxw3pe_mb(void)
{
	/* No pairing barrier: the other party is the device, which refills the buffer. */
	mb();
}

struct pxw3pe_irqlock *pxw3pe_irqlock_new(void)
{
	struct pxw3pe_irqlock *lock = kmalloc(sizeof(*lock), GFP_KERNEL);

	if (lock)
		spin_lock_init(&lock->lock);
	return lock;
}

void pxw3pe_irqlock_free(struct pxw3pe_irqlock *lock)
{
	kfree(lock);
}

unsigned long pxw3pe_irqlock_lock(struct pxw3pe_irqlock *lock)
{
	unsigned long flags;

	spin_lock_irqsave(&lock->lock, flags);
	return flags;
}

void pxw3pe_irqlock_unlock(struct pxw3pe_irqlock *lock, unsigned long flags)
{
	spin_unlock_irqrestore(&lock->lock, flags);
}

/*
 * The reboot notifier chain is blocking: blocking_notifier_call_chain() (kernel/notifier.c:368)
 * runs the callbacks in process context under the chain's read lock, from kernel_restart_prepare()
 * and kernel_shutdown_prepare() (kernel/reboot.c:101, 303) before device_shutdown().
 * Unregistering (kernel/notifier.c:313) takes the write lock and so waits for a running callback.
 */
static int pxw3pe_reboot_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct pxw3pe_reboot *reboot = container_of(nb, struct pxw3pe_reboot, nb);

	reboot->prepare(reboot->ctx);
	return NOTIFY_DONE;
}

struct pxw3pe_reboot *pxw3pe_reboot_new(void)
{
	struct pxw3pe_reboot *reboot = kzalloc(sizeof(*reboot), GFP_KERNEL);

	if (reboot)
		reboot->nb.notifier_call = pxw3pe_reboot_call;
	return reboot;
}

int pxw3pe_reboot_register(struct pxw3pe_reboot *reboot, void (*prepare)(void *ctx), void *ctx)
{
	int ret;

	reboot->prepare = prepare;
	reboot->ctx = ctx;
	ret = register_reboot_notifier(&reboot->nb);
	if (!ret)
		reboot->registered = true;
	return ret;
}

void pxw3pe_reboot_unregister(struct pxw3pe_reboot *reboot)
{
	if (!reboot->registered)
		return;
	WARN_ON(unregister_reboot_notifier(&reboot->nb));
	reboot->registered = false;
}

void pxw3pe_reboot_free(struct pxw3pe_reboot *reboot)
{
	pxw3pe_reboot_unregister(reboot);
	kfree(reboot);
}

static void pxw3pe_poller_work(struct work_struct *work)
{
	struct pxw3pe_poller *poller = container_of(to_delayed_work(work), struct pxw3pe_poller,
						    work);

	poller->fn(poller->ctx);
	/*
	 * Refused once pxw3pe_poller_free() has disabled the work: queue_delayed_work_on() fails
	 * in clear_pending_if_disabled() (kernel/workqueue.c:2388) for a work whose disable count
	 * is set, also while its handler is the caller.
	 */
	queue_delayed_work(system_dfl_wq, &poller->work, poller->period);
}

struct pxw3pe_poller *pxw3pe_poller_new(void (*fn)(void *ctx), void *ctx, unsigned int period_ms)
{
	struct pxw3pe_poller *poller = kzalloc(sizeof(*poller), GFP_KERNEL);

	if (!poller)
		return NULL;
	poller->fn = fn;
	poller->ctx = ctx;
	poller->period = msecs_to_jiffies(period_ms);
	INIT_DELAYED_WORK(&poller->work, pxw3pe_poller_work);
	return poller;
}

void pxw3pe_poller_start(struct pxw3pe_poller *poller)
{
	queue_delayed_work(system_dfl_wq, &poller->work, poller->period);
}

/*
 * disable_delayed_work_sync() (kernel/workqueue.c:4620) cancels a pending run, waits for one that
 * is executing and leaves the work disabled, so the handler's own queue_delayed_work() fails and
 * nothing runs after it returns.
 */
void pxw3pe_poller_free(struct pxw3pe_poller *poller)
{
	disable_delayed_work_sync(&poller->work);
	kfree(poller);
}

struct pxw3pe_latency *pxw3pe_latency_new(void)
{
	return kzalloc(sizeof(struct pxw3pe_latency), GFP_KERNEL);
}

void pxw3pe_latency_hold(struct pxw3pe_latency *latency, int us)
{
	cpu_latency_qos_add_request(&latency->req, us);
}

void pxw3pe_latency_release(struct pxw3pe_latency *latency)
{
	if (cpu_latency_qos_request_active(&latency->req))
		cpu_latency_qos_remove_request(&latency->req);
}

void pxw3pe_latency_free(struct pxw3pe_latency *latency)
{
	pxw3pe_latency_release(latency);
	kfree(latency);
}
