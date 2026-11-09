// SPDX-License-Identifier: GPL-2.0
/*
 * Character device glue for the Rust pxw3pe driver; see chrdev_glue.h.
 *
 * The file_operations carry the module as their owner. misc_open() takes the module reference
 * through fops_get() and the last fput() drops it after release(), so rmmod fails while a file is
 * open instead of unloading code that the file would still run.
 *
 * Unregistration. misc_open() calls ->open with misc_mtx held and misc_deregister() takes that
 * mutex, so once misc_deregister() has returned no open is in progress or can start. The poll
 * work is disabled afterwards; a disabled work cannot be queued, not even by its own handler, so
 * after pxw3pe_chrdev_unregister() no callback that uses the registration context runs and the
 * driver's context may go. Files that are still open do not depend on the registration: each
 * carries the ops table and what open returned.
 */

#include <linux/build_bug.h>
#include <linux/capability.h>
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/workqueue.h>

#include "chrdev_glue.h"

/*
 * The layout of the interface structure, which chrdev.rs declares by hand. chrdev.rs asserts the
 * same layout, so a structure changed on one side only fails the build; change both together.
 */
#define PXW3PE_PTR sizeof(void *)
static_assert(sizeof(struct pxw3pe_chrdev_ops) == 4 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_chrdev_ops, release) == 1 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_chrdev_ops, ioctl) == 2 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_chrdev_ops, poll) == 3 * PXW3PE_PTR);
#undef PXW3PE_PTR

/*
 * Sampling period. The card reader needs two consecutive card-detect samples to agree.
 */
#define PXW3PE_CHRDEV_POLL_JIFFIES msecs_to_jiffies(100)

struct pxw3pe_chrdev {
	struct miscdevice misc;
	struct delayed_work poll;
	const struct pxw3pe_chrdev_ops *ops;
	void *ctx;
};

/* private_data of an open file. */
struct pxw3pe_chrdev_file {
	const struct pxw3pe_chrdev_ops *ops;
	/* What ops->open returned. */
	void *priv;
};

static int pxw3pe_chrdev_open(struct inode *inode, struct file *file)
{
	/* misc_open() stored the miscdevice in private_data. */
	struct pxw3pe_chrdev *dev = container_of(file->private_data, struct pxw3pe_chrdev, misc);
	struct pxw3pe_chrdev_file *cf;
	void *priv;

	cf = kmalloc(sizeof(*cf), GFP_KERNEL_ACCOUNT);
	if (!cf)
		return -ENOMEM;
	priv = dev->ops->open(dev->ctx);
	if (IS_ERR(priv)) {
		kfree(cf);
		return PTR_ERR(priv);
	}
	cf->ops = dev->ops;
	cf->priv = priv;
	file->private_data = cf;
	return 0;
}

static int pxw3pe_chrdev_release(struct inode *inode, struct file *file)
{
	struct pxw3pe_chrdev_file *cf = file->private_data;

	cf->ops->release(cf->priv);
	kfree(cf);
	return 0;
}

static long pxw3pe_chrdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pxw3pe_chrdev_file *cf = file->private_data;

	return cf->ops->ioctl(cf->priv, cmd, arg);
}

static const struct file_operations pxw3pe_chrdev_fops = {
	.owner = THIS_MODULE,
	.open = pxw3pe_chrdev_open,
	.release = pxw3pe_chrdev_release,
	.unlocked_ioctl = pxw3pe_chrdev_ioctl,
	/* Every argument is a pointer to a structure of fixed layout. */
	.compat_ioctl = compat_ptr_ioctl,
};

/* Queued only for a device that has a poll callback. */
static void pxw3pe_chrdev_poll(struct work_struct *work)
{
	struct pxw3pe_chrdev *dev = container_of(to_delayed_work(work), struct pxw3pe_chrdev,
						 poll);

	dev->ops->poll(dev->ctx);
	/* Refused once pxw3pe_chrdev_unregister() has disabled the work. */
	queue_delayed_work(system_dfl_wq, &dev->poll, PXW3PE_CHRDEV_POLL_JIFFIES);
}

struct pxw3pe_chrdev *pxw3pe_chrdev_register(const char *name,
					     const struct pxw3pe_chrdev_ops *ops, void *ctx)
{
	struct pxw3pe_chrdev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);
	dev->ops = ops;
	dev->ctx = ctx;
	dev->misc.minor = MISC_DYNAMIC_MINOR;
	dev->misc.name = name;
	dev->misc.fops = &pxw3pe_chrdev_fops;
	/* Initialised either way: unregistration disables it whether it was ever queued or not. */
	INIT_DELAYED_WORK(&dev->poll, pxw3pe_chrdev_poll);

	ret = misc_register(&dev->misc);
	if (ret) {
		kfree(dev);
		return ERR_PTR(ret);
	}
	if (ops->poll)
		queue_delayed_work(system_dfl_wq, &dev->poll, PXW3PE_CHRDEV_POLL_JIFFIES);
	return dev;
}

void pxw3pe_chrdev_unregister(struct pxw3pe_chrdev *dev)
{
	misc_deregister(&dev->misc);
	disable_delayed_work_sync(&dev->poll);
	kfree(dev);
}

int pxw3pe_mutex_lock_killable(struct mutex *lock)
{
	return mutex_lock_killable(lock);
}

bool pxw3pe_fatal_signal_pending(void)
{
	return fatal_signal_pending(current);
}

bool pxw3pe_capable_rawio(void)
{
	return capable(CAP_SYS_RAWIO);
}
