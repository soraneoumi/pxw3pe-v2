/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interface between the Rust driver and its character devices (the card reader and the LNB
 * supply status).
 *
 * The Rust miscdevice abstraction builds its file_operations without an owner, so an open file
 * would not keep the module loaded. This small C layer registers a device with the owner set and
 * forwards the file operations the driver implements through struct pxw3pe_chrdev_ops. It also
 * runs a device's periodic sampling, if it has one, since the Rust workqueue abstraction cannot
 * cancel a delayed work. The Rust side mirrors these declarations in chrdev.rs; any change here
 * must be made there too.
 */
#ifndef PXW3PE_CHRDEV_GLUE_H
#define PXW3PE_CHRDEV_GLUE_H

#include <linux/types.h>

struct mutex;
struct pxw3pe_chrdev;

/*
 * Callbacks into the driver. All run in process context and may sleep.
 *
 * open and poll get the context given at registration and run only while the device is
 * registered: pxw3pe_chrdev_unregister() waits for those still running, and none starts after it
 * has returned. release and ioctl get what open returned and may run after unregistration, for
 * files that are still open; the module stays loaded until the last of them is closed. The
 * table therefore has to live as long as the module.
 */
struct pxw3pe_chrdev_ops {
	/* Returns the context of the new file, or an ERR_PTR. */
	void *(*open)(void *ctx);
	/* Drops what open returned. */
	void (*release)(void *file);
	/* Returns 0 or a negative errno. The argument is a user pointer. */
	long (*ioctl)(void *file, unsigned int cmd, unsigned long arg);
	/* Called every 100 ms; NULL for a device without periodic work. */
	void (*poll)(void *ctx);
};

/*
 * Registers the device node /dev/<name>; name must stay valid until unregistration. Returns an
 * ERR_PTR on failure.
 */
struct pxw3pe_chrdev *pxw3pe_chrdev_register(const char *name,
					     const struct pxw3pe_chrdev_ops *ops, void *ctx);
void pxw3pe_chrdev_unregister(struct pxw3pe_chrdev *dev);

/* mutex_lock_killable(): 0 with the lock held, -EINTR if a fatal signal arrived first. */
int pxw3pe_mutex_lock_killable(struct mutex *lock);

/* Whether the current task has been killed. */
bool pxw3pe_fatal_signal_pending(void);

/* capable(CAP_SYS_RAWIO) for the current task. */
bool pxw3pe_capable_rawio(void);

#endif
