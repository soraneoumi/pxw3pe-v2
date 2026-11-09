// SPDX-License-Identifier: GPL-2.0
/*
 * DVB core glue for the Rust pxw3pe driver; see dvb_glue.h.
 *
 * Lifetimes. The DVB core keeps the frontend alive while one of its files is still open after
 * unregistration, and on the last close it still dereferences fe->dvb, the adapter. So the
 * adapter, demux, dmxdev and frontend live in one allocation, which is freed only by the
 * frontend's release callback, run when the last frontend reference goes away.
 *
 * A call into a frontend file that is already in progress, or one that slips in while the
 * frontend is being torn down, may still reach our callbacks after dvb_unregister_frontend().
 * The callbacks therefore run under cb_lock and refuse to call the driver once "removed" is set;
 * pxw3pe_dvb_unregister() sets it under the write lock as its last step, which also waits for
 * callbacks still running. After that, no driver callback runs and the driver's context may go.
 */

#include <linux/build_bug.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>
#include <media/dvbdev.h>

#include "dvb_glue.h"

/*
 * The layout of the interface structures, which dvb.rs declares by hand. dvb.rs asserts the same
 * layout, so a structure changed on one side only fails the build; change both together.
 */
#define PXW3PE_PTR sizeof(void *)
static_assert(sizeof(struct pxw3pe_tune) == 8);
static_assert(offsetof(struct pxw3pe_tune, stream_id) == 4);
static_assert(sizeof(struct pxw3pe_signal) == 12);
static_assert(offsetof(struct pxw3pe_signal, cnr_mdb) == 4);
static_assert(offsetof(struct pxw3pe_signal, cnr_valid) == 8);
static_assert(sizeof(struct pxw3pe_dvb_ops) == 7 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, sleep) == 1 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, tune) == 2 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, read_signal) == 3 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, set_lnb) == 4 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, start_streaming) == 5 * PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_ops, stop_streaming) == 6 * PXW3PE_PTR);
static_assert(sizeof(struct pxw3pe_dvb_info) == PXW3PE_PTR + 16);
static_assert(offsetof(struct pxw3pe_dvb_info, satellite) == PXW3PE_PTR);
static_assert(offsetof(struct pxw3pe_dvb_info, frequency_min_hz) == PXW3PE_PTR + 4);
static_assert(offsetof(struct pxw3pe_dvb_info, frequency_max_hz) == PXW3PE_PTR + 8);
static_assert(offsetof(struct pxw3pe_dvb_info, frequency_stepsize_hz) == PXW3PE_PTR + 12);
#undef PXW3PE_PTR

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

/* The bridge has no PID filter, so all filtering happens in this software demux. */
#define PXW3PE_FILTERS 256

/* Interval at which the frontend thread polls the signal state. */
#define PXW3PE_POLL_JIFFIES (HZ / 5)

struct pxw3pe_dvb {
	struct dvb_frontend fe;
	struct dvb_adapter adapter;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	const struct pxw3pe_dvb_ops *ops;
	void *ctx;
	/* Taken for reading around every driver callback; see the comment at the top. */
	struct rw_semaphore cb_lock;
	bool removed;
	/* Running feeds; protected by demux.mutex, under which the core calls the feed hooks. */
	unsigned int feeds;
};

static struct pxw3pe_dvb *fe_to_dvb(struct dvb_frontend *fe)
{
	return container_of(fe, struct pxw3pe_dvb, fe);
}

/* Enters a driver callback; false once the driver has gone. */
static bool cb_enter(struct pxw3pe_dvb *dvb)
{
	down_read(&dvb->cb_lock);
	if (dvb->removed) {
		up_read(&dvb->cb_lock);
		return false;
	}
	return true;
}

static void cb_exit(struct pxw3pe_dvb *dvb)
{
	up_read(&dvb->cb_lock);
}

static int pxw3pe_fe_init(struct dvb_frontend *fe)
{
	struct pxw3pe_dvb *dvb = fe_to_dvb(fe);
	int ret;

	if (!cb_enter(dvb))
		return -ENODEV;
	ret = dvb->ops->init(dvb->ctx);
	cb_exit(dvb);
	return ret;
}

static int pxw3pe_fe_sleep(struct dvb_frontend *fe)
{
	struct pxw3pe_dvb *dvb = fe_to_dvb(fe);
	int ret;

	if (!cb_enter(dvb))
		return -ENODEV;
	ret = dvb->ops->sleep(dvb->ctx);
	cb_exit(dvb);
	return ret;
}

/*
 * Fills *status and the C/N statistic. On failure the status is cleared rather than left at its
 * previous value, so a failing bus never keeps reporting an old lock.
 */
static int read_status_locked(struct pxw3pe_dvb *dvb, enum fe_status *status)
{
	struct dtv_frontend_properties *c = &dvb->fe.dtv_property_cache;
	struct pxw3pe_signal signal = {};
	int ret;

	ret = dvb->ops->read_signal(dvb->ctx, &signal);
	c->cnr.len = 1;
	if (ret) {
		*status = 0;
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return ret;
	}
	*status = signal.status;
	if (signal.cnr_valid) {
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = signal.cnr_mdb;
	} else {
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
	return 0;
}

static int pxw3pe_fe_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct pxw3pe_dvb *dvb = fe_to_dvb(fe);
	int ret;

	if (!cb_enter(dvb)) {
		*status = 0;
		return -ENODEV;
	}
	ret = read_status_locked(dvb, status);
	cb_exit(dvb);
	return ret;
}

static int pxw3pe_fe_tune(struct dvb_frontend *fe, bool re_tune, unsigned int mode_flags,
			  unsigned int *delay, enum fe_status *status)
{
	struct pxw3pe_dvb *dvb = fe_to_dvb(fe);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	*delay = PXW3PE_POLL_JIFFIES;
	if (!cb_enter(dvb)) {
		*status = 0;
		return -ENODEV;
	}
	if (re_tune) {
		struct pxw3pe_tune tune = {
			.frequency = c->frequency,
			.stream_id = c->stream_id,
		};

		ret = dvb->ops->tune(dvb->ctx, &tune);
	}
	/*
	 * The core does not retry a failed tune and ignores the error; the driver remembers the
	 * failure and reports no lock on later polls until the next tune.
	 */
	if (ret)
		*status = 0;
	else
		ret = read_status_locked(dvb, status);
	cb_exit(dvb);
	return ret;
}

static enum dvbfe_algo pxw3pe_fe_get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int pxw3pe_fe_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct pxw3pe_dvb *dvb = fe_to_dvb(fe);
	bool on;
	int ret;

	switch (voltage) {
	case SEC_VOLTAGE_OFF:
		on = false;
		break;
	/* The board has a single fixed 15 V supply; both requests switch it on. */
	case SEC_VOLTAGE_13:
	case SEC_VOLTAGE_18:
		on = true;
		break;
	default:
		return -EINVAL;
	}
	if (!cb_enter(dvb))
		return -ENODEV;
	ret = dvb->ops->set_lnb(dvb->ctx, on);
	cb_exit(dvb);
	return ret;
}

/* Last frontend reference gone: nothing refers to the adapter any more. */
static void pxw3pe_fe_release(struct dvb_frontend *fe)
{
	kfree(fe_to_dvb(fe));
}

static const struct dvb_frontend_ops pxw3pe_fe_ops_isdbs = {
	.delsys = { SYS_ISDBS },
	.info = {
		.caps = FE_CAN_INVERSION_AUTO | FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_MULTISTREAM,
	},
	.init = pxw3pe_fe_init,
	.sleep = pxw3pe_fe_sleep,
	.tune = pxw3pe_fe_tune,
	.get_frontend_algo = pxw3pe_fe_get_algo,
	.read_status = pxw3pe_fe_read_status,
	.set_voltage = pxw3pe_fe_set_voltage,
	.release = pxw3pe_fe_release,
};

static const struct dvb_frontend_ops pxw3pe_fe_ops_isdbt = {
	.delsys = { SYS_ISDBT },
	.info = {
		.caps = FE_CAN_INVERSION_AUTO | FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO,
	},
	.init = pxw3pe_fe_init,
	.sleep = pxw3pe_fe_sleep,
	.tune = pxw3pe_fe_tune,
	.get_frontend_algo = pxw3pe_fe_get_algo,
	.read_status = pxw3pe_fe_read_status,
	.release = pxw3pe_fe_release,
};

/*
 * The feed hooks need no cb_lock: the core only calls them while demux or dvr files are open,
 * and pxw3pe_dvb_unregister() waits for those to close before it sets "removed".
 */
static int pxw3pe_start_feed(struct dvb_demux_feed *feed)
{
	struct pxw3pe_dvb *dvb = feed->demux->priv;
	int ret;

	if (dvb->feeds++)
		return 0;
	ret = dvb->ops->start_streaming(dvb->ctx, dvb);
	if (ret)
		dvb->feeds--;
	return ret;
}

static int pxw3pe_stop_feed(struct dvb_demux_feed *feed)
{
	struct pxw3pe_dvb *dvb = feed->demux->priv;

	/*
	 * Only a feed whose start succeeded was counted, and the core sets GO once start_feed has
	 * succeeded (dmx_ts_feed_start_filtering(), dmx_section_feed_start_filtering()).
	 * dmx_section_feed_stop_filtering() calls stop_feed without checking the state, so it also
	 * reaches us for a section feed whose start failed (dvb_dmxdev_feed_restart() ignores start
	 * errors) and for one that is stopped already; counting those would underflow "feeds".
	 * dmx_ts_feed_stop_filtering() rejects feeds that are not GO itself.
	 */
	if (feed->state != DMX_STATE_GO)
		return 0;
	if (!--dvb->feeds)
		dvb->ops->stop_streaming(dvb->ctx);
	return 0;
}

/*
 * Drops the registration's frontend reference. With CONFIG_MEDIA_ATTACH the core puts the module
 * that holds ops.release after calling it, which balances the symbol_get() of dvb_attach().
 * pxw3pe_fe_release() was not attached that way, so the reference that put consumes is taken
 * here. While a frontend file keeps the release pending, that reference also keeps this module,
 * and with it the release code, loaded.
 */
static void detach_frontend(struct pxw3pe_dvb *dvb)
{
	if (IS_ENABLED(CONFIG_MEDIA_ATTACH))
		__module_get(THIS_MODULE);
	dvb_frontend_detach(&dvb->fe);
}

struct pxw3pe_dvb *pxw3pe_dvb_register(struct device *dev, struct module *owner,
				       const struct pxw3pe_dvb_info *info,
				       const struct pxw3pe_dvb_ops *ops, void *ctx)
{
	struct pxw3pe_dvb *dvb;
	int ret;

	dvb = kzalloc(sizeof(*dvb), GFP_KERNEL);
	if (!dvb)
		return ERR_PTR(-ENOMEM);
	dvb->ops = ops;
	dvb->ctx = ctx;
	init_rwsem(&dvb->cb_lock);

	ret = dvb_register_adapter(&dvb->adapter, info->name, owner, dev, adapter_nr);
	if (ret < 0)
		goto free_dvb;

	dvb->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING;
	dvb->demux.priv = dvb;
	dvb->demux.filternum = PXW3PE_FILTERS;
	dvb->demux.feednum = PXW3PE_FILTERS;
	dvb->demux.start_feed = pxw3pe_start_feed;
	dvb->demux.stop_feed = pxw3pe_stop_feed;
	ret = dvb_dmx_init(&dvb->demux);
	if (ret)
		goto unregister_adapter;

	dvb->dmxdev.filternum = PXW3PE_FILTERS;
	dvb->dmxdev.demux = &dvb->demux.dmx;
	ret = dvb_dmxdev_init(&dvb->dmxdev, &dvb->adapter);
	if (ret)
		goto release_dmx;

	dvb->fe.ops = info->satellite ? pxw3pe_fe_ops_isdbs : pxw3pe_fe_ops_isdbt;
	strscpy(dvb->fe.ops.info.name, info->name, sizeof(dvb->fe.ops.info.name));
	dvb->fe.ops.info.frequency_min_hz = info->frequency_min_hz;
	dvb->fe.ops.info.frequency_max_hz = info->frequency_max_hz;
	dvb->fe.ops.info.frequency_stepsize_hz = info->frequency_stepsize_hz;
	ret = dvb_register_frontend(&dvb->adapter, &dvb->fe);
	if (ret)
		goto release_dmxdev;
	return dvb;

release_dmxdev:
	dvb_dmxdev_release(&dvb->dmxdev);
release_dmx:
	dvb_dmx_release(&dvb->demux);
unregister_adapter:
	dvb_unregister_adapter(&dvb->adapter);
free_dvb:
	/*
	 * A failed dvb_register_frontend() may have set up the frontend's private data and left one
	 * reference; dropping it frees that data and, through ops.release, the whole allocation.
	 */
	if (dvb->fe.frontend_priv)
		detach_frontend(dvb);
	else
		kfree(dvb);
	return ERR_PTR(ret);
}

void pxw3pe_dvb_unregister(struct pxw3pe_dvb *dvb)
{
	dvb_unregister_frontend(&dvb->fe);
	/* Waits until the demux and dvr files are closed; their feeds stop on the way. */
	dvb_dmxdev_release(&dvb->dmxdev);
	dvb_dmx_release(&dvb->demux);
	dvb_unregister_adapter(&dvb->adapter);
	down_write(&dvb->cb_lock);
	dvb->removed = true;
	up_write(&dvb->cb_lock);
	/* The allocation goes with the last frontend user. */
	detach_frontend(dvb);
}

void pxw3pe_dvb_feed(struct pxw3pe_dvb *dvb, const u8 *packets, size_t count)
{
	dvb_dmx_swfilter_packets(&dvb->demux, packets, count);
}

int pxw3pe_dvb_adapter_num(const struct pxw3pe_dvb *dvb)
{
	return dvb->adapter.num;
}
