/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interface between the Rust driver and the C DVB core.
 *
 * The DVB core has no Rust abstractions, so this small C layer owns every DVB structure
 * (adapter, demux, dmxdev, frontend) and forwards the callbacks the driver has to implement
 * through struct pxw3pe_dvb_ops. The Rust side mirrors these declarations in dvb.rs; any change
 * here must be made there too.
 */
#ifndef PXW3PE_DVB_GLUE_H
#define PXW3PE_DVB_GLUE_H

#include <linux/types.h>

struct device;
struct module;
struct pxw3pe_dvb;

/* Tuning request, taken from the DVB property cache. */
struct pxw3pe_tune {
	/* Hz for ISDB-T, kHz for ISDB-S (the DVB API convention for satellite systems). */
	u32 frequency;
	/* DTV_STREAM_ID; NO_STREAM_ID_FILTER when user space did not set one. */
	u32 stream_id;
};

/* Signal report filled in by the driver. */
struct pxw3pe_signal {
	/* enum fe_status bits. */
	u32 status;
	/* Carrier to noise ratio in 0.001 dB; only meaningful when cnr_valid is set. */
	s32 cnr_mdb;
	u8 cnr_valid;
};

/*
 * Callbacks into the driver. All run in process context and may sleep; the DVB core serialises
 * the frontend callbacks of one adapter, and start_feed/stop_feed run under the demux mutex.
 * None is called after pxw3pe_dvb_unregister() has returned.
 */
struct pxw3pe_dvb_ops {
	int (*init)(void *ctx);
	int (*sleep)(void *ctx);
	int (*tune)(void *ctx, const struct pxw3pe_tune *tune);
	int (*read_signal)(void *ctx, struct pxw3pe_signal *signal);
	/* Satellite adapters only: switch the LNB supply. */
	int (*set_lnb)(void *ctx, bool on);
	/*
	 * Called when the first feed of the adapter starts and when the last one stops. dvb is the
	 * adapter the feed belongs to, the handle for pxw3pe_dvb_feed(); it stays valid until the
	 * matching stop_streaming() has returned.
	 */
	int (*start_streaming)(void *ctx, struct pxw3pe_dvb *dvb);
	void (*stop_streaming)(void *ctx);
};

/* Static description of one adapter. */
struct pxw3pe_dvb_info {
	const char *name;
	bool satellite;
	u32 frequency_min_hz;
	u32 frequency_max_hz;
	u32 frequency_stepsize_hz;
};

/* Returns an ERR_PTR on failure. */
struct pxw3pe_dvb *pxw3pe_dvb_register(struct device *dev, struct module *owner,
				       const struct pxw3pe_dvb_info *info,
				       const struct pxw3pe_dvb_ops *ops, void *ctx);
void pxw3pe_dvb_unregister(struct pxw3pe_dvb *dvb);

/* Hands complete 188-byte packets to the demux. Callable from any non-hardirq context. */
void pxw3pe_dvb_feed(struct pxw3pe_dvb *dvb, const u8 *packets, size_t count);

/* Number of the registered DVB adapter, for log messages. */
int pxw3pe_dvb_adapter_num(const struct pxw3pe_dvb *dvb);

#endif
