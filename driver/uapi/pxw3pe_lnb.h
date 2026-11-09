/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * User-space interface of the PX-W3PE LNB supply.
 *
 * Each board appears as /dev/pxw3pe-lnbN, with the same N as its card reader /dev/pxw3pe-cardN.
 * The supply delivers 15 V to the antenna connector. It is switched only through the two ISDB-S
 * DVB frontends: FE_SET_VOLTAGE with SEC_VOLTAGE_13 or SEC_VOLTAGE_18 requests it, and
 * SEC_VOLTAGE_OFF or closing the frontend withdraws the request. It is on while either frontend
 * requests it. This device reports the state of the supply, and gives raw access to the
 * board's hardware for testing (the last three calls below).
 *
 * Errors common to all calls: ENODEV once the driver has switched the supply off for good (the
 * board is being removed or the system is restarting; an open file stays valid and keeps
 * failing this way), ENOTTY for an unknown request, EFAULT for a bad user pointer.
 *
 * The Rust driver mirrors these definitions in lnb.rs; any change here must be made there too.
 */
#ifndef _UAPI_PXW3PE_LNB_H
#define _UAPI_PXW3PE_LNB_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* The supply is switched on. */
#define PXW3PE_LNB_ON (1u << 0)
/*
 * The last switch failed on the I2C bus, so the supply may be on or off; ON is clear. The driver
 * repeats the switch about every 100 ms until it succeeds.
 */
#define PXW3PE_LNB_UNKNOWN (1u << 1)
/*
 * The supply reports power good, read by this call. Only with ON: the bit means nothing while
 * the supply is off. ON without POWER_GOOD is not yet a short: the driver judges the bit only
 * from 200 ms after switch-on, and only two bad checks in a row make a FAULT.
 */
#define PXW3PE_LNB_POWER_GOOD (1u << 2)
/*
 * The supply reported no power good in two consecutive checks (a short), and the driver switched
 * it off. The driver checks every 100 ms from 200 ms after switch-on; a read that fails on the
 * I2C bus is no check. The supply stays off, and FE_SET_VOLTAGE requests fail with EIO, until
 * both frontends have withdrawn their requests. A request refused this way still counts as one
 * in REQUEST_S0 and REQUEST_S1, and has to be withdrawn like any other.
 */
#define PXW3PE_LNB_FAULT (1u << 3)
/* The frontend of ISDB-S tuner 0 ("PX-W3PE V2 ISDB-S 0") requests the supply. */
#define PXW3PE_LNB_REQUEST_S0 (1u << 4)
/* The frontend of ISDB-S tuner 1 ("PX-W3PE V2 ISDB-S 1") requests the supply. */
#define PXW3PE_LNB_REQUEST_S1 (1u << 5)

struct pxw3pe_lnb_status {
	__u32 flags;
};

#define PXW3PE_LNB_IOC 0xBA

/*
 * Reports the state of the supply and which frontends request it. Errors: ENODEV; EIO when
 * reading the power-good bit failed on the I2C bus.
 */
#define PXW3PE_LNB_GET_STATUS _IOR(PXW3PE_LNB_IOC, 0x10, struct pxw3pe_lnb_status)

/*
 * Raw hardware access for testing. Each call needs CAP_SYS_RAWIO (EPERM otherwise). A wrong
 * value can stop the streams, switch chips off or make the bridge's DMA write anywhere in memory.
 * The calls take the I2C bus as the driver does, so a gateway access is never split, but they
 * can come between two accesses of a sequence the driver is running, and the driver's view of
 * the chips is not updated by them. Errors: EPERM, ENODEV, EINVAL for a value out of range or a
 * write field other than 0 and 1.
 *
 * The I2C calls report a transaction the bus refused in status, as a positive errno (EIO: no
 * acknowledge, ETIMEDOUT: no completion), and return success: probing an address that does not
 * answer is a result, not a failure of the call.
 */

/* Bytes of data an I2C or gateway call moves. */
#define PXW3PE_RAW_DATA 16

/*
 * A 32-bit register of the bridge at offset (a multiple of 4, below 0xCA0). write 0 reads it
 * into value, write 1 writes value.
 */
struct pxw3pe_raw_reg {
	__u32 offset;
	__u32 value;
	__u32 write;
};

/*
 * Registers of the I2C device at the 7-bit address addr, from register reg on: write 0 reads len
 * (1..16) bytes into data (S addr reg Sr addr+1 data P), write 1 writes len bytes of data
 * (S addr reg data P).
 */
struct pxw3pe_raw_i2c {
	__u32 addr;
	__u32 reg;
	__u32 len;
	__u32 write;
	__u32 status;
	__u8 data[PXW3PE_RAW_DATA];
};

/*
 * The tuner at the 8-bit write address tuner (even; 0xA8 satellite, 0xCE terrestrial) behind the
 * gateway (register 0xFE) of the demodulator core at the 7-bit address demod. With read_len 0,
 * writes the len (1..16) bytes of data to the tuner in one transaction. Otherwise, in one bus
 * hold, writes them as the address part of a read and reads read_len (1..16) bytes into data.
 */
struct pxw3pe_raw_gate {
	__u32 demod;
	__u32 tuner;
	__u32 len;
	__u32 read_len;
	__u32 status;
	__u8 data[PXW3PE_RAW_DATA];
};

#define PXW3PE_RAW_REG _IOWR(PXW3PE_LNB_IOC, 0x80, struct pxw3pe_raw_reg)
#define PXW3PE_RAW_I2C _IOWR(PXW3PE_LNB_IOC, 0x83, struct pxw3pe_raw_i2c)
#define PXW3PE_RAW_GATE _IOWR(PXW3PE_LNB_IOC, 0x84, struct pxw3pe_raw_gate)

#endif
