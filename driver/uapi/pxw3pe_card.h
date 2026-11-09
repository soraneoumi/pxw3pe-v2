/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * User-space interface of the PX-W3PE built-in smart-card reader.
 *
 * Each card appears as /dev/pxw3pe-cardN. The driver runs the complete T=1 protocol
 * (ISO/IEC 7816-3 in the ARIB STD-B25 profile) itself, so user space exchanges whole APDUs.
 * All calls are serialised per card; a call blocks while another one is in progress.
 *
 * Errors common to all calls: ENODEV when the board has been removed (an open file stays valid
 * and keeps failing this way), ENOTTY for an unknown request, EFAULT for a bad user pointer, and
 * EINTR when the caller is killed. EINTR while a call waits for another one means that it did
 * nothing. Otherwise it cut the operation short and left the card in reset with no session.
 *
 * A card pulled while a session is open ends the session once two samples of the detect switch
 * at least 100 ms apart, with none in between finding the card, have read "absent", whether an
 * operation was running or the driver noticed between two operations (it samples the switch
 * about every 100 ms). A single dropout shorter than that is a bouncing contact. The card is then held in reset, ACTIVE goes away, the next
 * GET_STATUS reports REMOVED, and TRANSMIT fails with ENOTCONN (or ENODEV, if it finds the card
 * missing itself) until the card is reset.
 *
 * The Rust driver mirrors these definitions in card.rs; any change here must be made there too.
 */
#ifndef _UAPI_PXW3PE_CARD_H
#define _UAPI_PXW3PE_CARD_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define PXW3PE_CARD_ATR_MAX 33
/* Smallest command: an APDU has at least CLA INS P1 P2 (ISO/IEC 7816-4). */
#define PXW3PE_CARD_APDU_MIN 4
/* Largest command or response the protocol can carry in one block (IFSD = 254). */
#define PXW3PE_CARD_APDU_MAX 254

/*
 * A card is inserted. While a session is open, a card that has been absent for less than the
 * 100 ms above still counts as inserted.
 */
#define PXW3PE_CARD_PRESENT (1u << 0)
/* The card has been reset and a protocol session is open. */
#define PXW3PE_CARD_ACTIVE (1u << 1)
/*
 * Since the previous GET_STATUS, a session ended because its card was pulled. Reported once, to
 * whichever file asks first: it is meant for the one process that manages the reader (pcscd).
 * It may come with PRESENT: a card pulled and put back (or replaced) between two calls.
 */
#define PXW3PE_CARD_REMOVED (1u << 2)

struct pxw3pe_card_status {
	__u32 flags;
};

struct pxw3pe_card_atr {
	__u32 len;
	__u8 atr[PXW3PE_CARD_ATR_MAX];
	__u8 reserved[3];
};

struct pxw3pe_card_transfer {
	/* In: command length. */
	__u32 command_len;
	/* Out: response length, including SW1 SW2. */
	__u32 response_len;
	__u8 command[256];
	__u8 response[256];
};

/*
 * Counters since the reader was set up (each time the driver binds to the board); they help tell
 * contact problems from card problems.
 */
struct pxw3pe_card_stats {
	__u32 edc_errors;
	__u32 other_errors;
	__u32 retransmissions;
	__u32 resynchs;
	__u32 wtx;
	__u32 resets;
	__u32 failed_resets;
	__u32 reserved;
};

#define PXW3PE_CARD_IOC 0xBA

/*
 * Reports whether a card is inserted, whether a session is open, and whether a session ended
 * with a removal since the previous call. A session whose card has been absent for 100 ms is
 * closed. Errors: ENODEV, EINTR.
 */
#define PXW3PE_CARD_GET_STATUS _IOR(PXW3PE_CARD_IOC, 0, struct pxw3pe_card_status)
/*
 * Resets the card (opening a new session) and returns its ATR. Errors:
 * ENODEV  no card, or it was pulled during the reset;
 * EIO     the card did not answer the reset or the IFS negotiation, or its answer took too long;
 * EPROTO  the ATR is malformed or announces parameters this reader cannot operate (only
 *         T=1 specific mode, F = 372, D = 2, N = 255 is supported);
 * EINTR   the caller was killed.
 * An error leaves the card in reset, with no session.
 */
#define PXW3PE_CARD_RESET _IOR(PXW3PE_CARD_IOC, 1, struct pxw3pe_card_atr)
/* Holds the card in reset, ending the session. Errors: ENODEV, EINTR. */
#define PXW3PE_CARD_POWER_OFF _IO(PXW3PE_CARD_IOC, 2)
/*
 * Sends one APDU and returns the card's answer. The command has command_len bytes, from
 * PXW3PE_CARD_APDU_MIN to PXW3PE_CARD_APDU_MAX. Errors:
 * EINVAL    command_len is below PXW3PE_CARD_APDU_MIN;
 * EMSGSIZE  command_len is above PXW3PE_CARD_APDU_MAX or above the card's IFSC (chaining is
 *           not used); the session stays open;
 * ENOTCONN  no session is open: the card was not reset, was powered off, lost its session in an
 *           earlier error, or was pulled;
 * ENODEV    the card was pulled during the exchange;
 * EIO       the exchange failed: no valid answer after the recovery steps of ISO/IEC 7816-3,
 *           none within 30 s, or after a resynchronisation the command no longer fit the card's
 *           IFSC (the card may have executed it).
 * EINVAL, EMSGSIZE and ENOTCONN change nothing. Any other error leaves the card in reset with
 * no session, so it has to be reset before the next command.
 */
#define PXW3PE_CARD_TRANSMIT _IOWR(PXW3PE_CARD_IOC, 3, struct pxw3pe_card_transfer)
/* Reads the counters. Errors: ENODEV, EINTR. */
#define PXW3PE_CARD_GET_STATS _IOR(PXW3PE_CARD_IOC, 4, struct pxw3pe_card_stats)

#endif
