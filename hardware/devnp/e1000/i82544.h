/******************************************************************************

  Copyright (c) 2001-2008, Intel Corporation 
  All rights reserved.
  Copyright (c) 2007-2008, QNX Software Systems GmbH & Co. KG.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.
  
   2. Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution.
  
   3. None of the names of Intel Corporation, QNX Software Systems
      GmbH & Co KG, or the names of any of their contributors may be used
	  to endorse or promote products derived from this software without
	  specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNERS OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/


#ifndef NIC_I8554_H_INCLUDED
#define NIC_I8554_H_INCLUDED

#define NIC_PRIORITY				21
#define I82544_UNUSED_DESCR			4

#define MIN_NUM_RX_DESCRIPTORS		16
#define MIN_NUM_TX_DESCRIPTORS		64
#define MAX_NUM_RX_DESCRIPTORS		4096
#define MAX_NUM_TX_DESCRIPTORS		4096

#ifdef __PPC__  // terrible hack for PR 22581 and 20915
#define	I82544_DEFAULT_NUM_RX		64
#define	I82544_DEFAULT_NUM_TX		64
#else
#define	I82544_DEFAULT_NUM_RX		512		// 1M total cost
#define	I82544_DEFAULT_NUM_TX		4096	//64K total cost
#endif
#define	I82544_INTERRUPT_PULSE		0x5a
#define	I82544_TIMER_PULSE			0x55
#define	I82544_TIMER_TEST			0x54
#define	I82544_DEFRAG_PACKET		(1<<31)
#define	DEFAULT_TX_REAP				64

#define	MAX_INTS_PER_SEC			20000
#define	DEFAULT_ITR					1000000000/(MAX_INTS_PER_SEC * 256)

#define SPEED_MODE_BIT				 (1 << 21)

#ifndef i82544_MO_TYPE
/* Valid values are 0, 1, 2, 3 */
#define i82544_MO_TYPE 0
#endif
#if (i82544_MO_TYPE - 0) == 0
#define i82544_MCHASH_LOSHIFT 4
#define i82544_MCHASH_HISHIFT 4
#elif (i82544_MO_TYPE - 0) == 1
#define i82544_MCHASH_LOSHIFT 3
#define i82544_MCHASH_HISHIFT 5
#elif (i82544_MO_TYPE - 0) == 2
#define i82544_MCHASH_LOSHIFT 2
#define i82544_MCHASH_HISHIFT 6
#elif (i82544_MO_TYPE - 0) == 3
#define i82544_MCHASH_LOSHIFT 0
#define i82544_MCHASH_HISHIFT 8
#else
#error invalid i82544_MO_TYPE
#endif

#define	E1000_MTA_SIZE	128

//
// We will arbitrarily transmit a packet with
// a maximum of 32 fragments (descriptors).
//
#define I82544_MAX_FRAGS			32

#include <io-pkt/iopkt_driver.h>
#include <stdio.h>
#include <errno.h>
#include <atomic.h>
#include <unistd.h>
//#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/siginfo.h>
#include <sys/syspage.h>
#include <sys/neutrino.h>
#include <sys/mbuf.h>
#include <sys/slogcodes.h>
#include <sys/platform.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <sys/io-pkt.h>
#include <sys/cache.h>
#include <sys/callout.h>
#include <sys/device.h>
#include <hw/inout.h>
#include <drvr/mdi.h>
#include <drvr/eth.h>
#include <hw/nicinfo.h>
#include <sys/device.h>
#define	_STDDEF_H_INCLUDED
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <siglock.h>
#include <dev/mii/miivar.h>

#include "e1000_hw.h"
#include "e1000_nvm.h"
#include "e1000_mac.h"
#include "e1000_api.h"

#include <_pack1.h>

typedef volatile struct {
	uint64_t	bufaddr;
	uint16_t	length;
	uint16_t	cksum;
	uint8_t		status;
	uint8_t		errors;
	uint16_t	special;
} i82544_rdesc_t;

typedef volatile struct {
	uint64_t	bufaddr;
	uint32_t	cmd_type_len;
	uint32_t	opts_stat;
} i82544_tdesc_t;

/* Transmit checksum context descriptor */
typedef volatile struct {
	union {
		struct {
			uint8_t		ipcss; /* start  */
			uint8_t		ipcso; /* offset */
			uint16_t	ipcse; /* end    */
		} ipcs_parts;
		uint32_t ipcs_ctxt;
	} ipcs_ctxt;

	union {
		struct {
			uint8_t		tucss; /* start  */
			uint8_t		tucso; /* offset */
			uint16_t	tucse; /* end    */
		} tucs_parts;
		uint32_t tucs_ctxt;
	} tucs_ctxt;
	uint32_t	tucmd_type_len;
	uint8_t		status;
	uint8_t		hdrlen;
	uint16_t	mss;
} i82544_tx_cs_ctx_t;

#define CS_CTXT_START(x) (((x) & 0xff)   <<  0)
#define CS_CTXT_OFF(x)   (((x) & 0xff)   <<  8)
#define CS_CTXT_END(x)   (((x) & 0xffff) << 16)
#define CS_CTXT_PAYLEN(x) ((x) & 0xfffff)

#define	roundup2(x, y)	(((x)+((y)-1))&(~((y)-1)))

struct i82544_s {
	struct ethercom		ecom;
	struct callout		hk_callout;
	struct callout		tst_callout;
	nic_config_t		cfg;
	nic_stats_t			stats;
	struct _iopkt_self	*iopkt;
	void				*pci_dev_hdl;
	uint64_t			bmtrans;
	struct cache_ctrl	cachectl;
	const struct sigevent *(*isrp)(void *, int);
	const struct sigevent *(*isrp_lnk)(void *, int);

	int					prom_multi_busted;
	int					msi_cap;
	int					msi_enabled;

	void				*rx_descriptor_area;
	void				*tx_descriptor_area;
	void				*sd_hook;

	/*
	 * We separate and pad the variables used in RX vs. TX path to
	 * remove cacheline contention on SMP.
	 */
	unsigned			__fill[15];
	/* TX descriptor and buffer tracking */
	struct evcnt		ev_txdrop __attribute__((aligned (NET_CACHELINE_SIZE)));
	int					num_transmit;	/* No. of TX descriptors cmdline override */
	int					tx_free;
	i82544_tdesc_t		*tdesc;
	int					cur_tx_rptr;
	int					cur_tx_wptr;
	struct mbuf			**tx_mbuf;
	/* Hardware checksum support -- used in TX path (host order) */
	uint32_t			last_ipcs_ctxt;
	uint32_t			last_tucs_ctxt;
	uint32_t			last_cmdlen;
	uint16_t			last_mss;
	uint16_t			vf_ifp;
	uint8_t				last_total_hdrlen;

	uint8_t				__fill2[15];

	/* RX descriptor and buffer tracking */
	struct _iopkt_inter	inter __attribute__((aligned (NET_CACHELINE_SIZE)));
	struct _iopkt_inter	inter_lnk __attribute__((aligned (NET_CACHELINE_SIZE)));
	int					num_receive;	/* No. of RX descriptors cmdline override */
	int					cur_rx_rptr;
	i82544_rdesc_t		*rdesc;
	struct mbuf			**rx_mbuf;
	struct mbuf			*rx_head;
	struct mbuf			**rx_tail;
	int					rx_len;
	int					rx_discard;
	uint32_t			eims;
	uint32_t			link_mask;

	int					iid __attribute__((aligned (NET_CACHELINE_SIZE)));
	int					iid_lnk __attribute__((aligned (NET_CACHELINE_SIZE)));
	
	uint32_t			intrmask;
	uint32_t			eiac_mask;
//	timer_t				hk_timer;	/* For housekeeping */
//	timer_t				tst_timer;	/* For testing */
//	struct itimerspec	t_timer;

	/* Media related state */
	int					linkup;		/* Current link status */
	int					force_link;
	int					cmd_force;
	int					link_config;
	struct e1000_hw		hw;
	struct mii_data		bsd_mii;
	int					pause_xmit;			// tx flow control enabled?
	int					pause_receive;		// rx flow control enabled?
	int					flow;
	// these options override auto-negotiated values and set the above 
	int					pause_tx_disable;	// set to never tx pause frames
	int					pause_tx_enable;	// set to always tx pause frames
	int					pause_rx_disable;	// set to ignore rxd pause frames
	int					pause_rx_enable;	// set to act on rxd pause frames

	uint32_t			irq_cause;

	/* Command line options */
	int					pause_ignore;
	int					pause_suppress;

	uint32_t			flash_map_size;
	int					start_running;
	int					dying;
	int					kermask;
	int					tx_reap;
	unsigned			rx_delay;
	unsigned			rx_abs;
	unsigned			irq_thresh;
	uint32_t			total_rx_bytes;
	uint32_t			total_rx_pkts;
	uint32_t			itr_setting;
	uint32_t			itr;
	uint32_t			rx_itr_val;
	int					set_itr;
	int					itr_set;	/* Command line override */
	int					max_read;
	uint16_t			rx_itr;

	uint8_t				msix_cap_ptr;
	uint8_t				bus;
	uint8_t				devfn;
	uint8_t				hk;
	int					i82544_if_flags;
	int					init_run;
};

struct	i82544_dev {
	struct device		sc_dev;
	i82544_dev_t		*sc_i82544;
	char				filler[sizeof(i82544_dev_t) + NET_CACHELINE_SIZE];
};

#include <_packpop.h>

/* Exported functions */
int i82544_flush(int reg_hdl, void *func_hdl);
int i82544_ioctl(struct ifnet *, unsigned long, caddr_t);
void i82544_filter(i82544_dev_t *i82544);

const struct sigevent *i82544_isr(void *, int);
const struct sigevent *i82544_isr_rx (void *, int);
const struct sigevent *i82544_isr_lnk (void *, int);
const struct sigevent * i82544_isr_kermask(void *, int);
const struct sigevent *i82544_isr_rx_kermask (void *, int);
const struct sigevent *i82544_isr_lnk_kermask (void *, int);
int i82544_process_interrupt(void *, struct nw_work_thread *);
int i82544_enable_interrupt(void *);
int i82544_enable_interrupt_kermask(void *);
void i82544_hk_callout(void *arg);

void i82544_start(struct ifnet *);
int  i82544_offload_setup (i82544_dev_t *, struct mbuf *, int, uint32_t *, uint32_t *);
void i82544_checksum_init(i82544_dev_t *i82544, uint32_t ipcs_ctxt, uint32_t tucs_ctxt);
int i82544_transmit_complete(i82544_dev_t *i82544, int from_send);
void i82544_reap(i82544_dev_t *i82544);
void update_link_status (i82544_dev_t *i82544);

/* Internal functions */
int i82544_parse_options( i82544_dev_t *i82544, const char *optstring, nic_config_t *cfg);
void i82544_enable(i82544_dev_t *i82544);

int i82544_link_event (void *arg, struct nw_work_thread *wtp);
int	i82544_lnk_enable_interrupt (void *arg);
int	i82544_rx_enable_interrupt (void *arg);
int	i82544_receive (void *arg, struct nw_work_thread *wtp);

/*
 * Note: the TAILQ macros were put back in to make the driver
 *       backward compatible with 6.0.1a release  & 
 *       should be (were) removed for 6.1.1
 */
#ifndef TAILQ_FIRST

#define TAILQ_EMPTY(head) ((head)->tqh_first == NULL)
#define TAILQ_FIRST(head) ((head)->tqh_first)
#define TAILQ_LAST(head) ((head)->tqh_last)
#define TAILQ_NEXT(elm, field) ((elm)->field.tqe_next)
#define TAILQ_PREV(elm, field) ((elm)->field.tqe_prev)

#endif

#endif /* NIC_I8554_H_INCLUDED */



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/i82544.h $ $Rev: 708496 $")
#endif
