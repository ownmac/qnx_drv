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
/*$FreeBSD: $*/


/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _E1000_H_
#define _E1000_H_

#include "e1000_api.h"

#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

#define INTEL_E1000_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

struct e1000_adapter;

#define E1000_DBG(args...)

#define E1000_ERR(args...) nic_slogf(_SLOGC_NETWORK, _SLOG_ERROR, "e1000: " args)

#define PFX "e1000: "
#if	0
#define DPRINTK(nlevel, klevel, fmt, args...) \
	(void)((NETIF_MSG_##nlevel & adapter->msg_enable) && \
	printk(KERN_##klevel PFX "%s: %s: " fmt, adapter->netdev->name, \
		__FUNCTION__ , ## args))
#endif

#define E1000_MAX_INTR 10

/* TX/RX descriptor defines */
#define E1000_DEFAULT_TXD                  256
#define E1000_MAX_TXD                      256
#define E1000_MIN_TXD                       80
#define E1000_MAX_82544_TXD               4096

#define E1000_DEFAULT_RXD                  256
#define E1000_MAX_RXD                      256

#define E1000_MIN_RXD                       80
#define E1000_MAX_82544_RXD               4096

#define E1000_MIN_ITR_USECS                 10 /* 100000 irq/sec */
#define E1000_MAX_ITR_USECS              10000 /* 100    irq/sec */


/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* Supported Rx Buffer Sizes */
#define E1000_RXBUFFER_128   128
#define E1000_RXBUFFER_256   256
#define E1000_RXBUFFER_512   512
#define E1000_RXBUFFER_1024  1024
#define E1000_RXBUFFER_2048  2048
#define E1000_RXBUFFER_4096  4096
#define E1000_RXBUFFER_8192  8192
#define E1000_RXBUFFER_16384 16384

/* SmartSpeed delimiters */
#define E1000_SMARTSPEED_DOWNSHIFT 3
#define E1000_SMARTSPEED_MAX       15

/* Packet Buffer allocations */
#define E1000_PBA_BYTES_SHIFT 0xA
#define E1000_TX_HEAD_ADDR_SHIFT 7
#define E1000_PBA_TX_MASK 0xFFFF0000

/* Early Receive defines */
#define E1000_ERT_2048 0x100

#define E1000_FC_PAUSE_TIME 0x0680 /* 858 usec */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16	/* Must be power of 2 */

#define AUTO_ALL_MODES            0
#define E1000_EEPROM_82544_APM    0x0004
#define E1000_EEPROM_APME         0x0400

#ifndef E1000_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define E1000_MASTER_SLAVE	e1000_ms_hw_default
#endif

#ifdef NETIF_F_HW_VLAN_TX
#define E1000_MNG_VLAN_NONE -1
#endif

/* CEM Support */
#define FW_HDR_LEN           0x4
#define FW_CMD_DRV_INFO      0xDD
#define FW_CMD_DRV_INFO_LEN  0x5
#define FW_CMD_RESERVED      0X0
#define FW_RESP_SUCCESS      0x1
#define FW_UNUSED_VER        0x0
#define FW_MAX_RETRIES       3
#define FW_STATUS_SUCCESS    0x1
#define FW_FAMILY_DRV_VER    0Xffffffff

struct e1000_fw_hdr {
	u8 cmd;
	u8 buf_len;
	union
	{
		u8 cmd_resv;
		u8 ret_status;
	} cmd_or_resp;
	u8 checksum;
};

struct e1000_fw_drv_info {
	struct e1000_fw_hdr hdr;
	u8 port_num;
	u8 fill1 [3];
	u32 drv_version;
};

#define E1000_DESC_UNUSED(R) \
	((((R)->next_to_clean > (R)->next_to_use) ? 0 : (R)->count) + \
	(R)->next_to_clean - (R)->next_to_use - 1)

#define E1000_RX_DESC_EXT(R, i)	    \
	(&(((union e1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)

#ifdef SIOCGMIIPHY
/* PHY register snapshot values */
struct e1000_phy_regs {
	u16 bmcr;		/* basic mode control register    */
	u16 bmsr;		/* basic mode status register     */
	u16 advertise;		/* auto-negotiation advertisement */
	u16 lpa;		/* link partner ability register  */
	u16 expansion;		/* auto-negotiation expansion reg */
	u16 ctrl1000;		/* 1000BASE-T control register    */
	u16 stat1000;		/* 1000BASE-T status register     */
	u16 estatus;		/* extended status register       */
};
#endif

#define E1000_FLAG_HAS_SMBUS                (1 << 0)
#define E1000_FLAG_HAS_INTR_MODERATION      (1 << 4)
#define E1000_FLAG_BAD_TX_CARRIER_STATS_FD  (1 << 6)
#define E1000_FLAG_QUAD_PORT_A              (1 << 8)
#define E1000_FLAG_SMART_POWER_DOWN         (1 << 9)
#ifdef NETIF_F_TSO
#define E1000_FLAG_HAS_TSO                  (1 << 10)
#ifdef NETIF_F_TSO6
#define E1000_FLAG_HAS_TSO6                 (1 << 11)
#endif
#define E1000_FLAG_TSO_FORCE                (1 << 12)
#endif

enum e1000_state_t {
	__E1000_TESTING,
	__E1000_RESETTING,
	__E1000_DOWN
};

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

extern char e1000_driver_name[];
extern const char e1000_driver_version[];

extern void e1000_power_up_phy(struct e1000_hw *hw);



#endif /* _E1000_H_ */



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/e1000.h $ $Rev: 708496 $")
#endif
