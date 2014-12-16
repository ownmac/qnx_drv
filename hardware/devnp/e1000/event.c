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

#include "bpfilter.h"
#include <i82544.h>
#include <e1000.h>
#include <drvr/mdi.h>
#include <netinet/in.h>

#if NBPFILTER > 0
#include <net/bpf.h>
#include <net/bpfdesc.h>
#endif

#include	<stdio.h>
#include	<string.h>


#if	0		/* This has still to be finalized */
/**
 * e1000_update_itr - update the dynamic ITR value based on statistics
 * @i82544: pointer to i82544
 * @itr_setting: current i82544->itr
 * @packets: the number of packets during this measurement interval
 * @bytes: the number of bytes during this measurement interval
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.  This functionality is controlled
 *      by the InterruptThrottleRate module parameter.
 **/

static unsigned int e1000_update_itr (i82544_dev_t *i82544, uint16_t itr_setting,
				int packets, int bytes)

{
unsigned int retval = itr_setting;

	if (packets == 0)
		goto update_itr_done;

	switch (itr_setting) {
		case	lowest_latency:
			/* handle TSO and jumbo frames */
			if (bytes / packets > 8000)
				retval = bulk_latency;
			else if ((packets < 5) && (bytes > 512))
				retval = low_latency;
			break;
		case	low_latency:	/* 50 usec aka 20000 ints/s */
			if (bytes > 10000) {
				/* this if handles the TSO accounting */
				if (bytes / packets > 8000)
					retval = bulk_latency;
				else if ((packets < 10) || ((bytes / packets) > 1200))
					retval = bulk_latency;
				else if ((packets > 35))
					retval = lowest_latency;
			} else if (bytes / packets > 2000) {
				retval = bulk_latency;
			} else if (packets <= 2 && bytes < 512) {
				retval = lowest_latency;
			}
			break;
		case	bulk_latency:	/* 250 usec aka 4000 ints/s */
			if (bytes > 25000) {
				if (packets > 35)
					retval = low_latency;
			} else if (bytes < 6000) {
				retval = low_latency;
			}
			break;
		}

update_itr_done:
	return retval;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static void e1000_set_itr (i82544_dev_t *i82544)

{
struct e1000_hw *hw = &i82544->hw;
uint16_t		current_itr;
uint32_t		new_itr = i82544->itr, reg;

	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (i82544->cfg.media_rate != SPEED_1000 * 1000) {
		current_itr = 0;
		new_itr = 4000;
		goto set_itr_now;
		}

//	if (i82544->flags2 & FLAG2_DISABLE_AIM) {
//		new_itr = 0;
//		goto set_itr_now;
//		}

	i82544->rx_itr = e1000_update_itr (i82544, i82544->rx_itr,
					   i82544->total_rx_pkts, i82544->total_rx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (i82544->itr_setting == 3 && i82544->rx_itr == lowest_latency)
		i82544->rx_itr = low_latency;

	current_itr = i82544->rx_itr;

	switch (current_itr) {
		/* counts and packets in update_itr are dependent on these numbers */
		case	lowest_latency:
			new_itr = 70000;
			break;
		case	low_latency:
			new_itr = 20000;	/* aka hwitr = ~200 */
			break;
		case	bulk_latency:
			new_itr = 4000;
			break;
		default:
			break;
		}

set_itr_now:
	if (new_itr != i82544->itr) {
		/*
		 * this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing
		 */
		new_itr = new_itr > i82544->itr ?
		    min (i82544->itr + (new_itr >> 2), new_itr) : new_itr;
		i82544->itr = new_itr;
		reg = (hw->mac.type == e1000_82574) ? E1000_EITR_82574(0) : E1000_ITR;
		if (i82544->msi_enabled == PCI_MSIX)
			i82544->set_itr = 1;
		else if (new_itr)
			E1000_WRITE_REG (hw, reg, 1000000000 / (new_itr * 256));
		else
			E1000_WRITE_REG (hw, reg, 0);
		}
}
#endif

/**************************************************************************/
/* Receive routine using advanced descriptors.                            */
/**************************************************************************/

static	int	i82544_adv_receive (i82544_dev_t *i82544, struct nw_work_thread *wtp)

{
int				cur_rx_rptr = i82544->cur_rx_rptr;
union			e1000_adv_rx_desc	*adesc;
struct mbuf		*m, *rm;
unsigned		status;
uint8_t			errors = 0;
off64_t			phys;
uint32_t		total_bytes = 0, total_pkts = 0;
struct ifnet	*ifp = &i82544->ecom.ec_if;
struct e1000_hw	*hw = &i82544->hw;

	adesc = (union e1000_adv_rx_desc *) &i82544->rdesc [cur_rx_rptr];

	while ((status = ENDIAN_LE32 (adesc->wb.upper.status_error)) & E1000_RXD_STAT_DD) {
		if (status & E1000_RXDEXT_ERR_FRAME_ERR_MASK) {
			ifp->if_ierrors++;
			goto nextpkt;
			}
	
		if (i82544->rx_discard) {
			if (status & E1000_RXD_STAT_EOP)
				i82544->rx_discard = 0;
			ifp->if_ierrors++;
			goto nextpkt;
			}

		/* Get a packet/buffer to replace the one that was filled */
		m = m_getcl_wtp (M_DONTWAIT, MT_DATA, M_PKTHDR, wtp);
		if (m == NULL) {
			i82544->rx_discard = 1;
			ifp->if_ierrors++;
			i82544->stats.rx_failed_allocs++;
			if (i82544->rx_head) {
				m_freem (i82544->rx_head);
				i82544->rx_len  = 0;
				i82544->rx_head = NULL;
				i82544->rx_tail = &i82544->rx_head;
				}
			goto nextpkt;
			}

		rm = i82544->rx_mbuf[cur_rx_rptr];
		i82544->rx_mbuf[cur_rx_rptr] = m;

		phys = pool_phys (m->m_data, m->m_ext.ext_page);
		rm->m_len = ENDIAN_LE16 (adesc->wb.upper.length);

		CACHE_INVAL (&i82544->cachectl, m->m_data, phys, m->m_ext.ext_size);
		adesc->read.pkt_addr = ENDIAN_LE64 (phys + i82544->bmtrans);
		adesc->read.hdr_addr = 0;

		rm->m_pkthdr.rcvif = ifp;
		*i82544->rx_tail = rm;
		i82544->rx_tail = &rm->m_next;
		i82544->rx_len += rm->m_len;

		if ((status & E1000_RXD_STAT_EOP) == 0)
			goto nextpkt;

		rm = i82544->rx_head;
		rm->m_pkthdr.len = i82544->rx_len;
		total_bytes += i82544->rx_len;

		i82544->rx_len  = 0;
		i82544->rx_head = NULL;
		i82544->rx_tail = &i82544->rx_head;

		if (!(status & E1000_RXD_STAT_IXSM)) {
			/* "checked" bits are valid */
			if (status & E1000_RXD_STAT_IPCS) {
				/* IP was checked */
				rm->m_pkthdr.csum_flags |= M_CSUM_IPv4;
				errors = (uint8_t) (status >> 24);
				if (errors & E1000_RXD_ERR_IPE)
					rm->m_pkthdr.csum_flags |= M_CSUM_IPv4_BAD;
				}
			if (status & E1000_RXD_STAT_TCPCS) {
				/* TCP/UDP was checked */
				rm->m_pkthdr.csum_flags |= M_CSUM_TCPv4 | M_CSUM_UDPv4;
				if (errors & E1000_RXD_ERR_TCPE)
					rm->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
				}
			}

#if NBPFILTER > 0
		/* Pass this up to any BPF listeners. */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, rm);
#endif

		/* Send it up */
		total_pkts++;
		ifp->if_ipackets++;
		(*ifp->if_input)(ifp, rm);

nextpkt:

		/* Give the descriptor back to the hardware */
		E1000_WRITE_REG (hw, E1000_RDT(0), cur_rx_rptr);

		cur_rx_rptr = (cur_rx_rptr + 1) % i82544->num_receive;
		adesc = (union e1000_adv_rx_desc *) &i82544->rdesc[cur_rx_rptr];
		}

	i82544->cur_rx_rptr = cur_rx_rptr;
	i82544->total_rx_bytes += total_bytes;
	i82544->total_rx_pkts += total_pkts;

	return (1);
}

/**************************************************************************/
/* Standard receive interrupt service routine.                            */
/**************************************************************************/

int		i82544_receive (void *arg, struct nw_work_thread *wtp)

{
i82544_dev_t	*i82544 = arg;
int				cur_rx_rptr = i82544->cur_rx_rptr;
i82544_rdesc_t	*rdesc = &i82544->rdesc[cur_rx_rptr];
struct mbuf		*m, *rm;
unsigned		status;
off64_t			phys;
uint32_t		total_bytes = 0, total_pkts = 0;
struct ifnet	*ifp = &i82544->ecom.ec_if;
struct e1000_hw	*hw = &i82544->hw;

	/* The 82575, 82576, 82580 & i350 use extended descriptors */
	if (hw->mac.type >= e1000_82575) {
		return (i82544_adv_receive (i82544, wtp));
		}

	while ((status = rdesc->status) & E1000_RXD_STAT_DD) {
		if (rdesc->errors & (E1000_RXD_ERR_CE |
		    E1000_RXD_ERR_SE | E1000_RXD_ERR_SEQ |
		    E1000_RXD_ERR_CXE | E1000_RXD_ERR_RXE)) {
			ifp->if_ierrors++;
			goto nextpkt;
			}
	
		if (i82544->rx_discard) {
			if (status & E1000_RXD_STAT_EOP)
				i82544->rx_discard = 0;
			ifp->if_ierrors++;
			goto nextpkt;
			}

		/* Get a packet/buffer to replace the one that was filled */
		m = m_getcl_wtp (M_DONTWAIT, MT_DATA, M_PKTHDR, wtp);
		if (m == NULL) {
			i82544->rx_discard = 1;
			ifp->if_ierrors++;
			i82544->stats.rx_failed_allocs++;
			if (i82544->rx_head) {
				m_freem (i82544->rx_head);
				i82544->rx_len  = 0;
				i82544->rx_head = NULL;
				i82544->rx_tail = &i82544->rx_head;
				}
			goto nextpkt;
			}

		rm = i82544->rx_mbuf[cur_rx_rptr];
		i82544->rx_mbuf[cur_rx_rptr] = m;

		phys = pool_phys (m->m_data, m->m_ext.ext_page);
		CACHE_INVAL (&i82544->cachectl, m->m_data, phys, m->m_ext.ext_size);
		rdesc->bufaddr = ENDIAN_LE64 (phys + i82544->bmtrans);

		rm->m_pkthdr.rcvif = ifp;
		rm->m_len = ENDIAN_LE16 (rdesc->length);

		*i82544->rx_tail = rm;
		i82544->rx_tail = &rm->m_next;
		i82544->rx_len += rm->m_len;
		total_bytes += i82544->rx_len;

		if ((status & E1000_RXD_STAT_EOP) == 0)
			goto nextpkt;

		rm = i82544->rx_head;
		rm->m_pkthdr.len = i82544->rx_len;

		i82544->rx_len  = 0;
		i82544->rx_head = NULL;
		i82544->rx_tail = &i82544->rx_head;

		if (!(status & E1000_RXD_STAT_IXSM)) {
			/* "checked" bits are valid */
			if (status & E1000_RXD_STAT_IPCS) {
				/* IP was checked */
				rm->m_pkthdr.csum_flags |= M_CSUM_IPv4;
				if (rdesc->errors & E1000_RXD_ERR_IPE)
					rm->m_pkthdr.csum_flags |= M_CSUM_IPv4_BAD;
				}
			if (status & E1000_RXD_STAT_TCPCS) {
				/* TCP/UDP was checked */
				rm->m_pkthdr.csum_flags |= M_CSUM_TCPv4 | M_CSUM_UDPv4;
				if (rdesc->errors & E1000_RXD_ERR_TCPE)
					rm->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
				}
			}

#if NBPFILTER > 0
		/* Pass this up to any BPF listeners. */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, rm);
#endif

		/* Send it up */
		total_pkts++;
		ifp->if_ipackets++;
		(*ifp->if_input)(ifp, rm);

nextpkt:
		rdesc->status = 0;

		/* Give the descriptor back to the hardware */
		E1000_WRITE_REG (hw, E1000_RDT(0), cur_rx_rptr);

		cur_rx_rptr = (cur_rx_rptr + 1) % i82544->num_receive;
		rdesc = &i82544->rdesc[cur_rx_rptr];
		}

	i82544->cur_rx_rptr = cur_rx_rptr;
	i82544->total_rx_bytes += total_bytes;
	i82544->total_rx_pkts += total_pkts;

	return (1);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

void	update_link_status (i82544_dev_t *i82544)

{
int					ret_val = 0;
int					link_active = FALSE;
uint16_t			speed, duplex;
uint32_t			tctl, ctrl, thstat;
struct e1000_hw		*hw = &i82544->hw;
struct ifnet		*ifp;

	ifp = &i82544->ecom.ec_if;
	speed = duplex = 0;

	/* ICH8 workaround-- Call gig speed drop workaround on cable
	 * disconnect (LSC) before accessing any PHY registers */
	if ((hw->mac.type == e1000_ich8lan) &&
	    (hw->phy.type == e1000_phy_igp_3) &&
	    (!(E1000_READ_REG (hw, E1000_STATUS) & E1000_STATUS_LU))) {
		e1000_gig_downshift_workaround_ich8lan (hw);
		}

	switch (hw->phy.media_type) {
		case	e1000_media_type_copper:
			if (hw->mac.get_link_status) {
				ret_val = e1000_check_for_link (hw);
				link_active = !hw->mac.get_link_status;
				}
			else {
				link_active = TRUE;
				}
			break;
		case	e1000_media_type_fiber:
			ret_val = e1000_check_for_link (hw);
			link_active = !!(E1000_READ_REG (hw, E1000_STATUS) &
			                 E1000_STATUS_LU);
			break;
		case	e1000_media_type_internal_serdes:
			ret_val = e1000_check_for_link (hw);
			link_active = hw->mac.serdes_has_link;
			break;
	/* VF device is type_unknown */
		case	e1000_media_type_unknown:
			e1000_check_for_link (hw);
			link_active = !hw->mac.get_link_status;
			/* Fall thru */
		default:
			break;
		}

	/* Check for thermal downshift or shutdown */
	if (hw->mac.type == e1000_i350) {
		thstat = E1000_READ_REG(hw, E1000_THSTAT);
		ctrl = E1000_READ_REG(hw, E1000_CTRL_EXT);
	}

	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (E1000_READ_REG (hw, E1000_CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		slogf (_SLOGC_NETWORK, _SLOG_WARNING, "Gigabit has been disabled, downgrading speed");
		}

	if (link_active && i82544->linkup == 0) {
		i82544->linkup = 1;
		i82544->cfg.flags &= ~NIC_FLAG_LINK_DOWN;
		e1000_get_speed_and_duplex (hw, &speed, &duplex);
		i82544->cfg.media_rate = speed * 1000;
		if (duplex == FULL_DUPLEX)
			i82544->cfg.duplex = 1;
		else
			i82544->cfg.duplex = 0;
		if ((hw->mac.type == e1000_82571 || hw->mac.type == e1000_82572) && speed < 1000) {
			uint32_t	tarc0;
			tarc0 = E1000_READ_REG (hw, E1000_TARC(0));
			tarc0 &= ~SPEED_MODE_BIT;
			E1000_WRITE_REG (hw, E1000_TARC(0), tarc0);
			}
		/* Disable TSO if link speed < 1G */
		if (speed < 1000 && (ifp->if_capabilities_tx & (IFCAP_TSOv4 | IFCAP_TSOv6))) {
			ifp->if_capabilities_tx &= ~(IFCAP_TSOv4 | IFCAP_TSOv6);
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "TSO disabled for link speed < 1G");
			}
		/* enable transmits in the hardware, need to do this
		 * after setting TARC0 */
		tctl = E1000_READ_REG (hw, E1000_TCTL);
		tctl |= E1000_TCTL_EN;
		E1000_WRITE_REG (hw, E1000_TCTL, tctl);
		if_link_state_change (ifp, LINK_STATE_UP);
		ctrl = E1000_READ_REG (hw, E1000_CTRL);
		if (i82544->cfg.verbose)
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s CTRL %x", __FUNCTION__, ctrl);
		i82544->pause_receive = (ctrl & E1000_CTRL_RFCE) ? 1 : 0;
		i82544->pause_xmit = (ctrl & E1000_CTRL_TFCE) ? 1 : 0;
		}
	else {
		if (!link_active) {
			i82544->cfg.media_rate = 0;
			i82544->cfg.duplex = 0;
			i82544->linkup = 0;
			i82544->cfg.flags |= NIC_FLAG_LINK_DOWN;
			if_link_state_change (ifp, LINK_STATE_DOWN);
			}
		}

	if (i82544->cfg.verbose) {
		if (link_active) {
			if (speed != 0) {
				slogf (_SLOGC_NETWORK, _SLOG_INFO, "Link up speed %d Mbps - duplex %s", speed, (duplex == FULL_DUPLEX) ? "Full" : "Half");
				}
			}
		else {
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "Link down");
			}
		}
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

int		i82544_link_event (void *arg, struct nw_work_thread *wtp)

{
i82544_dev_t		*i82544 = arg;
struct e1000_hw		*hw = &i82544->hw;

	hw->mac.get_link_status = 1;
	update_link_status (i82544);
	E1000_WRITE_REG (hw, E1000_IMS, E1000_IMS_LSC | 0x01000000);
	return (1);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

int		i82544_process_interrupt (void *arg, struct nw_work_thread *wtp)

{
i82544_dev_t		*i82544 = arg;
struct e1000_hw		*hw = &i82544->hw;
uint32_t			eims, ims, missing;

	if ((i82544->hw.mac.type >= e1000_82571) &&
		(i82544->irq_cause & E1000_ICR_INT_ASSERTED) == 0) {
		return (1);
		}

	/* 
	 * Reading ICR implicitly ACK's interrupts except
	 * when (IMS != 0) && ((ICR & IMS) == 0) we need
	 * to write ICR|IMS to ICR to ACK intrs instead
	 */
	do {
		if (i82544->irq_cause & E1000_ICR_LSC) {
			i82544_link_event (i82544, wtp);
			if (i82544->msi_enabled == PCI_MSIX) {
				E1000_WRITE_REG (hw, E1000_EIMS, i82544->link_mask);
				}
			}
		if ((i82544->irq_cause & E1000_ICR_RXT0) || (i82544->irq_cause & E1000_ICR_RXQ0)) {
			i82544->total_rx_bytes = 0;
			i82544->total_rx_pkts = 0;
			i82544_receive (i82544, wtp);
			if (i82544->msi_enabled == PCI_MSIX) {
				E1000_WRITE_REG (hw, E1000_EIMS, i82544->eims);
				eims = E1000_READ_REG (hw, E1000_EIMS);
				if (! (eims & i82544->eims))
					E1000_WRITE_REG (hw, E1000_EIMS, i82544->eims);
				}
/* Work in progress */
//			if (i82544->itr_setting & 3)
//				e1000_set_itr (i82544);
			}
		if (i82544->cfg.verbose > 3) {
			if ((i82544->irq_cause & i82544->intrmask) == 0 && (i82544->irq_cause & E1000_ICR_RXQ0) == 0) {
				slogf (_SLOGC_NETWORK, _SLOG_INFO, "e1000: Unexpected irq cause 0x%x", i82544->irq_cause);
				}
			}

		ims = E1000_READ_REG(hw, E1000_IMS);
		if (ims && !(ims & i82544->irq_cause)){
			/* write the bits missing between ims and irq_cause to ICR */
			missing = ~ims & i82544->irq_cause;
			E1000_WRITE_REG(hw, E1000_ICR, missing);
			//slogf (_SLOGC_NETWORK, _SLOG_INFO, "e1000: irq_cause %#x ims %#x missing %#x - ICR cleared", i82544->irq_cause, ims, missing);
			}

			/* Only loop on interrupt causes that we are interested in. */
			i82544->irq_cause = E1000_READ_REG (hw, E1000_ICR);
		} while ((i82544->irq_cause & i82544->intrmask) != 0);

	// if the transmit side is quiet, process txd descriptors now
	if (!i82544->start_running) {
		struct ifnet *ifp = &i82544->ecom.ec_if;
		struct _iopkt_self *iopkt = i82544->iopkt;
		struct nw_work_thread	*wtp = WTP;

		NW_SIGLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
		i82544_reap (i82544);
		NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
		}

	return 1;
}

/**************************************************************************/
/* This is the interrupt service routine for MSI and INTx interrupts.     */
/**************************************************************************/

const struct sigevent *i82544_isr (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter;

	/*
	 * First level cause filter.  Things will still
	 * work without this but less efficiently in a
	 * shared interrupt environment.
	 *
	 * Make sure we're not on_list before reading
	 * cause reg (which also clears it) so we don't
	 * interfere with process level irupt handling
	 * which may be doing the same thing.  This works
	 * because if we're on_list we know we're still
	 * masked (spurious) and on_list is knocked down
	 * after all process level processing.
	 */
	if (ient->on_list == 0) {
	    i82544->irq_cause = E1000_READ_REG (&i82544->hw, E1000_ICR);
		if ((i82544->irq_cause & E1000_ICR_INT_ASSERTED) == 0) {
			if (i82544->hw.mac.type >= e1000_82571) {
				/* IRQ not caused by this card. */
				ient->spurious++;
				return NULL;
				}
			}
		}

	/*
	 * We have to make sure the interrupt is masked regardless
	 * of our on_list status.  This is because of a window where
	 * a shared (spurious) interrupt comes after on_list
	 * is knocked down but before the enable() callout is made.
	 * If enable() then happened to run after we masked, we
	 * could end up on the list without the interrupt masked
	 * which would cause the kernel more than a little grief
	 * if one of our real interrupts then came in.
	 *
	 * This window doesn't exist when using kermask since the
	 * interrupt isn't unmasked until all the enable()s run
	 * (mask count is tracked by kernel).
	 *
	 * The window also doesn't exist with the first level cause
	 * filter above since we've filtered out spurious interrupts
	 * when on_list == 0.  The result is a redundant mask op when
	 * on_list == 1 (subset of shared interrupt env) and first
	 * level cause filter present.
	 */

	E1000_WRITE_REG (&i82544->hw, E1000_IMC, 0xffffffff);

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/* This is the MSI-X link interrupt service routine.                      */
/**************************************************************************/

const struct sigevent *i82544_isr_lnk (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter_lnk;
uint32_t			tmp = 0;

	if (ient->on_list == 0) {
		i82544->irq_cause = E1000_ICR_LSC;
	    tmp = E1000_READ_REG (&i82544->hw, E1000_ICR);
		if (tmp & E1000_ICR_LSC && i82544->hw.mac.type != e1000_82574) {
			E1000_WRITE_REG (&i82544->hw, E1000_IMS, E1000_IMS_LSC | 0x01000000);
			E1000_WRITE_REG (&i82544->hw, E1000_EIMS, i82544->link_mask);
			}
		}

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/* MSIX link interrupt enable. Already done above.                        */
/**************************************************************************/

int		i82544_lnk_enable_interrupt (void *arg)

{
	return (1);
}

/**************************************************************************/
/* This is the MSI-X receive interrupt service routine.                   */
/**************************************************************************/

const struct sigevent *i82544_isr_rx (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter;
uint32_t			tmp;

	if (ient->on_list == 0) {
		i82544->irq_cause = E1000_ICR_RXQ0;
	    tmp = E1000_READ_REG (&i82544->hw, E1000_ICR);
		E1000_WRITE_REG (&i82544->hw, E1000_EIMC, i82544->eims);
		}

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/* Re-enable MSI-X receive interrupts.                                    */
/**************************************************************************/

int		i82544_rx_enable_interrupt (void *arg)

{
i82544_dev_t		*i82544 = arg;

	E1000_WRITE_REG (&i82544->hw, E1000_EIMS, i82544->eims);
	return (1);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

const struct sigevent *i82544_isr_kermask (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter;

	if (ient->on_list == 0 &&
	    (i82544->irq_cause = E1000_READ_REG (&i82544->hw, E1000_ICR) == 0)) {
		/* IRQ not caused by this card. */
		ient->spurious++;
		return NULL;
		}


	/*
	 * Close window where this is referenced in
	 * i82544_enable_interrupt_kermask().  We may get
	 * an interrupt, return a sigevent and have
	 * another thread start processing on SMP before
	 * the InterruptAttach() has returned.
	 */
	i82544->iid = iid;

	InterruptMask (i82544->cfg.irq[0], iid);

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

const struct sigevent *i82544_isr_lnk_kermask (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter_lnk;

	if (ient->on_list == 0 &&
	    (i82544->irq_cause = E1000_READ_REG (&i82544->hw, E1000_ICR)) == 0) {
		/* IRQ not caused by this card. */
		ient->spurious++;
		return NULL;
		}


	/*
	 * Close window where this is referenced in
	 * i82544_enable_interrupt_kermask().  We may get
	 * an interrupt, return a sigevent and have
	 * another thread start processing on SMP before
	 * the InterruptAttach() has returned.
	 */
	i82544->iid = iid;

	InterruptMask (i82544->cfg.irq[2], iid);

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

const struct sigevent *i82544_isr_rx_kermask (void *arg, int iid)

{
i82544_dev_t		*i82544 = arg;
struct _iopkt_inter	*ient = &i82544->inter;

	if (ient->on_list == 0 &&
	    (i82544->irq_cause = E1000_READ_REG (&i82544->hw, E1000_ICR)) == 0) {
		/* IRQ not caused by this card. */
		ient->spurious++;
		return NULL;
		}


	/*
	 * Close window where this is referenced in
	 * i82544_enable_interrupt_kermask().  We may get
	 * an interrupt, return a sigevent and have
	 * another thread start processing on SMP before
	 * the InterruptAttach() has returned.
	 */
	i82544->iid = iid;

	InterruptMask (i82544->cfg.irq[0], iid);

	return interrupt_queue (i82544->iopkt, ient);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

void	i82544_hk_callout(void *arg)

{
i82544_dev_t			*i82544 = arg;
struct ifnet			*ifp = &i82544->ecom.ec_if;
struct _iopkt_self		*iopkt = i82544->iopkt;
struct nw_work_thread	*wtp = WTP;

	// if the transmit side is quiet, process txd descriptors now
	if (!i82544->start_running) {
		NW_SIGLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
		i82544->hk = 1;
		i82544_reap (i82544);
		i82544->hk = 0;
		NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
		}
	/* This code has been added to overcome a problem with 82577 adapters
	   where the line doesn't come up occasionally. */
	if (! (i82544->linkup) && i82544->hw.mac.type == e1000_pchlan) {
		i82544->hw.mac.get_link_status = 1;
		update_link_status (i82544);
		}
	callout_msec (&i82544->hk_callout, 1 * 1000, i82544_hk_callout, i82544);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

int		i82544_enable_interrupt (void *arg)

{
i82544_dev_t *i82544 = arg;

	if (i82544->msi_enabled == PCI_MSIX) {
		E1000_WRITE_REG (&i82544->hw, E1000_IMS, E1000_ICR_RXQ0 | E1000_ICR_OTHER);
		}
	else {
		E1000_WRITE_REG (&i82544->hw, E1000_IMS, i82544->intrmask);
		}
	return 1;
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

int		i82544_enable_interrupt_kermask (void *arg)

{
i82544_dev_t *i82544 = arg;

	InterruptUnmask (i82544->cfg.irq[0], i82544->iid);
	if (i82544->msi_enabled == PCI_MSIX) {
		InterruptUnmask (i82544->cfg.irq[2], i82544->iid_lnk);
		}
	return 1;
}



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/event.c $ $Rev: 751115 $")
#endif
