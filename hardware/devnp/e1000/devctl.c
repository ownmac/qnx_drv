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


#include <i82544.h>
#include <net/ifdrvcom.h>
#include <sys/sockio.h>

/*****************************************************************************/
/* Only called by stack thread                                               */
/*****************************************************************************/

void	i82544_filter (i82544_dev_t *i82544)

{
struct ethercom		*ec;
struct ifnet		*ifp;
struct ether_multi	*enm;
struct ether_multistep step;
struct e1000_hw		*hw = &i82544->hw;
uint32_t			hash, reg, bit, rctl, rctl_bsize;
int					i;

	ec = &i82544->ecom;
	ifp = &ec->ec_if;

	ifp->if_flags &= ~IFF_ALLMULTI;
	
	//
	// The default stack cluster size is 2k.  But
	// this can be changed using the mclbytes and
	// pagesize options to the stack, eg:
	//
	//   "mclbytes=8192,pagesize=8192"
	//
	// which results in MCLBYTES being bumped up to 
	// a value which is a power of 2, max 16k
	//
	// The advantage of this is that a jumbo packet (for
	// a particular mtu) can be passed up in a single cluster,
	// which reduces rx fragment processing by the stack 
	// and has been observed to increase throughput.
	//
	switch (MCLBYTES) {
		case	(4 * 1024):
			rctl_bsize = E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX;
			break;
		case	(8 * 1024):
			rctl_bsize = E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX;
			break;
		case	(16 * 1024):
			rctl_bsize = E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX;
			break;
		default:
			rctl_bsize = E1000_RCTL_SZ_2048;
			break;		
		}
	if (hw->mac.type == e1000_82575 || hw->mac.type == e1000_82576 ||
		hw->mac.type == e1000_82580) {
		rctl_bsize = E1000_RCTL_SZ_2048;
		}

	switch (i82544_MO_TYPE) {
		case	0:
			rctl = E1000_RCTL_MO_0;
			break;
		case	1:
			rctl = E1000_RCTL_MO_1;
			break;
		case	2:
			rctl = E1000_RCTL_MO_2;
			break;
		case	3:
			rctl = E1000_RCTL_MO_3;
			break;
		}

	rctl |= E1000_RCTL_SECRC | E1000_RCTL_BAM | E1000_RCTL_EN | E1000_RCTL_LBM_NO |
			E1000_RCTL_RDMTS_HALF | E1000_RCTL_LPE | E1000_RCTL_DPF | rctl_bsize;

	if (ifp->if_flags & IFF_PROMISC) {
		rctl |= E1000_RCTL_UPE;
		i82544->cfg.flags |= NIC_FLAG_PROMISCUOUS;
allmulti:
		rctl |= E1000_RCTL_MPE;
		ifp->if_flags |= IFF_ALLMULTI;

		E1000_WRITE_REG (&i82544->hw, E1000_RCTL, rctl);

		return;
		}

	i82544->cfg.flags &= ~NIC_FLAG_PROMISCUOUS;

	/* Clear out the multicast table. */
	for (i = 0; i < hw->mac.mta_reg_count; i++)
		E1000_WRITE_REG_ARRAY (hw, E1000_MTA,i, 0);

	ETHER_FIRST_MULTI (step, ec, enm);
	while (enm != NULL) {
		if (memcmp (enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			/*
			 * We must listen to a range of multicast addresses.
			 * For now, just accept all multicasts, rather than
			 * trying to set only those filter bits needed to match
			 * the range.  (At this time, the only use of address
			 * ranges is for IP multicast routing, for which the
			 * range is big enough to require all bits set.)
			 */
			goto allmulti;
			}

		hash = e1000_hash_mc_addr_generic (hw, enm->enm_addrlo);

		reg = (hash >> 5) & (hw->mac.mta_reg_count - 1);
		bit = hash & 0x1f;

		hash = E1000_READ_REG_ARRAY (hw, E1000_MTA, reg);
		hash |= 1U << bit;

		/* Hardware bug?? */
		if ((reg & 0xe) == 1) {
			bit = E1000_READ_REG_ARRAY (hw, E1000_MTA, (reg - 1));
			E1000_WRITE_REG_ARRAY (hw, E1000_MTA, reg, hash);
			E1000_WRITE_REG_ARRAY (hw, E1000_MTA, (reg - 1), bit);
			}
		else
			E1000_WRITE_REG_ARRAY (hw, E1000_MTA, reg, hash);

		ETHER_NEXT_MULTI (step, enm);
		}

	E1000_WRITE_REG (hw, E1000_RCTL, rctl);

	return;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static void	update_stats (i82544_dev_t *i82544)

{
nic_stats_t				*gstats = &i82544->stats;
nic_ethernet_stats_t	*estats = &i82544->stats.un.estats;
struct	e1000_hw		*hw = &i82544->hw;

	gstats->octets_txed_ok += E1000_READ_REG (hw, E1000_GOTCL) |
	    (uint64_t)E1000_READ_REG (hw, E1000_GOTCH) << 32;
	gstats->txed_ok += E1000_READ_REG (hw, E1000_GPTC);
	gstats->txed_multicast += E1000_READ_REG (hw, E1000_MPTC);
	gstats->txed_broadcast += E1000_READ_REG (hw, E1000_BPTC);
	estats->single_collisions += E1000_READ_REG (hw, E1000_COLC);
	estats->no_carrier += E1000_READ_REG (hw, E1000_TNCRS);
	estats->tx_deferred += E1000_READ_REG (hw, E1000_DC);
	estats->xcoll_aborted += E1000_READ_REG (hw, E1000_ECOL);
	estats->late_collisions += E1000_READ_REG (hw, E1000_LATECOL);

	gstats->octets_rxed_ok += E1000_READ_REG (hw, E1000_GORCL) |
	    (uint64_t)E1000_READ_REG (hw, E1000_GORCH) << 32;
	gstats->rxed_ok += E1000_READ_REG (hw, E1000_GPRC);
	gstats->rxed_multicast += E1000_READ_REG (hw, E1000_MPRC);
	gstats->rxed_broadcast += E1000_READ_REG (hw, E1000_BPRC);
	estats->align_errors += E1000_READ_REG (hw, E1000_ALGNERRC);
	estats->internal_rx_errors += E1000_READ_REG (hw, E1000_RNBC);
	estats->fcs_errors += E1000_READ_REG (hw, E1000_CRCERRS);
	estats->oversized_packets += E1000_READ_REG (hw, E1000_ROC);
	estats->symbol_errors += E1000_READ_REG (hw, E1000_SYMERRS);
	estats->jabber_detected += E1000_READ_REG (hw, E1000_RJC);
	estats->short_packets += E1000_READ_REG (hw, E1000_RUC);
	estats->sqe_errors += E1000_READ_REG (hw, E1000_MPC);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

int		i82544_ioctl (struct ifnet * ifp, unsigned long cmd, caddr_t data)

{
int						error = 0;
i82544_dev_t			*i82544 = ifp->if_softc;
struct drvcom_config	*dcfgp;
struct drvcom_stats		*dstp;
struct ifdrv_com		*ifdc;

	switch (cmd) {
		case	SIOCGDRVCOM:
			ifdc = (struct ifdrv_com *)data;
			switch (ifdc->ifdc_cmd) {
				case	DRVCOM_CONFIG:
					dcfgp = (struct drvcom_config *)ifdc;

					if (ifdc->ifdc_len != sizeof (nic_config_t)) {
						error = EINVAL;
						break;
						}
					memcpy (&dcfgp->dcom_config, &i82544->cfg, sizeof (i82544->cfg));
					break;

				case	DRVCOM_STATS:
					dstp = (struct drvcom_stats *)ifdc;

					if (ifdc->ifdc_len != sizeof (nic_stats_t)) {
						error = EINVAL;
						break;
						}
					update_stats (i82544);
					memcpy (&dstp->dcom_stats, &i82544->stats, sizeof (i82544->stats));
					break;

				default:
					error = ENOTTY;
				}
			break;


	    case	SIOCSIFMEDIA:
	    case	SIOCGIFMEDIA: {
			struct ifreq *ifr = (struct ifreq *)data;

	        error = ifmedia_ioctl (ifp, ifr, &i82544->bsd_mii.mii_media, cmd);
	        break;
			}

		default:
			error = ether_ioctl (ifp, cmd, data);
			if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the
			 * hardware filter accordingly.
			 */
				i82544_filter (i82544);
				error = 0;
				}
			break;
		}

	return error;
}



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/devctl.c $ $Rev: 736526 $")
#endif
