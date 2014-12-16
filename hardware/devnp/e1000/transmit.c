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

#include "i82544.h"

#if NBPFILTER > 0
#include <net/bpf.h>
#include <net/bpfdesc.h>
#endif

#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/tcp.h>
#include <net/if_vlanvar.h>

static	inline	uint16_t in_pseudo (uint32_t a, uint32_t b, uint32_t c)
{
uint64_t	sum;

	sum = (uint64_t) a + b + c;
	sum = (sum & 0xffff) + (sum >> 16);
	if (sum > 0xffff)
		sum -= 0xffff;
	return (sum);
}

#define	CSUM_OFFLOAD	(M_CSUM_IPv4 | M_CSUM_TCPv4 | M_CSUM_UDPv4 | \
                         M_CSUM_TCPv6 | M_CSUM_UDPv6)

/*****************************************************************************/
/* this function is called only if the packet is ridiculously                */
/* fragmented.  If we are lucky, the entire packet will fit into             */
/* one cluster so we manually defrag.  However, if this is a jumbo           */
/* packet, and io-pkt is running without large values for mclbytes           */
/* and pagesize, we are left with no choice but to dup the packet            */
/*****************************************************************************/

static struct mbuf *i82544_defrag (struct mbuf *m)

{
struct mbuf	*m2;

	if (m->m_pkthdr.len > MCLBYTES) {
		//
		// this is a jumbo packet which just happens to
		// be larger than a single cluster, which means
		// that the normal stuff below wont work.  So,
		// we try a "deep" copy of this packet, which is
		// time-consuming, but is probably better than
		// just dropping the packet
		//
		int num_frag;

		m2 = m_dup (m, 0, m->m_pkthdr.len, M_DONTWAIT);
		if (!m2) {
			m_freem (m);
			return NULL;
			}
		m = m2;  // m is now the defragged packet

		// ensure that dupd packet is not excessively fragmented
		for (num_frag=0, m2=m; m2; num_frag++) {
			m2 = m2->m_next;
			}
		if (num_frag > I82544_MAX_FRAGS) {
			// this should never happen
			m_freem (m);
			return NULL;
			}

		return m;
		}

	// the entire packet should fit into one cluster
	MGET (m2, M_DONTWAIT, MT_DATA);
	if (m2 == NULL) {
		m_freem (m);
		return NULL;
		}

	M_COPY_PKTHDR (m2, m);

	MCLGET (m2, M_DONTWAIT);
	if ((m2->m_flags & M_EXT) == 0) {
		m_freem (m);
		m_freem (m2);
		return NULL;
		}

	// this is NOT paranoid - this can happen with jumbo packets bigger
	// than a single cluster - should be handled above
	if (m->m_pkthdr.len > m2->m_ext.ext_size) {
		m_freem (m);
		m_freem (m2);
		return NULL;
		}

	m_copydata (m, 0, m->m_pkthdr.len, mtod (m2, caddr_t));
	m2->m_pkthdr.len = m2->m_len = m->m_pkthdr.len;

	m_freem (m);

	return m2;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void	i82544_reap (i82544_dev_t *i82544)

{
int				cur_tx_rptr = i82544->cur_tx_rptr;
struct ifnet	*ifp = &i82544->ecom.ec_if;
struct mbuf		*m;
int				i, to_reap = 0, cnt = 0;
i82544_tdesc_t	*desc;

	if (i82544->tx_free == i82544->num_transmit)
		return;
	for (i = 0, cnt = cur_tx_rptr; ; i++) {
		desc = &i82544->tdesc [cnt];
		if (desc->opts_stat & ENDIAN_LE32 (E1000_TXD_STAT_DD))
			to_reap++;
		cnt = (cnt + 1) % i82544->num_transmit;
		if (cnt == i82544->cur_tx_wptr)
			break;
		}
	if (to_reap == 0)
		return;
	cnt = 0;
	while (to_reap) {
		desc = &i82544->tdesc [cur_tx_rptr];
		if ((m = i82544->tx_mbuf[cur_tx_rptr]) != NULL) {
			m_freem (m);
			ifp->if_opackets++;
			i82544->tx_mbuf[cur_tx_rptr] = NULL;
			}
		if (desc->opts_stat & ENDIAN_LE32 (E1000_TXD_STAT_DD))
			to_reap--;
		desc->opts_stat = 0;
		desc->bufaddr = 0;
		cur_tx_rptr = (cur_tx_rptr + 1) % i82544->num_transmit;
		i82544->tx_free++;
		/* This is to avoid the reap routine looping for too long */
		if (++cnt == i82544->tx_reap)
			break;
		}

	i82544->cur_tx_rptr = cur_tx_rptr;

	if (ifp->if_flags_tx & IFF_OACTIVE) {
		if (i82544->tx_free >= (I82544_MAX_FRAGS + I82544_UNUSED_DESCR)) {
			ifp->if_flags_tx &= ~IFF_OACTIVE;
			if (!i82544->start_running)
				i82544_start (ifp);
			}
		}
}

/*****************************************************************************/
/* perform i82544 hardware checksum offload and                              */
/* transmit segmentation offload setup as instructed by stack                */
/*****************************************************************************/

int	i82544_offload_setup (i82544_dev_t *i82544, struct mbuf *m0, int offload_flags,
							uint32_t *cmdp, uint32_t *fieldsp)
							
{
int						mac_offset=0;  	// offset into mbuf, past ethernet mac header
int						mac_hdrlen=0;  	// length of ethernet mac header
int						ip_hdrlen=0;	// length of IP header
int						tcp_hdrlen=0; 	// length of TCP header
struct ether_header		*eh;
struct mbuf 			*m;
uint16_t				etype=0;
uint32_t				ipcs_ctxt = i82544->last_ipcs_ctxt;  // reload previous value
uint32_t				tucs_ctxt = i82544->last_tucs_ctxt;  // reload previous value
//uint32_t				cmdlen = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_C | E1000_TXD_CMD_RS;  // context desr below
uint32_t				cmdlen = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_C;  // context desr below
uint8_t					total_hdrlen=0;
uint16_t				mss=0;
int						min_hdrlen=0;


	//
	// get the IP header length and figure out the minimum IP/TCP header
	// length which will allow us to fudge the TCP header for TSO
	//
	if (offload_flags & (M_CSUM_IPv4 | M_CSUM_TCPv4 | M_CSUM_UDPv4 | M_CSUM_TSOv4)) {
		ip_hdrlen  = M_CSUM_DATA_IPv4_IPHL (m0->m_pkthdr.csum_data);
		min_hdrlen = sizeof (struct ip) + sizeof (struct tcphdr);

		}
	else if (offload_flags & (M_CSUM_TCPv6 | M_CSUM_UDPv6 | M_CSUM_TSOv6)) {
		ip_hdrlen = M_CSUM_DATA_IPv6_HL (m0->m_pkthdr.csum_data);
		min_hdrlen = sizeof (struct ip6_hdr) + sizeof (struct tcphdr);

		}
	else {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): unsupported hardware offload flags: 0x%X",
				__FUNCTION__, offload_flags);
		return -1;
		}

	/* Ethernet header will always be in first frag */
	eh = mtod (m0, struct ether_header *);
	
	/* Get in host order */
	etype = ENDIAN_BE16 (eh->ether_type);
	
	// how long is the ethernet mac headers?
	switch (etype) {
		case	ETHERTYPE_IP:
		case	ETHERTYPE_IPV6:
			mac_offset = mac_hdrlen = ETHER_HDR_LEN;
			break;
	
		case	ETHERTYPE_VLAN:
			mac_offset = mac_hdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
			break;
	
		default:
		    slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): hw csum unsupported etype 0x%X",
					__FUNCTION__, etype);
			return -1;
			break;
		}
	
	//	
	// skip over the 14 byte ethernet header (18 if vlan)	
	//	
	for (m=m0; m && (m->m_len <= mac_offset); m=m->m_next) {
		mac_offset -= m->m_len;
		}
	if (!m) {
		// ridiculously short packet - should never happen
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): dropped short packet: mac_offset %d  mac_hdrlen %d",
					__FUNCTION__, mac_offset, mac_hdrlen);
		return -1;
		}
	
	// stuff these bits into data descriptors in i82544_start()
	*cmdp = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;

	//
	// Transmit Segmentation Offload setup, both IPv4 and IPv6
	//
	if (offload_flags & (M_CSUM_TSOv4 | M_CSUM_TSOv6)) {
		struct tcphdr  *th=0;  // N.B. we assume always TCP with TSO

		//
		// ensure that what remains of the ethernet mac header, the 
		// IPv4 (or IPv6) header and the TCP header is contiguous 
		// within the current mbuf, because we need to write to them
		//
		if (m->m_len < (mac_offset + min_hdrlen)) {
			//
			// XXX fix me - do the ugly mbuf coalescing to
			// get the IP and TCP headers in the same mbuf
			//
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): dropped fragmented mbuf header packet: "
					"mac_offset %d  m->m_len %d  min_hdrlen %d", 
					__FUNCTION__, mac_offset, m->m_len, min_hdrlen);
			return -1;
			}

		if (offload_flags & M_CSUM_TSOv4) {
			struct ip  *ip = (void *)(mtod(m, caddr_t) + mac_offset);
	
			// fudge headers as required by hardware
			ip->ip_len = 0;
	
			th = (void *)((char *)ip + ip_hdrlen);
			th->th_sum = in_cksum_phdr (ip->ip_src.s_addr, ip->ip_dst.s_addr, htons(IPPROTO_TCP));

			}
		else {  // TSOv6
			struct ip6_hdr  *ip6 = (void *)(mtod (m, caddr_t) + mac_offset);

			// fudge headers as required by hardware
			ip6->ip6_plen = 0;

			th = (void *)((char *)ip6 + ip_hdrlen);
			th->th_sum = in6_cksum_phdr (&ip6->ip6_src, &ip6->ip6_dst, 0, htonl(IPPROTO_TCP));

			}

		// fish out tcp header length
		tcp_hdrlen = th->th_off << 2;

		// enable TSO in data descriptors in i82544_start()
		// and ask for ethernet checksum in FIRST packet
		*cmdp |= E1000_TXD_CMD_TSE | E1000_TXD_CMD_IFCS;

		//
		// set values for context descriptor below.  Note 
		// that total_hdrlen, mss and cmdlen are declared
		// above with exactly the same width as the descriptor
		// fields.  We hope that no overflow/truncation
		// occurs here.  Specifically, we hope that that
		//
		// - total_hdrlen does not try to exceed 256, and
		// - mss does not try to exceed 64k, and
		// - payload length in cmdlen does not try to exceed 1 MB
		//
		total_hdrlen = mac_hdrlen + ip_hdrlen + tcp_hdrlen;
		mss = m0->m_pkthdr.segsz;
		cmdlen |= E1000_TXD_CMD_TSE | E1000_TXD_CMD_TCP |
				CS_CTXT_PAYLEN (m0->m_pkthdr.len - total_hdrlen);

		if (offload_flags & M_CSUM_TSOv4) {
			cmdlen |= E1000_TXD_CMD_IP;  // N.B.  not set means IPv6	
			}
		}

	//
	// IP checksum offload setup
	//
	// N.B.  If we are doing TSO, chip requires that
	// we also set up hardware checksumming
	//
	if (offload_flags & (M_CSUM_IPv4 | M_CSUM_TSOv4)) {
		ipcs_ctxt = CS_CTXT_START (mac_hdrlen) |
		    CS_CTXT_OFF (mac_hdrlen + offsetof (struct ip, ip_sum)) |
		    CS_CTXT_END (mac_hdrlen + ip_hdrlen - 1);

		*fieldsp |= (E1000_TXD_POPTS_IXSM << E1000_TXD_POPTS_SHIFT);

		}
	else if (offload_flags & M_CSUM_TSOv6) { 
		ipcs_ctxt = CS_CTXT_START (mac_hdrlen) |
		    CS_CTXT_OFF (mac_hdrlen + offsetof (struct ip, ip_sum)) |
		    CS_CTXT_END (0);

		// IXSM bit is NOT set - there is no checksum in the IPv6 header!
		}

	//
	// TCP/UDP checksum offload setup
	//
	// N.B.  If we are doing TSO, chip requires that
	// we also set up hardware checksumming.
	//
	// N.B.  This is a bit silly - both of the csum_data OFFSET
	// macros do the same thing - bottom 16 bits - but in case
	// they ever change in the future, we will duplicate the code
	//
	if (offload_flags & (M_CSUM_TCPv4 | M_CSUM_UDPv4 | M_CSUM_TSOv4)) {
		int hdr_offset = mac_hdrlen + ip_hdrlen;

		tucs_ctxt = CS_CTXT_START (hdr_offset) |
		  CS_CTXT_OFF (hdr_offset + M_CSUM_DATA_IPv4_OFFSET (m0->m_pkthdr.csum_data)) |
		  CS_CTXT_END (0); /* rest of packet */

		*fieldsp |= (E1000_TXD_POPTS_TXSM << E1000_TXD_POPTS_SHIFT);

		}
	else if (offload_flags & (M_CSUM_TCPv6 | M_CSUM_UDPv6 | M_CSUM_TSOv6)) {
		int hdr_offset = mac_hdrlen + ip_hdrlen;

		tucs_ctxt = CS_CTXT_START (hdr_offset) |
		  CS_CTXT_OFF (hdr_offset + M_CSUM_DATA_IPv6_OFFSET (m0->m_pkthdr.csum_data)) |
		  CS_CTXT_END (0); /* rest of packet */

		*fieldsp |= (E1000_TXD_POPTS_TXSM << E1000_TXD_POPTS_SHIFT);
		}

	//
	// We load a context descriptor every time we get a packet that 
	// has different specifications than the last one.  This sets up 
	// the devices internal transmit checksum context for all subsequent 
	// packets.
	//

	if (ipcs_ctxt != i82544->last_ipcs_ctxt || 
		tucs_ctxt != i82544->last_tucs_ctxt ||
        cmdlen != i82544->last_cmdlen || 
		total_hdrlen != i82544->last_total_hdrlen || mss != i82544->last_mss) {

		// grab next transmit descriptor
		i82544_tx_cs_ctx_t  *tdesc = (void *)&i82544->tdesc[i82544->cur_tx_wptr];

		// step over this one
		i82544->cur_tx_wptr = (i82544->cur_tx_wptr + 1) % i82544->num_transmit;
		i82544->tx_free--;
	
		// finally, after all this incredible setup, load context descriptor
		tdesc->ipcs_ctxt.ipcs_ctxt = ENDIAN_LE32 (ipcs_ctxt);
		tdesc->tucs_ctxt.tucs_ctxt = ENDIAN_LE32 (tucs_ctxt);
		tdesc->tucmd_type_len = ENDIAN_LE32 (cmdlen);
		tdesc->hdrlen = total_hdrlen;  // single byte, no endian headache
		tdesc->mss = ENDIAN_LE16 (mss);
	
		if (i82544->cfg.verbose > 2) {
  	        slogf (_SLOGC_NETWORK, _SLOG_WARNING, 
			  "%s(): hardware checksum/TSO context descriptor written: " 
			  "offload_flags 0x%X, etype 0x%X, "
			  "mac_offset %d, mac_hdrlen %d, ip_hdrlen %d, tcp_hdrlen %d, "   
			  "ipcs_ctxt 0x%X, tucs_ctxt 0x%X, cmdlen 0x%X, total_hdrlen %d, mss %d, "
			  "*cmdp %X, *fieldsp %X",
			  __FUNCTION__, offload_flags, etype, 
			  mac_offset, mac_hdrlen, ip_hdrlen, tcp_hdrlen, 
			  ipcs_ctxt, tucs_ctxt, cmdlen, total_hdrlen, mss, 
			  cmdp ? *cmdp : 0, fieldsp ? *fieldsp : 0);
			}

		// save current context descriptor values for comparison next time we are called
		i82544->last_ipcs_ctxt = ipcs_ctxt;
		i82544->last_tucs_ctxt = tucs_ctxt;
		i82544->last_cmdlen = cmdlen;
		i82544->last_total_hdrlen = total_hdrlen;
		i82544->last_mss = mss;
		}

	return 0;  // worked ok
}

/*****************************************************************************/
/* Extended descriptor format offload setup                                  */
/*****************************************************************************/

int		i82544_ctx_setup (i82544_dev_t *i82544, struct mbuf *m)

{
struct 		e1000_adv_tx_context_desc *txd;
uint32_t	vlan_macip_lens, type_tucmd_mlhl, mss_l4len_idx;
struct		ether_vlan_header *eh;
struct		ip *ip = NULL;
struct		ip6_hdr *ip6;
struct		m_tag *mtag;
int			ehdrlen, cur_tx_wptr, ip_hlen = 0;
uint16_t	etype, vtag = 0;
uint8_t		ipproto = 0;
bool		offload = TRUE;

	if ((m->m_pkthdr.csum_flags & CSUM_OFFLOAD) == 0)
		offload = FALSE;

	vlan_macip_lens = type_tucmd_mlhl = mss_l4len_idx = 0;
	cur_tx_wptr = i82544->cur_tx_wptr;
	txd = (struct e1000_adv_tx_context_desc *) &i82544->tdesc [cur_tx_wptr];

	/*
	** In advanced descriptors the vlan tag must 
	** be placed into the context descriptor, thus
	** we need to be here just for that setup.
	*/
	mtag = m_tag_find (m, PACKET_TAG_VLAN, NULL);
	if (mtag != NULL) {
		vtag = ENDIAN_LE16 (VLAN_TAG_VALUE (mtag));
		vlan_macip_lens |= (vtag << E1000_ADVTXD_VLAN_SHIFT);
		}
	else if (offload == FALSE)
		return FALSE;

	/*
	 * Determine where frame payload starts.
	 * Jump over vlan headers if already present,
	 * helpful for QinQ too.
	 */
	eh = mtod (m, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
		etype = ntohs(eh->evl_proto);
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
		}
	else {
		etype = ntohs(eh->evl_encap_proto);
		ehdrlen = ETHER_HDR_LEN;
		}

	/* Set the ether header length */
	vlan_macip_lens |= ehdrlen << E1000_ADVTXD_MACLEN_SHIFT;

	switch (etype) {
		case ETHERTYPE_IP:
			ip = (struct ip *)(m->m_data + ehdrlen);
			ip_hlen  = M_CSUM_DATA_IPv4_IPHL (m->m_pkthdr.csum_data);
			if (m->m_len < ehdrlen + ip_hlen) {
				if(m->m_len == ehdrlen && m->m_next && m->m_next->m_len >= ip_hlen)
					ip = (struct ip *)(m->m_next->m_data); /* IP4 hdr in next mbuf */
				else {
					offload = FALSE;
					break;
					}
				}
			ipproto = ip->ip_p;
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV4;
			break;
		case ETHERTYPE_IPV6:
			ip6 = (struct ip6_hdr *)(m->m_data + ehdrlen);
			ip_hlen = sizeof(struct ip6_hdr);
			if (m->m_len < ehdrlen + ip_hlen) {
				if(m->m_len == ehdrlen && m->m_next && m->m_next->m_len >= ip_hlen)
					ip6 = (struct ip6_hdr *)(m->m_next->m_data); /* IP6 hdr in next mbuf */
				else
					return (FALSE);
				}
			ipproto = ip6->ip6_nxt;
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV6;
			break;
		default:
			offload = FALSE;
			break;
		}

	vlan_macip_lens |= ip_hlen;
	type_tucmd_mlhl |= E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;

	switch (ipproto) {
		case IPPROTO_TCP:
			if (m->m_pkthdr.csum_flags & M_CSUM_TCPv4)
				type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_TCP;
			break;

		case IPPROTO_UDP:
			if (m->m_pkthdr.csum_flags & M_CSUM_UDPv4)
				type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_UDP;
			break;

		case IPPROTO_ICMP:
			offload = TRUE;
			break;

		default:
			offload = FALSE;
			break;
		}

	/* 82575 needs the queue index added */
//	if (i82544->hw.mac.type == e1000_82575)
//		mss_l4len_idx = txr->me << 4;

	/* Now copy bits into descriptor */
	txd->vlan_macip_lens = ENDIAN_LE32 (vlan_macip_lens);
	txd->type_tucmd_mlhl = ENDIAN_LE32 (type_tucmd_mlhl);
	txd->seqnum_seed = 0;
	txd->mss_l4len_idx = ENDIAN_LE32 (mss_l4len_idx);

	/* We've consumed the first desc, adjust counters */
	cur_tx_wptr = (cur_tx_wptr + 1) % i82544->num_transmit;
	i82544->cur_tx_wptr = cur_tx_wptr;
	i82544->tx_free -= 1;
	return (offload);
//	return (1);
}

/*****************************************************************************/
/* Extended descriptor format offload setup                                  */
/*****************************************************************************/

int		i82544_tso_setup (i82544_dev_t *i82544, struct mbuf *m, uint32_t *hdrlen)

{
struct	e1000_adv_tx_context_desc	*txd;
struct	ether_vlan_header			*eh;
struct	ip							*ip;
struct	tcphdr						*th;
int		ehdrlen, ip_hlen, tcp_hlen;
int		cur_tx_wptr;
uint32_t	vlan_macip_lens = 0, type_tucmd_mlhl = 0;
uint32_t	mss_l4len_idx = 0;

	eh = mtod (m, struct ether_vlan_header *);
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN))
		ehdrlen = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
	else
		ehdrlen = ETHER_HDR_LEN;
	if (m->m_len < ehdrlen + sizeof (struct ip) + sizeof (struct tcphdr))
		return (0);
	cur_tx_wptr = i82544->cur_tx_wptr;
	txd = (struct e1000_adv_tx_context_desc *) &i82544->tdesc [cur_tx_wptr];

	ip = (struct ip *)(m->m_data + ehdrlen);
	if (ip->ip_p != IPPROTO_TCP)
		return (0);
	ip->ip_sum = 0;
	ip_hlen = ip->ip_hl << 2;
	th = (struct tcphdr *)((caddr_t) ip + ip_hlen);
	th->th_sum = in_pseudo (ip->ip_src.s_addr, ip->ip_dst.s_addr, htons (IPPROTO_TCP));
	tcp_hlen = th->th_off << 2;
	*hdrlen = ehdrlen + ip_hlen + tcp_hlen;
	vlan_macip_lens |= (ehdrlen << E1000_ADVTXD_MACLEN_SHIFT);
	vlan_macip_lens |= ip_hlen;
	txd->vlan_macip_lens = ENDIAN_LE32 (vlan_macip_lens);

	/* ADV DTYPE TUCMD */
	type_tucmd_mlhl |= E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;
	type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_TCP;
	type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_IPV4;
	txd->type_tucmd_mlhl = ENDIAN_LE32 (type_tucmd_mlhl);

	/* MSS L4LEN IDX */
	mss_l4len_idx |= (m->m_pkthdr.segsz << E1000_ADVTXD_MSS_SHIFT);
	mss_l4len_idx |= (tcp_hlen << E1000_ADVTXD_L4LEN_SHIFT);
		
	txd->mss_l4len_idx = ENDIAN_LE32 (mss_l4len_idx);

	txd->seqnum_seed = 0;

	cur_tx_wptr = (cur_tx_wptr + 1) % i82544->num_transmit;
	i82544->cur_tx_wptr = cur_tx_wptr;
	i82544->tx_free -= 1;

	return (1);
}

/*****************************************************************************/
/* Extended descriptor format transmit routine                               */
/*****************************************************************************/

void	i82544_xmit_ext (i82544_dev_t *i82544, struct ifnet *ifp, struct mbuf *m0, int off_flags, int num_frag)

{
uint32_t	cmd_type_len = 0;
uint32_t	hdrlen = 0;
uint32_t	olinfo_status = 0;
//uint16_t	etype = 0;
int			cur_tx_wptr, free_wptr = 0;
off64_t		phys;
struct		mbuf *m;
union		e1000_adv_tx_desc	*txd = NULL;
//struct		ether_header	*eh;

	/* Set basic descriptor constants */
	cmd_type_len |= E1000_ADVTXD_DTYP_DATA;
	cmd_type_len |= E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;

	/* Ethernet header will always be in first frag */
//	eh = mtod (m0, struct ether_header *);
	
	/* Get in host order */
//	etype = ENDIAN_BE16 (eh->ether_type);
//	if (etype == ETHERTYPE_VLAN)
//		cmd_type_len |= E1000_ADVTXD_DCMD_VLE;

	if (off_flags & (M_CSUM_TSOv4 | M_CSUM_TSOv6)) {
		if (i82544_tso_setup (i82544, m0, &hdrlen)) {
			cmd_type_len |= E1000_ADVTXD_DCMD_TSE;
			olinfo_status |= E1000_TXD_POPTS_IXSM << 8;
			olinfo_status |= E1000_TXD_POPTS_TXSM << 8;
			}
		else {
			return;
			}
		}
	else {
		if (i82544_ctx_setup (i82544, m0)) {
			if (m0->m_pkthdr.csum_flags & M_CSUM_TCPv4)
				olinfo_status |= E1000_TXD_POPTS_TXSM << 8;
			if (m0->m_pkthdr.csum_flags & M_CSUM_IPv4)
				olinfo_status |= E1000_TXD_POPTS_IXSM << 8;
			if (m0->m_pkthdr.csum_flags & M_CSUM_UDPv4)
				olinfo_status |= E1000_TXD_POPTS_TXSM << 8;
			}
		}
	/* Calculate payload length */
	olinfo_status |= ((m0->m_pkthdr.len - hdrlen) << E1000_ADVTXD_PAYLEN_SHIFT);

	cur_tx_wptr = i82544->cur_tx_wptr;
	for (m = m0; m; m = m->m_next) {
		if (!m->m_len) {
		// harmless except when hw csum is turned on
			num_frag--;  // must adjust for free descr count below
			continue;
			}
		txd = (union e1000_adv_tx_desc *) &i82544->tdesc [cur_tx_wptr];
		phys = mbuf_phys (m);
		CACHE_FLUSH (&i82544->cachectl, m->m_data, phys, m->m_len);
		txd->read.buffer_addr = ENDIAN_LE64 (phys + i82544->bmtrans);
		txd->read.cmd_type_len = ENDIAN_LE32 (m->m_len | cmd_type_len | E1000_TXD_CMD_IFCS);
		txd->read.olinfo_status = ENDIAN_LE32 (olinfo_status);
		free_wptr = cur_tx_wptr;
		cur_tx_wptr = (cur_tx_wptr + 1) % i82544->num_transmit;
		}
	/* Last descriptor requires EOP and RS */
	txd->read.cmd_type_len |= ENDIAN_LE32 (E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);

	/* Update the write pointer - submits packet for transmission */
	E1000_WRITE_REG (&i82544->hw, E1000_TDT(0), cur_tx_wptr);

	i82544->cur_tx_wptr = cur_tx_wptr;

	/* Store a pointer to the packet for freeing later */
	i82544->tx_mbuf[free_wptr] = m0;

	i82544->tx_free -= num_frag;
		
#if NBPFILTER > 0
	/* Pass the packet to any BPF listeners. */
	if (ifp->if_bpf) {
		bpf_mtap(ifp->if_bpf, m0);
		}
#endif
}

/*****************************************************************************/
/* Entry point for Tx from the upper level                                   */
/*****************************************************************************/

void	i82544_start (struct ifnet *ifp)

{
i82544_dev_t		*i82544 = ifp->if_softc;
i82544_tdesc_t		*tdesc = NULL;
int					free_wptr = free_wptr;
int					offload_flags;
int					cur_tx_wptr;
int					hdrlen = hdrlen; /* silence gcc warning */
int					offset = offset; /* silence gcc warning */
struct mbuf 		*m0, *m;
uint32_t			cmd, fields;
off64_t				phys;
struct _iopkt_self	*iopkt = i82544->iopkt;
struct nw_work_thread	*wtp = WTP;
struct	e1000_hw	*hw = &i82544->hw;
int            		num_frag;

	if ((ifp->if_flags_tx & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING) {
		if (!i82544->hk) {
			NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
			}
		return;
		}

	if (!i82544->linkup) {
		if (!i82544->hk) {
			NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
			}
		return;
		}

	i82544->start_running = 1;

	for (;;) {

		// only call reap if we cannot transmit
        if (i82544->tx_free < (I82544_MAX_FRAGS + I82544_UNUSED_DESCR)) {

            // see if we can clean out txd pkts from descr ring
			i82544_reap (i82544);

            // did we free any up?
            if (i82544->tx_free < (I82544_MAX_FRAGS + I82544_UNUSED_DESCR)) {

				ifp->if_flags_tx |= IFF_OACTIVE;

				i82544->stats.tx_failed_allocs++;
                goto done; // not enough tx descriptors, try later
    	        }
	        }

		IFQ_DEQUEUE (&ifp->if_snd, m0);
		if (!m0) {
			goto done;  // nothing to tx, we go home now
			}

        // count up mbuf fragments
        for (num_frag = 0, m = m0; m; num_frag++) {
            m = m->m_next;
	        }

        // ridiculously fragmented?
        if (num_frag > I82544_MAX_FRAGS) {
			//
			// This should very rarely (hopefully never) happen
			//
			// Is this a huge payload?
			//
			if (m0->m_pkthdr.len > MCLBYTES) {
				//
				// Could be TSO or jumbo.  Either way, we dont call defrag
				// So, will it actually fit into the descriptor ring, as is?
				//
	            // Make sure there are as many free as possible before we check
				//
				i82544_reap (i82544);

				if (i82544->tx_free > (num_frag + I82544_UNUSED_DESCR)) {
					// we got lucky - it will fit into the tx descr ring
					if (i82544->cfg.verbose) {
		        	    slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: warning: heavily fragmented large packet transmitted: "
						  "size %d, num_frag %d, free tx descr %d", 
						  __FUNCTION__, m0->m_pkthdr.len, num_frag, i82544->tx_free);
						}
					}
				else {
					slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: dropped heavily fragmented huge packet: "
					  "size %d, num_frag %d, free tx descr %d", 
					  __FUNCTION__, m0->m_pkthdr.len, num_frag, i82544->tx_free);
            		m_freem (m0);
		            i82544->stats.tx_failed_allocs++;
		            ifp->if_oerrors++;
		            goto done;
					}
				}
			else {
				// 
				// its safe to call the defrag routine - we know
				// the entire payload will fit into a cluster
				//
				if ((m = i82544_defrag (m0)) == NULL) {
		            slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: i82544_defrag() failed", __FUNCTION__);
		            i82544->stats.tx_failed_allocs++;
		            ifp->if_oerrors++;
		            goto done;
					}
				else if (i82544->cfg.verbose) {
	       			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: warning: heavily fragmented normal packet transmitted: "
						"defrag worked: size %d, orignal num_frag %d", 
						__FUNCTION__, m->m_pkthdr.len, num_frag);
					}
				m0 = m;
			
		        // must re-count mbuf fragments again
		        for (num_frag=0, m=m0; m; num_frag++) {
		            m = m->m_next;
			        }
				}
			}

		//
		// hardware checksumming stuff
		//
		cmd = 0;
		fields = 0;
		offload_flags = m0->m_pkthdr.csum_flags &
			(M_CSUM_IPv4 | M_CSUM_TCPv4 | M_CSUM_UDPv4 | M_CSUM_TSOv4 | 
                         M_CSUM_TCPv6 | M_CSUM_UDPv6 | M_CSUM_TSOv6);

		/* The 82575, 82576, 82580 & i350 use extended descriptors */
		if (hw->mac.type >= e1000_82575) {
			i82544_xmit_ext (i82544, ifp, m0, offload_flags, num_frag);
			continue;
			}

		if (offload_flags) {
			if (i82544_offload_setup (i82544, m0, offload_flags, &cmd, &fields) == -1) {		
				slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: i82544_offload_setup() failed, tx packet dropped!", __FUNCTION__);
				m_freem (m0);
	            continue;
				}
			}

		// 
		// we know that we have room in the tx descriptor ring
		// for this transmission, so run through all the linked
		// mbufs, flushing the cpu cache and loading the descriptors 
		// 
		cur_tx_wptr = i82544->cur_tx_wptr;
		for (m = m0; m; m = m->m_next) {
			if (!m->m_len) {
				// harmless except when hw csum is turned on
				num_frag--;  // must adjust for free descr count below
				continue;
				}
			tdesc = &i82544->tdesc[cur_tx_wptr];
			phys = mbuf_phys(m);
			CACHE_FLUSH (&i82544->cachectl, m->m_data, phys, m->m_len);
			tdesc->bufaddr = ENDIAN_LE64 (phys + i82544->bmtrans);
			tdesc->cmd_type_len = ENDIAN_LE32 (m->m_len | cmd | E1000_TXD_CMD_IFCS);
			tdesc->opts_stat = ENDIAN_LE32 (fields);
			free_wptr = cur_tx_wptr;
			cur_tx_wptr = (cur_tx_wptr + 1) % i82544->num_transmit;
			}

		tdesc->cmd_type_len |= ENDIAN_LE32 (E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
		
		/* Update the write pointer - submits packet for transmission */
		E1000_WRITE_REG (&i82544->hw, E1000_TDT(0), cur_tx_wptr);

		i82544->cur_tx_wptr = cur_tx_wptr;

		/* Store a pointer to the packet for freeing later */
		i82544->tx_mbuf[free_wptr] = m0;

		i82544->tx_free -= num_frag;
		
#if NBPFILTER > 0
		/* Pass the packet to any BPF listeners. */
		if (ifp->if_bpf) {
			bpf_mtap(ifp->if_bpf, m0);
			}
#endif

		}  // for

done:
	i82544->start_running = 0;
	/* If we have been called by the house-keeping callout, then don't unlock,
	   as the unlock will be done by the house-keeping callout */
	if (!i82544->hk) {
		NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
		}

	return;
}




#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/transmit.c $ $Rev: 737447 $")
#endif
