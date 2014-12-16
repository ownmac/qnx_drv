/*
 * $QNXLicenseC:
 * Copyright 2007, QNX Software Systems. All Rights Reserved.
 * 
 * You must obtain a written license from and pay applicable license fees to QNX 
 * Software Systems before you may reproduce, modify or distribute this software, 
 * or any work that includes all or part of this software.   Free development 
 * licenses are available for evaluation and non-commercial purposes.  For more 
 * information visit http://licensing.qnx.com or email licensing@qnx.com.
 *  
 * This file may contain contributions from others.  Please review this entire 
 * file for other proprietary rights or license notices, as well as the QNX 
 * Development Suite License Guide at http://licensing.qnx.com/license-guide/ 
 * for other information.
 * $
 */



#include <i82544.h>
#include <sys/malloc.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <device_qnx.h>


#include <sys/mman.h>


//
// this is a callback, made by the bsd media code.  We passed
// a pointer to this function during the ifmedia_init() call
// in bsd_mii_initmedia()
//
void
bsd_mii_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	i82544_dev_t *i82544 = ifp->if_softc;

	i82544->bsd_mii.mii_media_active = IFM_ETHER;
	i82544->bsd_mii.mii_media_status = IFM_AVALID;

	if (i82544->force_link) {

		if (i82544->linkup) {
			i82544->bsd_mii.mii_media_status |= IFM_ACTIVE;
		}

		// report back the previously forced values
		switch(i82544->cfg.media_rate) {
			case 0:
			i82544->bsd_mii.mii_media_active |= IFM_NONE;
			break;	

			case 1000*10:
			i82544->bsd_mii.mii_media_active |= IFM_10_T;
			break;

			case 1000*100:
			i82544->bsd_mii.mii_media_active |= IFM_100_TX;
			break;

			case 1000*1000:
			i82544->bsd_mii.mii_media_active |= IFM_1000_T;
			break;

			default:	// this shouldnt really happen, but ...
			i82544->bsd_mii.mii_media_active |= IFM_NONE;
			break;
		}
		if (i82544->cfg.duplex) {
			i82544->bsd_mii.mii_media_active |= IFM_FDX;
		}

		/* Sort out flow control status */
		i82544->bsd_mii.mii_media_status |= i82544->flow;

	} else if (i82544->linkup) {  // link is auto-detect and up

		i82544->bsd_mii.mii_media_status |= IFM_ACTIVE;

		switch(i82544->cfg.media_rate) {
			case 1000*10:
			i82544->bsd_mii.mii_media_active |= IFM_10_T;
			break;

			case 1000*100:
			i82544->bsd_mii.mii_media_active |= IFM_100_TX;
			break;

			case 1000*1000:
			i82544->bsd_mii.mii_media_active |= IFM_1000_T;
			break;

			default:	// this shouldnt really happen, but ...
			i82544->bsd_mii.mii_media_active |= IFM_NONE;
			break;
		}

		if (i82544->cfg.duplex) {
			i82544->bsd_mii.mii_media_active |= IFM_FDX;
		}

		// these media state variables are set by the link interrupt
		if (i82544->pause_xmit || i82544->pause_receive) {
			i82544->bsd_mii.mii_media_active |= IFM_FLOW;
		
			if (i82544->pause_xmit) {
				i82544->bsd_mii.mii_media_active |= IFM_ETH_TXPAUSE;
			}
	
			if (i82544->pause_receive) {
				i82544->bsd_mii.mii_media_active |= IFM_ETH_RXPAUSE;
			}
		}

		// could move this to event.c so there was no lag
		ifmedia_set(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_AUTO);

	} else {	// link is auto-detect and down
		i82544->bsd_mii.mii_media_active |= IFM_NONE;
		i82544->bsd_mii.mii_media_status = 0;

		// could move this to event.c so there was no lag
		ifmedia_set(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_NONE);
	}

	// stuff parameter values with hoked-up bsd values
    ifmr->ifm_status = i82544->bsd_mii.mii_media_status;
    ifmr->ifm_active = i82544->bsd_mii.mii_media_active;
}


//
// this is a callback, made by the bsd media code.  We passed
// a pointer to this function during the ifmedia_init() call
// in bsd_mii_initmedia().  This function is called when
// someone makes an ioctl into us, we call into the generic
// ifmedia source, and it make this callback to actually 
// force the speed and duplex, just as if the user had
// set the cmd line options
//

int		bsd_mii_mediachange(struct ifnet *ifp)

{
i82544_dev_t	*i82544	= ifp->if_softc;
int				old_media_rate = i82544->cfg.media_rate;
int				old_duplex = i82544->cfg.duplex;
int				old_force_link = i82544->force_link;
struct ifmedia	*ifm = &i82544->bsd_mii.mii_media;
int				user_duplex = ifm->ifm_media & IFM_FDX ? 1 : 0;
int				user_media = ifm->ifm_media & IFM_TMASK;
int				user_flow = ifm->ifm_media & IFM_ETH_FMASK;
int				media, old_flow = i82544->flow;

    if (!(ifp->if_flags & IFF_UP)) {
		slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): i82544 interface isn't up, ioctl ignored", __FUNCTION__);
	    return 0;
	}

	if (!(ifm->ifm_media & IFM_ETHER)) {
		slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): i82544 interface - bad media: 0x%X", 
		  __FUNCTION__, ifm->ifm_media);
		return 0;	// should never happen
	}

	switch (user_media) {
		case IFM_AUTO:		// auto-select media
			i82544->force_link      =  0;
			i82544->cfg.media_rate	= -1;
			i82544->cfg.duplex		= -1;
			ifmedia_set(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_AUTO);
			ifp->if_init (ifp);
			return (0);
			break;

		case IFM_NONE:		// disable media
		//
		// forcing the link with a speed of zero means to disable the link
		//
			i82544->force_link		= 1;
			i82544->cfg.media_rate	= 0; 
			i82544->cfg.duplex		= 0;
			ifmedia_set(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_NONE);
			ifp->if_init (ifp);
			return (0);
			break;

		case IFM_10_T:		// force 10baseT
			i82544->force_link		= 1;
			i82544->cfg.media_rate	= 10 * 1000;
			i82544->cfg.duplex		= user_duplex;
			ifmedia_set(&i82544->bsd_mii.mii_media, 
				user_duplex ? IFM_ETHER|IFM_10_T|IFM_FDX : IFM_ETHER|IFM_10_T);
			break;

		case IFM_100_TX:	// force 100baseTX
			i82544->force_link		= 1;
			i82544->cfg.media_rate	= 100 * 1000;
			i82544->cfg.duplex		= user_duplex;
			ifmedia_set(&i82544->bsd_mii.mii_media, 
				user_duplex ? IFM_ETHER|IFM_100_TX|IFM_FDX : IFM_ETHER|IFM_100_TX);
			break;

		case IFM_1000_T:	// force 1000baseT
		//
		// N.B.  I have not had good luck, trying to get gige to work half
		// duplex.  Even with different gige switches, I can only force full duplex
		//
			i82544->force_link		= 1;
			i82544->cfg.media_rate	= 1000 * 1000;
			i82544->cfg.duplex		= user_duplex;
			ifmedia_set(&i82544->bsd_mii.mii_media, 
				user_duplex ? IFM_ETHER|IFM_1000_T|IFM_FDX : IFM_ETHER|IFM_1000_T);
			break;

		default:			// should never happen
			slogf(_SLOGC_NETWORK, _SLOG_WARNING,
			  "%s(): i82544 interface - unknown media: 0x%X", 
			  __FUNCTION__, user_media);
			return 0;
			break;
		}

	/* Forced duplex */
	i82544->cfg.duplex = user_duplex;
	if (user_duplex)
		media |= IFM_FDX;

	/* Forced flow control */
	i82544->flow = 0;
	if (user_flow & IFM_FLOW) {
		i82544->flow = user_flow;
		}
	if (user_flow & IFM_ETH_RXPAUSE) {
		i82544->flow |= IFM_ETH_RXPAUSE;
		}
	if (user_flow & IFM_ETH_TXPAUSE) {
		i82544->flow |= IFM_ETH_TXPAUSE;
		}

	media |= user_flow;

	ifmedia_set (&i82544->bsd_mii.mii_media, media);

	// does the user want something different than it already is?
	if ((i82544->cfg.media_rate != old_media_rate)    ||
		(i82544->cfg.duplex     != old_duplex)        ||
		(i82544->force_link     != old_force_link)    ||
		(i82544->flow           != old_flow)          ||
		(i82544->cfg.flags      &  NIC_FLAG_LINK_DOWN) ) {
		
		// re-initialize hardware with new parameters
		i82544->link_config = 1;
		ifp->if_init(ifp);

		}

    return 0;
}


//
// called from i82544_pci_attach() in init.c to hook up
// to the bsd media structure.  Not entirely unlike kissing
// a porcupine, we must do so carefully, because we do not
// want to use the bsd mii management structure, because
// this driver uses link interrupt
//

void	bsd_mii_initmedia(i82544_dev_t *i82544)

{
i82544->bsd_mii.mii_ifp = &i82544->ecom.ec_if;

	ifmedia_init(&i82544->bsd_mii.mii_media, IFM_IMASK, bsd_mii_mediachange,
	  bsd_mii_mediastatus);

	// we do NOT call mii_attach() - we do our own link management

	//
	// must create these entries to make ifconfig media work
	// see lib/socket/public/net/if_media.h for defines
	//

	// ifconfig wm0 none (x22)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_NONE, 0, NULL);

	// ifconfig wm0 auto (x20)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_AUTO, 0, NULL);

	// ifconfig wm0 10baseT (x23 - half duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_10_T, 0, NULL);

	// ifconfig wm0 10baseT-FDX mediaopt fdx (x100023)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_10_T|IFM_FDX, 0, NULL);

	// ifconfig wm0 10baseT-FDX mediaopt flow (x100023)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_FLOW, 0, NULL);

	// ifconfig wm0 10baseT-FDX mediaopt rxpause (x100023)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);

	// ifconfig wm0 10baseT-FDX mediaopt txpause (x100023)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_ETH_TXPAUSE, 0, NULL);

	// ifconfig wm0 100baseTX (x26 - half duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_100_TX, 0, NULL);

	// ifconfig wm0 100baseTX-FDX mediaopt fdx (x100026 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_100_TX|IFM_FDX, 0, NULL);

	// ifconfig wm0 100baseTX-FDX mediaopt flow (x100026 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_FLOW, 0, NULL);

	// ifconfig wm0 100baseTX-FDX mediaopt rxpause (x100026 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);

	// ifconfig wm0 100baseTX-FDX mediaopt txpause (x100026 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_ETH_TXPAUSE, 0, NULL);

	// ifconfig wm0 1000baseT (x30 - half duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_1000_T, 0, NULL);

	// ifconfig wm0 1000baseT mediaopt fdx (x100030 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_1000_T|IFM_FDX, 0, NULL);

	// ifconfig wm0 1000baseT mediaopt flow (x100030 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_FLOW, 0, NULL);

	// ifconfig wm0 1000baseT mediaopt rxpause (x100030 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);

	// ifconfig wm0 1000baseT mediaopt txpause (x100030 - full duplex)
    ifmedia_add(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);

	// add more entries to support flow control via ifconfig media

	// link is initially down
	ifmedia_set(&i82544->bsd_mii.mii_media, IFM_ETHER|IFM_NONE);
}





#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/bsd_media.c $ $Rev: 708496 $")
#endif
