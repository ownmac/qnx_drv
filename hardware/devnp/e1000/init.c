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


#include <i82544.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <device_qnx.h>

#include <sys/mman.h>
#include "e1000.h"

static int i82544_entry(void *dll_hdl, struct _iopkt_self *iopkt, char *options);

struct _iopkt_drvr_entry IOPKT_DRVR_ENTRY_SYM(i82544) = IOPKT_DRVR_ENTRY_SYM_INIT(i82544_entry);

#ifdef VARIANT_a
#include <nw_dl.h>
/* This is what gets specified in the stack's dl.c */
struct nw_dll_syms i82544_syms[] = {
        {"iopkt_drvr_entry", &IOPKT_DRVR_ENTRY_SYM(i82544)},
        {NULL, NULL}
};
#endif

static int i82544_pci_attach(struct device *, struct device *, void *);
static int i82544_pci_detach(struct device *, int);
static int i82544_init(struct ifnet *);
static void i82544_stop(struct ifnet *ifp, int disable);
static void i82544_rxdrain(i82544_dev_t *i82544);
void i82544_shutdown(void *);
void bsd_mii_initmedia(i82544_dev_t *);

CFATTACH_DECL(i82544_pci,
	sizeof(struct i82544_dev),
	NULL,
	i82544_pci_attach,
	i82544_pci_detach,
	NULL);

static unsigned known_device_ids[] = {
	PCI_DEVICE_ID_INTEL_DH89XXCC_SGMII,				/* 0x0438 */
	PCI_DEVICE_ID_INTEL_DH89XXCC_SERDES,			/* 0x043A */
	PCI_DEVICE_ID_INTEL_DH89XXCC_BACKPLANE,			/* 0x043C */
	PCI_DEVICE_ID_INTEL_DH89XXCC_SFP,				/* 0x0440 */
	PCI_DEVICE_ID_INTEL_82542,						/* 0x1000 */
	PCI_DEVICE_ID_INTEL_82543GC_FIBER,				/* 0x1001 */
	PCI_DEVICE_ID_INTEL_82543GC_COPPER,				/* 0x1004 */
	PCI_DEVICE_ID_INTEL_82544EI_COPPER,				/* 0x1008 */
	PCI_DEVICE_ID_INTEL_82544EI_FIBER,				/* 0x1009 */
	PCI_DEVICE_ID_INTEL_82544GC_COPPER,				/* 0x100C */
	PCI_DEVICE_ID_INTEL_82544GC_LOM,				/* 0x100D */
	PCI_DEVICE_ID_INTEL_82540EM,					/* 0x100E */
	PCI_DEVICE_ID_INTEL_82545EM_COPPER,				/* 0x100F */
	PCI_DEVICE_ID_INTEL_82546EB_COPPER,				/* 0x1010 PCI */
	PCI_DEVICE_ID_INTEL_82545EM_FIBER,				/* 0x1011 */
	PCI_DEVICE_ID_INTEL_82546EB_FIBER,				/* 0x1012 */
	PCI_DEVICE_ID_INTEL_82541EI,					/* 0x1013 */
	PCI_DEVICE_ID_INTEL_82541ER_LOM,				/* 0x1014 */
	PCI_DEVICE_ID_INTEL_82540EM_LOM,				/* 0x1015 */
	PCI_DEVICE_ID_INTEL_82540EP_LOM,				/* 0x1016 */
	PCI_DEVICE_ID_INTEL_82540EP,					/* 0x1017 */
	PCI_DEVICE_ID_INTEL_82541EI_MOBILE,				/* 0x1018 */
	PCI_DEVICE_ID_INTEL_82547EI,					/* 0x1019 */
	PCI_DEVICE_ID_INTEL_82547EI_MOBILE,				/* 0x101A */
	PCI_DEVICE_ID_INTEL_82546EB_QUAD_COPPER,		/* 0x101D */
	PCI_DEVICE_ID_INTEL_82540EP_LP,					/* 0x101E */
	PCI_DEVICE_ID_INTEL_82545GM_COPPER,				/* 0x1026 */
	PCI_DEVICE_ID_INTEL_82545GM_FIBER,				/* 0x1027 PCI */
	PCI_DEVICE_ID_INTEL_82545GM_SERDES,				/* 0x1028 */
	PCI_DEVICE_ID_INTEL_ICH8_IGP_M_AMT,				/* 0x1049 */
	PCI_DEVICE_ID_INTEL_ICH8_IGP_AMT,				/* 0x104A PCIe LOM */
	PCI_DEVICE_ID_INTEL_ICH8_IGP_C,					/* 0x104B PCIe LOM */
	PCI_DEVICE_ID_INTEL_ICH8_IFE,					/* 0x104C */
	PCI_DEVICE_ID_INTEL_ICH8_IGP_M,					/* 0x104D */
	PCI_DEVICE_ID_INTEL_82571EB_COPPER,				/* 0x105E PCIe */
	PCI_DEVICE_ID_INTEL_82571EB_FIBER,				/* 0x105F PCIe */
	PCI_DEVICE_ID_INTEL_82571EB_SERDES,				/* 0x1060 */
	PCI_DEVICE_ID_INTEL_82547GI,					/* 0x1075 */
	PCI_DEVICE_ID_INTEL_82541GI,					/* 0x1076 PCI */
	PCI_DEVICE_ID_INTEL_82541GI_MOBILE,				/* 0x1077 */
	PCI_DEVICE_ID_INTEL_82541ER,					/* 0x1078 */
	PCI_DEVICE_ID_INTEL_82546GB_COPPER,				/* 0x1079 PCI */
	PCI_DEVICE_ID_INTEL_82546GB_FIBER,				/* 0x107A PCI */
	PCI_DEVICE_ID_INTEL_82546GB_SERDES,				/* 0x107B */
	PCI_DEVICE_ID_INTEL_82541GI_LF,					/* 0x107C PCI */
	PCI_DEVICE_ID_INTEL_82572EI_COPPER,				/* 0x107D */
	PCI_DEVICE_ID_INTEL_82572EI_FIBER,				/* 0x107E PCIe */
	PCI_DEVICE_ID_INTEL_82572EI_SERDES,				/* 0x107F */
	PCI_DEVICE_ID_INTEL_82546GB_PCIE,				/* 0x108A */
	PCI_DEVICE_ID_INTEL_82573E,						/* 0x108B */
	PCI_DEVICE_ID_INTEL_82573E_IAMT,				/* 0x108C PCIe LOM */
	PCI_DEVICE_ID_INTEL_80003ES2LAN_COPPER_DPT,		/* 0x1096 */
	PCI_DEVICE_ID_INTEL_80003ES2LAN_SERDES_DPT,		/* 0x1098 */
	PCI_DEVICE_ID_INTEL_82546GB_QUAD_COPPER,		/* 0x1099 */
	PCI_DEVICE_ID_INTEL_82573L,						/* 0x109A PCIe LOM */
	PCI_DEVICE_ID_INTEL_82571EB_QUAD_COPPER,		/* 0x10A4 */
	PCI_DEVICE_ID_INTEL_82571EB_QUAD_FIBER,			/* 0x10A5 */
	PCI_DEVICE_ID_INTEL_82575EB_COPPER,				/* 0x10a7 */
	PCI_DEVICE_ID_INTEL_82575EB_FIBER_SERDES,		/* 0x10a9 */
	PCI_DEVICE_ID_INTEL_82546GB_QUAD_COPPER_KSP3,	/* 0x10B5 */
	PCI_DEVICE_ID_INTEL_82572EI,					/* 0x10B9 */
	PCI_DEVICE_ID_INTEL_80003ES2LAN_COPPER_SPT,		/* 0x10BA */
	PCI_DEVICE_ID_INTEL_80003ES2LAN_SERDES_SPT,		/* 0x10BB */
	PCI_DEVICE_ID_INTEL_82571EB_QUAD_COPPER_LP,		/* 0x10BC PCIe */
	PCI_DEVICE_ID_INTEL_ICH9_IGP_AMT,				/* 0x10BD PCIe LOM */
	PCI_DEVICE_ID_INTEL_ICH9_IGP_M,					/* 0x10BF */
	PCI_DEVICE_ID_INTEL_ICH9_IFE,					/* 0x10c0 */
	PCI_DEVICE_ID_INTEL_ICH9_IFE_G,					/* 0x10c2 */
	PCI_DEVICE_ID_INTEL_ICH9_IFE_GT,				/* 0x10c3 */
	PCI_DEVICE_ID_INTEL_ICH8_IFE_GT,				/* 0x10C4 */
	PCI_DEVICE_ID_INTEL_ICH8_IFE_G,					/* 0x10C5 */
	PCI_DEVICE_ID_INTEL_82576,						/* 0x10C9 */
	PCI_DEVICE_ID_INTEL_82576_VF,					/* 0x10CA */
	PCI_DEVICE_ID_INTEL_ICH9_IGP_M_V,				/* 0x10CB */
	PCI_DEVICE_ID_INTEL_ICH10_R_BM_LM,				/* 0x10CC */
	PCI_DEVICE_ID_INTEL_ICH10_R_BM_LF,				/* 0x10CD */
	PCI_DEVICE_ID_INTEL_ICH10_R_BM_V,				/* 0x10CE */
	PCI_DEVICE_ID_INTEL_82574L,						/* 0x10D3 */
	PCI_DEVICE_ID_INTEL_82571PT_QUAD_COPPER,		/* 0x10d5 */
	PCI_DEVICE_ID_INTEL_82575GB_QUAD_COPPER,		/* 0x10d6 PCIe */
	PCI_DEVICE_ID_INTEL_82571EB_SERDES_DUAL,		/* 0x10d9 */
	PCI_DEVICE_ID_INTEL_82571EB_SERDES_QUAD,		/* 0x10da */
	PCI_DEVICE_ID_INTEL_ICH10_D_BM_LM,				/* 0x10DE */
	PCI_DEVICE_ID_INTEL_ICH10_D_BM_LF,				/* 0x10DF */
	PCI_DEVICE_ID_INTEL_ICH9_BM,					/* 0x10E5 */
	PCI_DEVICE_ID_INTEL_82576_FIBER,				/* 0x10E6 */
	PCI_DEVICE_ID_INTEL_82576_SERDES,				/* 0x10E7 */
	PCI_DEVICE_ID_INTEL_82576_QUAD_COPPER,			/* 0x10E8 */
	PCI_DEVICE_ID_INTEL_82577LM,					/* 0x10EA */
	PCI_DEVICE_ID_INTEL_82577LC,					/* 0x10EB */
	PCI_DEVICE_ID_INTEL_82578DM,					/* 0x10EF */
	PCI_DEVICE_ID_INTEL_82578DC,					/* 0x10F0 */
	PCI_DEVICE_ID_INTEL_ICH9_IGP_M_AMT,				/* 0x10F5 */
	PCI_DEVICE_ID_INTEL_ICH8_82567V_3,				/* 0x1501 */
	PCI_DEVICE_ID_INTEL_ICH8_82579_LM,				/* 0x1502 */
	PCI_DEVICE_ID_INTEL_ICH8_82579_V,				/* 0x1503 */
	PCI_DEVICE_ID_INTEL_82583V,						/* 0x150c */
	PCI_DEVICE_ID_INTEL_82580_COPPER,				/* 0x150E */
	PCI_DEVICE_ID_INTEL_82580_FIBER,				/* 0x150F */
	PCI_DEVICE_ID_INTEL_82580_SERDES,				/* 0x1510 */
	PCI_DEVICE_ID_INTEL_82580_SGMII,				/* 0x1511 */
	PCI_DEVICE_ID_INTEL_82580_COPPER_DUAL,			/* 0x1516 */
	PCI_DEVICE_ID_INTEL_82580_ER,					/* 0x151D */
	PCI_DEVICE_ID_INTEL_82580_ER_DUAL,				/* 0x151E */
	PCI_DEVICE_ID_INTEL_I350_VF,					/* 0x1520 */
	PCI_DEVICE_ID_INTEL_I350_COPPER,				/* 0x1521 */
	PCI_DEVICE_ID_INTEL_I350_FIBER,					/* 0x1522 */
	PCI_DEVICE_ID_INTEL_I350_SERDES,				/* 0x1523 */
	PCI_DEVICE_ID_INTEL_I350_SGMII,					/* 0x1524 */
	PCI_DEVICE_ID_INTEL_ICH10_D_BM_V,				/* 0x1525 */
	PCI_DEVICE_ID_INTEL_82576_QUAD_COPPER_ET2,		/* 0x1526 */
	PCI_DEVICE_ID_INTEL_82580_QUAD_FIBER,			/* 0x1527 */
	PCI_DEVICE_ID_INTEL_I210_NVMLESS,				/* 0x1531 */
	PCI_DEVICE_ID_INTEL_I210_COPPER,				/* 0x1533 */
	PCI_DEVICE_ID_INTEL_I210_COPPER_OEM1,			/* 0x1534 */
	PCI_DEVICE_ID_INTEL_I210_COPPER_IT,				/* 0x1535 */
	PCI_DEVICE_ID_INTEL_I210_FIBER,					/* 0x1536 */
	PCI_DEVICE_ID_INTEL_I210_SERDES,				/* 0x1537 */
	PCI_DEVICE_ID_INTEL_I210_SGMII,					/* 0x1538 */
	PCI_DEVICE_ID_INTEL_I211_COPPER,				/* 0x1539 */
	PCI_DEVICE_ID_INTEL_PCH_LPT_I217_LM,			/* 0x153A */
	PCI_DEVICE_ID_INTEL_PCH_LPT_I217_V,				/* 0x153B */
	PCI_DEVICE_ID_INTEL_PCH_LPTLP_I218_LM,			/* 0x155A */
	PCI_DEVICE_ID_INTEL_PCH_LPTLP_I218_V,			/* 0x1559 */
	PCI_DEVICE_ID_INTEL_I350_DA4,					/* 0x1546 */
	PCI_DEVICE_ID_INTEL_I210_COPPER_FLASHLESS,		/* 0x157B */
	PCI_DEVICE_ID_INTEL_I210_SERDES_FLASHLESS,		/* 0x157C */
	PCI_DEVICE_ID_INTEL_I354_BACKPLANE_1GBPS,		/* 0x1F40 */
	PCI_DEVICE_ID_INTEL_I354_SGMII,					/* 0x1F41 */
	PCI_DEVICE_ID_INTEL_ICH9_IGP_C,					/* 0x294c */
	PCI_DEVICE_ID_INTEL_ICH10_HANKSVILLE,			/* 0xF0FE */

	0		/* End of list */
};

struct i_attach_args {
	struct _iopkt_self	*iopkt;
	char			*options;
	unsigned		busvendor, busdevice;
	int				busindex;
	uint8_t			bus, devfn;
	uint8_t			fill [2];
};

/*****************************************************************************/
/* Initial driver entry point.                                               */
/* Returns -1 on failure; errno will be set to ENODEV if no devices detected.*/
/*****************************************************************************/

int		i82544_entry (void *dll_hdl, struct _iopkt_self *iopkt, char *options)

{
nic_config_t	cfg;
int				devid, idx;
unsigned		bus, dev_func;
int				err, single;
int				instance, pci_hdl;
struct device	*dev;
struct i_attach_args	iargs;

	memset (&cfg, 0, sizeof (cfg));

	if (options != NULL) {
		cfg.vendor_id = 0xffffffff;
		cfg.device_id = 0xffffffff;
		cfg.device_index = -1;
		
		if ((err = i82544_parse_options (NULL, options, &cfg)) != EOK) {
			return (err);
			}

		if (cfg.device_id != 0xffffffff) {
			known_device_ids[0] = cfg.device_id;
			known_device_ids[1] = 0;
			}
		if (cfg.vendor_id == 0xffffffff)
			cfg.vendor_id = 0x8086;
			}
		else {
			cfg.vendor_id = 0x8086;
			cfg.device_index = -1;
			}

	memset (&iargs, 0x00, sizeof(iargs));
	iargs.iopkt = iopkt;
	iargs.options = options;

	instance = single = 0;

	if ((pci_hdl = pci_attach (0)) < 0) {
		err = errno;
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "devnp-e1000: Could not attach to PCI server");
		errno = err;
		return -1;
		}

	for (devid = 0; known_device_ids[devid] != 0; devid++) {
		idx = ((cfg.device_index == -1) ? 0 : cfg.device_index);
		while (1) {
			if (pci_find_device (known_device_ids[devid],
			    cfg.vendor_id, idx, &bus, &dev_func) != PCI_SUCCESS) {
				break;
				}
			iargs.busvendor = cfg.vendor_id;
			iargs.busdevice = known_device_ids[devid];
			iargs.busindex = idx;
			iargs.bus = (uint8_t) bus;
			iargs.devfn = (uint8_t) dev_func;
			dev = NULL; /* No Parent */
			if (dev_attach ("wm", options, &i82544_pci_ca, &iargs, &single,
			    &dev, NULL) != EOK) {
				goto done;
				}
			dev->dv_dll_hdl = dll_hdl;
			instance++;

			if (cfg.device_index != -1)
				/* Only looking at a specific index */
				break;
			idx++;
			}
		}

done:
	if (instance)
		return (EOK);

	return (ENODEV);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

int	e1000_set_spd_dplx (struct e1000_hw *hw)

{
struct e1000_mac_info *mac = &hw->mac;
int				spddplx;
i82544_dev_t	*i82544 = hw->i82544;

	mac->autoneg = i82544->cmd_force ? 0 : 1;

	if (i82544->cfg.media_rate != -1) {
		switch (i82544->cfg.media_rate / 1000) {
			case	10:
				spddplx = SPEED_10;
				break;
			case	100:
				spddplx = SPEED_100;
				break;
			case	1000:
				spddplx = SPEED_1000;
				break;
			default:
				slogf (_SLOGC_NETWORK, _SLOG_ERROR, "Invalid speed setting %d", i82544->cfg.media_rate / 1000);
				return (-1);
			}
		spddplx += ((i82544->cfg.duplex != -1) && i82544->cfg.duplex) ? FULL_DUPLEX : HALF_DUPLEX;
		}
	else {
		spddplx = SPEED_1000 + FULL_DUPLEX;
		}

	/* Fiber NICs only allow 1000 gbps Full duplex */
	if ((hw->phy.media_type == e1000_media_type_fiber ||
		hw->phy.media_type == e1000_media_type_internal_serdes) &&
		spddplx != (SPEED_1000 + FULL_DUPLEX)) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "Unsupported Speed/Duplex configuration");
		return -EINVAL;
		}

	switch (spddplx) {
		case SPEED_10 + HALF_DUPLEX:
			mac->forced_speed_duplex = ADVERTISE_10_HALF;
			if (mac->autoneg)
				hw->phy.autoneg_advertised = ADVERTISE_10_HALF;
			break;
		case SPEED_10 + FULL_DUPLEX:
			mac->forced_speed_duplex = ADVERTISE_10_FULL;
			if (mac->autoneg)
				hw->phy.autoneg_advertised = ADVERTISE_10_FULL;
			break;
		case SPEED_100 + HALF_DUPLEX:
			mac->forced_speed_duplex = ADVERTISE_100_HALF;
			if (mac->autoneg)
				hw->phy.autoneg_advertised = ADVERTISE_100_HALF;
			break;
		case SPEED_100 + FULL_DUPLEX:
			mac->forced_speed_duplex = ADVERTISE_100_FULL;
			if (mac->autoneg)
				hw->phy.autoneg_advertised = ADVERTISE_100_FULL;
			break;
		case SPEED_1000 + FULL_DUPLEX:
			mac->autoneg = 1;
			hw->phy.autoneg_advertised = ADVERTISE_1000_FULL;
			break;
		case SPEED_1000 + HALF_DUPLEX: /* not supported */
		default:
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "Unsupported Speed/Duplex configuration");
			return -EINVAL;
		}
	return 0;
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

static	void e1000_get_hw_control (struct e1000_hw *hw)

{
uint32_t	ctrl_ext;
uint32_t	swsm;

	/* Let firmware know the driver has taken over */
	switch (hw->mac.type) {
		case	e1000_82573:
			swsm = E1000_READ_REG (hw, E1000_SWSM);
			E1000_WRITE_REG (hw, E1000_SWSM,
					swsm | E1000_SWSM_DRV_LOAD);
			break;
		case	e1000_82571:
		case	e1000_82572:
		case	e1000_82574:
		case	e1000_82575:
		case	e1000_82576:
		case	e1000_82580:
		case	e1000_80003es2lan:
		case	e1000_ich8lan:
		case	e1000_ich9lan:
		case	e1000_ich10lan:
		case	e1000_pchlan:
		case	e1000_pch2lan:
		case	e1000_pch_lpt:
		case	e1000_i350:
		case	e1000_i354:
		case	e1000_i210:
		case	e1000_i211:
			ctrl_ext = E1000_READ_REG (hw, E1000_CTRL_EXT);
			E1000_WRITE_REG (hw, E1000_CTRL_EXT,
					ctrl_ext | E1000_CTRL_EXT_DRV_LOAD);
			break;
		default:
			break;
		}
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

static	void e1000_release_hw_control (struct e1000_hw *hw)

{
uint32_t	ctrl_ext;
uint32_t	swsm;

	/* Let firmware know the driver has taken over */
	switch (hw->mac.type) {
		case	e1000_82573:
			swsm = E1000_READ_REG (hw, E1000_SWSM);
			E1000_WRITE_REG (hw, E1000_SWSM, swsm & ~E1000_SWSM_DRV_LOAD);
			break;
		case	e1000_82571:
		case	e1000_82572:
		case	e1000_82574:
		case	e1000_82575:
		case	e1000_82576:
		case	e1000_82580:
		case	e1000_80003es2lan:
		case	e1000_ich8lan:
		case	e1000_ich9lan:
		case	e1000_ich10lan:
		case	e1000_pchlan:
		case	e1000_pch2lan:
		case	e1000_pch_lpt:
		case	e1000_i350:
		case	e1000_i354:
		case	e1000_i210:
		case	e1000_i211:
			ctrl_ext = E1000_READ_REG (hw, E1000_CTRL_EXT);
			E1000_WRITE_REG (hw, E1000_CTRL_EXT,
					ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
			break;
		default:
			break;
		}
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static void igb_init_fw(struct e1000_hw *hw) 

{
struct	e1000_fw_drv_info fw_cmd;
int		i;
	
	if (!e1000_get_hw_semaphore_generic(hw)) { 
		for (i = 0; i <= FW_MAX_RETRIES; i++) {
			E1000_WRITE_REG(hw, E1000_FWSTS, E1000_FWSTS_FWRI);
			fw_cmd.hdr.cmd = FW_CMD_DRV_INFO;
			fw_cmd.hdr.buf_len = FW_CMD_DRV_INFO_LEN;
			fw_cmd.hdr.cmd_or_resp.cmd_resv = FW_CMD_RESERVED;
			fw_cmd.port_num = hw->bus.func;
			fw_cmd.drv_version = FW_FAMILY_DRV_VER;
			fw_cmd.hdr.checksum = e1000_calculate_checksum((u8 *)&fw_cmd,
			                                           (FW_HDR_LEN +
			                                            fw_cmd.hdr.buf_len));
			 e1000_host_interface_command(hw, (u8*)&fw_cmd,
			                             sizeof(fw_cmd)); 
			if (fw_cmd.hdr.cmd_or_resp.ret_status == FW_STATUS_SUCCESS)
				break;
			}
		}
	else
		slogf (_SLOGC_NETWORK, _SLOG_INFO, 
			 "Unable to get semaphore, firmware init failed.");
	e1000_put_hw_semaphore_generic(hw);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static int	i82544_pci_detach_cleanup(i82544_dev_t *i82544, int how)

{
struct ifnet		*ifp;
struct _iopkt_self	*iopkt;

	ifp = &i82544->ecom.ec_if;
	iopkt = i82544->iopkt;

	switch (how) {
		case	-1:
		/*
		 * Don't init() while we're dying.  Yes it can happen:
		 * ether_ifdetach() calls bridge_ifdetach() which
		 * tries to take us out of promiscuous mode with an
		 * init().
		 */
			i82544->dying = 1;
			i82544_stop (ifp, 1);

			ether_ifdetach (ifp);
			if_detach (ifp);
			e1000_release_hw_control (&i82544->hw);

		case	11:
			free (i82544->tx_mbuf, M_DEVBUF);

		case	10:
			free (i82544->rx_mbuf, M_DEVBUF);

		case	9:
			cache_fini (&i82544->cachectl);

		case	8:
			munmap (i82544->tx_descriptor_area, i82544->num_transmit * sizeof (i82544_tdesc_t));

		case	7:
			munmap (i82544->rx_descriptor_area, i82544->num_receive * sizeof (i82544_rdesc_t));

		case	6:
			if ((i82544->hw.mac.type == e1000_ich8lan) ||
			    (i82544->hw.mac.type == e1000_ich9lan) ||
			    (i82544->hw.mac.type == e1000_pchlan) ||
			    (i82544->hw.mac.type == e1000_pch2lan) ||
				(i82544->hw.mac.type == e1000_ich10lan)) {
				munmap_device_memory ((void *)i82544->hw.flash_address,i82544->flash_map_size);
				}
		case	5:
			munmap_device_memory ((void *)i82544->hw.hw_addr, 0x20000);

		case	4:
			evcnt_detach (&i82544->ev_txdrop);
			pci_detach_device (i82544->pci_dev_hdl);

		case	3:
		case	2:
		case	1:
			if (i82544->sd_hook != NULL)
				shutdownhook_disestablish (i82544->sd_hook);
		}

	return EOK;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static int	i82544_pci_detach(struct device *dev, int flags)

{
struct i82544_dev	*idev;

	idev = (struct i82544_dev *)dev;

	return i82544_pci_detach_cleanup (idev->sc_i82544, -1);
}

/*****************************************************************************/
/* Check the PCI capabilities to see whether we have MSI or MSI-X.           */
/*****************************************************************************/

static	int	check_capabilities (i82544_dev_t *i82544, uint8_t bus, uint8_t devfn)

{
uint8_t		cap, cap_ptr = 0;
uint16_t	device, dev_ctrl, mess_ctrl;
int			offset;

	if (pci_read_config8 (bus, devfn, offsetof (struct _pci_config_regs, Capabilities_Pointer), 1, &cap_ptr) != PCI_SUCCESS)
		return (-1);
	if (pci_read_config16 (bus, devfn, offsetof (struct _pci_config_regs, Device_ID), 1, &device) != PCI_SUCCESS)
		return (-1);
	if (cap_ptr) {
		while (1) {
			if (pci_read_config8 (bus, devfn, cap_ptr, 1, &cap))
				return (-1);
			if (cap == PCI_CAP_MSI)
				i82544->msi_cap = 1;
			if (cap == PCI_CAP_MSI_X) {
				i82544->msi_cap = 2;
				i82544->msix_cap_ptr = cap_ptr;
				if (pci_read_config16 (bus, devfn, cap_ptr + 2, 1, &mess_ctrl))
					return (-1);
				if ((mess_ctrl & 0x7ff) == 0)
					i82544->msi_cap = 1;	/* If only one vector, make it look like MSI */
				}
			if ((cap == PCI_CAP_PCI_EXPRESS) && (i82544->max_read != -1)) {
				offset = offsetof (struct _pci_capability_pci_express, Device_Control);
				if (pci_read_config16 (bus, devfn, cap_ptr + offset, 1, &dev_ctrl) != PCI_SUCCESS)
					return (-1);
				dev_ctrl &= 0x8fff;
				dev_ctrl |= (i82544->max_read << 12);
				pci_write_config16 (bus, devfn, cap_ptr + offset, 1, &dev_ctrl);
				}
			if (pci_read_config8 (bus, devfn, cap_ptr + 1, 1, &cap_ptr))
				return (-1);
			if (! cap_ptr) {
			/* For some reason the 82583V device advertises MSI-X, but according */
			/* to Intel, the device doesn't support MSI-X, so we use MSI instead */
				if (device == PCI_DEVICE_ID_INTEL_82583V)
					i82544->msi_cap = 1;
				break;
				}
			}
		}
	return (EOK);
}

/**************************************************************************/
/*                                                                        */
/**************************************************************************/

static	void	i82544_reset (i82544_dev_t *i82544)

{
struct e1000_hw		*hw;
int					rx_buffer_size;

	hw = &i82544->hw;
	if (hw->mac.type == e1000_80003es2lan)
		hw->fc.pause_time = 0xFFFF;
	else
		hw->fc.pause_time = E1000_FC_PAUSE_TIME;

	hw->fc.send_xon = TRUE;

	if (hw->mac.type == e1000_82576) {
		rx_buffer_size = ((E1000_READ_REG (hw, E1000_RXPBS) & 0xffff) << 10);
		}
	else {
		rx_buffer_size = ((E1000_READ_REG (hw, E1000_PBA) & 0xffff) << 10);
		}
	
	hw->fc.high_water = rx_buffer_size - roundup2 (i82544->cfg.mtu, 1024);
	hw->fc.low_water = hw->fc.high_water - 1500;
	hw->fc.pause_time = E1000_FC_PAUSE_TIME;
	hw->fc.send_xon = TRUE;
	if (!i82544->pause_rx_disable && !i82544->pause_tx_disable) {
		hw->fc.requested_mode = e1000_fc_full;
		}
	else {
		if (i82544->pause_rx_disable && !i82544->pause_tx_disable) {
			hw->fc.requested_mode = e1000_fc_tx_pause;
			}
		else {
			if (i82544->pause_rx_disable && i82544->pause_tx_disable) {
				hw->fc.requested_mode = e1000_fc_none;
				}
			else {
				if (!i82544->pause_rx_disable && i82544->pause_tx_disable) {
					hw->fc.requested_mode = e1000_fc_rx_pause;
					}
				}
			}
		}

	/* Override - workaround for PCHLAN issue */
	if (hw->mac.type == e1000_pchlan)
		hw->fc.requested_mode = e1000_fc_rx_pause;

	/* Override - settings for PCH2LAN, ya its magic :) */
	if (hw->mac.type == e1000_pch2lan || hw->mac.type == e1000_pch_lpt) {
		hw->fc.high_water = 0x5c20;
		hw->fc.low_water = 0x5048;
		hw->fc.pause_time = 0x0650;
		hw->fc.refresh_time = 0x0400;
		if (i82544->cfg.mtu > ETH_MAX_PKT_LEN)
			E1000_WRITE_REG (hw, E1000_PBA, 14);
		else
			E1000_WRITE_REG (hw, E1000_PBA, 26);
		}

	hw->fc.current_mode = hw->fc.requested_mode;

	e1000_reset_hw (hw);
	E1000_WRITE_REG (hw, E1000_WUC, 0);

	if (e1000_init_hw (hw) < 0) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "Hardware Initialization Failed");
		return;
		}

	E1000_WRITE_REG (hw, E1000_VET, ETHERTYPE_VLAN);
	e1000_get_phy_info (hw);
	e1000_check_for_link (hw);

	if (hw->phy.media_type == e1000_media_type_fiber) {
		i82544->cfg.connector = NIC_CONNECTOR_FIBER;
		}
	else {
		i82544->cfg.connector = NIC_CONNECTOR_MII;
		}

}

/*****************************************************************************/
/* Clear the unicast & multicast address arrays                              */
/*****************************************************************************/

void	i82544_clear_addresses (i82544_dev_t *i82544)

{
int		index;
struct	e1000_hw		*hw = &i82544->hw;

	for (index = 0; index < 32; index += 2) {
		E1000_WRITE_REG (hw, E1000_RAL(index), 0);
		E1000_WRITE_REG (hw, E1000_RAH(index), 0);
		}
	
	for (index = 0; index < 128; index++) {
		E1000_WRITE_REG_ARRAY (hw, E1000_MTA, index, 0);
		}
}

/**************************************************************************/
/* Setup and declare a NIC                                                */
/**************************************************************************/

static int	i82544_pci_attach(struct device *parent, struct device *self, void *aux)

{
int						i;
int						err = EOK;
struct i82544_dev		*idev;
i82544_dev_t			*i82544;
struct ifnet			*ifp;
void					*head;
char					*options;
size_t					size;
struct i_attach_args	*iargs;
struct _iopkt_self		*iopkt;
uint32_t				dword, flags;
struct pci_dev_info		info;
struct e1000_hw			*hw;
unsigned				busvendor, busdevice;
int						busindex, force_speed = 0;
uint8_t					bus, devfn;
nic_config_t    		*cfg;

	idev = (struct i82544_dev *)self;
	iargs = aux;

	iopkt = iargs->iopkt;
	options = iargs->options;
	busvendor = iargs->busvendor;
	busdevice = iargs->busdevice;
	busindex = iargs->busindex;
	bus = iargs->bus;
	devfn = iargs->devfn;

	head = idev->filler;

	i82544 = NET_CACHELINE_ALIGN (head);
	idev->sc_i82544 = i82544;

	i82544->iopkt = iopkt;
	i82544->iid = i82544->iid_lnk = -1; /* Not attached */
	i82544->bus = bus;
	i82544->devfn = devfn;

	i82544->rx_tail = &i82544->rx_head;

	hw = &i82544->hw;
	hw->i82544 = i82544;

	cfg = &i82544->cfg;

	ifp = &i82544->ecom.ec_if;
	ifp->if_softc = i82544;

	strcpy (ifp->if_xname, self->dv_xname);

	/* Ethernet stats we are interested in */
	i82544->stats.un.estats.valid_stats =
	    NIC_ETHER_STAT_SINGLE_COLLISIONS |
	    NIC_ETHER_STAT_MULTI_COLLISIONS |
	    NIC_ETHER_STAT_TX_DEFERRED |
	    NIC_ETHER_STAT_XCOLL_ABORTED |
	    NIC_ETHER_STAT_LATE_COLLISIONS |
	    NIC_ETHER_STAT_ALIGN_ERRORS |
	    NIC_ETHER_STAT_FCS_ERRORS |
	    NIC_ETHER_STAT_OVERSIZED_PACKETS |
	    NIC_ETHER_STAT_SYMBOL_ERRORS |
	    NIC_ETHER_STAT_JABBER_DETECTED |
	    NIC_ETHER_STAT_SHORT_PACKETS |
		NIC_ETHER_STAT_SQE_ERRORS |
	    NIC_ETHER_STAT_INTERNAL_RX_ERRORS; 

	/* Generic networking stats we are interested in */
	i82544->stats.valid_stats =
	    NIC_STAT_TX_FAILED_ALLOCS | NIC_STAT_RX_FAILED_ALLOCS |
	    NIC_STAT_TXED_MULTICAST | NIC_STAT_TXED_BROADCAST |
	    NIC_STAT_RXED_MULTICAST | NIC_STAT_RXED_BROADCAST;

	/* Parse the options; set up some defaults first */
	i82544->cfg.priority = IRUPT_PRIO_DEFAULT;
	i82544->cfg.lan = -1;
	i82544->cfg.media_rate = -1;
	i82544->cfg.duplex = -1;
	i82544->num_transmit = I82544_DEFAULT_NUM_TX;
	i82544->num_receive = I82544_DEFAULT_NUM_RX;
	i82544->cfg.mtu = ETH_MAX_DATA_LEN;
	i82544->cfg.mru = ETH_MAX_DATA_LEN;
	i82544->tx_reap = DEFAULT_TX_REAP;
	i82544->cfg.flags |= NIC_FLAG_MULTICAST;
//	i82544->irq_thresh = 9000; // default is max 9000 interrupts per second
	i82544->max_read = -1;

	strcpy ((char *) i82544->cfg.uptype, "en");

	err = i82544_parse_options (i82544, options, &i82544->cfg);

	if (err != EOK) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "devnp-e1000: error parsing options");
		i82544_pci_detach_cleanup (i82544, 1);
		return (err);
		}

    // did user specify either of speed or duplex on the cmd line?
	if ((cfg->media_rate != -1) || (cfg->duplex != -1)) {

		if (cfg->media_rate == -1) {
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must also specify speed when duplex is specified", __FUNCTION__);
			i82544_pci_detach_cleanup (i82544, 1);
            return (EINVAL);
	        }
		if (cfg->duplex == -1) {
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must also specify duplex when speed is specified", __FUNCTION__);
			i82544_pci_detach_cleanup (i82544, 1);
			return (EINVAL);
			}
		force_speed = 1;
		}

	// range check user-specified descriptor ring sizes
	i82544->num_receive &= ~3;
	if (i82544->num_receive < MIN_NUM_RX_DESCRIPTORS) {
		i82544->num_receive = MIN_NUM_RX_DESCRIPTORS;
		}
	if (i82544->num_receive > MAX_NUM_RX_DESCRIPTORS) {
		i82544->num_receive = MAX_NUM_RX_DESCRIPTORS;
		}

	i82544->num_transmit &= ~3;
	if (i82544->num_transmit < MIN_NUM_TX_DESCRIPTORS) {
		i82544->num_transmit = MIN_NUM_TX_DESCRIPTORS;
	    }
	if (i82544->num_transmit > MAX_NUM_TX_DESCRIPTORS) {
		i82544->num_transmit = MAX_NUM_TX_DESCRIPTORS;
	    }

	i82544->cfg.lan = idev->sc_dev.dv_unit;

	hw->device_id = busdevice;
	if (e1000_set_mac_type (hw)) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "set_mac_type failed %x", hw->device_id);
		i82544_pci_detach_cleanup (i82544, 1);
		err = ENODEV;
		return (err);
		}

	/* Are we a VF device? */
	if ((hw->mac.type == e1000_vfadapt) ||
	    (hw->mac.type == e1000_vfadapt_i350))
		i82544->vf_ifp = 1;
	else
		i82544->vf_ifp = 0;

	// initialize - until mii callback says we have a link ...
	cfg->flags |= NIC_FLAG_LINK_DOWN;

	memset (&info, 0, sizeof (info));
	i82544->cfg.vendor_id = info.VendorId = busvendor;
	i82544->cfg.device_id = info.DeviceId = busdevice;
	i82544->cfg.device_index = busindex;

	hw->verbose = i82544->cfg.verbose;
	if (info.VendorId == 0 || info.DeviceId == 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,
		    "devnp-e1000: must specify PCI device and vendor ID");
		err = EINVAL;
		i82544_pci_detach_cleanup (i82544, 1);
		return (err);
		}

	if (check_capabilities (i82544, bus, devfn)) {
		i82544_pci_detach_cleanup (i82544, 3);
		return (-1);
		}

	flags = PCI_INIT_ALL | PCI_INIT_IRQ | PCI_MASTER_ENABLE;
	if (i82544->msi_cap)
		flags |= (i82544->msi_cap == 1) ? PCI_USE_MSI : PCI_USE_MSIX;

	if ((i82544->pci_dev_hdl = pci_attach_device (NULL, flags, busindex, &info)) == NULL) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "pci_attach_device failed");
		err = errno;
		i82544_pci_detach_cleanup (i82544, 3);
		return (err);
		}
	i82544->msi_enabled = info.msi;

	evcnt_attach_dynamic (&i82544->ev_txdrop, EVCNT_TYPE_MISC,
	    NULL, ifp->if_xname, "txdrop");

	i82544->bmtrans = info.CpuBmstrTranslation;
	hw->revision_id = info.Revision;

	if (hw->mac.type >= e1000_82543) {
		ifp->if_capabilities_rx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 |
						IFCAP_CSUM_UDPv4;
		ifp->if_capabilities_tx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 |
						IFCAP_CSUM_UDPv4 | IFCAP_CSUM_TCPv6 | IFCAP_CSUM_UDPv6;
		}
	if ((hw->mac.type >= e1000_82544) && (hw->mac.type != e1000_82547)) {
		ifp->if_capabilities_tx |= IFCAP_TSOv4;
		}
	if (hw->mac.type > e1000_82547_rev_2) {
		ifp->if_capabilities_tx |= IFCAP_TSOv6;
		ifp->if_capabilities_rx |= (IFCAP_CSUM_TCPv6 | IFCAP_CSUM_UDPv6);
		}

	if (hw->mac.type == e1000_82541 || hw->mac.type == e1000_82541_rev_2) {
		ifp->if_capabilities_tx &= ~(IFCAP_TSOv4 | IFCAP_TSOv6);
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "TSO disabled for this chipset");
		}

	if (hw->mac.type != e1000_82542) {
		i82544->ecom.ec_capabilities |= ETHERCAP_JUMBO_MTU;
		}

	hw->fc.requested_mode = e1000_fc_default;

	/* Provide our config details, informational only */
	i82544->cfg.serial_number = busindex;
	i82544->cfg.num_irqs = 1;
	if (i82544->cfg.irq[0] == 0)
		i82544->cfg.irq[0] = info.Irq;
	/* Only the 82574, 82576 & 82580 have been tested with MSI-X */
	if (i82544->msi_enabled == PCI_MSIX) {
		switch (hw->mac.type) {
			case	e1000_82574:
			case	e1000_82576:
			case	e1000_82580:
			case	e1000_82575:
			case	e1000_i350:
			case	e1000_i354:
			case	e1000_i210:
			case	e1000_i211:
				i82544->cfg.num_irqs = 3;
				i82544->cfg.irq[1] = info.Irq + 1;
				i82544->cfg.irq[2] = info.Irq + 2;
				break;
			default:
				break;
			}
		}

	i82544->cfg.mem_window_base[0] = PCI_MEM_ADDR(info.CpuBaseAddress[0]);
	i82544->cfg.mem_window_size[0] = info.BaseAddressSize[0];
	i82544->cfg.num_mem_windows = 1;

	if ((hw->hw_addr = mmap_device_memory (NULL, 0x20000,
	    PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED, 
	    PCI_MEM_ADDR(info.CpuBaseAddress[0]))) == MAP_FAILED) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "mmap_device_memory failed");
		err = errno;
		i82544_pci_detach_cleanup (i82544, 4);
		return (err);
		}

	for (i = 1; i <= 5; i++) {
		if (PCI_IS_IO (info.CpuBaseAddress [i])) {
			if ((hw->io_base = mmap_device_io (info.BaseAddressSize [i],
				PCI_IO_ADDR (info.CpuBaseAddress [i]))) == (uintptr_t) MAP_FAILED) {
				slogf (_SLOGC_NETWORK, _SLOG_ERROR, "mmap_device_io failed");
				err = errno;
				i82544_pci_detach_cleanup (i82544, 5);
				return (err);
				}
			break;
			}
		}

	if (((hw->mac.type == e1000_ich8lan) ||
		 (hw->mac.type == e1000_pchlan) ||
		 (hw->mac.type == e1000_pch2lan) ||
		 (hw->mac.type == e1000_pch_lpt) ||
		 (hw->mac.type == e1000_ich9lan) ||
		 (hw->mac.type == e1000_ich10lan)) &&
		 PCI_IS_MEM(info.CpuBaseAddress[1])) {
		i82544->flash_map_size = info.BaseAddressSize[1];
		hw->flash_address = mmap_device_memory (NULL, info.BaseAddressSize[1],
			PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED, 
			PCI_MEM_ADDR (info.CpuBaseAddress[1]));
		if (hw->flash_address == MAP_FAILED) {
			slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "Flash mmap failed!");
			i82544_pci_detach_cleanup (i82544, 5);
			return (ENODEV);
			}
		}

	/* This reset is to overcome a problem on some ICH10 chipsets where the
	   hardware semaphore doesn't reset */
	if (hw->mac.type == e1000_ich10lan || hw->mac.type == e1000_pch2lan) {
		dword = E1000_READ_REG(hw, E1000_CTRL);
		dword |= (E1000_CTRL_PHY_RST | E1000_CTRL_RST);
		E1000_WRITE_REG(hw, E1000_CTRL, dword);
		delay(20);
		}

	/* PR:17599 */
	i82544_clear_addresses (i82544);	

	if (e1000_setup_init_funcs (hw, TRUE)) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "Hardware Initialization Failure");
		i82544_pci_detach_cleanup (i82544, 6);
		return (ENODEV);
		}

	e1000_power_up_phy (hw);
	e1000_get_bus_info (hw);

	if (hw->mac.type == e1000_82541)
		e1000_init_script_state_82541 (hw, 1);
	if (hw->mac.type == e1000_82543)
		e1000_set_tbi_compatibility_82543 (hw, 1);

	hw->mac.autoneg = 1;
	hw->phy.autoneg_wait_to_complete = 0;
	hw->phy.autoneg_advertised = AUTONEG_ADVERTISE_SPEED_DEFAULT;

	if (hw->phy.media_type == e1000_media_type_copper) {
		hw->phy.mdix = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = 0;
		hw->phy.ms_type = E1000_MASTER_SLAVE;
		}

	if (force_speed) {
		// we get here, we know both media_rate and duplex are set - but to what?

		if (e1000_set_spd_dplx (hw)) {
			i82544_pci_detach_cleanup (i82544, 11);
			return (ENODEV);
			}
		}
	else {
		hw->mac.autoneg = 1;
		if (hw->phy.media_type == e1000_media_type_fiber ||
			hw->phy.media_type == e1000_media_type_internal_serdes) {
			hw->phy.autoneg_advertised = ADVERTISE_1000_FULL;
			}
		else {
			hw->phy.autoneg_advertised = E1000_ALL_SPEED_DUPLEX;
			}
		}

	if (e1000_check_reset_block (hw)) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "PHY reset is blocked");
		}

	switch (hw->mac.type) {
		case e1000_i350:
			/* Enable EEE */
			e1000_set_eee_i350 (hw);
			/* send driver version info to firmware */
			igb_init_fw (hw);
			break;
		case e1000_i354:
			e1000_set_eee_i354 (hw);
			break;
		default:
			break;
		}

	/*
	** Start from a known state, this is
	** important in reading the nvm and
	** mac from that.
	*/
	e1000_reset_hw (hw);

	/* Make sure we have a good EEPROM before we read from it */
	if (((hw->mac.type != e1000_i210) &&
		(hw->mac.type != e1000_i211)) &&
		(e1000_validate_nvm_checksum (hw) < 0)) {
		/*
		** Some PCI-E parts fail the first check due to
		** the link being in sleep state, call it again,
		** if it fails a second time its a real issue.
		*/
		if (e1000_validate_nvm_checksum (hw) < 0) {
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "The NVM Checksum is invalid");
			if (memcmp(i82544->cfg.current_address, "\0\0\0\0\0\0", 6) == 0) {
				i82544_pci_detach_cleanup (i82544, 6);
				return (ENODEV);
				}
			else {
				memcpy(hw->mac.addr, i82544->cfg.current_address, ETH_MAC_LEN);
				}
			}
		}

	if (e1000_read_mac_addr (hw)) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "NVM Read Error");
		}

	i82544_reset (i82544);

	hw->mac.get_link_status = 1;
	update_link_status (i82544);

	if (((hw->mac.type == e1000_82573) ||
		(hw->mac.type == e1000_ich8lan) ||
		(hw->mac.type == e1000_pchlan) ||
		(hw->mac.type == e1000_pch2lan) ||
		(hw->mac.type == e1000_82574) ||
		(hw->mac.type == e1000_ich10lan) ||
		(hw->mac.type == e1000_ich9lan)) &&
		e1000_check_mng_mode (hw)) {
		e1000_get_hw_control (hw);
		}
	else {
		if ((hw->mac.type == e1000_82575) ||
			(hw->mac.type == e1000_82576) ||
			(hw->mac.type == e1000_82580) ||
			(hw->mac.type == e1000_pch_lpt) ||
			(hw->mac.type == e1000_i350) ||
			(hw->mac.type == e1000_i354) ||
			(hw->mac.type == e1000_i210) ||
			(hw->mac.type == e1000_i211)) {
			e1000_get_hw_control (hw);
			}
		}

//	e1000_get_phy_info (hw);

	i82544->cfg.phy_addr = hw->phy.addr;
	hw->mac.adaptive_ifs = 1;

	/* Setup the i82544_dev_t with the info from the options */
	if (memcmp(i82544->cfg.current_address, "\0\0\0\0\0\0", 6) == 0) {
		memcpy(i82544->cfg.current_address, hw->mac.addr, ETH_MAC_LEN);
		memcpy(i82544->cfg.permanent_address, hw->mac.addr, ETH_MAC_LEN);
		}

	switch (hw->mac.type) {
		case	e1000_82544:
		case	e1000_82575:
		case	e1000_82576:
		case	e1000_82580:
		case	e1000_i350:
		case	e1000_i354:
		case	e1000_i210:
		case	e1000_i211:
			E1000_WRITE_REG (hw, E1000_WUC, 0);
			break;
		default:
			break;
		}

	if (hw->mac.type == e1000_82574) {
		dword = E1000_READ_REG (hw, E1000_RFCTL);
		dword |= E1000_RFCTL_ACK_DIS;
		E1000_WRITE_REG (hw, E1000_RFCTL, dword);
		}

	/* Check to see if chipset supports jumbo frames */
	if (cfg->mtu > ETH_MAX_PKT_LEN) {
		if (hw->mac.type == e1000_82542 || hw->mac.type == e1000_82583 || hw->mac.type == e1000_ich8lan) {
			slogf (_SLOGC_NETWORK, _SLOG_WARNING, "This adapter does not support jumbo packets");
			cfg->mtu = ETH_MAX_DATA_LEN;
			}
		}

	if (cfg->mtu >= 4096) {		/* Increase RX FIFO for large mtu sizes */
		dword = E1000_READ_REG (hw, E1000_PBA);
		if ((dword & 0xffff) < 0x10) {
			E1000_WRITE_REG (hw, E1000_PBA, 0x10);
			}
		}

	/* Allocate memory for the receive and transmit descriptors */
	i82544->rx_descriptor_area = mmap (0, i82544->num_receive * sizeof (i82544_rdesc_t),
#ifdef __X86__
	    PROT_READ | PROT_WRITE, MAP_PHYS | MAP_ANON | MAP_PRIVATE, NOFD, 0);
#else
	    PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_PHYS | MAP_ANON | MAP_PRIVATE, NOFD, 0);
#endif
	if (i82544->rx_descriptor_area == NULL) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "Unable to mmap rx descriptors");
		err = errno;
		i82544_pci_detach_cleanup (i82544, 6);
		return (err);
		}

	i82544->tx_descriptor_area = mmap (0, i82544->num_transmit * sizeof (i82544_tdesc_t),
#ifdef __X86__
	    PROT_READ | PROT_WRITE, MAP_PHYS | MAP_ANON | MAP_PRIVATE, NOFD, 0);
#else
	    PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_PHYS | MAP_ANON | MAP_PRIVATE, NOFD, 0);
#endif
	if (i82544->tx_descriptor_area == NULL) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "Unable to mmap tx descriptors");
		err = errno;
		i82544_pci_detach_cleanup (i82544, 7);
		return (err);
		}

	i82544->rdesc = i82544->rx_descriptor_area;
	i82544->tdesc = i82544->tx_descriptor_area;

	i82544->cachectl.fd = NOFD;

	if (cache_init (0, &i82544->cachectl, NULL) == -1) {
		err = errno;
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "cache_init error");
		i82544_pci_detach_cleanup (i82544, 8);
		return (err);
		}

	/* Allocate array of mbuf pointers for receiving */
	size = i82544->num_receive * sizeof(*i82544->rx_mbuf);
	if ((i82544->rx_mbuf = malloc (size, M_DEVBUF, M_NOWAIT)) == NULL) {
		err = errno;
		i82544_pci_detach_cleanup (i82544, 9);
		return err;
		}
	memset (i82544->rx_mbuf, 0x00, size);

	/* Allocate array of mbuf pointers for tracking pending transmit packets */
	size = i82544->num_transmit * sizeof(*i82544->tx_mbuf);
	if ((i82544->tx_mbuf = malloc (size, M_DEVBUF, M_NOWAIT)) == NULL) {
		err = errno;
		i82544_pci_detach_cleanup (i82544, 10);
		return err;
		}
	memset (i82544->tx_mbuf, 0x00, size);

	if ((err = interrupt_entry_init (&i82544->inter, 0, NULL,
	    i82544->cfg.priority)) != EOK) {
		i82544_pci_detach_cleanup (i82544, 11);
		return err;
		}
	if ((err = interrupt_entry_init (&i82544->inter_lnk, 0, NULL,
	    i82544->cfg.priority)) != EOK) {
		i82544_pci_detach_cleanup (i82544, 11);
		return err;
		}

	/* Checksumming is off by default */
	E1000_WRITE_REG (hw, E1000_RXCSUM, 0);

	strcpy ((char *) i82544->cfg.device_description, "INTEL PRO/1000 Gigabit ");
	if (hw->phy.media_type == e1000_media_type_fiber)
		strcat ((char *) i82544->cfg.device_description, "(Fiber)");
	else
		if (hw->phy.media_type == e1000_media_type_internal_serdes)
			strcat ((char *) i82544->cfg.device_description, "(SerDes)");
		else
			strcat ((char *) i82544->cfg.device_description, "(Copper)");

	// hook up so media devctls work
	bsd_mii_initmedia (i82544);

	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = i82544_ioctl;
	ifp->if_start = i82544_start;
	ifp->if_init = i82544_init;
	ifp->if_stop = i82544_stop;
	IFQ_SET_READY(&ifp->if_snd);


	if_attach (ifp);
	ether_ifattach (ifp, i82544->cfg.current_address);

	i82544->inter.func = i82544_process_interrupt;
	i82544->inter_lnk.func = i82544_process_interrupt;
	if (i82544->kermask == 0) {
		i82544->inter.enable = i82544_enable_interrupt;
		i82544->inter_lnk.enable = i82544_enable_interrupt;
		if (i82544->msi_enabled == PCI_MSIX) {
			i82544->inter.func = i82544_receive;
			i82544->inter_lnk.func = i82544_link_event;
			i82544->isrp = i82544_isr_rx;
			i82544->isrp_lnk = i82544_isr_lnk;
			i82544->inter.enable = i82544_rx_enable_interrupt;
			i82544->inter_lnk.enable = i82544_lnk_enable_interrupt;
			}
		else {
			i82544->isrp = i82544_isr;
			}
		}
	else {
		i82544->inter.enable = i82544_enable_interrupt_kermask;
		i82544->inter_lnk.enable = i82544_enable_interrupt_kermask;
		if (i82544->msi_enabled == PCI_MSIX) {
			i82544->isrp = i82544_isr_rx_kermask;
			i82544->isrp_lnk = i82544_isr_lnk_kermask;
			}
		else {
			i82544->isrp = i82544_isr_kermask;
			}
		}
	i82544->inter.arg = i82544;
	i82544->inter_lnk.arg = i82544;

	/* Set up a timer for housekeeping */
	callout_init (&i82544->hk_callout);

	i82544->sd_hook = shutdownhook_establish (i82544_shutdown, i82544);

	if (i82544->cfg.verbose) 
		nic_dump_config (&i82544->cfg);

	return (0);
}

/*****************************************************************************/
/*      Stop transmission on the interface.                                  */
/*****************************************************************************/

static void	i82544_stop (struct ifnet *ifp, int disable)

{
i82544_dev_t		*i82544 = ifp->if_softc;
int					i;
struct mbuf			*m;
struct _iopkt_self	*iopkt = i82544->iopkt;
struct nw_work_thread	*wtp = WTP;
struct e1000_hw		*hw = &i82544->hw;
uint32_t			swsm, ctrl_ext;

	if (i82544->cfg.verbose) {
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "devnp-e1000: i82544_stop() called, disable = %d\n", disable);
		}

	/* Shut off the housekeeping callout */
	callout_stop (&i82544->hk_callout);

	/* Don't want any more interrupts, thanks */
	E1000_WRITE_REG (hw, E1000_IMC, 0xffffffff);

	/* Shut off the receiver */
	E1000_WRITE_REG (hw, E1000_RCTL, 0);

	/* Shut off the transmitter */
	E1000_WRITE_REG (hw, E1000_TCTL, 0);

	/* Lock out the transmit side */
	NW_SIGLOCK_P (&ifp->if_snd_ex, iopkt, wtp);

	ifp->if_flags_tx |= IFF_OACTIVE;

	/* Release any queued transmit buffers. */
	for (i = 0; i < i82544->num_transmit; i++) {
		if ((m = i82544->tx_mbuf[i]) != NULL) {
			m_freem (m);
			i82544->tx_mbuf[i] = NULL;
			}
		}

   	/* clear cache so we send new offload context descriptor on next packet */
	i82544->last_cmdlen = 0;
	i82544->last_total_hdrlen = 0;
	i82544->last_mss = 0;

	ifp->if_flags_tx &= ~(IFF_RUNNING | IFF_OACTIVE);
	/* Done with the transmit side */
	NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);

	if (disable) {
		if (i82544->iid != -1) {
			InterruptDetach (i82544->iid);
			i82544->iid = -1;
			}
		if (i82544->iid_lnk != -1) {
			InterruptDetach (i82544->iid_lnk);
			i82544->iid_lnk = -1;
			}

		/*
		 * Make sure our callout didn't sneak on between 
		 * disable and detach (shared interrupt) and that
		 * it isn't in flight.
		 */

		interrupt_entry_remove (&i82544->inter, NULL); /* Must be 'the stack' to call this */
		if (i82544->msi_enabled == PCI_MSIX) {
			interrupt_entry_remove (&i82544->inter_lnk, NULL); /* Must be 'the stack' to call this */
			}

//		i82544_rxdrain (i82544);

		if (hw->mac.type == e1000_82573) {
			swsm = E1000_READ_REG (hw, E1000_SWSM);
			E1000_WRITE_REG (hw, E1000_SWSM, swsm & ~E1000_SWSM_DRV_LOAD);
			}
		else {
			ctrl_ext = E1000_READ_REG (hw, E1000_CTRL_EXT);
			E1000_WRITE_REG (hw, E1000_CTRL_EXT, ctrl_ext & ~E1000_CTRL_EXT_DRV_LOAD);
			}
		}

	/* Mark the interface as down and cancel the watchdog timer. */
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ifp->if_timer = 0;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void	i82544_shutdown (void *arg)

{
i82544_dev_t	*i82544 = (i82544_dev_t *) arg;

	i82544_stop (&i82544->ecom.ec_if, 1);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static void	i82544_rxdrain (i82544_dev_t *i82544)

{
int					i;
struct mbuf			*m;

	for (i = 0; i < i82544->num_receive; i++) {
		if ((m = i82544->rx_mbuf[i]) != NULL) {
			m_freem (m);
			i82544->rx_mbuf[i] = NULL;
			}
		}

	if ((m = i82544->rx_head) != NULL) {
		m_freem (m);
		i82544->rx_len  = 0;
		i82544->rx_head = NULL;
		i82544->rx_tail = &i82544->rx_head;
		}
}

/*****************************************************************************/
/* Configure MSI-X registers for various chipsets.                           */
/*****************************************************************************/

static	void setup_msix (i82544_dev_t *i82544)

{
uint32_t			tmp;
struct e1000_hw		*hw;

	hw = &i82544->hw;
	switch (hw->mac.type) {
		case	e1000_82574:
			tmp = E1000_READ_REG (hw, E1000_RFCTL);
			tmp |= E1000_RFCTL_ACK_DIS;
			E1000_WRITE_REG (hw, E1000_RFCTL, tmp);
			tmp = E1000_READ_REG (hw, E1000_CTRL_EXT);
			tmp |= E1000_CTRL_EXT_PBA_CLR;
			E1000_WRITE_REG (hw, E1000_IAM, ~E1000_EIAC_MASK_82574 | E1000_IMS_OTHER);
			tmp |= E1000_CTRL_EXT_EIAME;
			E1000_WRITE_REG (hw, E1000_CTRL_EXT, tmp);
			tmp = E1000_IVAR_INT_ALLOC_VALID;
			tmp |= ((E1000_IVAR_INT_ALLOC_VALID | 2) << 16);
			E1000_WRITE_REG (hw, E1000_IVAR, tmp);
			E1000_WRITE_FLUSH (hw);
			break;
		case	e1000_82576:
		case	e1000_82580:
		case	e1000_i350:
		case	e1000_i354:
		case	e1000_i210:
		case	e1000_i211:
		case	e1000_vfadapt:
		case	e1000_vfadapt_i350:
			E1000_WRITE_REG (hw, E1000_GPIE, E1000_GPIE_MSIX_MODE | E1000_GPIE_EIAME |
				E1000_GPIE_PBA | E1000_GPIE_NSICR);
			E1000_WRITE_REG (hw, E1000_EITR(0), i82544->rx_itr_val);
			tmp = E1000_READ_REG_ARRAY (hw, E1000_IVAR0, 0);
			tmp &= 0xffff0000;
			tmp |= E1000_IVAR_VALID;
			tmp |= (1 | E1000_IVAR_VALID) << 8;
			i82544->eims = 1;
			i82544->link_mask = 0x04;
			E1000_WRITE_REG_ARRAY (hw, E1000_IVAR0, 0, tmp);
			tmp = (2 | E1000_IVAR_VALID) << 8;
			E1000_WRITE_REG (hw, E1000_IVAR_MISC, tmp);
			break;
		case	e1000_82575:
			tmp = E1000_READ_REG (hw, E1000_CTRL_EXT);
			tmp |= E1000_CTRL_EXT_PBA_CLR;
			tmp |= E1000_CTRL_EXT_EIAME;
			tmp |= E1000_CTRL_EXT_IRCA;
			E1000_WRITE_REG (hw, E1000_CTRL_EXT, tmp);
			E1000_WRITE_REG (hw, E1000_EITR(0), i82544->rx_itr_val);
			E1000_WRITE_REG (hw, E1000_MSIXBM(0), E1000_EICR_RX_QUEUE0);
			E1000_WRITE_REG (hw, E1000_MSIXBM(2), E1000_EIMS_OTHER);
			i82544->eims = E1000_EICR_RX_QUEUE0;
			i82544->link_mask = E1000_EIMS_OTHER;
			break;
		default:
			break;
		}
}

/*****************************************************************************/
/* Bit of a misnomer, what this really means is                              */
/* to enable OS management of the system... aka                              */
/* to disable special hardware management features                           */
/*****************************************************************************/

static void	e1000_init_manageability(struct e1000_hw *hw)

{
	/* A shared code workaround */
#define E1000_82542_MANC2H E1000_MANC2H
	if (e1000_check_mng_mode (hw)) {
		int manc2h = E1000_READ_REG(hw, E1000_MANC2H);
		int manc = E1000_READ_REG(hw, E1000_MANC);

		/* disable hardware interception of ARP */
		manc &= ~(E1000_MANC_ARP_EN);

                /* enable receiving management packets to the host */
		manc |= E1000_MANC_EN_MNG2HOST;
#define E1000_MNG2HOST_PORT_623 (1 << 5)
#define E1000_MNG2HOST_PORT_664 (1 << 6)
		manc2h |= E1000_MNG2HOST_PORT_623;
		manc2h |= E1000_MNG2HOST_PORT_664;
		E1000_WRITE_REG(hw, E1000_MANC2H, manc2h);
		E1000_WRITE_REG(hw, E1000_MANC, manc);
		}
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static	void	i82544_set_promisc (i82544_dev_t *i82544, struct ifnet *ifp)

{
struct e1000_hw		*hw = &i82544->hw;
uint32_t			reg_rctl;

	reg_rctl = E1000_READ_REG (hw, E1000_RCTL);

	if (i82544->cfg.flags & NIC_FLAG_PROMISCUOUS) {
		reg_rctl |= (E1000_RCTL_UPE | E1000_RCTL_MPE);
		/* Turn this on if you want to see bad packets */
		E1000_WRITE_REG (hw, E1000_RCTL, reg_rctl);
	} else if (ifp->if_flags & IFF_ALLMULTI) {
		reg_rctl |= E1000_RCTL_MPE;
		reg_rctl &= ~E1000_RCTL_UPE;
		E1000_WRITE_REG (hw, E1000_RCTL, reg_rctl);
	}
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static	int	i82544_initialize_receive_unit (i82544_dev_t *i82544, struct ifnet *ifp)

{
uint32_t			rctl, rxdctl;
uint32_t			srrctl;
uint64_t			base;
int					i;
struct e1000_hw		*hw = &i82544->hw;
uint8_t				*mac = i82544->cfg.current_address;

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG (hw, E1000_RCTL);
	E1000_WRITE_REG (hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	/*
	 * Set the interrupt throttling rate. Value is calculated
	 * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
	 */
	if ((hw->mac.type != e1000_82575) && (hw->mac.type != e1000_82576) &&
		(hw->mac.type != e1000_82580)) {
		if ((i82544->itr_setting != 0) && (i82544->itr != 0))
			E1000_WRITE_REG (hw, E1000_ITR, i82544->rx_itr_val);
		else
			E1000_WRITE_REG (hw, E1000_ITR, 0);
		}

	/*
	** When using MSIX interrupts we need to throttle
	** using the EITR register (82574 only)
	*/
	if (hw->mac.type == e1000_82574)
		for (i = 0; i < 4; i++)
			E1000_WRITE_REG (hw, E1000_EITR_82574(i), i82544->rx_itr_val);

	/* Disable accelerated ackknowledge */
	if (hw->mac.type == e1000_82574)
		E1000_WRITE_REG (hw, E1000_RFCTL, E1000_RFCTL_ACK_DIS);

	/*
	** On some systems with 82573
	** long latencies are observed, like Lenovo X60. This
	** change eliminates the problem, but since having positive
	** values in RDTR is a known source of problems on other
	** platforms another solution is being sought.
	*/
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);

	if (mem_offset64 ((void *)i82544->rdesc, NOFD, 1, (off64_t *) &base, NULL) == -1)
		return -1;

	base += i82544->bmtrans;
	E1000_WRITE_REG (hw, E1000_RDBAL(0), base & 0xffffffff);
	E1000_WRITE_REG (hw, E1000_RDBAH(0), base >> 32);
	E1000_WRITE_REG (hw, E1000_RDLEN(0), (i82544->num_receive * sizeof (i82544_rdesc_t)));

	/* Use extended descriptor format */
	if (hw->mac.type >= e1000_82575) {
		srrctl = E1000_READ_REG (hw, E1000_SRRCTL (0));
		srrctl |= E1000_SRRCTL_DESCTYPE_ADV_ONEBUF;
		if (i82544->cfg.mtu > ETH_MAX_PKT_LEN)
			srrctl |= 4096 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
		else
			srrctl |= 2048 >> E1000_SRRCTL_BSIZEPKT_SHIFT;
		E1000_WRITE_REG (hw, E1000_SRRCTL (0), srrctl);
		}

	/* Set early receive threshold on appropriate hw */
	if (((hw->mac.type == e1000_ich9lan) ||
	    (hw->mac.type == e1000_pch2lan) ||
	    (hw->mac.type == e1000_ich10lan)) &&
	    (i82544->cfg.mtu > ETH_MAX_PKT_LEN)) {
		rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		E1000_WRITE_REG (hw, E1000_RXDCTL(0), rxdctl | 3);
		E1000_WRITE_REG (hw, E1000_ERT, 0x100 | (1 << 13));
		}
		
	/* Set MAC addr for received packet filtering purposes */
	E1000_WRITE_REG (hw, E1000_RAL(0), (mac[0] | mac[1] << 8 | mac[2] << 16 | mac[3] << 24));
	E1000_WRITE_REG (hw, E1000_RAH(0), (mac[4] | mac[5] << 8 | 1<<31));

	rxdctl = E1000_READ_REG (hw, E1000_RXDCTL (0));
	if (hw->mac.type >= e1000_82575) {
		rxdctl |= E1000_RXDCTL_QUEUE_ENABLE;
		rxdctl &= 0xfff00000;
		rxdctl |= IGB_RX_PTHRESH;
		rxdctl |= IGB_RX_HTHRESH << 8;
		rxdctl |= IGB_RX_WTHRESH << 16;
		E1000_WRITE_REG (hw, E1000_RXDCTL (0), rxdctl);
		}

	if (hw->mac.type == e1000_pch2lan) {
		if (i82544->cfg.mtu > ETH_MAX_PKT_LEN)
			e1000_lv_jumbo_workaround_ich8lan(hw, TRUE);
		else
			e1000_lv_jumbo_workaround_ich8lan(hw, FALSE);
	}

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM |
	    E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF |
	    (hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* Strip the CRC */
	rctl |= E1000_RCTL_SECRC;

	/* Make sure VLAN Filters are off */
	rctl &= ~E1000_RCTL_VFE;
	rctl &= ~E1000_RCTL_SBP;

	if ((hw->mac.type != e1000_82575) && (hw->mac.type != e1000_82576) &&
		(hw->mac.type != e1000_82580) && (hw->mac.type != e1000_i350)) {
		if (i82544->cfg.mtu == ETH_MAX_DATA_LEN)
			rctl |= E1000_RCTL_SZ_2048;
		else if (i82544->cfg.mtu <= 4096)
			rctl |= E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX;
		else if (i82544->cfg.mtu <= 8192)
			rctl |= E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX;
		}
	else {
		rctl &= ~(E1000_RCTL_SBP | E1000_RCTL_SZ_256);
		}

	if (i82544->cfg.mtu > ETH_MAX_PKT_LEN)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Workaround Si errata on 82577/82578 - configure IPG for jumbos */
	if ((hw->mac.type == e1000_pchlan) && (rctl & E1000_RCTL_LPE)) {
		uint32_t	mac_data;
		uint16_t	phy_data;

		e1e_rphy (hw, PHY_REG (770, 26), &phy_data);
		phy_data &= 0xfff8;
		phy_data |= (1 << 2);
		e1e_wphy (hw, PHY_REG (770, 26), phy_data);

		mac_data = er32 (FFLT_DBG);
		mac_data |= (1 << 17);
		ew32 (FFLT_DBG, mac_data);

		if (hw->phy.type == e1000_phy_82577) {
			e1e_rphy (hw, 22, &phy_data);
			phy_data &= 0x0fff;
			phy_data |= (1 << 14);
			e1e_wphy (hw, 0x10, 0x2823);
			e1e_wphy (hw, 0x11, 0x0003);
			e1e_wphy (hw, 22, phy_data);
			}
		}

	/* Write out the settings */
	E1000_WRITE_REG (hw, E1000_RCTL, rctl);
	E1000_WRITE_REG (hw, E1000_RDH(0), 0);
	E1000_WRITE_REG (hw, E1000_RDT (0), i82544->num_receive - 1);

	return (0);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static	int	i82544_initialize_transmit_unit (i82544_dev_t *i82544)

{
uint64_t		base;
uint32_t		tipg, tctl;
uint32_t		tarc, ctrl;
struct	e1000_hw	*hw = &i82544->hw;

	if (mem_offset64 ((void *)i82544->tdesc, NOFD, 1, (off64_t *) &base, NULL) == -1)
		return -1;

	base += i82544->bmtrans;
	E1000_WRITE_REG (hw, E1000_TDLEN(0), (i82544->num_transmit * sizeof (i82544_tdesc_t)));
	E1000_WRITE_REG (hw, E1000_TDBAH(0), base >> 32);
	E1000_WRITE_REG (hw, E1000_TDBAL(0), base & 0xffffffff);
	E1000_WRITE_REG (hw, E1000_TDH(0), 0);
	E1000_WRITE_REG (hw, E1000_TDT(0), 0);

	/* Enable the transmit queue */
	if (hw->mac.type >= e1000_82575) {
		ctrl = E1000_READ_REG (hw, E1000_TXDCTL (0));
		ctrl |= E1000_TXDCTL_QUEUE_ENABLE;
		E1000_WRITE_REG (hw, E1000_TXDCTL (0), ctrl);
		}

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (hw->mac.type) {
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		tipg |= DEFAULT_82542_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82542_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
		break;
	case e1000_80003es2lan:
		tipg = DEFAULT_82543_TIPG_IPGR1;
		tipg |= DEFAULT_80003ES2LAN_TIPG_IPGR2 <<
		    E1000_TIPG_IPGR2_SHIFT;
		break;
	default:
		if ((hw->phy.media_type == e1000_media_type_fiber) ||
		    (hw->phy.media_type ==
		    e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(hw, E1000_TIDV, 8);

//	if (hw->mac.type >= e1000_82540)
//		E1000_WRITE_REG(hw, E1000_TADV,
//		    adapter->tx_abs_int_delay.value);

	if ((hw->mac.type == e1000_82571) ||
	    (hw->mac.type == e1000_82572)) {
		tarc = E1000_READ_REG (hw, E1000_TARC(0));
		tarc |= SPEED_MODE_BIT;
		E1000_WRITE_REG(hw, E1000_TARC(0), tarc);
	} else if (hw->mac.type == e1000_80003es2lan) {
		tarc = E1000_READ_REG (hw, E1000_TARC(0));
		tarc |= 1;
		E1000_WRITE_REG (hw, E1000_TARC(0), tarc);
		tarc = E1000_READ_REG (hw, E1000_TARC(1));
		tarc |= 1;
		E1000_WRITE_REG (hw, E1000_TARC(1), tarc);
	}

//	adapter->txd_cmd = E1000_TXD_CMD_IFCS;
//	if (adapter->tx_int_delay.value > 0)
//		adapter->txd_cmd |= E1000_TXD_CMD_IDE;

	/* Initialize some software state */
	i82544->tx_free = i82544->num_transmit;

	if (i82544->vf_ifp)
		return (0);

	/* Program the Transmit Control Register */
	tctl = E1000_READ_REG (hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		   (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	if (hw->mac.type >= e1000_82571)
		tctl |= E1000_TCTL_MULR;

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG (hw, E1000_TCTL, tctl);

	return (0);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

static	int	i82544_init (struct ifnet *ifp)

{
i82544_dev_t			*i82544;
uint32_t				ctrl;
uint32_t				pba;
struct mbuf				*m;
int						i, err;
int						mtu_change = 0;
uint16_t				hwm;
off64_t					phys;
struct _iopkt_self		*iopkt;
struct nw_work_thread	*wtp;
struct e1000_hw			*hw;
nic_config_t    		*cfg;
union e1000_adv_rx_desc *adesc;

	i82544 = ifp->if_softc;
	iopkt = i82544->iopkt;
	wtp = WTP;
	hw = &i82544->hw;
	cfg = &i82544->cfg;

	if (i82544->dying == 1)
		return (0);

	i82544_stop (ifp, 0);

	NW_SIGLOCK_P (&ifp->if_snd_ex, iopkt, wtp);

	ifp->if_flags_tx |= IFF_OACTIVE;

	if (i82544->flow & IFM_ETH_FMASK) {
		if (i82544->flow & IFM_FLOW)
			i82544->pause_rx_disable = i82544->pause_tx_disable = 0;
		else {
			if (i82544->flow & IFM_ETH_RXPAUSE) {
				i82544->pause_rx_disable = 0;
				i82544->pause_tx_disable = 1;
				}
			if (i82544->flow & IFM_ETH_TXPAUSE) {
				i82544->pause_tx_disable = 0;
				i82544->pause_rx_disable = 1;
				}
			}
		
		}

	i82544->itr_setting = 3;
	if ((i82544->itr == 0) && !(i82544->itr_set))
		i82544->itr = 20000;
	if (cfg->verbose)
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "ITR set to %d", i82544->itr);
	if (i82544->itr != 0) {
		i82544->rx_itr_val = 1000000000 / (i82544->itr * 256);
		}

	switch (hw->mac.type) {
		case	e1000_82571:
		case	e1000_82572:
		case	e1000_82575:
		case	e1000_80003es2lan:
			pba = E1000_PBA_32K;
			break;
		case	e1000_82573:
			pba = E1000_PBA_12K;
			break;
		case	e1000_82574:
		case	e1000_82583:
			pba = E1000_PBA_20K;
			break;
		case	e1000_ich8lan:
			pba = E1000_PBA_8K;
			break;
		case	e1000_ich9lan:
		case	e1000_ich10lan:
			pba = E1000_PBA_10K;
			break;
		case	e1000_82576:
		case	e1000_vfadapt:
			pba = E1000_READ_REG (hw, E1000_RXPBS);
			pba &= E1000_RXPBS_SIZE_MASK_82576;
			break;
		case	e1000_82580:
		case	e1000_i350:
		case	e1000_i354:
		case	e1000_vfadapt_i350:
			pba = E1000_READ_REG (hw, E1000_RXPBS);
			pba = e1000_rxpbs_adjust_82580 (pba);
			break;
		case	e1000_pchlan:
		case	e1000_pch2lan:
			pba = E1000_PBA_26K;
			break;
		case	e1000_i210:
		case	e1000_i211:
			pba = E1000_PBA_34K;
			break;
		default:
			if (cfg->mtu > 8192)
				pba = E1000_PBA_40K;
			else
				pba = E1000_PBA_48K;
			break;
		}
	E1000_WRITE_REG (hw, E1000_PBA, pba);

	hw->max_frame_size = cfg->mtu + ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN + ETHER_CRC_LEN;

	e1000_rar_set (hw, hw->mac.addr, 0);

	hw->phy.autoneg_wait_to_complete = 1;
	i82544_reset (i82544);
	hw->phy.autoneg_wait_to_complete = 0;
	update_link_status (i82544);

	E1000_WRITE_REG (hw, E1000_VET, ETHERTYPE_VLAN);

	e1000_init_manageability (hw);

	if (ifp->if_mtu != i82544->cfg.mtu) {
		mtu_change = 1;
		}

	// get mtu from stack, mostly for nicinfo
	i82544->cfg.mtu = ifp->if_mtu;
	i82544->cfg.mru = ifp->if_mtu;

	i82544->cur_rx_rptr = 0;

	if (hw->mac.type == e1000_82571 || hw->mac.type == e1000_82572) {
		uint16_t	phy_tmp = 0;
		e1000_read_phy_reg (hw, IGP02E1000_PHY_POWER_MGMT, &phy_tmp);
		phy_tmp &= ~IGP02E1000_PM_SPD;
		e1000_write_phy_reg (hw, IGP02E1000_PHY_POWER_MGMT, phy_tmp);
		}

	if (i82544->link_config) {
		i82544->link_config = 0;
		if (i82544->force_link) {
			if (e1000_set_spd_dplx (hw)) {
				err = ENODEV;
				goto do_err;
				}
			i82544->force_link = 0;
			}
		else {
			hw->mac.autoneg = 1;
			if (hw->phy.media_type == e1000_media_type_fiber ||
				hw->phy.media_type == e1000_media_type_internal_serdes) {
				hw->phy.autoneg_advertised = ADVERTISE_1000_FULL;
				}
			else {
				hw->phy.autoneg_advertised = E1000_ALL_SPEED_DUPLEX;
				}
			}
		// initialize - until mii callback says we have a link ...
		cfg->flags |= NIC_FLAG_LINK_DOWN;
		}

	if (hw->mac.type == e1000_82580) {
		u32 reg;

		hwm = (E1000_PBA_35K << 10) - (2 * i82544->cfg.mtu);
		/*
		 * 0x80000000 - enable DMA COAL
		 * 0x10000000 - use L0s as low power
		 * 0x20000000 - use L1 as low power
		 * X << 16 - exit dma coal when rx data exceeds X kB
		 * Y - upper limit to stay in dma coal in units of 32usecs
		 */
		E1000_WRITE_REG(hw, E1000_DMACR,
		    0xA0000006 | ((hwm << 6) & 0x00FF0000));

		/* set hwm to PBA -  2 * max frame size */
		E1000_WRITE_REG(hw, E1000_FCRTC, hwm);
		/*
		 * This sets the time to wait before requesting transition to
		 * low power state to number of usecs needed to receive 1 512
		 * byte frame at gigabit line rate
		 */
		E1000_WRITE_REG(hw, E1000_DMCTLX, 4);

		/* free space in tx packet buffer to wake from DMA coal */
		E1000_WRITE_REG(hw, E1000_DMCTXTH,
		    (20480 - (2 * i82544->cfg.mtu)) >> 6);

		/* make low power state decision controlled by DMA coal */
		reg = E1000_READ_REG(hw, E1000_PCIEMISC);
		E1000_WRITE_REG(hw, E1000_PCIEMISC,
		    reg | E1000_PCIEMISC_LX_DECISION);
	}

	/* Pre-allocate a receive buffer for each receive descriptor */
	if (mtu_change) {
		i82544_rxdrain (i82544);
		mtu_change = 0;
		}
	for (i = 0; i < i82544->num_receive; i++) {
		if (i82544->rx_mbuf[i] != NULL)
			continue;
		m = m_getcl_wtp (M_DONTWAIT, MT_DATA, M_PKTHDR, wtp);
		if (m == NULL) {
			i++;
			break;
			}

		i82544->rx_mbuf[i] = m;
		phys = pool_phys (m->m_data, m->m_ext.ext_page);

		CACHE_INVAL (&i82544->cachectl, m->m_data, phys, m->m_ext.ext_size);

		/* The 82575, 82576, 82580 & i350 use extended descriptors */
		if (hw->mac.type >= e1000_82575) {
			adesc = (union e1000_adv_rx_desc *) &i82544->rdesc [i];
			adesc->read.pkt_addr = ENDIAN_LE64 (phys + i82544->bmtrans);
			adesc->read.hdr_addr = 0;
			}
		else {
			i82544->rdesc[i].bufaddr = ENDIAN_LE64 (phys + i82544->bmtrans);
			i82544->rdesc[i].status = 0;
			}
		}

	if (i82544_initialize_transmit_unit (i82544)) {
		err = ENOMEM;
		goto do_err;
		}
	if (i82544_initialize_receive_unit (i82544, ifp)) {
		err = ENOMEM;
		goto do_err;
		}
	i82544_set_promisc (i82544, ifp);

	i82544->cur_tx_wptr = i82544->cur_tx_rptr = 0;
	i82544->cur_rx_rptr = 0;

	ctrl = E1000_READ_REG (hw, E1000_RXCSUM) & ~((E1000_TXD_POPTS_IXSM | E1000_TXD_POPTS_TXSM) << E1000_TXD_POPTS_SHIFT);

	if (ifp->if_capenable_rx & IFCAP_CSUM_IPv4) {
		ctrl |= E1000_RXCSUM_IPOFL;
		}

	if (ifp->if_capenable_rx & (IFCAP_CSUM_TCPv4 | IFCAP_CSUM_UDPv4)) {
		ctrl |= E1000_RXCSUM_TUOFL;
		}

	E1000_WRITE_REG (hw, E1000_RXCSUM, ctrl);

	if (i82544->msi_enabled == PCI_MSIX) {
		setup_msix (i82544);
		}

	if (hw->mac.type >= e1000_82544 && hw->mac.type <= e1000_82547_rev_2 &&
		hw->mac.autoneg == 1 && hw->phy.autoneg_advertised == ADVERTISE_1000_FULL) {
		uint32_t	ctrl = E1000_READ_REG (hw, E1000_CTRL);
		ctrl &= ~E1000_CTRL_SWDPIN3;
		E1000_WRITE_REG (hw, E1000_CTRL, ctrl);
		}

	if (((hw->mac.type == e1000_82573) ||
		(hw->mac.type == e1000_ich8lan) ||
		(hw->mac.type == e1000_pchlan) ||
		(hw->mac.type == e1000_pch2lan) ||
		(hw->mac.type == e1000_82574) ||
		(hw->mac.type == e1000_ich10lan) ||
		(hw->mac.type == e1000_ich9lan)) &&
		e1000_check_mng_mode (hw)) {
		e1000_get_hw_control (hw);
		}
	else {
		if ((hw->mac.type == e1000_82575) ||
			(hw->mac.type == e1000_82576) ||
			(hw->mac.type == e1000_82580) ||
			(hw->mac.type == e1000_pch_lpt) ||
			(hw->mac.type == e1000_i350) ||
			(hw->mac.type == e1000_i354) ||
			(hw->mac.type == e1000_i210) ||
			(hw->mac.type == e1000_i211)) {
			e1000_get_hw_control (hw);
			}
		}

	/* Set multicast or promiscuous */
	i82544_filter (i82544);

	e1000_clear_hw_cntrs_base_generic (hw);

	/* Everything should be up.  Attach to the interrupt */
	if (i82544->iid == -1) {
		if ((err = InterruptAttach_r (i82544->cfg.irq[0], i82544->isrp,
		    i82544, sizeof (*i82544), _NTO_INTR_FLAGS_TRK_MSK)) < 0) {
			err = -err;
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "devnp-e1000: InterruptAttach: %d", err);
			goto do_err;
			}

		i82544->iid = err;
		}

	if (i82544->msi_enabled == PCI_MSIX) {
		if (i82544->iid_lnk == -1) {
			if ((err = InterruptAttach_r (i82544->cfg.irq[2], i82544->isrp_lnk,
			    i82544, sizeof (*i82544), _NTO_INTR_FLAGS_TRK_MSK)) < 0) {
				err = -err;
				slogf (_SLOGC_NETWORK, _SLOG_ERROR, "devnp-e1000: InterruptAttach1: %d", err);
				goto do_err;
				}

			i82544->iid_lnk = err;
			}
		if (hw->mac.type == e1000_82574) {
			i82544->eiac_mask |= E1000_EIAC_MASK_82574;
//			E1000_WRITE_REG (hw, E1000_EIAC_82574, E1000_EIAC_MASK_82574);
			}
		if (hw->mac.type >= e1000_82575) {
			E1000_WRITE_REG (hw, E1000_EIAC, 0x05);
			E1000_WRITE_REG (hw, E1000_EIAM, 0x05);
			E1000_WRITE_REG (hw, E1000_EIMS, 0x05);
			}
		}

	/* Interesting interrupts */
	if (i82544->msi_enabled == PCI_MSIX) {
		if (hw->mac.type == e1000_82574) {
			i82544->intrmask = (E1000_ICR_OTHER | E1000_ICR_LSC | i82544->eiac_mask);
			}
		else {
			i82544->intrmask = E1000_ICR_LSC;
			}
		}
	else {
		i82544->intrmask = (E1000_ICR_LSC | E1000_ICR_RXT0);
		}

	if (hw->mac.type == e1000_pch_lpt)
		E1000_WRITE_REG (hw, E1000_IMS, i82544->intrmask | E1000_IMS_ECCER);
	else
		E1000_WRITE_REG (hw, E1000_IMS, i82544->intrmask);

	ifp->if_flags_tx |= IFF_RUNNING;
	ifp->if_flags_tx &= ~IFF_OACTIVE;
	NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);
	ifp->if_flags |= IFF_RUNNING;

	// start housekeeping callout - give it 10 seconds
	callout_msec (&i82544->hk_callout, 10 * 1000, i82544_hk_callout, i82544);

	hw->phy.reset_disable = 1;

	i82544->i82544_if_flags = ifp->if_flags;

	return (0);

do_err:
	ifp->if_flags_tx &= ~IFF_OACTIVE;
	NW_SIGUNLOCK_P (&ifp->if_snd_ex, iopkt, wtp);

	slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: not running.", ifp->if_xname);

	return (err);
}




#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/init.c $ $Rev: 751115 $")
#endif
