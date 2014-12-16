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

#include	<i82544.h>

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void e1000_pci_set_mwi(struct e1000_hw *hw)

{
uint16_t	 	cmd;
i82544_dev_t	*i82544 = hw->i82544;

	pci_read_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Command), 1, 2, &cmd);
	cmd |= PCI_COMMAND_INVALIDATE_ENABLE;
	pci_write_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Command), 1, 2, &cmd);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void e1000_pci_clear_mwi(struct e1000_hw *hw)

{
uint16_t		 cmd;
i82544_dev_t	*i82544 = hw->i82544;

	pci_read_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Command), 1, 2, &cmd);
	cmd &= ~PCI_COMMAND_INVALIDATE_ENABLE;
	pci_write_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Command), 1, 2, &cmd);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void e1000_read_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *value)

{
i82544_dev_t	*i82544 = hw->i82544;

	pci_read_config (i82544->pci_dev_hdl, reg, 1, 2, value);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void e1000_write_pci_cfg(struct e1000_hw *hw, uint32_t reg, uint16_t *value)

{
i82544_dev_t	*i82544 = hw->i82544;

	pci_write_config (i82544->pci_dev_hdl, reg, 1, 2, value);
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

int	e1000_read_pcie_cap_reg (struct e1000_hw *hw, uint32_t reg, uint16_t *value)

{
uint8_t			cap_offset = 0;
uint8_t			cap;
i82544_dev_t	*i82544 = hw->i82544;

	pci_read_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Capabilities_Pointer), 1, 1, &cap_offset);
	if (!cap_offset)
		return -E1000_ERR_CONFIG;
	while (1) {
		if (pci_read_config (i82544->pci_dev_hdl, cap_offset, 1, 1, &cap))
			return (-E1000_ERR_CONFIG);
		if (cap == PCI_CAP_PCI_EXPRESS)
			break;
		if (pci_read_config (i82544->pci_dev_hdl, cap_offset + 1, 1, 1, &cap_offset))
			return (-E1000_ERR_CONFIG);
		if (!cap_offset)
			return (-E1000_ERR_CONFIG);
		}

	pci_read_config (i82544->pci_dev_hdl, cap_offset + reg, 1, 2, value);

	return E1000_SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

int	e1000_write_pcie_cap_reg (struct e1000_hw *hw, uint32_t reg, uint16_t *value)

{
uint8_t			cap_offset = 0;
uint8_t			cap;
i82544_dev_t	*i82544 = hw->i82544;

	pci_read_config (i82544->pci_dev_hdl, offsetof (struct _pci_config_regs, Capabilities_Pointer), 1, 1, &cap_offset);
	if (!cap_offset)
		return -E1000_ERR_CONFIG;
	while (1) {
		if (pci_read_config (i82544->pci_dev_hdl, cap_offset, 1, 1, &cap))
			return (-E1000_ERR_CONFIG);
		if (cap == PCI_CAP_PCI_EXPRESS)
			break;
		if (pci_read_config (i82544->pci_dev_hdl, cap_offset + 1, 1, 1, &cap_offset))
			return (-E1000_ERR_CONFIG);
		if (!cap_offset)
			return (-E1000_ERR_CONFIG);
		}

	pci_write_config (i82544->pci_dev_hdl, cap_offset + reg, 1, 2, value);

	return E1000_SUCCESS;
}




#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/e1000_main.c $ $Rev: 708496 $")
#endif
