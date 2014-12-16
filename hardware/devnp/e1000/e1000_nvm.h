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

#ifndef _E1000_NVM_H_
#define _E1000_NVM_H_

struct e1000_pba {
	u16 word[2];
	u16 *pba_block;
};

struct e1000_fw_version {
	u32 etrack_id;
	u16 eep_major;
	u16 eep_minor;
	u16	eep_build;

	u8 invm_major;
	u8 invm_minor;
	u8 invm_img_type;

	bool or_valid;
	u16 or_major;
	u16 or_build;
	u16 or_patch;
};


void e1000_init_nvm_ops_generic(struct e1000_hw *hw);
s32  e1000_null_read_nvm(struct e1000_hw *hw, u16 a, u16 b, u16 *c);
void e1000_null_nvm_generic(struct e1000_hw *hw);
s32  e1000_null_led_default(struct e1000_hw *hw, u16 *data);
s32  e1000_null_write_nvm(struct e1000_hw *hw, u16 a, u16 b, u16 *c);
s32  e1000_acquire_nvm_generic(struct e1000_hw *hw);

s32  e1000_poll_eerd_eewr_done(struct e1000_hw *hw, int ee_reg);
s32  e1000_read_mac_addr_generic(struct e1000_hw *hw);
s32  e1000_read_pba_num_generic(struct e1000_hw *hw, u32 *pba_num);
s32  e1000_read_pba_string_generic(struct e1000_hw *hw, u8 *pba_num,
                                   u32 pba_num_size);
s32  e1000_read_pba_length_generic(struct e1000_hw *hw, u32 *pba_num_size);
s32  e1000_read_nvm_spi(struct e1000_hw *hw, u16 offset, u16 words, u16 *data);
s32  e1000_read_nvm_microwire(struct e1000_hw *hw, u16 offset,
                              u16 words, u16 *data);
s32  e1000_read_nvm_eerd(struct e1000_hw *hw, u16 offset, u16 words,
                         u16 *data);
s32  e1000_valid_led_default_generic(struct e1000_hw *hw, u16 *data);
s32  e1000_validate_nvm_checksum_generic(struct e1000_hw *hw);
s32  e1000_write_nvm_eewr(struct e1000_hw *hw, u16 offset,
                          u16 words, u16 *data);
s32  e1000_write_nvm_microwire(struct e1000_hw *hw, u16 offset,
                               u16 words, u16 *data);
s32  e1000_write_nvm_spi(struct e1000_hw *hw, u16 offset, u16 words,
                         u16 *data);
s32  e1000_update_nvm_checksum_generic(struct e1000_hw *hw);
void e1000_stop_nvm(struct e1000_hw *hw);
void e1000_release_nvm_generic(struct e1000_hw *hw);
void e1000_get_fw_version(struct e1000_hw *hw,
			  struct e1000_fw_version *fw_vers);

#define E1000_STM_OPCODE  0xDB00

#endif



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/e1000_nvm.h $ $Rev: 751115 $")
#endif
