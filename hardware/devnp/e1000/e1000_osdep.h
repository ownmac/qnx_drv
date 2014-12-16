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


/* glue for the OS-dependent part of e1000
 * includes register access macros
 */

#ifndef _E1000_OSDEP_H_
#define _E1000_OSDEP_H_

#include <io-pkt/iopkt_driver.h>
#include <stdio.h>
#include <errno.h>
#include <atomic.h>
#include <unistd.h>
//#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/siginfo.h>
#include <sys/syspage.h>
#include <sys/neutrino.h>
#include <sys/dcmd_io-net.h>
#include <sys/mbuf.h>
#include <sys/slogcodes.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
//#include <sys/io-pkt.h>
#include <sys/cache.h>
#include <hw/inout.h>
#include <drvr/mdi.h>
#include <drvr/eth.h>
#include <drvr/nicsupport.h>
#define  _STDDEF_H_INCLUDED
#include <hw/pci.h>
#include <hw/pci_devices.h>
#include <stdbool.h>

#define usec_delay(x) nanospin_ns(x * 1000)
#ifndef msec_delay
#define msec_delay(x)	delay(x)

/* Some workarounds require millisecond delays and are run during interrupt
 * context.  Most notably, when establishing link, the phy may need tweaking
 * but cannot process phy register reads/writes faster than millisecond
 * intervals...and we establish link due to a "link status change" interrupt.
 */
#define msec_delay_irq(x) delay(x)
#endif

#define	u8		uint8_t
#define	u16		uint16_t
#define	s16		short
#define	u32		uint32_t
#define	u64		uint64_t
#define s64		int64_t
#define	s32		int
//#define	bool	int

#define	E1000_MUTEX		pthread_mutex_t
#define	E1000_MUTEX_INIT(mutex)		pthread_mutex_init(mutex,NULL)
#define	E1000_MUTEX_DESTROY(mutex)	pthread_mutex_destroy(mutex)
#define	E1000_MUTEX_LOCK(mutex)		pthread_mutex_lock(mutex)
#define	E1000_MUTEX_TRYLOCK(mutex)	pthread_mutex_trylock(mutex)
#define	E1000_MUTEX_UNLOCK(mutex)	pthread_mutex_unlock(mutex)

#define PCI_COMMAND_REGISTER   offsetof(struct _pci_config_regs, Command)
#define CMD_MEM_WRT_INVALIDATE PCI_COMMAND_INVALIDATE_ENABLE
//#define ETH_ADDR_LEN           ETH_MAC_LEN

#ifdef __BIG_ENDIAN
#define E1000_BIG_ENDIAN __BIG_ENDIAN
#endif


#define	DEBUGFUNC(_x)	do { \
						if (hw->verbose > 3) \
						slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, _x); \
						} while (0);
#define	DEBUGOUT(_x)	do { \
						if (hw->verbose > 3) \
						slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, _x); \
						} while (0);
#define	DEBUGOUT1(_x, _y)	do { \
						if (hw->verbose > 3) \
						slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, _x, _y); \
						} while (0);
#define	DEBUGOUT2(_x, _y, _z)	do { \
						if (hw->verbose > 3) \
						slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, _x, _y, _z); \
						} while (0);

#define	DEBUGOUT3(_w, _x, _y, _z)	do { \
						if (hw->verbose > 3) \
						slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, _w, _x, _y, _z); \
						} while (0);

static inline unsigned char readb(const volatile void *addr)
{
	return *(volatile unsigned char *) addr;
}
static inline unsigned short readw(const volatile void *addr)
{
	return ENDIAN_LE16 (*(volatile unsigned short *) addr);
}
static inline unsigned int readl(const volatile void *addr)
{
	return ENDIAN_LE32 (*(volatile unsigned int *) addr);
}

static inline void writeb(unsigned char b, volatile void *addr)
{
	*(volatile unsigned char *) addr = b;
}
static inline void writew(unsigned short b, volatile void *addr)
{
	*(volatile unsigned short *) addr = ENDIAN_LE16(b);
}
static inline void writel(unsigned int b, volatile void *addr)
{
	*(volatile unsigned int *) addr = ENDIAN_LE32(b);
}

#define E1000_REGISTER(a, reg) (((a)->mac.type >= e1000_82543) \
                               ? reg                           \
                               : e1000_translate_register_82542(reg))

#define E1000_WRITE_REG(a, reg, value) ( \
    writel((value), ((a)->hw_addr + E1000_REGISTER(a, reg))))

#define E1000_READ_REG(a, reg) (readl((a)->hw_addr + E1000_REGISTER(a, reg)))

#define E1000_WRITE_REG_ARRAY(a, reg, offset, value) ( \
    writel((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 2))))

#define E1000_READ_REG_ARRAY(a, reg, offset) ( \
    readl((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 2)))

#define E1000_READ_REG_ARRAY_DWORD E1000_READ_REG_ARRAY
#define E1000_WRITE_REG_ARRAY_DWORD E1000_WRITE_REG_ARRAY

#define E1000_WRITE_REG_ARRAY_WORD(a, reg, offset, value) ( \
    writew((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 1))))

#define E1000_READ_REG_ARRAY_WORD(a, reg, offset) ( \
    readw((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 1)))

#define E1000_WRITE_REG_ARRAY_BYTE(a, reg, offset, value) ( \
    writeb((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + (offset))))

#define E1000_READ_REG_ARRAY_BYTE(a, reg, offset) ( \
    readb((a)->hw_addr + E1000_REGISTER(a, reg) + (offset)))

#define E1000_WRITE_REG_IO(a, reg, offset) do { \
    outle32(((a)->io_base), reg);                  \
    outle32(((a)->io_base + 4), offset);      } while(0)

#define E1000_WRITE_FLUSH(a) E1000_READ_REG(a, E1000_STATUS)

#define E1000_WRITE_FLASH_REG(a, reg, value) ( \
    writel((value), ((a)->flash_address + reg)))

#define E1000_WRITE_FLASH_REG16(a, reg, value) ( \
    writew((value), ((a)->flash_address + reg)))

#define E1000_READ_FLASH_REG(a, reg) (readl((a)->flash_address + reg))

#define E1000_READ_FLASH_REG16(a, reg) (readw((a)->flash_address + reg))

# define do_div(n,base) ({					\
	uint32_t __base = (base);				\
	uint32_t __rem;						\
	__rem = ((uint64_t)(n)) % __base;			\
	(n) = ((uint64_t)(n)) / __base;				\
	__rem;							\
 })

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define max_t(type,x,y) ({ \
	type _x = (x); \
	type _y = (y); \
	_x > _y ? _x : _y; })

#endif /* _E1000_OSDEP_H_ */



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/e1000_osdep.h $ $Rev: 708496 $")
#endif
