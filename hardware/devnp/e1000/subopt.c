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

static char *i82544_opts[] = {
	"receive",           // 0
	"transmit",          // 1
	"pause_rx_disable",  // 2
	"pause_tx_disable",  // 3
	"pause_rx_enable",   // 4
	"pause_tx_enable",   // 5
	"kermask",           // 6
	"rx_delay",          // 7
	"rx_abs",            // 8
	"irq_thresh",        // 9
	"tx_reap",			 // 10
	"force_link",		 // 11
	"int_mod",			 // 12
	"max_read",			 // 13
	NULL
};

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

int 	i82544_parse_options (i82544_dev_t *i82544, const char *optstring, nic_config_t *cfg)

{
char    *value, *options, *freeptr, *c;
int     opt, invalid, rc = EOK;
int		tmp;

	if (optstring == NULL)
		return 0;

	/* getsubopt() is destructive */
	options = malloc (strlen (optstring) + 1, M_TEMP, M_NOWAIT);
	if (options == NULL)
		return ENOMEM;
	strcpy (options, optstring);
	freeptr = options;

	while (options && *options != '\0') {
		c = options;
		invalid = 0;
		if ((opt = getsubopt (&options, i82544_opts, &value)) != -1) {
			if (i82544 == NULL)
				continue;

			switch (opt) {
				case	0:
					i82544->num_receive = strtoul(value, 0, 0);
					break;

				case	1:
					i82544->num_transmit = strtoul(value, 0, 0);
					break;

				case	2: // disregard received pause (flow control) frames
					i82544->pause_rx_disable = 1;
					break;

				case	3:  // do not transmit pause (flow control) frames
					i82544->pause_tx_disable = 1;
					break;

				case	4: // always act on received pause (flow control) frames
					i82544->pause_rx_enable = 1;
					break;

				case	5:  // always transmit pause (flow control) frames
					i82544->pause_tx_enable = 1;
					break;

				case	6:
					if (value != NULL) {
						i82544->kermask = strtol (value, 0, 0);
						if (i82544->kermask != 0)
							i82544->kermask = 1;
						}
					break;

				case	7:
					if (value != NULL) {
						i82544->rx_delay = strtol (value, 0, 0);
						i82544->rx_delay = min (i82544->rx_delay, 50);
						}
					break;

				case	8:
					if (value != NULL) {
						i82544->rx_abs = strtol (value, 0, 0);
						i82544->rx_abs = min (i82544->rx_abs, 200);
						}
					break;

				case	9:
					if (value != NULL) {
						i82544->irq_thresh = strtol (value, 0, 0);
						}
					break;

				case	10:
					if (value != NULL) {
						i82544->tx_reap = strtol (value, 0, 0);
						if (i82544->tx_reap == 0)
							i82544->tx_reap = DEFAULT_TX_REAP;
						}
					break;

				case	11:
					i82544->cmd_force = 1;
					break;

				case	12:
					if (value != NULL) {
						i82544->itr = strtol (value, 0, 0);
						i82544->itr_set = 1;
						}
					break;

				case	13:
					if (value != NULL) {
						tmp = strtol (value, 0, 0);
						switch (tmp) {
							case	128:
								i82544->max_read = 0;
								break;
							case	256:
								i82544->max_read = 1;
								break;
							case	512:
								i82544->max_read = 2;
								break;
							case	1024:
								i82544->max_read = 3;
								break;
							case	2048:
								i82544->max_read = 4;
								break;
							case	4096:
								i82544->max_read = 5;
								break;
							default:
								slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "Invalid max read request size %d", tmp);
								free (freeptr, M_TEMP);
								return (EINVAL);
							}
								
						}
					break;

				default:
					rc = EINVAL;
					invalid = 1;
					break;
					}
				}
			else
				if (nic_parse_options (cfg, value) != EOK) {
					rc = EINVAL;
					invalid = 1;
					}

		if (invalid) {
			slogf (_SLOGC_NETWORK, _SLOG_WARNING, "devnp-e1000: unknown option %s", c);
			}
		}

	free (freeptr, M_TEMP);

	return rc;
}



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.5.0/trunk/lib/io-pkt/sys/dev_qnx/e1000/subopt.c $ $Rev: 708496 $")
#endif
