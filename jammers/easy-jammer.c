/*
 * Copyright (c) 2021, RISE Research Institutes of Sweden AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *	Easy interference.
 *	Transmitts packages into the air on a specific channel.
 */

/*
 *	Problem:
 *	Affects all the channels.
 */

#include "contiki.h"

#include <stdio.h>
#include "dev/radio.h"
#include "net/netstack.h"
#include "packetbuf.h"

/*---------------------------------------------------------------------------*/
PROCESS(easy_interference_process, "easy_interference process");
AUTOSTART_PROCESSES(&easy_interference_process);

/*---------------------------------------------------------------------------*/
static radio_value_t channel_c = 26;

/*---------------------------------------------------------------------------*/
static void
set_channel(void)
{
  printf("Set channel\n");
  if(NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, channel_c) != RADIO_RESULT_OK) {
    printf("ERROR: failed to change radio channel\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
run_transmit(void)
{
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen()); // 0 == OK
  NETSTACK_RADIO.transmit(120);	// 0 == OK
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(easy_interference_process, ev, data)
{
  PROCESS_BEGIN();

  set_channel();
  /* RSSI:-49-49-91-49-49-93-49-49-92-49-49-92-49-49-92-49 */ //On distance
  printf("%d netstack_mac_off\n", NETSTACK_MAC.off());
  printf("%d netstack_radio_on\n", NETSTACK_RADIO.on());

  //NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, RADIO_CONST_TXPOWER_MAX); //Activating this makes the interference weaker.

  while(1) {
    run_transmit();
    PROCESS_PAUSE();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
