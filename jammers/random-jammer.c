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

/**
 * \file
 *          A proactive random jammer that alters between two different states.
 *          It does either jam or stay concealed.
 *          It uses nrfs52840's direct test mode in order to emit random bits.
 *
 * \author
 *          John Kanwar <johnkanwar@hotmail.com>
 */

#include "contiki.h"

#include <stdio.h>
#include "dev/radio.h"
#include "net/netstack.h"
#include "packetbuf.h"
#include "ble_dtm.h"
#include "sys/timer.h"
#include "boards.h"
#include "random.h"

#define RANDOM_MIN 2 //Will be divided with 10 so it is actually 0.2
#define RANDOM_MAX 20
#define BLE_CHANNEL 39
#define PACKET_LENGTH 254

/*---------------------------------------------------------------------------*/
PROCESS(random_jammer_process, "random_jammer_process");
AUTOSTART_PROCESSES(&random_jammer_process);

/*---------------------------------------------------------------------------*/
static struct timer timer_timer;
static int is_jamming = 0;
static int is_init = 0;
/*---------------------------------------------------------------------------*/
static uint32_t
dtm_cmd_put(int channel_input, int cmd_input, int length_input)
{
  dtm_cmd_t command_code = cmd_input;
  dtm_freq_t channel = channel_input;
  uint32_t length = length_input;
  dtm_pkt_type_t payload = DTM_PKT_PRBS9;
  return dtm_cmd(command_code, channel, length, payload);
}
/*---------------------------------------------------------------------------*/
static void
start_jamming(void)
{
  dtm_cmd_put(BLE_CHANNEL, LE_TRANSMITTER_TEST, PACKET_LENGTH);
  is_jamming = 1;
}
/*---------------------------------------------------------------------------*/
static void
stop_jamming(void)
{
  dtm_cmd_put(BLE_CHANNEL, LE_TEST_END,PACKET_LENGTH);
  is_init = 0;
  is_jamming = 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(random_jammer_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();
  printf("%d netstack_mac_off\n", NETSTACK_MAC.off());
  printf("%d netstack_radio_on\n", NETSTACK_RADIO.on());
  random_init(2);
  timer_set(&timer_timer, CLOCK_SECOND * random_rand() % ((RANDOM_MAX + 1) - RANDOM_MIN) + RANDOM_MIN);
  float jamming_duration = 0;

  etimer_set(&et, CLOCK_SECOND * random_rand() % ((RANDOM_MAX + 1) - RANDOM_MIN) + RANDOM_MIN);

  while(true) {

    /*Jam for n random amount of time*/
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    if(is_jamming) {
      stop_jamming();
      printf("Stop\n");
    } else {
      if(!is_init) {
	dtm_init();
	is_init = 1;
      }
      start_jamming();
      printf("Start\n");
    }
    jamming_duration = (float)(random_rand() % ((RANDOM_MAX + 1) - RANDOM_MIN) + RANDOM_MIN) / 10;
    etimer_set(&et, CLOCK_SECOND * jamming_duration);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
