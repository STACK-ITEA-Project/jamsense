/**
 * \file
 *          A Contiki application that detects multiple
 *          concurrent channel activity in the 2.4 GHz band.
 *          The application works on low-power sensor node platforms
 *          that feature the cc2420 radio from Texas Instruments.
 *
 * \author
 *          Venkatraman Iyer <iyer.venkat9@gmail.com>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
#include <stdlib.h>
#include <string.h>
#include "dev/radio.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "lib/random.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "kmeans.h"

/*RPL-udp CLIENT includes*/
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"

#include "sys/log.h"
#include "dev/radio.h"
#include "net/netstack.h"
#include "dev/leds.h"
#include "dev/etc/rgb-led/rgb-led.h"

/*---------------------------------------------------------------------------*/

/*Jamming detection, could be placed in a makefile*/
#if J_D == 1
#define RSSI_SIZE 140
#define POWER_LEVELS 20
#else
#define RSSI_SIZE 120
#define POWER_LEVELS 16
#endif
#define MAX_DURATION 1000

/*RPL-UDP + SpeckSense++ variables*/
/*Could be in a sperate file*/
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO
#define WITH_SERVER_REPLY 1
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678
#define SEND_INTERVAL (60 * CLOCK_SECOND)
#define INIT_MSG a

/*---------------------------------------------------------------------------*/
#if PROCESS_ID == 0
PROCESS(specksense, "SpeckSense");
// AUTOSTART_PROCESSES(&specksense);
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);
#elif PROCESS_ID == 1
PROCESS(specksense, "SpeckSense");
AUTOSTART_PROCESSES(&specksense);
#elif PROCESS_ID == 2
PROCESS(channel_allocation, "Spectrum monitoring");
AUTOSTART_PROCESSES(&channel_allocation);
#else
#error "Choose a valid process 1. specksense on RADIO_CHANNEL, 2. scanning all channels."
#endif
/*---------------------------------------------------------------------------*/

//! Global variables

static int print_counter = 0;

static uint16_t n_clusters;

//! Global variables for RSSI scan
static int rssi_val, /*rssi_valB,*/ rle_ptr = -1, /*rle_ptrB = -1,*/
		   step_count = 1, cond, itr,
		   n_samples, max_samples, rssi_val_mod;

static unsigned rssi_levels[RSSI_SIZE];
static rtimer_clock_t sample_st, sample_end;
static struct record record;


#if CHANNEL_METRIC == 2
static uint16_t cidx;
static int itr_j;
static int current_channel = RADIO_CHANNEL;
#endif

static void print_rssi_rle()
{
	int i;
	printf("RSSI");
	for (i = 0; i <= rle_ptr; i++)
		printf(" (%d,%d)", record.rssi_rle[i][0], record.rssi_rle[i][1]);
	printf("\nrle_ptr:%d,%d,%d\n", rle_ptr, max_samples, n_samples);
}

/*---------------------------------------------------------------------------*/
static void rssi_sampler(int time_window_ms)
{
	// sample_st = RTIMER_NOW();
	max_samples = time_window_ms * 20;
	step_count = 1;
	rle_ptr = -1;
	record.sequence_num = 0;
	int globalCounter = 0;
	print_counter = 0;

	printf("\n START \n");

	// for (int i = 0; i < (RUN_LENGTH); i++)
	// {
	// 	// printf("Old values: [0]: %d  [1]: %d \n", record.rssi_rle[i][0],record.rssi_rle[i][1]);

	// 	record.rssi_rle[i][0] = 0;
	// 	record.rssi_rle[i][1] = 0;
	// }

	record.sequence_num++;
	rle_ptr = -1;
	record.rssi_rle[0][1] = 0;
	record.rssi_rle[0][0] = 0;
	n_samples = max_samples * 10;
	//   watchdog_stop();
	print_counter = 0;
	int times = 0;
	watchdog_periodic();

	while ((rle_ptr < RUN_LENGTH))
	{
		times++;
		/*Get RSSI value*/

		/*Start time*/
		if (NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &rssi_val) != RADIO_RESULT_OK)
		{
			printf(" ff");
		}

		rssi_val -= 45; /* compensation offset */
		int16_t debug_rssi = rssi_val;

		/*If power level is <= 2 set it to power level 1*/
		if (debug_rssi <= -132)
		{
			debug_rssi = -139;
		}

		/*Power level most be higher than one */
		if (rssi_levels[-debug_rssi - 1] > 1)
		{
			n_samples = n_samples - 1;
			cond = 0x01 & ((record.rssi_rle[rle_ptr][0] != rssi_levels[-rssi_val - 1]) | (record.rssi_rle[rle_ptr][1] == 32767));

			/*Max_duration achieved, move to next value*/
			if (record.rssi_rle[rle_ptr][1] >= MAX_DURATION)
			{
				cond = 1; /*Jump to next value*/
			}

			/*Increase rle_ptr when new powerlevel starts recording.*/
			rle_ptr = rle_ptr + cond;
			rssi_val_mod = -rssi_val - 1;

			/*Check out of bounds*/
			if (rssi_val_mod >= 140)
			{
				rssi_val_mod = 139;
				// printf("out of loop I guess\n");
			}

			/*Create 2D vector*/
			record.rssi_rle[rle_ptr][0] = rssi_levels[rssi_val_mod];
			record.rssi_rle[rle_ptr][1] = (record.rssi_rle[rle_ptr][1]) * (1 - cond) + 1;
		}
		else
		{ /*I think a problem might be that it loops here without printing anything for a very long amount of time. */
			if (rle_ptr == 498){}
		}
	}
	printf("This is how many times the loop looped: %d \n", times);
	watchdog_start();

	// sample_end = RTIMER_NOW();
	if (rle_ptr < RUN_LENGTH)
		rle_ptr++;

	printf("\nNumber of sampels %d : rle_ptr %d\n", globalCounter, rle_ptr);
	printf(" \n");
	printf(" \n");
}
/*---------------------------------------------------------------------------*/

static int values_read = 0;
/*---------------------------------------------------------------------------*/
static void rssi_sampler_old(int time_window_ms)
{

	sample_st = RTIMER_NOW();
	max_samples = time_window_ms * 20;
	step_count = 1;
	rle_ptr = -1;
	record.sequence_num = 0;
	int times = 0;

	record.sequence_num++;
	rle_ptr = -1;
	record.rssi_rle[0][1] = 0;
	record.rssi_rle[0][0] = 0;
	n_samples = max_samples * 10;

	while ((rle_ptr < RUN_LENGTH) && (n_samples))
	{
		if (NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &rssi_val) != RADIO_RESULT_OK)
		{
			printf(" ff");
		}

		values_read += 1;
		if (rssi_levels[-rssi_val] > 2)
		{

			n_samples = n_samples - 1;

			rssi_val = ((signed char)rssi_val);
			cond = 0x01 & ((record.rssi_rle[rle_ptr][0] != rssi_levels[-rssi_val - 1]) | (record.rssi_rle[rle_ptr][1] == 32767));

			rle_ptr = rle_ptr + cond;
			record.rssi_rle[rle_ptr][0] = rssi_levels[-rssi_val - 1];
			record.rssi_rle[rle_ptr][1] = (record.rssi_rle[rle_ptr][1]) * (1 - cond) + 1;
		}
	}

	printf("This is how many times the loop looped: %d \n", times);
	watchdog_start();
	step_count++;

	if (rle_ptr < RUN_LENGTH)
	{
		rle_ptr++;
	}

	// printf("\nNumber of sampels %d : rle_ptr %d\n", globalCounter, rle_ptr);
	printf(" \n");

	sample_end = RTIMER_NOW();
}

static void init_power_levels()
{
	printf("POWER_LEVELS: %d", POWER_LEVELS);
#if POWER_LEVELS == 2
	for (itr = 0; itr < 120; itr++)
		if (itr < 90)
			rssi_levels[itr] = 2;
		else
			rssi_levels[itr] = 1;
#elif POWER_LEVELS == 4
	for (itr = 0; itr < 120; itr++)
		if (itr < 30)
			rssi_levels[itr] = 4;
		else if (itr >= 30 && itr < 60)
			rssi_levels[itr] = 3;
		else if (itr >= 60 && itr < 90)
			rssi_levels[itr] = 2;
		else
			rssi_levels[itr] = 1;
#elif POWER_LEVELS == 8
	for (itr = 0; itr < 120; itr++)
		if (itr < 14)
			rssi_levels[itr] = 8;
		else if (itr >= 12 && itr < 25)
			rssi_levels[itr] = 7;
		else if (itr >= 25 && itr < 38)
			rssi_levels[itr] = 6;
		else if (itr >= 38 && itr < 51)
			rssi_levels[itr] = 5;
		else if (itr >= 51 && itr < 64)
			rssi_levels[itr] = 4;
		else if (itr >= 64 && itr < 77)
			rssi_levels[itr] = 3;
		else if (itr >= 77 && itr < 90)
			rssi_levels[itr] = 2;
		else
			rssi_levels[itr] = 1;
#elif POWER_LEVELS == 16
	for (itr = 0; itr < 120; itr++)
		if (itr < 6)
			rssi_levels[itr] = 16;
		else if (itr >= 6 && itr < 12)
			rssi_levels[itr] = 15;
		else if (itr >= 12 && itr < 18)
			rssi_levels[itr] = 14;
		else if (itr >= 18 && itr < 24)
			rssi_levels[itr] = 13;
		else if (itr >= 24 && itr < 30)
			rssi_levels[itr] = 12;
		else if (itr >= 30 && itr < 36)
			rssi_levels[itr] = 11;
		else if (itr >= 36 && itr < 42)
			rssi_levels[itr] = 10;
		else if (itr >= 42 && itr < 48)
			rssi_levels[itr] = 9;
		else if (itr >= 48 && itr < 54)
			rssi_levels[itr] = 8;
		else if (itr >= 54 && itr < 60)
			rssi_levels[itr] = 7;
		else if (itr >= 60 && itr < 66)
			rssi_levels[itr] = 6;
		else if (itr >= 66 && itr < 72)
			rssi_levels[itr] = 5;
		else if (itr >= 72 && itr < 78)
			rssi_levels[itr] = 4;
		else if (itr >= 78 && itr < 84)
			rssi_levels[itr] = 3;
		else if (itr >= 84 && itr < 90)
			rssi_levels[itr] = 2;
		else
			rssi_levels[itr] = 1;
#elif POWER_LEVELS == 120
	int i_c = 140 - 1;
	for (itr = 1; itr <= 140; itr++)
	{
		rssi_levels[i_c] = itr;
		i_c--;
	}
#elif POWER_LEVELS == 20 /*Power level is read in the opposite way. */
	for (itr = 0; itr < 140; itr++)
		if (/*itr >= 60 &&*/ itr < 64)
			rssi_levels[itr] = 20;
		else if (itr >= 64 && itr < 68)
			rssi_levels[itr] = 19;
		else if (itr >= 68 && itr < 72)
			rssi_levels[itr] = 18;
		else if (itr >= 72 && itr < 76)
			rssi_levels[itr] = 17;
		else if (itr >= 76 && itr < 80)
			rssi_levels[itr] = 16;
		else if (itr >= 80 && itr < 84)
			rssi_levels[itr] = 15;
		else if (itr >= 84 && itr < 88)
			rssi_levels[itr] = 14;
		else if (itr >= 88 && itr < 92)
			rssi_levels[itr] = 13;
		else if (itr >= 92 && itr < 96)
			rssi_levels[itr] = 12;
		else if (itr >= 96 && itr < 100)
			rssi_levels[itr] = 11;
		else if (itr >= 100 && itr < 104)
			rssi_levels[itr] = 10;
		else if (itr >= 104 && itr < 108)
			rssi_levels[itr] = 9;
		else if (itr >= 108 && itr < 112)
			rssi_levels[itr] = 8;
		else if (itr >= 112 && itr < 116)
			rssi_levels[itr] = 7;
		else if (itr >= 116 && itr < 120)
			rssi_levels[itr] = 6;
		else if (itr >= 120 && itr < 124)
			rssi_levels[itr] = 5;
		else if (itr >= 124 && itr < 128)
			rssi_levels[itr] = 4;
		else if (itr >= 128 && itr < 132)
			rssi_levels[itr] = 3;
		else if (itr >= 132 && itr < 136)
			rssi_levels[itr] = 2;
		else if (itr >= 136 && itr < 140)
			rssi_levels[itr] = 1;
		else
			rssi_levels[itr] = 1; // Will never happen

#else
#error "Power levels should be one of the following values: 2, 4, 8, 16 or 120"
#endif
}

/*Just add a make file for which to use*/
/*---------------------------------------------------------------------------*/
#if PROCESS_ID == 0
PROCESS_THREAD(udp_client_process, ev, data)
{
	static struct etimer et;
	static unsigned count;
	static char str[32];
	static int nr_failures = 0;

	PROCESS_BEGIN();

	NETSTACK_MAC.off();
	NETSTACK_RADIO.on();
	if (NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RADIO_CHANNEL) != RADIO_RESULT_OK)
	{
		printf("ERROR: failed to change radio channel\n");
		break;
	}
	/* Initialize UDP connection */
	simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
						UDP_SERVER_PORT, udp_rx_callback);

	etimer_set(&et, CLOCK_SECOND * 0.5);
	while (1)
	{

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		etimer_reset(&et);

		if (NETSTACK_ROUTING.node_is_reachable())
		{
			printf("First one down\n");
			if (NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr))
			{
				nr_failures = 0;

				if (!init)
				{
					LOG_INFO("Init function!! \n");

					/*Send init*/
					udp_init_send();
					init = 1;
				}
				else
				{

					/*Flash led light green*/
					rgb_led_set(RGB_LED_GREEN);
					LOG_INFO("This is the client! \n");
					LOG_INFO("Sending request %u to ", count);
					LOG_INFO_6ADDR(&dest_ipaddr);
					LOG_INFO_("\n");
					snprintf(str, sizeof(str), "5 %d", count);
					simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
					count++;
				}
			}
			else
			{
				LOG_INFO("Didn't get root_ipaddr\n");
			}
		}
		else
		{
			rgb_led_set(RGB_LED_RED);
			LOG_INFO("Not reachable yet: %d\n", cnt);
			if (nr_failures > 20)
			{
				nr_failures = 0;
				process_start(&specksense, NULL);
				PROCESS_EXIT();
			}
			nr_failures++;
			cnt++;
		}

		rgb_led_off();
	}

	PROCESS_END();
}
PROCESS_THREAD(specksense, ev, data)
{
	static int nr_times = 0;
	PROCESS_BEGIN();
	NETSTACK_MAC.off();
	NETSTACK_RADIO.on();

	watchdog_start();
	init_power_levels();

	if (NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RADIO_CHANNEL) != RADIO_RESULT_OK)
	{
		printf("ERROR: failed to change radio channel\n");
		break;
	}
	etimer_set(&et, CLOCK_SECOND * 2);

	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	while (nr_times < 10)
	{
		record.sequence_num = 0;

		rssi_sampler(TIME_WINDOW);
		//  leds_off(LEDS_GREEN);
		//  leds_off(LEDS_RED);

		n_clusters = kmeans(&record, rle_ptr);
		check_similarity(PROFILING);
		nr_times++;
		PROCESS_PAUSE();
	}
	nr_times = 0;
	process_start(&udp_client_process, NULL);
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#elif PROCESS_ID == 1
PROCESS_THREAD(specksense, ev, data)
{
	//    static struct etimer et;

	PROCESS_BEGIN();
	NETSTACK_MAC.off();
	NETSTACK_RADIO.on();

	watchdog_start();
	init_power_levels(); // Populate the rssi quantization levels.

	if (NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RADIO_CHANNEL) != RADIO_RESULT_OK)
	{
		printf("ERROR: failed to change radio channel\n");
		break;
	}

	while (1)
	{

		record.sequence_num = 0;
		/*
			They need to have the same format after the RSSI sampler in order to be as one process
		*/

		// printf("J_D: %d RSSI_SIZE:%d POWER_LEVELS: %d \n", J_D, RSSI_SIZE, POWER_LEVELS);
		if (J_D)
		{
			rssi_sampler(TIME_WINDOW);
			if (0)
			{
				print_rssi_rle();
			}
			n_clusters = kmeans(&record, rle_ptr);
			check_similarity(/*PROFILING*/ 0);
			printf("Number of cluster %d\n", n_clusters);
		}
		else
		{
			rssi_sampler_old(TIME_WINDOW);
			n_clusters = kmeans_old(&record, rle_ptr);
			// print_rssi_rle();
			check_unintentional_interference(n_clusters);
			// print_interarrival(RADIO_CHANNEL, n_clusters);
		}

		PROCESS_PAUSE();
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#elif PROCESS_ID == 2
PROCESS_THREAD(channel_allocation, ev, data)
{
	PROCESS_BEGIN();
	NETSTACK_MAC.off();
	NETSTACK_RADIO.on();
	init_power_levels();

	for (itr = 0; itr < 16; itr++)
		channel_metric[itr] = 0;

	cidx = 11;
	// cc2420_set_channel(cidx);
	if (NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, cidx) != RADIO_RESULT_OK)
	{
		printf("ERROR: failed to change radio channel\n");
		break;
	}
	itr = 0;
	while (itr < 3)
	{

		record.sequence_num = 0;

		leds_on(LEDS_RED);
		rssi_sampler(TIME_WINDOW);
#if DEBUG_RSSI == 1
		print_rssi_rle();
#endif
		//   watchdog_stop();
#if CHANNEL_METRIC == 1

		printf("Channel %d:", cidx); /*TODO get the radio channel.*/

		n_clusters = kmeans(&record, rle_ptr);
		print_interarrival(cidx, n_clusters);
#elif CHANNEL_METRIC == 2
		channel_metric[cidx - 11] = channel_metric[cidx - 11] +
									channel_metric_rssi_threshold(&record, rle_ptr);
#endif
		channel_rate(&record, n_clusters);
		watchdog_start();

		//   leds_off(LEDS_RED);
		NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &current_channel)
			cidx = (current_channel == 26) ? 11 : current_channel + 1;
		NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, cidx)
			//   cc2420_set_channel(cidx);
			if (cidx == 11)
				itr++;
	}
#if CHANNEL_METRIC == 2
	for (itr = 0; itr < 16; itr++)
	{
		channel_metric[itr] = channel_metric[itr] / 3.0;
		printf("Channel %d: Channel metric: %ld.%03u\n",
			   itr + 11, (long)channel_metric[itr],
			   (unsigned)((channel_metric[itr] -
						   floor(channel_metric[itr])) *
						  1000));
	}

	for (itr = 0; itr < 15; itr++)
		for (itr_j = itr + 1; itr_j < 16; itr_j++)
			if (channel_metric[itr] > channel_metric[itr_j])
			{
				int tmp_channel;
				float tmp_val = channel_metric[itr];

				channel_metric[itr] = channel_metric[itr_j];
				channel_metric[itr_j] = tmp_val;

				tmp_channel = channel_arr[itr];
				channel_arr[itr] = channel_arr[itr_j];
				channel_arr[itr_j] = tmp_channel;
			}

	printf("Channel ordering:");
	for (itr = 15; itr >= 0; itr--)
	{
		printf(" %d", channel_arr[itr]);
		if (itr)
			printf(",");
	}

#endif
	PROCESS_END();
}
#endif
