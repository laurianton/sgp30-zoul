/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * SGP30 sensor demo
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/sgp30.h"
#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define ADC_PIN             2
#define LOOP_PERIOD         2
#define LOOP_INTERVAL       (CLOCK_SECOND * LOOP_PERIOD)
#define LEDS_PERIODIC       LEDS_GREEN
#define BUTTON_PRESS_EVENT_INTERVAL (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
static struct etimer et;

static uint16_t counter;

uint8_t send_out, read_in;

/*---------------------------------------------------------------------------*/
PROCESS(test_sgp30_sensor_process, "test SGP30 sensor process");
AUTOSTART_PROCESSES(&test_sgp30_sensor_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_sgp30_sensor_process, ev, data)
{

  PROCESS_BEGIN();

  counter = 0;

  /* Configure the ADC ports */
  /* Use pin number not mask, for example if using the PA5 pin then use 5 */
  /*adc_sensors.configure(ANALOG_SGP30_SENSOR, ADC_PIN);*/

  printf("SGP30 test application\n");
  leds_on(LEDS_BLUE);
  
  printf("SGP30 init %u\n", SGP30_init() );
  etimer_set(&et, CLOCK_SECOND);

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      leds_set(LEDS_GREEN);

      printf("Counter 0x%X	", counter);

      send_out = SGP30_send_command();
      if (send_out) 
      {
         printf("ERROR:  send_out %u\n",send_out); 
      }

      etimer_set(&et, CLOCK_SECOND );
      counter++;
    }

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    if(ev == PROCESS_EVENT_TIMER) {

      read_in = SGP30_read();
      if (read_in) 
      {
         printf("ERROR:   read_in %u\n",read_in); 
      }

      leds_set(LEDS_YELLOW);

      etimer_set(&et, CLOCK_SECOND);
      counter++;
    }
    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

