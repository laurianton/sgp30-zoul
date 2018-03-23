/*
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-sensors
 * @{
 *
 * \defgroup zoul-sgp30-sensor SGP30 Sensor
 *
 * Driver for the SGP30 sensor
 *
 *
 * \file
 * Header file for the SGP30 Sensor Driver
 */

/*

Python example

import smbus
import time

bus = smbus.SMBus(1)
address = 0x58

bus.write_i2c_block_data(address, 0x20, [0x03])
time.sleep(.5)

while 1:
   bus.write_i2c_block_data(address, 0x20, [0x08])
   time.sleep(.6)
   data = bus.read_i2c_block_data(address, 0)
   m1 = (data[0] << 8) + data[1]
   m2 = (data[3] << 8) + data[4]
   print m1, " CRC",data[2], "  ", m2, " CRC", data[5]
   time.sleep(5)

*/

/*
* SGP30 General data
* I2C address		0x58
* Init_air_quality	0x2003
* Measure_air_quality	0x2008
*/
/*---------------------------------------------------------------------------*/
#ifndef SGP30_H_
#define SGP30_H_
#include <stdio.h>
#include "i2c.h"
/* -------------------------------------------------------------------------- */
/**
 * \name Generic SGP30 sensor
 * @{
 */
/* -------------------------------------------------------------------------- */
#define SGP30_ADDR           0x58 /**< SGP30 slave address */
#define SGP30_INIT           0x2003 /**< SGP30 data register */
#define SGP30_MEASURE        0x2008 /**< SGP30 data register */
/** @} */
/* -------------------------------------------------------------------------- */
#endif /* ifndef SGP30_H_ */
/*---------------------------------------------------------------------------*/

/** \brief Initialiser for the SGP30 sensor driver */
uint8_t SGP30_init(void);

/** \brief Send read command  */
uint8_t SGP30_send_command(void);

/** \brief Get a reading from the SGP30 sensor */
uint8_t SGP30_read(void);

/**
 * @}
 * @}
 */
