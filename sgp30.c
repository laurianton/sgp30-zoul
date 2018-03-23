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
 */
/*---------------------------------------------------------------------------*/
/**
 *  zoul-sgp30-sensor
 *  Driver for the SGP30 CO2 sensor
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "dev/i2c.h"
#include "sgp30.h"

uint8_t  
SGP30_init(void)
{
  uint8_t data[] = {0x20, 0x03};

  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);

                                                           /* initialize measurement */
/* if(i2c_single_send(SGP30_ADDR, SGP30_INIT) == I2C_MASTER_ERR_NONE)  */

  if (i2c_burst_send(SGP30_ADDR, data, 2) == I2C_MASTER_ERR_NONE)
  {
    return I2C_MASTER_ERR_NONE;
  }
  return i2c_master_error();
}

/*---------------------------------------------------------------------------*/

/** \brief Send a command  */
uint8_t SGP30_send_command(uint8_t command[])
{
  /* Send the command to the sensor */
  if (i2c_burst_send(SGP30_ADDR, command, 2) == I2C_MASTER_ERR_NONE) 
  {
    return I2C_MASTER_ERR_NONE;
  }
  return i2c_master_error();
}


/** \brief Send command to start air quality reading */
uint8_t SGP30_measure_air_quality(void)
{
  /* Send command 0x2008 to the sensor to start reading */

  uint8_t data[] = {0x20, 0x08};

  if (i2c_burst_send(SGP30_ADDR, data, 2) == I2C_MASTER_ERR_NONE) 
  {
    return I2C_MASTER_ERR_NONE;
  }
  return i2c_master_error();
}

uint8_t
SGP30_read(uint16_t* co2_reading, uint16_t* tvoc_reading)
{
  uint8_t buf[6], crc1, crc2;
  uint16_t m_CO2eq, m_TVOC;

  /* Read six bytes only */
  if(i2c_burst_receive(SGP30_ADDR, buf, 6) == I2C_MASTER_ERR_NONE) {
    m_CO2eq = (buf[0] << 8) + (buf[1]);
    crc1 = buf[2];
    m_TVOC = (buf[3] << 8) + (buf[4]);
    crc2 = buf[5];
   
/* CRC checking is missing */
 
/*    printf("CO2 %u	0x%X	crc 0x%X	TVOC %u	0x%X crc 0x%X\n",m_CO2eq, m_CO2eq, crc1, m_TVOC, m_TVOC, crc2); */

    *co2_reading = m_CO2eq;
    *tvoc_reading = m_TVOC;
    return I2C_MASTER_ERR_NONE;
  }
  return i2c_master_error();
}
/*---------------------------------------------------------------------------*/
/** @} */
