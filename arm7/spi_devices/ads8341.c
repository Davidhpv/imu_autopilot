/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file Driver for ADS8341 analog to digital converter
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#include "ads8341.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "conf.h"
#include "spi.h"

uint16_t ads8341_value[3][4];
uint8_t ads8341_current_channel;
uint8_t ads8341_current_adc_id;

static void ads8341_unselect(void);
static void ads8341_select(void);
static void ads8341_on_spi_int(void);


void ads8341_init(void)
{
	/* configure SS pin */
	ADS8341_0_SS_IODIR |= 1 << ADS8341_0_SS_PIN; /* pin is output  */
	ads8341_current_adc_id = ADS8341_0;
	ads8341_unselect(); /* pin idles high */

	//ADS8341_1_SS_IODIR |= 1 << ADS8341_1_SS_PIN; /* pin is output  */
	//ads8341_current_adc_id = ADS8341_1;
	//ads8341_unselect(); /* pin idles high */

	//ADS8341_2_SS_IODIR |= 1 << ADS8341_2_SS_PIN; /* pin is output  */
	//ads8341_current_adc_id = ADS8341_2;
	//ads8341_unselect(); /* pin idles high */
}
static void ads8341_unselect(void)
{
	if (ads8341_current_adc_id == ADS8341_0){
		ADS8341_0_SS_IOSET |= 1 << ADS8341_0_SS_PIN;
	}
	/*case ADS8341_1:
		ADS8341_1_SS_IOSET |= 1 << ADS8341_1_SS_PIN;
		break;
	case ADS8341_2:
		ADS8341_2_SS_IOSET |= 1 << ADS8341_2_SS_PIN;
		break;
	}*/
}
static void ads8341_select(void)
{
	if (ads8341_current_adc_id == ADS8341_0){
		ADS8341_0_SS_IOCLR |= 1 << ADS8341_0_SS_PIN;
	}
	/*case ADS8341_1:
		ADS8341_1_SS_IOCLR |= 1 << ADS8341_1_SS_PIN;
		break;
	case ADS8341_2:
		ADS8341_2_SS_IOCLR |= 1 << ADS8341_2_SS_PIN;
		break;
	}*/
}


void ads8341_read(uint8_t adc_id, uint8_t channel)
{
	ads8341_current_channel=channel;
	ads8341_current_adc_id=adc_id;
	unsigned char cmd1 = (1 << 4);
	switch (channel)
	{
	case 0:
		cmd1 |= (1 << 1);
		break;
	case 1:
		cmd1 |= (1 << 3 | 1 << 1);
		break;
	case 2:
		cmd1 |= (1 << 2);
		break;
	case 3:
		cmd1 |= (1 << 3 | 1 << 2);
		break;
	default:
		return;
	}
	unsigned char cmd2 = (1 << 4) | (1 << 3) | (1 << 2);
	spi_package package;
	package.bit_mode = SPI_5_BIT_MODE;
	package.data[0] = cmd1;
	package.data[1] = cmd2;
	package.data[2] = 0;
	package.data[3] = 0;
	package.data[4] = 0;
	package.length = 5;
	package.slave_select = &ads8341_select;
	package.slave_unselect = &ads8341_unselect;
	package.spi_interrupt_handler = &ads8341_on_spi_int;
	spi_transmit(&package);

}

static void ads8341_on_spi_int(void)
{
	unsigned char foo __attribute__ ((unused)) = SSPDR;
	unsigned int data = (SSPDR&1)<<15;
	data += SSPDR<<10;
	data += SSPDR<<5;
	data += SSPDR<<0;
	ads8341_value[ads8341_current_adc_id][ads8341_current_channel] = data;
}

// TODO Enable the use of multiple ADCs of the same type
uint16_t ads8341_get_value(uint8_t adc_id, uint8_t channel)
{
	return ads8341_value[adc_id][channel];
}

