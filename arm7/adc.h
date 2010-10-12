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
* @file
*   @brief Driver for LPC2000 series ADC
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#ifndef ADC_H_
#define ADC_H_

#include "inttypes.h"
#include "conf.h"

#if defined(BOARD_PIXHAWK_V100) || defined(BOARD_TWOG_BOOZ)
#define ADC_BAT_VDC_CHANNEL			4
#define ADC_BPLANCD_IO_0_CHANNEL	0
#define ADC_BPLANCD_IO_2_CHANNEL	2
#endif



#if (FEATURE_ADC==FEATURE_ADC_PIXHAWK)
#define ADC_BAT_VDC_CHANNEL			5
#define ADC_3_CHANNEL				6
#define ADC_5_CHANNEL				2
#define ADC_6_CHANNEL				1
#define ADC_7_CHANNEL				3
#endif

void adc_init(void);

uint16_t adc_get_value(uint8_t channel);

#endif /*ADC_H_*/
