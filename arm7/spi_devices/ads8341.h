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

#ifndef ADS8341_H_
#define ADS8341_H_
#include "inttypes.h"

void ads8341_init(void);
uint16_t ads8341_get_value(uint8_t adc_id, uint8_t channel);
void ads8341_read(uint8_t adc_id, uint8_t channel);

#endif /* ADS8341_H_ */
