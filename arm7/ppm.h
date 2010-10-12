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

#ifndef PPM_H
#define PPM_H

#include "LPC21xx.h"
int ppm_get_channel(unsigned int nr);

/**
 * @brief Initialize the ppm input.
 */
void ppm_init(void);

/**
 * @brief Check if ppm information is valid
 */
int ppm_is_valid(void);

/**
 * @brief PPM interrupt routine. This routine is called by systime if a timer capture interrupt has occurred.
 */
void PPM_ISR(void);

#endif /* PPM_H */
