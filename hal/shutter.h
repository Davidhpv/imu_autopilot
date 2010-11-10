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
* @file Camera shutter triggering
*   @author Lorenz Meier
*
*/

#ifndef SHUTTER_H_
#define SHUTTER_H_

#include <stdbool.h>
#include <inttypes.h>

void shutter_init(void);

void shutter_control(uint8_t enable);

void shutter_set(uint32_t shutter_interval, uint32_t exposure);

uint32_t shutter_get_seq(void);
bool shutter_loop(void);

#endif /* SHUTTER_H_ */
