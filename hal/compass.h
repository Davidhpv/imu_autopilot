/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann
Contributing Authors (in alphabetical order):
  Dominik Honegger
  Tobias Naegeli



(c) 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/
/**
* @file compass.h
*
* @brief get magnet compass values
*
* This file contains a functions to get values from the magnet compass.
*
**/

#include "ms2100.h"
#include <math.h>

#ifndef _COMPASS_H_
#define _COMPASS_H_

static inline float compass_get_angle_si(void){
	int x = ms2100_get_value(MS2100_X_AXIS);
	int y = ms2100_get_value(MS2100_Y_AXIS);
	float angle = atan2(y,x);
	return angle;
}

#endif /* COMPASS_H_ */
