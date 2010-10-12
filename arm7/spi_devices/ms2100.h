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
* @file Driver for MS2100 magnetometer
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#ifndef MS2100_H_
#define MS2100_H_

#include "conf.h"
#include "inttypes.h"

#define MS2100_RESET_TIMEOUT_BASE	((PCLK/10000000UL)?(PCLK/10000000UL):1)

//other functions/////////////////////////////////////////////////////////
void ms2100_init(void);
void ms2100_send_request(uint8_t axis);
short ms2100_get_value(int axis);
int ms2100_get_status(void);

#define MS2100_SAMPLING_TIMES  10

#define MS2100_X_AXIS		   0
#define MS2100_Y_AXIS          1
#define MS2100_Z_AXIS          2

#define MS2100_IDLE            0
#define MS2100_BUSY            1
#define MS2100_SENDING_REQ     2
#define MS2100_WAITING_EOC     3
#define MS2100_GOT_EOC         4
#define MS2100_READING_RES     5
#define MS2100_DATA_AVAILABLE  6


#endif /* MS2100_H_ */
