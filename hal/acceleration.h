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
* @file Linear accelerations
*   @author Lorenz Meier
*
*/

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include "conf.h"
#include "arm7/spi.h"
#include "sca3100.h"
#include "sca3000.h"
#include "global_data.h"

static inline void acc_init(void)
{
	sca3100_init();
}

static inline void acc_read(void)
{
	// Sending accel readout request
	sca3100_read_res();
	// waiting until data arrives
	while(spi_running());
	global_data.accel_raw.x = (int)sca3100_get_value(SCA3100_X_AXIS);
	global_data.accel_raw.y = (int)sca3100_get_value(SCA3100_Y_AXIS);
	global_data.accel_raw.z = -(int)sca3100_get_value(SCA3100_Z_AXIS);

	// Convert milli g to m/s^2
	global_data.accel_si.x = (float)global_data.accel_raw.x*0.00981f;
	global_data.accel_si.y = (float)global_data.accel_raw.y*0.00981f;
	global_data.accel_si.z = (float)global_data.accel_raw.z*0.00981f;
}

static inline float acc_get_x_si(void)
{
	return global_data.accel_si.x;
}

static inline uint32_t acc_get_x_raw(void)
{
	return global_data.accel_si.y;
}

static inline float acc_get_y_si(void)
{
	return global_data.accel_si.z;
}

static inline uint32_t acc_get_y_raw(void)
{
	return global_data.accel_raw.x;
}

static inline float acc_get_z_si(void)
{
	return global_data.accel_raw.y;
}

static inline uint32_t acc_get_z_raw(void)
{
	return global_data.accel_raw.z;
}


#endif /* ACCELERATION_H_ */
