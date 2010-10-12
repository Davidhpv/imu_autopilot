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
* @file Angular velocity
*   @author Lorenz Meier
*
*/

#ifndef GYROS_H_
#define GYROS_H_

#include "conf.h"
#include "debug.h"
#include "spi.h"
#include "ads8341.h"

static inline void gyro_init(void)
{
	uint32_t sum_x=0;
	uint32_t sum_y=0;
	uint32_t sum_z=0;
	int i;
	int sample_size = 1000;
	for(i=0;i<sample_size;i++){
		ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_x += ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);

		ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_y += ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);

		ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_z += ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);
	}
// not used anymore now storing in parameters and only on request.
//	global_data.gyros_autocal_raw_offset.x=(uint16_t)(sum_x/sample_size);
//	global_data.gyros_autocal_raw_offset.y=(uint16_t)(sum_y/sample_size);
//	global_data.gyros_autocal_raw_offset.z=(uint16_t)(sum_z/sample_size);

}

static inline void gyro_calibrate(void)
{
	uint32_t sum_x=0;
	uint32_t sum_y=0;
	uint32_t sum_z=0;
	int i;
	const int sample_size = 10000; //make sure that this times 30000 doesn't exceed 2^32
	for(i=0;i<sample_size;i++){
		ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_x += ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);

		ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_y += ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);

		ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
		while (spi_running());
		sum_z += ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);
	}

	global_data.param[PARAM_GYRO_OFFSET_X]=(sum_x/sample_size);
	global_data.param[PARAM_GYRO_OFFSET_Y]=(sum_y/sample_size);
	global_data.param[PARAM_GYRO_OFFSET_Z]=(sum_z/sample_size);

}

static inline void gyro_read(void)
{
	ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.gyros_raw.x = ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);

	ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.gyros_raw.y = ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);

	ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.gyros_raw.z = ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);

	ads8341_read(ADS8341_0, GYROS_TEMPERATURE_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.temperature_gyros = ads8341_get_value(ADS8341_0,GYROS_TEMPERATURE_ADS8341_0_CHANNEL);

	if (global_data.param[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE] == 1)
	{
		//With temperature compensation, linear fit.
//		float calibration_temperature_correction_offset_x = -0.060333834627133
//				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_X];
//		float calibration_temperature_correction_offset_y = 0.020519379344279
//				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Y];
//		float calibration_temperature_correction_offset_z = -0.024202371781532
//				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Z];

		// With temperature calibration, linear fit, with autocalibration offset
		float calibration_temperature_correction_offset_x = -0.060333834627133
				* global_data.temperature_gyros + global_data.param[PARAM_GYRO_OFFSET_X];
		float calibration_temperature_correction_offset_y = 0.020519379344279
				* global_data.temperature_gyros + global_data.param[PARAM_GYRO_OFFSET_Y];
		float calibration_temperature_correction_offset_z = -0.024202371781532
				* global_data.temperature_gyros + global_data.param[PARAM_GYRO_OFFSET_Z];

		//Only update SI, if value was not smaller than gyro_treshold ( Sometimes it get 0. No idea why. 20100901)
		uint16_t gyro_treshold = 1024;
		if (global_data.gyros_raw.x >= gyro_treshold)
		{
			global_data.gyros_si.x = -IDG_500_GYRO_SCALE_X
					* (global_data.gyros_raw.x
							- calibration_temperature_correction_offset_x);
		}
		else
		{
			debug_message_buffer("GYRO_RAW_X was ZERO!!!");
		}
		if (global_data.gyros_raw.y >= gyro_treshold)
		{
			global_data.gyros_si.y = IDG_500_GYRO_SCALE_Y
					* (global_data.gyros_raw.y
							- calibration_temperature_correction_offset_y);
		}
		else
		{
			debug_message_buffer("GYRO_RAW_Y was ZERO!!!");
		}
		if (global_data.gyros_raw.z >= gyro_treshold)
		{
			global_data.gyros_si.z = -IXZ_500_GYRO_SCALE_Z
					* (global_data.gyros_raw.z
							- calibration_temperature_correction_offset_z);
		}
		else
		{
			debug_message_buffer("GYRO_RAW_Z was ZERO!!!");
		}
	}
	else
	{
		// Without temp compensation
		global_data.gyros_si.x = -IDG_500_GYRO_SCALE_X * (global_data.gyros_raw.x
				- global_data.param[PARAM_GYRO_OFFSET_X]);
		global_data.gyros_si.y = IDG_500_GYRO_SCALE_Y * (global_data.gyros_raw.y
				- global_data.param[PARAM_GYRO_OFFSET_Y]);// global_data.param[PARAM_GYRO_OFFSET_Y]);
		global_data.gyros_si.z = -IXZ_500_GYRO_SCALE_Z * (global_data.gyros_raw.z
				- global_data.param[PARAM_GYRO_OFFSET_Z]);
	}
}

static inline float gyro_get_roll_si(void)
{
	return global_data.gyros_si.x;
}

static inline uint32_t gyro_get_roll_raw(void)
{
	return global_data.gyros_si.y;
}

static inline float gyro_get_pitch_si(void)
{
	return global_data.gyros_si.z;
}

static inline uint32_t gyro_get_pitch_raw(void)
{
	return global_data.gyros_raw.x;
}

static inline float gyro_get_yaw_si(void)
{
	return global_data.gyros_raw.y;
}

static inline uint32_t gyro_get_yaw_raw(void)
{
	return global_data.gyros_raw.z;
}


#endif /* GYROS_H_ */
