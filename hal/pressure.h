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
* @file Pressure sensor readout
*   @author Lorenz Meier
*
*/

#ifndef PRESSURE_H_
#define PRESSURE_H_

#include "bmp085.h"

#define air_gas_constant 8.31432f
#define air_density_sea_level 1.225f
#define absolute_null_kelvin 273.15f

uint8_t bmp085_measurement_mode;

static inline void pressure_init(void)
{
	bmp085_init();
	bmp085_measurement_mode = 255;
}

static inline float pressure_differential_read(void)
{

	uint16_t adc_value = adc_get_value(ADC_6_CHANNEL);
	//float adc_volt = ((float) adc_value) / 310.0f;

	global_data.pressure_diff_raw = adc_value;
	// Convert ADC units to volt
	// then scale 1.0 - 3.0 V to 0..1
	// then multiply with max range of sensor of 3920 Pascal to get SI units
	global_data.pressure_diff_si = (((adc_value / 310.0f) - 1.0f) / (3.0f - 1.0f)) * 3920;
}

static inline float pressure_differential_get_si(void)
{
	return global_data.pressure_diff_si;
}

static inline void pressure_ambient_read(void)
{
	switch (bmp085_measurement_mode)
	{
	case 255:
		bmp085_start_temp_measurement();
		bmp085_measurement_mode = 0;
		break;
	case 50:
		global_data.pressure_raw = bmp085_get_pressure();
		global_data.pressure_si = global_data.pressure_raw;
		bmp085_start_temp_measurement();
		bmp085_measurement_mode = 0;
		break;
	default:
		if(bmp085_measurement_mode == 0) {
			global_data.temperature = bmp085_get_temperature();
			global_data.temperature_si = global_data.temperature / 10;
		}
		else{
			global_data.pressure_raw = bmp085_get_pressure();
		}
		bmp085_start_pressure_measurement();
		bmp085_measurement_mode++;
	}
}


static inline float pressure_ambient_get_si(void)
{
	return global_data.pressure_raw;
}

static inline float air_density_get_si(void)
{
	return global_data.pressure_si/(air_gas_constant * (global_data.temperature_si + absolute_null_kelvin));
}


#endif /* PRESSURE_H_ */
