/*
 * sensors.h
 *
 *  Created on: Jun 15, 2009
 *      Author: mavteam, Christian Dobler
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "conf.h"
#include "sca3100.h"
#include "sca3000.h"
#include "bmp085.h"
#include "ads8341.h"
#include "ms2100.h"
#include "LPC21xx.h"
#include "global_data.h"
#include "spi.h"
#include "i2c.h"
#include "sys_time.h"
#include "inttypes.h"
#include "adc.h"
#include "dac.h"
#include "hmc5843.h"
#include <stdio.h>

#include "comm.h"
#include "led.h"


#define BURST_TIMES		1

#if(FEATURE_SENSORS==FEATURE_SENSORS_ENABLED)

uint8_t mag_axis;
uint8_t bmp085_measurement_mode;

static inline void sensors_init(void)
{
	//ms2100_init();
	ads8341_init();
	mag_axis = 0;
	bmp085_init(); //disabled for demo
	bmp085_measurement_mode = 255;
}

//static inline void sensors_gyros_init(void)
//{
//	uint32_t sum_x=0;
//	uint32_t sum_y=0;
//	uint32_t sum_z=0;
//	int i;
//	int sample_size = 3000;
//	for(i=0;i<sample_size;i++){
//		ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
//		while (spi_running());
//		sum_x += ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);
//
//		ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
//		while (spi_running());
//		sum_y += ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);
//
//		ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
//		while (spi_running());
//		sum_z += ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);
//	}
//
//	global_data.gyros_autocal_raw_offset.x=(uint16_t)(sum_x/sample_size);
//	global_data.gyros_autocal_raw_offset.y=(uint16_t)(sum_y/sample_size);
//	global_data.gyros_autocal_raw_offset.z=(uint16_t)(sum_z/sample_size);
//
//}

//static inline void sensors_read_gyro(void)
//{
//	ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
//	while (spi_running());
//	global_data.gyros_raw.x = ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);
//
//	ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
//	while (spi_running());
//	global_data.gyros_raw.y = ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);
//
//	ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
//	while (spi_running());
//	global_data.gyros_raw.z = ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);
//
//	ads8341_read(ADS8341_0, GYROS_TEMPERATURE_ADS8341_0_CHANNEL);
//	while (spi_running());
//	global_data.temperature_gyros = ads8341_get_value(ADS8341_0,GYROS_TEMPERATURE_ADS8341_0_CHANNEL);
//
//	if (global_data.param[PARAM_CAL_GYRO_TEMP_FIT_ACTIVE] == 1)
//	{
//		//With temperature compensation, linear fit.
////		float calibration_temperature_correction_offset_x = -0.060333834627133
////				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_X];
////		float calibration_temperature_correction_offset_y = 0.020519379344279
////				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Y];
////		float calibration_temperature_correction_offset_z = -0.024202371781532
////				* global_data.temperature_gyros + global_data.param[PARAM_CAL_GYRO_TEMP_FIT_Z];
//
//		// With temperature calibration, linear fit, with autocalibration offset
//		float calibration_temperature_correction_offset_x = -0.060333834627133
//				* global_data.temperature_gyros + global_data.gyros_autocal_raw_offset.x;
//		float calibration_temperature_correction_offset_y = 0.020519379344279
//				* global_data.temperature_gyros + global_data.gyros_autocal_raw_offset.y;
//		float calibration_temperature_correction_offset_z = -0.024202371781532
//				* global_data.temperature_gyros + global_data.gyros_autocal_raw_offset.z;
//
//		global_data.gyros_si.x = -IDG_500_GYRO_SCALE_X * (global_data.gyros_raw.x
//				- calibration_temperature_correction_offset_x);
//		global_data.gyros_si.y = IDG_500_GYRO_SCALE_Y * (global_data.gyros_raw.y
//				- calibration_temperature_correction_offset_y);
//		global_data.gyros_si.z = -IXZ_500_GYRO_SCALE_Z * (global_data.gyros_raw.z
//				- calibration_temperature_correction_offset_z);
//	}
//	else
//	{
//		// Without temp compensation
//		global_data.gyros_si.x = -IDG_500_GYRO_SCALE_X * (global_data.gyros_raw.x
//				- global_data.gyros_autocal_raw_offset.x);
//		global_data.gyros_si.y = IDG_500_GYRO_SCALE_Y * (global_data.gyros_raw.y
//				- global_data.gyros_autocal_raw_offset.y);// global_data.param[PARAM_GYRO_OFFSET_Y]);
//		global_data.gyros_si.z = -IXZ_500_GYRO_SCALE_Z * (global_data.gyros_raw.z
//				- global_data.gyros_autocal_raw_offset.z);
//	}
//}

static inline void sensors_read_acc(void)
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

static inline void sensors_read_mag(void)
{
	//Read out magnet data. It will be available later.
	//hmc5843_start_read();
	//copy last data and update state
	global_data.state.magnet_ok = hmc5843_data_ok();
	hmc5843_get_data(&global_data.magnet_raw);
	global_data.state.magnet_ok &= hmc5843_data_ok();//if interrupt came in between

	// Correct offset and scale to milli Gauss
	global_data.magnet_corrected.x =  (global_data.magnet_raw.x - global_data.param[PARAM_CAL_MAG_OFFSET_X]);
	global_data.magnet_corrected.y = -(global_data.magnet_raw.y - global_data.param[PARAM_CAL_MAG_OFFSET_Y]);
	global_data.magnet_corrected.z =  (global_data.magnet_raw.z - global_data.param[PARAM_CAL_MAG_OFFSET_Z]);
}

static inline void sensors_pressure_bmp085_read_out(void)
{

	switch (bmp085_measurement_mode){
	case 255:
		bmp085_start_temp_measurement();
		bmp085_measurement_mode = 0;
		break;
	case 50:
		global_data.pressure_raw = bmp085_get_pressure();
		bmp085_start_temp_measurement();
		bmp085_measurement_mode = 0;
		break;
	default:
		if(bmp085_measurement_mode == 0) {
			global_data.temperature = bmp085_get_temperature();
			global_data.temperature_si = global_data.temperature / 10.0f;
		}
		else{
			global_data.pressure_raw = bmp085_get_pressure();
		}
		bmp085_start_pressure_measurement();
		bmp085_measurement_mode++;
	}

}


static inline void sensors_gyro_acc_read_out(void)
{

	ads8341_read(ADS8341_0, GYROS_ROLL_ADS8341_0_CHANNEL);
	while (spi_running());
	ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);
	global_data.gyros_raw.x = ads8341_get_value(ADS8341_0,GYROS_ROLL_ADS8341_0_CHANNEL);

	ads8341_read(ADS8341_0, GYROS_PITCH_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.gyros_raw.y = ads8341_get_value(ADS8341_0,GYROS_PITCH_ADS8341_0_CHANNEL);

	ads8341_read(ADS8341_0, GYROS_YAW_ADS8341_0_CHANNEL);
	while (spi_running());
	global_data.gyros_raw.z = ads8341_get_value(ADS8341_0,GYROS_YAW_ADS8341_0_CHANNEL);


	sca3100_read_res();
	while(spi_running());
	global_data.accel_raw.x = (int)sca3100_get_value(SCA3100_X_AXIS);
	global_data.accel_raw.y = (int)sca3100_get_value(SCA3100_Y_AXIS);
	global_data.accel_raw.z = (int)sca3100_get_value(SCA3100_Z_AXIS);

}

#endif


#endif /* SENSORS_H_ */
