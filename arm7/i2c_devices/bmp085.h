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
 
/*
 * @file 
 *   @brief BMP085 pressure sensor device driver
 *
 *   This file contains the device driver for the Bosch BMP085 pressure sensor.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#ifndef BMP085_H_
#define BMP085_H_
 
#include "inttypes.h"
#include "i2c.h"

#if (FEATURE_SENSOR_PRESSURE==FEATURE_SENSOR_PRESSURE_BMP085)
 
/**
 * @struct bmp085_smd500_calibration_param_t This structure holds all device specific calibration parameters.
 */
typedef struct {
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;      		   
} bmp085_smd500_calibration_param_t;


/**
 * @struct bmp085_t This struct holds all control, measurement and status data of the BMP085 pressure sensor.
 */
typedef struct  {	
	bmp085_smd500_calibration_param_t cal_param;	///< structure that holds calibration data
	uint8_t chip_id, ml_version, al_version;		///< chip identification data
	uint8_t dev_addr;								///< I2C slave address of BMP085
	uint8_t bus_number;								///< number of the I2C bus the BMP085 is connected to
	uint8_t sensortype;								///< value of sensor type (should be 0x55)

	int32_t param_b5;								///< calibration parameter calculated by bmp085_get_temperature()
	uint8_t oversampling_setting;					///< setting of the pressure conversion mode (internal oversampling)

	uint32_t current_pressure;						///< value of the most recent pressure measurement
	uint32_t current_temp;							///< value of the most recent temperature measurement

	uint8_t busy;									///< set to 1 during measurements
	uint8_t measurement_type;						///< temporarily stores current measurement type (for driver internal use only)
} bmp085_t;


/*  Settings for pressure sensor */

/** BMP085 I2C slave addresses */
#define BMP085_I2C_ADDR				0xEE 		///< I2C slave address of the BMP085

/** BMP085 register addresses */
#define BMP085_CONTROL_REG			0xF4 		///< address of the BMP085 control register
#define BMP085_DATA_MSB_REG 		0xF6		///< address of the BMP085 data (MSB) register
#define BMP085_DATA_LSB_REG 		0xF7		///< address of the BMP085 data (LSB) register
#define BMP085_DATA_XLSB_REG 		0xF7		///< address of the BMP085 data (XLSB) register (optional for ultra high resolution mode)

/** BMP085 control register settings */
#define BMP085_T_MEASURE        	0x2E		///< writing this value to the BMP085 control register starts temperature measurement
#define BMP085_P_MEASURE        	0x34		///< writing this value to the BMP085 control register starts pressure measurement


#define BMP085_CHIP_ID				0x55		///< ID value of every BMP085 chip
#define BOSCH_PRESSURE_BMP085		85			///< ID value in decimal representation

#define BMP085_PROM_START__ADDR		0xAA		///< start address of the internal EEPROM containing the factory calibration data
#define BMP085_PROM_DATA__LEN		22			///< size of the internal EEPROM

#define BMP085_CHIP_ID_REG			0xD0		///< address of the register containing the chip ID
#define BMP085_VERSION_REG			0xD1		///< address of the register containing the chip version

/** defines to extract chip ID correctly */
#define BMP085_CHIP_ID__POS			0
#define BMP085_CHIP_ID__MSK			0xFF
#define BMP085_CHIP_ID__LEN			8
#define BMP085_CHIP_ID__REG			BMP085_CHIP_ID_REG

/** defines to extract ML version correctly */
#define BMP085_ML_VERSION__POS		0
#define BMP085_ML_VERSION__LEN		4
#define BMP085_ML_VERSION__MSK		0x0F
#define BMP085_ML_VERSION__REG		BMP085_VERSION_REG

/** defines to extract AL version correctly */
#define BMP085_AL_VERSION__POS  	4
#define BMP085_AL_VERSION__LEN  	4
#define BMP085_AL_VERSION__MSK		0xF0
#define BMP085_AL_VERSION__REG		BMP085_VERSION_REG

/** bmp085.sensortype value if no device detected */
#define E_SENSOR_NOT_DETECTED   	0

/** standard calibration parameters */
#define BMP085_PARAM_MG      		3038        ///<calibration parameter
#define BMP085_PARAM_MH     		-7357       ///<calibration parameter
#define BMP085_PARAM_MI      		3791        ///<calibration parameter


/** conversion timeouts for different pressure modes */
#define BMP085_PRESS_0_CONVERSION_TIME		6000
#define BMP085_PRESS_1_CONVERSION_TIME		9000
#define BMP085_PRESS_2_CONVERSION_TIME		15000

/** conversion timeout for temperature measurement mode */
#define BMP085_TEMP_CONVERSION_TIME			6000



/** control bit extraction function */
#define BMP085_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS

#define BMP085_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)


/**
 * @brief Initialization of the BMP085 pressure sensor
 *
 * This function initializes the BMP085 pressure sensor. The I2C subsystem
 * has to be initialized beforehand; otherwise this will fail.
 */
void bmp085_init(void);

/**
 * @brief Function to read the BMP085 chip ID register
 *
 * This function is invoked by the I2C subsystem during the initialization
 * of the BMP085 pressure sensor. This function does not need to be called
 * by application software.
 *
 * @param package I2C package as described in i2c.h
 */
void bmp085_read_chip_id_on_init(i2c_package *package);

/**
 * @brief Function to read the BMP085 chip ID register
 *
 * This function is invoked by the I2C subsystem during the initialization
 * of the BMP085 pressure sensor. This function does not need to be called
 * by application software.
 *
 * @param package I2C package as described in i2c.h
 */
void bmp085_read_version_on_init(i2c_package *package);

/**
 * @brief Starts read-out of factory calibration data from the sensor on init
 *
 * This function is invoked by the I2C subsystem during the initialization
 * of the BMP085 pressure sensor. This function does not need to be called
 * by application software.
 */
void bmp085_get_cal_param(void);

/**
 * @brief Stores factory calibration data read out from the sensor on init
 *
 * This function is invoked by the I2C subsystem during the initialization
 * of the BMP085 pressure sensor. This function does not need to be called
 * by application software.
 *
 * @param package I2C package as described in i2c.h
 */
void bmp085_store_cal_param_on_init(i2c_package *package);

/**
 * @brief Function to start temperature measurement
 *
 * This function has to be called by the application software to start a temperature
 * measurement on the BMP085. The BMP085 needs 4.5ms until the temperature measurement
 * is done. Measurement read-out is done by bmp085_start_measurement_read() called by
 * the EOC interrupt.
 *
 * This function needs to be called once before starting the first pressure measurement.
 */
void bmp085_start_temp_measurement(void);

/**
 * @brief Function to start temperature measurement
 *
 * This function has to be called by the application software to start a pressure
 * measurement on the BMP085. The BMP085 needs 4.5ms - 13.5ms (depending on the oversampling
 * setting selected by BMP085_PRESSURE_CONVERSION_MODE) until the temperature measurement is
 * done. Measurement read-out is done by bmp085_start_measurement_read() called by the EOC
 * interrupt.
 *
 * Execute one complete temperature measurement before calling this function.
 */
void bmp085_start_pressure_measurement(void);

/**
 * @brief Function to read out measurement result
 *
 * This function reads out the the 16 bit measurement result register from the BMP085.
 * It is invoked by the EOC interrupt (PH v200: timer interrupt, PH v210: EINT2). The
 * application software should not call this function.
 */
void bmp085_start_measurement_read(void);

/**
 * @brief Stores measurement data read out from the sensor in the driver
 *
 * This function is invoked by the I2C subsystem during the initialization
 * of the BMP085 pressure sensor. This function does not need to be called
 * by application software.
 *
 * @param package I2C package as described in i2c.h
 */
void bmp085_save_measurement(i2c_package *package);

/**
 * @brief Function to get the most recent temperature value
 *
 * This function has to be called by the application software to get the most recent
 * temperature value. This function returns immediately (no delays). Call this function
 * at least once before calling bmp085_get_pressure().
 *
 * @return 16-bit temperature value in 0.1�C
 */
int16_t bmp085_get_temperature(void);

/**
 * @brief Function to get the most recent pressure value
 *
 * This function has to be called by the application software to get the most recent
 * pressure value. This function returns immediately (no delays). Execute one complete
 * temperature measurement before calling this function.
 *
 * @return 32-bit pressure value in 1 Pa
 */
int32_t bmp085_get_pressure(void);
#endif // IMU_V200
#endif // BMP085_H_
