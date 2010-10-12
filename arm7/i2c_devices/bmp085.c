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
 *   @brief BMP085 pressure sensor device driver
 *
 *   This file contains the device driver for the Bosch BMP085 pressure sensor.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#include "inttypes.h"
#include "bmp085.h"
#include "conf.h"
#include "i2c.h"
#include "armVIC.h"
#include "LPC21xx.h"
#include "sys_time.h"

#include "comm.h"
#include "led.h"
#include <stdio.h>

#include "mavlink.h"

#if (FEATURE_SENSOR_PRESSURE==FEATURE_SENSOR_PRESSURE_BMP085)


#if (FEATURE_SENSOR_PRESSURE_INTERUPT==FEATURE_SENSOR_PRESSURE_INTERUPT_ENABLED)
static void EOC_EXTINT_ISR(void) __attribute__((naked));
#endif

bmp085_t bmp085;


void bmp085_init(void)
{
	i2c_package package_write, package_read;

	// set up static data in device descriptor structure bmp085
	bmp085.sensortype = E_SENSOR_NOT_DETECTED;			// no sensor detected yet
	bmp085.dev_addr = BMP085_I2C_ADDR;                  // preset BMP085 slave I2C_addr
	bmp085.bus_number = BMP085_I2C_BUS_NUMBER;			// number of the I2C bus, that the BMP085 is connected to
	bmp085.oversampling_setting = BMP085_PRESSURE_CONVERSION_MODE;	// mode for sensor internal oversampling of pressure values
	bmp085.busy = 0;									// reset BMP085 state variable
	bmp085.measurement_type = 0;						// no measurement type specified yet (1: temperature, 2: pressure)

//	/* configure EOC pin */
//	/* connect BMP085_EOC pin to EINT2 */
//	BMP085_EOC_PINSEL |= BMP085_EOC_PINSEL_VAL << BMP085_EOC_PINSEL_BIT;
//	EXTMODE|= 1 << BMP085_EOC_EINT; 		/* EINT2 is edge triggered */
//	EXTPOLAR|= 1 << BMP085_EOC_EINT; 		/* EINT2 is triggered on rising edge */
//	EXTINT|=  1 << BMP085_EOC_EINT; 		/* clear pending EINT2 */
//
//	/* initialize interrupt vector */
//	VICIntSelect &= ~VIC_BIT(VIC_EINT2); 	/* select EINT2 as IRQ source */
//	VICIntEnable = VIC_BIT(VIC_EINT2); 		/* enable it                 */
//	_VIC_CNTL(PRESSURE_EOC_VIC_SLOT) = VIC_ENABLE | VIC_EINT2;
//	_VIC_ADDR(PRESSURE_EOC_VIC_SLOT) = (unsigned int) EOC_EXTINT_ISR; // address of the ISR

	// first read chip ID register to certify that everything is correctly connected
	// set up I2C write package
	package_write.data[0] = BMP085_CHIP_ID__REG;	// address of BMP085 chip ID register
	package_write.length = 1;						// just write the 8-bit address to be read
	package_write.direction = I2C_WRITE;
	package_write.slave_address = bmp085.dev_addr;	// I2C slave address of BM085
	package_write.bus_number = bmp085.bus_number;	// number of the I2C bus, that the BMP085 is connected to
	package_write.write_read = 1;					// make repeated start after this package to receive data
	package_write.i2c_done_handler = NULL;			// nothing to be done at end of I2C write op

	// set up I2C read package
	package_read.length = 1;
	package_read.direction = I2C_READ;
	package_read.slave_address = bmp085.dev_addr;
	package_read.bus_number = bmp085.bus_number;
	package_read.write_read = 1;
	package_read.i2c_done_handler = (void*)&bmp085_read_chip_id_on_init; // bmp085_read_chip_id_on_init() is invoked at the end of the I2C operation

	i2c_write_read(&package_write, &package_read);  /* read Chip Id */


	return;
}

void bmp085_read_chip_id_on_init(i2c_package *package) {

	i2c_package package_write, package_read;

	// extract chip ID from received I2C package
	bmp085.chip_id = BMP085_GET_BITSLICE(package->data[0], BMP085_CHIP_ID);		/* get bitslice */

	if (bmp085.chip_id == BMP085_CHIP_ID) {				// that's what the chip ID should be

		bmp085.sensortype = BOSCH_PRESSURE_BMP085;		// set sensor type to Bosch BMP085

		// now read out chip version
		// set up I2C write package
		package_write.data[0] = BMP085_VERSION_REG;		// address of BMP085 version register
		package_write.length = 1;						// just write the 8-bit address to be read
		package_write.direction = I2C_WRITE;
		package_write.slave_address = bmp085.dev_addr;
		package_write.bus_number = bmp085.bus_number;
		package_write.write_read = 1;					// make repeated start after this package to receive data
		package_write.i2c_done_handler = NULL;
		
		// set up I2C read package
		package_read.length = 1;
		package_read.direction = I2C_READ;
		package_read.slave_address = bmp085.dev_addr;
		package_read.bus_number = bmp085.bus_number;
		package_read.write_read = 1;
		package_read.i2c_done_handler = (void*)&bmp085_read_version_on_init;	// bmp085_read_version_on_init() is invoked at the end of the I2C operation

		i2c_write_read(&package_write, &package_read);  /* read Version reg */

	}
  
	else {		// this should not happen, if everything is correctly connected
		//uint8_t buffer[100];	// string buffer for debug messages
		//sprintf((char *)buffer, "BMP085 error: wrong chip ID on bus\n");
		//message_send_debug(COMM_1, buffer);

	}

}

void bmp085_read_version_on_init(i2c_package *package) {

	// extract version numbers from received I2C package
	bmp085.ml_version = BMP085_GET_BITSLICE(package->data[0], BMP085_ML_VERSION);        /* get ML Version */
	bmp085.al_version = BMP085_GET_BITSLICE(package->data[0], BMP085_AL_VERSION); 		/* get AL Version */
	
	bmp085_get_cal_param( ); 		/* readout bmp085 calibration parameters */

}

void bmp085_get_cal_param(void)
{
	i2c_package package_write, package_read;

	// initiate now readout of bmp085 calibration parameters

	package_write.data[0] = BMP085_PROM_START__ADDR;	// start address of the internal EEPROM of the BMP085
	package_write.length = 1;							// just write the 8-bit address to be read
	package_write.direction = I2C_WRITE;
	package_write.slave_address = bmp085.dev_addr;
	package_write.bus_number = bmp085.bus_number;
	package_write.write_read = 1;
	package_write.i2c_done_handler = NULL;
	
	package_read.length = BMP085_PROM_DATA__LEN;		// number of calibration bytes to be read fromthe BMP085
	package_read.direction = I2C_READ;
	package_read.slave_address = bmp085.dev_addr;
	package_read.bus_number = bmp085.bus_number;
	package_read.write_read = 1;						// make repeated start after this package to receive data
	package_read.i2c_done_handler = (void*)&bmp085_store_cal_param_on_init;	// bmp085_store_cal_param_on_init() is invoked at the end of the I2C operation
	i2c_write_read(&package_write, &package_read);  /* read calibration parameters over I2C */
  
}

// stores calibration data read out from the EEPROM in the sensor
void bmp085_store_cal_param_on_init(i2c_package *package) {
	
	/*parameters AC1-AC6*/
	bmp085.cal_param.ac1 =  (package->data[0] <<8) | package->data[1];
	bmp085.cal_param.ac2 =  (package->data[2] <<8) | package->data[3];
	bmp085.cal_param.ac3 =  (package->data[4] <<8) | package->data[5];
	bmp085.cal_param.ac4 =  (package->data[6] <<8) | package->data[7];
	bmp085.cal_param.ac5 =  (package->data[8] <<8) | package->data[9];
	bmp085.cal_param.ac6 =  (package->data[10] <<8) | package->data[11];

	/*parameters B1,B2*/
	bmp085.cal_param.b1 =  (package->data[12] <<8) | package->data[13];
	bmp085.cal_param.b2 =  (package->data[14] <<8) | package->data[15];

	/*parameters MB,MC,MD*/
	bmp085.cal_param.mb =  (package->data[16] <<8) | package->data[17];
	bmp085.cal_param.mc =  (package->data[18] <<8) | package->data[19];
	bmp085.cal_param.md =  (package->data[20] <<8) | package->data[21];

	// init done

}


// start temperature measurement
void bmp085_start_temp_measurement()
{

	i2c_package package;

	// set up command package
	package.data[0] = BMP085_CONTROL_REG;  	// write to the BMP085 control register
	package.data[1] = BMP085_T_MEASURE;		// the command to start a temperature measurement
	package.length = 2;						// that is 2 bytes
	package.direction = I2C_WRITE;
	package.slave_address = bmp085.dev_addr;
	package.bus_number = bmp085.bus_number;
	package.write_read = 0;					// make stop condition on I2C bus after this command
	package.i2c_done_handler = NULL;

	if(!bmp085.busy){						// wait if BMP085 is already locked by other process

	bmp085.busy = 1;						// lock BMP085 resource
	bmp085.measurement_type = 1;			// set measurement type to 1 := temperature measurement
	i2c_op(&package);  /* write register to be read */

	// on a v2.1 IMU the BMP085_EOC interrupt will designate end-of-conversion of sensor value (= data is ready)

#if(FEATURE_SENSOR_PRESSURE_INTERUPT==FEATURE_SENSOR_PRESSURE_INTERUPT_DISABLED)		// on a v2.0 IMU timer interrupt is used to specify end-of-conversion
	T0MCR |= TMCR_MR0_I;
	T0MR0 = T0TC + SYS_TICS_OF_USEC(BMP085_TEMP_CONVERSION_TIME);	// timer interrupt will be generated after temperature conversion timeout
	#endif
	}
}

// start pressure measurement
void bmp085_start_pressure_measurement()
{

	i2c_package package;

	// set up command package
	package.data[0] = BMP085_CONTROL_REG + (bmp085.oversampling_setting << 6);	// write to the BMP085 control register
	package.data[1] = BMP085_P_MEASURE;			// the command to start a pressure measurement
	package.length = 2;							// that is 2 bytes
	package.direction = I2C_WRITE;
	package.slave_address = bmp085.dev_addr;
	package.bus_number = bmp085.bus_number;
	package.write_read = 0;						// make stop condition on I2C bus after this command
	package.i2c_done_handler = NULL;

	if(!bmp085.busy){					// wait if BMP085 is already locked by other process

	bmp085.busy = 1;							// lock BMP085 resource
	bmp085.measurement_type = 2;				// set measurement type to 1 := temperature measurement

	i2c_op(&package);  /* write register to be read */

	// on a v2.1 IMU the BMP085_EOC interrupt will designate end-of-conversion of sensor value (= data is ready)

#if(FEATURE_SENSOR_PRESSURE_INTERUPT==FEATURE_SENSOR_PRESSURE_INTERUPT_DISABLED)		// on a v2.0 IMU timer interrupt is used to specify end-of-conversion
	T0MCR |= TMCR_MR0_I;				// timer interrupt will be generated after temperature conversion timeout which depends on oversampling setting
	//uint8_t buffer[100];	// string buffer for debug messages
	switch(bmp085.oversampling_setting) {
		case 0:
			T0MR0 = T0TC + SYS_TICS_OF_USEC(BMP085_PRESS_0_CONVERSION_TIME);
			break;
		case 1:
			T0MR0 = T0TC + SYS_TICS_OF_USEC(BMP085_PRESS_1_CONVERSION_TIME);
			break;
		case 2:
			T0MR0 = T0TC + SYS_TICS_OF_USEC(BMP085_PRESS_2_CONVERSION_TIME);
			break;
		default:
			
			//sprintf((char *)buffer, "BMP085 error: non-existing conversion mode defined\n");
			//message_send_debug(COMM_1, buffer);
			break;
	}
	#endif
	}
}

// start reading out measurement from the sensor (invoked by either timer or EOC interrupt)
void bmp085_start_measurement_read() {

	i2c_package package_write, package_read;

	// set up I2C write package
	package_write.data[0] = BMP085_DATA_MSB_REG;	// register address to read the measurement
	package_write.length = 1;						// this command is one byte
	package_write.direction = I2C_WRITE;
	package_write.slave_address = bmp085.dev_addr;
	package_write.bus_number = bmp085.bus_number;
	package_write.write_read = 1;
	package_write.i2c_done_handler = NULL;

	// set up I2C read package
	package_read.length = 2;						// measurement is a 16 bit value = 2 bytes
	package_read.direction = I2C_READ;
	package_read.slave_address = bmp085.dev_addr;
	package_read.bus_number = bmp085.bus_number;
	package_read.write_read = 1;					// make repeated start after this package to receive data
	package_read.i2c_done_handler = (void*)&bmp085_save_measurement;	// bmp085_save_measurement() is invoked at the end of the I2C operation

	i2c_write_read(&package_write, &package_read);  // start I2C operation

}

// stores measurements temporarily within the driver
void bmp085_save_measurement(i2c_package *package)
{

	//uint8_t buffer[100];	// string buffer for debug messages
	switch(bmp085.measurement_type) {
		case 1:		// if temperature has been measured, store temperature
			bmp085.current_temp = (package->data[0] <<8) | package->data[1];
			break;
		case 2:		// if pressure has been measured, store temperature
			bmp085.current_pressure = (package->data[0] <<8) | package->data[1];
			break;
		default:	// no other measurement modes available
			//sprintf((char *)buffer, "BMP085 error: no measurement type defined\n");
			//message_send_debug(COMM_1, buffer);
			break;
	}

	bmp085.busy = 0;	// release locked BMP085 resource

}

// function to convert and scale temperature values to 0.1�C (taken from the BMP085 API from BOSCH)
int16_t bmp085_get_temperature(void)
{

	int16_t temperature;
	int32_t x1,x2;

	x1 = (((int32_t) bmp085.current_temp - (int32_t) bmp085.cal_param.ac6) * (int32_t) bmp085.cal_param.ac5) >> 15;
	x2 = ((int32_t) bmp085.cal_param.mc << 11) / (x1 + bmp085.cal_param.md);
	bmp085.param_b5 = x1 + x2;

	temperature = ((bmp085.param_b5 + 8) >> 4);  // temperature in 0.1�C

	return (temperature);
}

// function to convert and scale pressure values to Pascal (taken from the BMP085 API from BOSCH)
int32_t bmp085_get_pressure(void)
{

   int32_t pressure,x1,x2,x3,b3,b6;
   uint32_t b4, b7;

   b6 = bmp085.param_b5 - 4000;
   //*****calculate B3************
   x1 = (b6*b6) >> 12;
   x1 *= bmp085.cal_param.b2;
   x1 >>=11;

   x2 = (bmp085.cal_param.ac2*b6);
   x2 >>=11;

   x3 = x1 +x2;

   b3 = (((((int32_t)bmp085.cal_param.ac1 )*4 + x3) <<((int32_t)bmp085.oversampling_setting)) + 2) >> 2;

   //*****calculate B4************
   x1 = (bmp085.cal_param.ac3* b6) >> 13;
   x2 = (bmp085.cal_param.b1 * ((b6*b6) >> 12) ) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (bmp085.cal_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;

   b7 = ((uint32_t)(bmp085.current_pressure - b3) * (50000>>(int32_t)bmp085.oversampling_setting));
   if (b7 < 0x80000000)
   {
     pressure = (b7 << 1) / b4;
   }
   else
   {
     pressure = (b7 / b4) << 1;
   }

   x1 = pressure >> 8;
   x1 *= x1;
   x1 = (x1 * BMP085_PARAM_MG) >> 16;
   x2 = (pressure * BMP085_PARAM_MH) >> 16;
   pressure += (x1 + x2 + BMP085_PARAM_MI) >> 4;	// pressure in Pa

   return (pressure);
}




#if(FEATURE_SENSOR_PRESSURE_INTERUPT==FEATURE_SENSOR_PRESSURE_INTERUPT_ENABLED)		// can not be used on Pixhawk v2.0 IMU
/* Interrupt handler for BMP085 End-of-Conversion (EOC) interrupt */
static void EOC_EXTINT_ISR(void) {

	ISR_ENTRY();				// enter ISR

	bmp085_start_measurement_read();	// invoke function bmp085_start_measurement_read()

	/* clear EINT */
	EXTINT|=  1 << BMP085_EOC_EINT;

	VICVectAddr = 0x00000000; 	/* clear this interrupt from the VIC */
	ISR_EXIT();					// exit ISR
}
#endif

#endif
