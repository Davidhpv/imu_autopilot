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
 *   @brief I2C motor controller driver
 *
 *   This file contains the device driver for the I2C motor controllers.
 *   @author Christian Dobler <mavteam@student.ethz.ch>
 *
 */

#include "inttypes.h"
#include "i2c_motor_controller.h"
#include "conf.h"
#include "i2c.h"

#include "comm.h"
#include "led.h"
#include <stdio.h>

#if defined(IMU_PIXHAWK_V200) || defined(IMU_PIXHAWK_V210) //TODO change to features

static void mot_save_rpm_after_read(i2c_package *package);
volatile uint16_t current_rpm = 0;
volatile uint8_t mot_rpm_data_ready = 0;

void motor_i2c_set_rpm(uint8_t mot_i2c_dev_addr, uint16_t rpm)
{
	i2c_package package;
	package.data[1] = (uint8_t)rpm;						// LSB of RPM value to be set
	package.data[0] = (uint8_t)(rpm>>8);				// MSB of RPM value to be set
	package.length = 2;									// 2 bytes for RPM setting
	package.direction = I2C_WRITE;						// I2C write operation
	package.slave_address = mot_i2c_dev_addr;			// I2C slave address of selected motor controller
	package.bus_number = MOT_I2C_BUS_NUMBER;			// number of the I2C bus, that the motor controller is connected to
	package.write_read = 0;								// no repeated start condition
	package.i2c_done_handler = NULL;					// nothing to be done at I2C completion
	i2c_op(&package);
}

uint16_t motor_i2c_get_rpm(uint16_t mot_i2c_dev_addr)
{
	i2c_package package_write, package_read;

	// set up I2C package for the read command to be passed to the I2C subsystem
	package_write.data[0] = MOT_GET_RPM_CMD;					// command to read RPM from motor controller
	package_write.length = 1;									// 1 command byte
	package_write.direction = I2C_WRITE;						// I2C write operation
	package_write.slave_address = mot_i2c_dev_addr; 			// I2C slave address of selected motor controller
	package_write.bus_number = MOT_I2C_BUS_NUMBER;				// number of the I2C bus, that the motor controller is connected to
	package_write.write_read = 1;								// repeated start condition at completion
	package_write.i2c_done_handler = NULL;						// nothing to be done at I2C completion

	// set up I2C package for the data read-out to be passed to the I2C subsystem
	package_read.length = 2;									// 2 bytes for MSB and LSB of current RPM value to be read
	package_read.direction = I2C_READ;							// I2C read operation
	package_read.slave_address = mot_i2c_dev_addr;				// I2C slave address of selected motor controller
	package_read.bus_number = MOT_I2C_BUS_NUMBER;				// number of the I2C bus, that the motor controller is connected to
	package_read.write_read = 1;								// repeated start condition at start
	package_read.i2c_done_handler = (void*)&mot_save_rpm_after_read;	// mot_save_rpm_after_read() is called at I2C completion

	mot_rpm_data_ready = 0;										// invalidate old RPM value

	i2c_write_read(&package_write, &package_read);  			// start RPM read operation

	while(!mot_rpm_data_ready) ;								// wait for I2C operation to complete

	return current_rpm;											// return current RPM value
}

static void mot_save_rpm_after_read(i2c_package *package)
{
	current_rpm = (((uint16_t)(package->data[0]))<<8) || ((uint16_t)(package->data[1]));	// temporarily store RPM value contained in the I2C package to current_rpm
	mot_rpm_data_ready = 1;																	// set new RPM value to be valid
}

#endif
