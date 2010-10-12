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
 *   This file contains the device driver for the I2C motor controllers from Mikrokopter.
 *   @author Laurens Mackay <mavteam@student.ethz.ch>
 *
 */

#include "inttypes.h"
#include "i2c_motor_mikrokopter.h"
#include "features.h"
#include "conf.h"
#include "i2c.h"

#include "comm.h"
#include "led.h"
#include <stdio.h>


void motor_i2c_set_pwm(uint8_t mot_i2c_dev_addr, uint8_t pwm)
{
	i2c_package package;
	package.data[0] = pwm;								// PWM value to be set
	package.length = 1;									// 1 bytes for PWM setting
	package.direction = I2C_WRITE;						// I2C write operation
	package.slave_address = mot_i2c_dev_addr;			// I2C slave address of selected motor controller
	package.bus_number = MOT_I2C_BUS_NUMBER;			// number of the I2C bus, that the motor controller is connected to
	package.write_read = 0;								// no repeated start condition
	package.i2c_done_handler = NULL;					// nothing to be done at I2C completion
	i2c_op(&package);
}
