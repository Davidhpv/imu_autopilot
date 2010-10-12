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

#ifndef I2C_MOTOR_CONTROLLER_H_
#define I2C_MOTOR_CONTROLLER_H_

#include "inttypes.h"

#define MOT_GET_RPM_CMD			0x11	///< command to read RPM from motor controllers (0b00010001)

/**
 * @brief Function to set RPM value on one of the motors
 *
 * This function sends the RPM value specified in rpm to the motor
 * controller specified by mot_i2c_dev_addr.
 *
 * @param mot_i2c_dev_addr I2C slave address of a motor controller. Possible values are: MOT1_I2C_SLAVE_ADDRESS, MOT2_I2C_SLAVE_ADDRESS, MOT3_I2C_SLAVE_ADDRESS or MOT4_I2C_SLAVE_ADDRESS
 * @param rpm	RPM value to be set
 */
void motor_i2c_set_rpm(uint8_t mot_i2c_dev_addr, uint16_t rpm);

/**
 * @brief Function to read current RPM value from one of the motors
 *
 * This function reads the current RPM value of the motor
 * controller specified by mot_i2c_dev_addr.
 *
 * @param mot_i2c_dev_addr I2C slave address of a motor controller. Possible values are: MOT1_I2C_SLAVE_ADDRESS, MOT2_I2C_SLAVE_ADDRESS, MOT3_I2C_SLAVE_ADDRESS or MOT4_I2C_SLAVE_ADDRESS
 * @return RPM value to be set
 */
uint16_t motor_i2c_get_rpm(uint16_t mot_i2c_dev_addr);

#endif /* I2C_MOTOR_CONTROLLER_H_ */
