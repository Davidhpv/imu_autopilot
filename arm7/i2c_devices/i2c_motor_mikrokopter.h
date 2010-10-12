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

#ifndef I2C_MOTOR_MIKROKOPTER_H_
#define I2C_MOTOR_MIKROKOPTER_H_

#include "inttypes.h"

/**
 * @brief Function to set PWM value on one of the motors
 *
 * This function sends the PWM value specified in pwm to the motor
 * controller specified by mot_i2c_dev_addr.
 *
 * @param mot_i2c_dev_addr I2C slave address of a motor controller. Possible values are: MOT1_I2C_SLAVE_ADDRESS, MOT2_I2C_SLAVE_ADDRESS, MOT3_I2C_SLAVE_ADDRESS or MOT4_I2C_SLAVE_ADDRESS
 * @param pmw PWM value to be set (Duty Cycle)
 */
void motor_i2c_set_pwm(uint8_t mot_i2c_dev_addr, uint8_t pwm);

#endif /* I2C_MOTOR_MIKROKOPTER_H_ */

