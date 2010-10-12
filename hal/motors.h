/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann
Contributing Authors (in alphabetical order):
  Dominik Honegger
  Tobias Naegeli



(c) 2008, 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/
/**
* @file motors.h
*
* @brief set motor values
*
* This file contains a function to set the speed of both motors.
*
**/
#ifndef MOTORS_H_
#define MOTORS_H_

#include "pwm.h"
#include "conf.h"

/** @addtogroup HAL */
//@{
/** @name Motor functions
 *  Set Motor speed */
//@{

#if defined(BOARD_PIXHAWK_V100) || defined(BOARD_TWOG_BOOZ)

/**
 * @brief This function initialize the motors
 */
static inline void motors_init(void){
	pwm_set_channel(1000, MOTORS_LOWER_PWM_CHANNEL);
	pwm_set_channel(1000, MOTORS_UPPER_PWM_CHANNEL);
	return;
}

/**
 * @brief This function sets the speed of the upper motor
 * @param set the speed of the motor, this has to be a float between
 * 0 and 1. If it is bigger or smaller than one it will be set to 0 or 1.
 */
static inline void motors_set_lower(float speed){
	if(speed<0){
		speed=0;
	}
	if(speed>1){
		speed=1;
	}

	unsigned int pwm_length = (unsigned int) (1000*speed + 1000);

	pwm_set_channel(pwm_length, MOTORS_LOWER_PWM_CHANNEL);
}

/**
 * @brief This function sets the speed of the lower motor
 * @param set the speed of the motor, this has to be a float between
 * 0 and 1. If it is bigger or smaller than one it will be set to 0 or 1.
 */
static inline void motors_set_upper(float speed){
	if(speed<0){
		speed=0;
	}
	if(speed>1){
		speed=1;
	}

	unsigned int pwm_length = (unsigned int) (1000*speed + 1000);

	pwm_set_channel(pwm_length, MOTORS_UPPER_PWM_CHANNEL);
}
//@}}

#endif

#endif /* MOTORS_H_ */
