/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann
Contributing Authors (in alphabetical order):
  Dominik Honegger
  Tobias Naegeli
  Martin Rutschmann



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
* @file servos.h
*
* @brief setting servo positions
*
* This file contains functions to initialize and set servos.*
**/

#ifndef SERVOS_H_
#define SERVOS_H_

// Maximal decisive of the servos (50 degrees)
#define SERVOS_MAX_RAD (float)0.872664626

#include "pwm.h"
#include "conf.h"

/** @addtogroup HAL */
//@{
/** @name servo functions
 *  Initializing and setting servos */
//@{


/**
 * @brief Initializing of the servos
 */
static inline void servos_init(void)
{
	for (int i = 0; i < PWM_NB_CHANNELS; i++)
	{
		pwm_set_channel(1500, i);
	}
	return;
}

/**
 *
 * @param servo Index of the servo, 0: First servo, 8: last servo
 * @param angle Angle of the servo in radians
 * @return 1 on success, 0 on failure
 */
static inline uint8_t servos_set(int servo, float angle)
{
	if (servo < PWM_NB_CHANNELS)
	{
		float normed = (+angle) / SERVOS_MAX_RAD;
		unsigned int pwm_length = (unsigned int) (normed * 500 + 1500);
		pwm_set_channel(pwm_length, servo);
		return 1;
	}
	else
	{
		return 0;
	}
}

//@}}

#endif /* MOTORS_H_ */
