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
*   @brief generate pwm signals for servos and motors.
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*  For the generation of the pwm signals a 4017 decade counter is used.
*  With it you can generate 8 pwm signal with using only two processor
*  pins.
*/

#ifndef PWM_H
#define PWM_H

#include "LPC21xx.h"
#include "sys_time.h"
#include "conf.h"

/**
 * @brief pwm initialization
 *
 * Initialization of the pwm generation. If you want to use pwm signals
 * you also have to initialize sys_time. The pwm signal is generated with
 * a 4017 decade counter. Before initializing pwm, you frist should
 * initialize sys_time.h
 *
 * @see sys_time.h
 */
void pwm_init(void);

/**
 * @brief setting the pwm with of a channel
 *
 * You can set the width of a channel with this function.
 *
 * @param length_usec	pulse length in usec
 * @param channel_nr	channel number
 */
void pwm_set_channel(unsigned int length_usec, unsigned int channel_nr);

/**
 * @brief Interruptt routine for pwm generation.
 *
 * This function is called by the timer interrupt.
 *
 * @see sys_time.h
 */
void PWM_ISR(void);

#endif /* PWM_H */
