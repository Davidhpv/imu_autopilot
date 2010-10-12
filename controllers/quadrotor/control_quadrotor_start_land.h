/*
 * control_quadrotor_start_land.h
 *
 *  Created on: 13.07.2010
 *      Author: Laurens Mackay
 */

#ifndef CONTROL_QUADROTOR_START_LAND_H_
#define CONTROL_QUADROTOR_START_LAND_H_

#include "inttypes.h"
void quadrotor_start_land_handler(uint64_t loop_start_time);

float quadrotor_start_land_motor_thrust(void);

#endif /* CONTROL_QUADROTOR_START_LAND_H_ */
