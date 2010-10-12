/*=====================================================================
 
PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>
 
(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
 
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
 

#include "pid.h"
/**
 * @file
 *   @brief Fixed wing attitude control
 *
 *   @author
 *
 */
 

#ifndef CONTROL_FIXED_WING_ATTITUDE_H_
#define CONTROL_FIXED_WING_ATTITUDE_H_

PID_t fw_yaw_controller;
PID_t fw_nick_controller;
PID_t fw_roll_controller;

void control_fixed_wing_attitude_init(void);

void control_fixed_wing_attitude(void);

#endif /* CONTROL_FIXED_WING_ATTITUDE_H_ */


