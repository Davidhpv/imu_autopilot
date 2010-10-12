//10-Jun-2009 10:30:16
//10-Jun-2009 10:29:06
//10-Jun-2009 10:19:19
//10-Jun-2009 10:19:03
//29-May-2009 15:26:12
//28-May-2009 14:04:15
//28-May-2009 14:02:42
//28-May-2009 14:02:41
//12-May-2009 12:41:57
//12-May-2009 12:38:37
//12-May-2009 12:38:36
//12-May-2009 12:23:09
//12-May-2009 12:08:41
//12-May-2009 12:08:23
//12-May-2009 12:07:58
//12-May-2009 12:06:12
/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Joseph Kruijen, jkruijen@student.ethz.ch
Contributing Authors (in alphabetical order):



(c) 2009 PIXHAWK PROJECT

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

#include "control_yawSpeed.h"

void control_yawSpeed(float yawSpeedRef, float yawSpeed, float* delta_w) {

	// Instantiate result variables with zero content
	float deltaW_temp = 0;

	//Update controller with "update_control_yawSpeed.m"  not yet implemented
	//L1
	yawSpeedRef = yawSpeedRef * 0.31831;      	//normalize
	yawSpeed = yawSpeed * 0.31831;      		//normalize
	deltaW_temp = 0.2*2*(yawSpeedRef-yawSpeed);
	deltaW_temp = deltaW_temp * 0.24234;      	//denormalize
	//end_L1


	//for real use
	*delta_w = deltaW_temp;


	//L2
//
//end_L2
}

