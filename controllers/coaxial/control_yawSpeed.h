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

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _CONTROL_YAWSPEED_H_
#define _CONTROL_YAWSPEED_H_

/**
 * @brief Controls the yaw angle and altitude of the helicopter
 *
 * @todo Describe which kind of controller is used here.
 *
 * @param yawSpeedRef   The reference yaw speed in [rad/s]
 * @param yawSpeed		The current yaw speed in [rad/s]
 * @param delta_w	    The half the difference of the motor controller inputs
 *
 */

void control_yawSpeed(float yawSpeedRef, float yawSpeed, float* delta_w);



#endif /* _CONTROL_YAWSPEED_H_ */

#ifdef __cplusplus
}
#endif
