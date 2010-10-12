/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Joseph Kruijen, jkruijen@student.ethz.ch
Contributing Authors (in alphabetical order):



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

//#ifdef __cplusplus
//extern "C" {
//#endif

#ifndef _CONTROL_POSITION_H_
#define _CONTROL_POSITION_H_

/**
 * @brief Controls the atitude, nick and roll of the helicopter
 *
 * Describe which kind of controller is used here.
 *
 * @param x						The current position in meter
 * @param xRef					The current reference position in meter
 * @param xSpeed				The current speed in m/s
 * @param xSpeedRef				The current reference speed in m/s
 * @param y						The current position in meter
 * @param yRef					The current reference position in meter
 * @param ySpeed				The current speed in m/s
 * @param ySpeedRef				The current reference speed in m/s
 * @param roll					The angle of the helicopter in rad value range[-pi,pi]
 * @param rollSpeed				The current roll rate in [rad/s]
 * @param nick					The angle of the helicopter in rad value range[-pi,pi]
 * @param nickSpeed			The current nick rate in [rad/s]

 //Output values used in control_attitude.c
 * @param rollRef				The set reference angle of the helicopter in rad [-pi,pi]
 * @param rollSpeedRef			The set reference roll rate reference in [rad/s]
 * @param nickRef				The set reference angle of the helicopter in rad [-pi,pi]
 * @param nickSpeedRef			The set reference nick rate reference in [rad/s]
 *
 */

void control_position(float x, float xRef, float xSpeed, float xSpeedRef, float y, float yRef, float ySpeed, float ySpeedRef, float nick, float nickSpeed, float roll, float rollSpeed, float* nickRef, float* nickSpeedRef, float* rollRef, float* rollSpeedRef);



#endif /* _CONTROL_POSITION_H_ */

//#ifdef __cplusplus
//}
//#endif
