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

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _CONTROL_ATTITUDE_H_
#define _CONTROL_ATTITUDE_H_

/**
 * @brief Controls the atitude, nick and roll of the helicopter
 *
 * Describe which kind of controller is used here.
 *
 * @param roll					The current roll angle in	[rad] value range[-pi,pi]
 * @param rollRef				The roll reference angle in [rad] value range[-pi,pi]
 * @param rollSpeed				The current roll rate in	[rad/s]
 * @param rollSpeedRef			The current roll rate reference in [rad/s]
 * @param nick					The current nick angle in	[rad] value range[-pi,pi]
 * @param nickRef				The nick reference angle in [rad] value range[-pi,pi]
 * @param nickSpeed			The current nick rate in		[rad/s]
 * @param nickSpeedRef			The current nick rate reference in [rad/s]
 * @param setServoLeft          The angle of left servo in	[-] value range[-1,1]
 * @param setServoRight         The angle of right servo in [-] value range[-1,1]
 *
 */

void control_attitude(float roll, float rollRef, float rollSpeed, float rollSpeedRef, float nick, float nickRef, float nickSpeed, float nickSpeedRef, float* setServoPosY, float* setServoNegY);

#endif /* _CONTROL_ATTITUDE_H_ */

#ifdef __cplusplus
}
#endif
