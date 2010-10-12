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
 * @brief Auto-calibration of remote control, magnetometer, etc.
 *   @author Lorenz Meier
 *   @author Laurens MacKay
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

bool calibration_enter(void);
void calibration_exit(void);
void start_rc_calibration(void);
void start_mag_calibration(void);
void start_pressure_calibration(void);
void start_gyro_calibration(void);

#endif /* CALIBRATION_H_ */
