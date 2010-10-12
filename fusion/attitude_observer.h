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
 *   @brief Attitude Observer Filter
 *
 *   @author Martin Rutschmann <pixhawk@student.ethz.ch>
 *
 * This Kalman Filter, filters the measurement of a vector which is constant in the
 * World Frame. For prediction it uses the gyroscope measurements and for the correction
 * the measurement of the vector (accelerometer or magnetometer). It has a constant Kalman
 * Gain Matrix which can be calculated using MATLAB
 */
 
#ifndef ATTITUDE_OBSERVER_H_
#define ATTITUDE_OBSERVER_H_

#include "mav_vect.h"

void attitude_observer_init(float_vect3 init_state_accel, float_vect3 init_state_magnet);

void attitude_observer_predict(float_vect3 gyros);

void attitude_observer_correct_accel(int16_vect3 accel);

void attitude_observer_correct_magnet(int16_vect3 magnet);

void attitude_observer_get_angles(float_vect3* angles);

#endif /* ATTITUDE_OBSERVER_H_ */

