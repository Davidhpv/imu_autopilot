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
 *   @brief This is an example file
 *
 *   This is the more detailed description of this file.
 *
 *   @author Laurens Mackay <mavteam@student.ethz.ch>
 *   @author Amir <mavteam@student.ethz.ch>
 *
 */

#include "mav_vect.h"

/* @brief Tranform a vector from body-frame to navigation frame-frame
 * @param vector Vector to fransform
 * @param angles Angles to do the transformation with
 * @param result Vector to save the result in
 * */
void turn_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result);
void navi2body_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result);
void body2navi_xy_plane(const float_vect3* vector, const float yaw,
		float_vect3* result);
void navi2body(const float_vect3* vector, const float_vect3* angles,
		float_vect3* result);
void body2navi(const float_vect3* vector, const float_vect3* angles,
		float_vect3* result);
