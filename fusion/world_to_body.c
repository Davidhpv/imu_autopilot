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
 *   @brief Implementation of coordinate transformation functions
 *
 *   Functions to transform from world to body and vice-versa
 *
 *   @author Martin Rutschmann <mavteam@student.ethz.ch>
 *   @author Dominik Honegger <mavteam@student.ethz.ch>
 *
 */

#include "world_to_body.h"
#include "mav_vect.h"
#include "math.h"

/**
 * @param world_vector input vector in world coordinates
 * @param attitude current attitude
 * @param body_vector output vector in body coordinates
 *
 */
void world_to_body(float_vect3 world_vector, float_vect3 attitude,float_vect3* body_vector)
{
	float pitch = attitude.y;
	// yaw angle is zero towards y direction;
	float yaw = attitude.z + 1.57079633f;
	float roll = attitude.x;

	(*body_vector).x = -cos(pitch)*cos(yaw)*world_vector.x 									+ cos(pitch)*sin(yaw)*world_vector.y 									+ sin(pitch)*world_vector.z;
	(*body_vector).y = -(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))*world_vector.x 	+ (sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw))*world_vector.y 	- sin(roll)*cos(pitch)*world_vector.z;
	(*body_vector).z = -(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*world_vector.x 	+ (cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))*world_vector.y		- cos(roll)*cos(pitch)*world_vector.z;
}
