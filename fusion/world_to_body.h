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
 *   @brief Definition of coordinate transformation functions
 *
 *   Functions to transform from world to body and vice-versa
 *
 *   @author Martin Rutschmann <mavteam@student.ethz.ch>
 *   @author Dominik Honegger <mavteam@student.ethz.ch>
 *
 */

#ifndef WORLD_TO_BODY_H
#define WORLD_TO_BODY_H

#include "mav_vect.h"

/** @brief Calculation from world coordinates to body coordinates */
void world_to_body(float_vect3 world_vector, float_vect3 attitude,float_vect3* body_vector);

#endif /* WORLD_TO_BODY_H */
