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
 *   @brief Definition of Complimentary Attitude Filter
 *
 *   @author Joseph Kruijen <jkruijen@student.ethz.ch>
 *   @author Florian Zurbriggen <florianz@student.ethz.ch>
 *
 */

#ifndef _ATTITUDE_COMPL_EULER_H_
#define _ATTITUDE_COMPL_EULER_H_

#ifdef __cplusplus
extern "C" {
#endif



#include "global_data.h"
#include "fast_atan2.h"
#include "lookup_sin_cos.h"

void complimentary_filter_predict_rad(int16_vect3 accel_raw, uint16_vect3 gyro_raw, int16_vect3 mag_raw, float_vect3* attitude, float_vect3* att_rates);



#ifdef __cplusplus
}
#endif

#endif /* _ATTITUDE_COMPL_EULER_H_ */
