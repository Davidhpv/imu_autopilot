/*
 * gps_transformations.h
 *
 *  Created on: 19.10.2010
 *      Author: Laurens Mackay
 */

#ifndef GPS_TRANSFORMATIONS_H_
#define GPS_TRANSFORMATIONS_H_


#include "mav_vect.h"

/* @brief set origin for tangential plane to actual gps position */
void gps_set_local_origin(void);

/* @brief Convert the coordinates from gps_* variables in gps.h to gps_local relative in meters
 * @param gps_local_position vector to save relative position in.*/
void gps_get_local_position(float_vect3 * gps_local_position);
/* Same for velocity. Only x-y plane.*/
void gps_get_local_velocity(float_vect3 * gps_local_velocity);

#endif /* GPS_TRANSFORMATIONS_H_ */
