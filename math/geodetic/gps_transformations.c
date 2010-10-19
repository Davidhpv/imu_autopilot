/*
 * gps_transformations.c
 *
 *  Created on: 19.10.2010
 *      Author: Laurens Mackay
 */
#include "gps_transformations.h"
#include "gps.h"
#include "debug.h"

static float_vect3 gps_local_origin;
static bool gps_local_origin_init = false;
static float gps_cos_origin_lat = 0;
const float r_earth = 6378140;

void gps_set_local_origin(void)
{
	gps_local_origin.x = gps_lat / 1e7f;
	gps_local_origin.y = gps_lon / 1e7f;
	gps_local_origin.z = gps_alt / 100e0f;
	gps_cos_origin_lat = cos(gps_local_origin.x * 3.1415 / 180);

	gps_local_origin_init = true;

	debug_message_buffer("GPS Local Origin saved");
}

void gps_get_local_position(float_vect3 * gps_local_position)
{
	if (gps_local_origin_init)
	{
		gps_local_position->x = r_earth * tan((gps_lat / 1e7f
				- gps_local_origin.x) * 3.1415 / 180);

		gps_local_position->y = r_earth * gps_cos_origin_lat * tan((gps_lon
				/ 1e7f - gps_local_origin.y) * 3.1415 / 180);

		gps_local_position->z = -(gps_alt / 100e0f - gps_local_origin.z);//z down

	}
	else
	{
		gps_set_local_origin(); //take actual position as origin

		gps_local_position->x = 0;
		gps_local_position->y = 0;
		gps_local_position->z = 0;
	}
}

void gps_get_local_velocity(float_vect3 * gps_local_velocity)
{
	gps_local_velocity->x = cos(gps_course * 10 * 3.1415 / 180) * gps_gspeed
			* 100;
	gps_local_velocity->y = sin(gps_course * 10 * 3.1415 / 180) * gps_gspeed
			* 100;
	//don't touch z-velocity.
}
