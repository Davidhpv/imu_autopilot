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
 * This Observer Filter, filters the measurement of a vector which is constant in the
 * World Frame. For prediction it uses the gyroscope measurements and for the correction
 * the measurement of the vector (accelerometer or magnetometer). It has a constant Kalman
 * Gain Matrix which can be calculated using MATLAB
 */

#include "attitude_observer.h"
#include "global_data.h"
#include "fast_atan2.h"
#include "debug.h"
#include <math.h>
#define TS 0.005f
//#define K_ACCEL 0.0033f now using PARAM
#define K_MAGNET 0.1f

static float_vect3 state_accel;
static float_vect3 state_magnet;

void attitude_observer_init(float_vect3 init_state_accel,
		float_vect3 init_state_magnet)
{
	state_accel = init_state_accel;
	state_magnet = init_state_magnet;
}

void attitude_observer_predict(float_vect3 gyros)
{
	float w1 = TS * gyros.x;
	float w2 = TS * gyros.y;
	float w3 = TS * gyros.z;

	//Predict accel Vector
	float x = state_accel.x + w3 * state_accel.y - w2 * state_accel.z;
	float y = -w3 * state_accel.x + state_accel.y + w1 * state_accel.z;
	float z = w2 * state_accel.x - w1 * state_accel.y + state_accel.z;
	state_accel.x = x;
	state_accel.y = y;
	state_accel.z = z;

	//Predict magnet Vector
	x = state_magnet.x + w3 * state_magnet.y - w2 * state_magnet.z;
	y = -w3 * state_magnet.x + state_magnet.y + w1 * state_magnet.z;
	z = w2 * state_magnet.x - w1 * state_magnet.y + state_magnet.z;

	state_magnet.x = x;
	state_magnet.y = y;
	state_magnet.z = z;
}

void attitude_observer_correct_accel(int16_vect3 accel)
{
	state_accel.x = state_accel.x * (1.0f
			- global_data.param[PARAM_ATT_KAL_KACC])
			+ global_data.param[PARAM_ATT_KAL_KACC] * (float) accel.x;
	state_accel.y = state_accel.y * (1.0f
			- global_data.param[PARAM_ATT_KAL_KACC])
			+ global_data.param[PARAM_ATT_KAL_KACC] * (float) accel.y;
	state_accel.z = state_accel.z * (1.0f
			- global_data.param[PARAM_ATT_KAL_KACC])
			+ global_data.param[PARAM_ATT_KAL_KACC] * (float) accel.z;
}

void attitude_observer_correct_magnet(int16_vect3 magnet)
{
	// Debug output
//
//	float_vect3 mag_cor;
//	mag_cor.x = magnet.x;
//	mag_cor.y = magnet.y;
//	mag_cor.z = magnet.z;
//
//	float_vect3 mag_pred;
//	mag_pred.x = state_magnet.x;
//	mag_pred.y = state_magnet.y;
//	mag_pred.z = state_magnet.z;
//
//	debug_vect("mgc", mag_cor);
//	debug_vect("mgp", mag_pred);

	state_magnet.x = state_magnet.x * (1.0f-K_MAGNET)
			+ K_MAGNET * (float) magnet.x;
	state_magnet.y = state_magnet.y * (1.0f-K_MAGNET)
			+ K_MAGNET * (float) magnet.y;
	state_magnet.z = state_magnet.z * (1.0f-K_MAGNET)
			+ K_MAGNET * (float) magnet.z;
}

void attitude_observer_get_angles(float_vect3* angles){
	angles->x = atan2(-state_accel.y, -state_accel.z);
	angles->y = asin(state_accel.x / 1000.0f);

//	angles->x = -state_accel.x;
//	angles->y = -state_accel.y;

	float x = cos(angles->y) * state_magnet.x + sin(angles->x) * sin(angles->y)
			* state_magnet.y + cos(angles->x) * sin(angles->y) * state_magnet.z;

	float y = cos(angles->x) * state_magnet.y - sin(angles->x) * state_magnet.z;
	angles->z = atan2(x, y);

	//mavlink_msg_debug_send(MAVLINK_COMM_0, 66, atan2(x, y));
	//mavlink_msg_debug_send(MAVLINK_COMM_0, 60, x);
	//mavlink_msg_debug_send(MAVLINK_COMM_0, 61, y);
}
