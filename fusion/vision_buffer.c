/*
 * vision_buffer.c
 *
 *  Created on: 13.05.2010
 *      Author: Laurens Mackay
 */

#ifndef VISION_BUFFER_C_
#define VISION_BUFFER_C_
#include "vision_buffer.h"
#include "float_checks.h"

#include "sys_time.h"
#include "global_data.h"
// Include comm
#include "comm.h"
#include "mavlink.h"

#include "shutter.h"

#include "global_pos.h"

#include "simple_altitude_moving_average.h"
#include "attitude_compl_euler.h"
#include "altitude_kalman.h"
#include "lookup_sin_cos.h"
#include "least_square.h"

#include "control_quadrotor_attitude.h"
#include "control_quadrotor_position.h"
#include "remote_control.h"
#include "position_kalman.h"

#include "debug.h"
#include "transformation.h"

#define VISION_BUFFER_COUNT 8

static vision_t vision_buffer[VISION_BUFFER_COUNT];
static uint8_t vision_buffer_index_write = 0;
static uint8_t vision_buffer_index_read = 0;
//static uint16_t vision_buffer_count = 0;
uint32_t vision_buffer_full_count = 0;
uint32_t vision_buffer_reject_count = 0;

void vision_buffer_buffer_init()
{
	for (int i = 0; i < VISION_BUFFER_COUNT; i++)
	{
		vision_buffer[i].time_captured = 0;
	}
}

void vision_buffer_buffer_camera_triggered(uint64_t usec,
		uint64_t loop_start_time, uint32_t seq)
{
	//debug_message_buffer("cam triggered");


	// Emit timestamp of this image
	//	mavlink_msg_image_triggered_send(MAVLINK_COMM_0,
	//			usec);


	if (vision_buffer_index_read - vision_buffer_index_write == 1
			|| (vision_buffer_index_read == 0 && vision_buffer_index_write
					== VISION_BUFFER_COUNT - 1))
	{
		//overwrite oldest
		vision_buffer_index_read = (vision_buffer_index_read + 1)
				% VISION_BUFFER_COUNT;

		//		if (vision_buffer_full_count++ % 16 == 0)
		if (vision_buffer_full_count++ % 256 == 0)
		{
			debug_message_buffer_sprintf("vision_buffer buffer full %u",
					vision_buffer_full_count);
		}

	}
	vision_buffer_index_write = (vision_buffer_index_write + 1)
			% VISION_BUFFER_COUNT;
	//vision_buffer_count++;

	// save position in buffer
	vision_buffer[vision_buffer_index_write].pos = global_data.position;
	vision_buffer[vision_buffer_index_write].ang = global_data.attitude;
	vision_buffer[vision_buffer_index_write].time_captured = usec;

	//debug_message_buffer("vision_buffer stored data");


	// Emit sensor data matching to this image
	//TODO: get this from buffer
	//	mavlink_msg_attitude_send(MAVLINK_COMM_0,
	//			usec,
	//			global_data.attitude.x, global_data.attitude.y,
	//			global_data.attitude.z, global_data.gyros_si.x,
	//			global_data.gyros_si.y, global_data.gyros_si.z);


}

/**
 * @brief Send one of the buffered messages
 * @param pos data from vision
 */
void vision_buffer_handle_data(mavlink_vision_position_estimate_t* pos)
{
	if (vision_buffer_index_write == vision_buffer_index_read)
	{
		//buffer empty
		return;

	}
	vision_buffer_index_read = (vision_buffer_index_read + 1)
			% VISION_BUFFER_COUNT;

	//TODO: find data and process it
	uint8_t for_count = 0;
	uint8_t i = vision_buffer_index_read;
	for (; (vision_buffer[i].time_captured < pos->usec)
			&& (vision_buffer_index_write - i != 1); i = (i + 1)
			% VISION_BUFFER_COUNT)
	{

		if (++for_count > VISION_BUFFER_COUNT)
		{
			debug_message_buffer("vision_buffer PREVENTED HANG");
			break;
		}
	}
	if (vision_buffer[i].time_captured == pos->usec)
	{

		//we found the right data
		if (!isnumber(pos->x) || !isnumber(pos->y) || !isnumber(pos->z)
				|| !isnumber(pos->roll) || !isnumber(pos->pitch) || !isnumber(pos->yaw)
				|| pos->x == 0.0 || pos->y == 0.0 || pos->z == 0.0)
		{
			//reject invalid data
			debug_message_buffer("vision_buffer invalid data (inf,nan,0) rejected");
		}
		else if (fabs(vision_buffer[i].ang.x - pos->roll)
				< global_data.param[PARAM_VISION_ANG_OUTLAYER_TRESHOLD]
				&& fabs(vision_buffer[i].ang.y - pos->pitch)
						< global_data.param[PARAM_VISION_ANG_OUTLAYER_TRESHOLD])
		{
			//Do outlayer rejection
			//debug_message_buffer("vision_buffer data found OK");

			// Update validity time
			global_data.pos_last_valid = sys_time_clock_get_time_usec();

			//Project correction to present

			float_vect3 pos_e;
			pos_e.x = pos->x - vision_buffer[i].pos.x;
			pos_e.y = pos->y - vision_buffer[i].pos.y;
			pos_e.z = pos->z - vision_buffer[i].pos.z;

//			debug_vect("pos_e", pos_e);

			//Correct yaw angle because it is not yet implemented in vision.
			float yaw_navi = pos->yaw
					- global_data.param[PARAM_VISION_YAWCORRECT];

			float yaw_e = yaw_navi - vision_buffer[i].ang.z;

			float_vect3 pos_diff;
			pos_diff.x = global_data.position.x - vision_buffer[i].pos.x;
			pos_diff.y = global_data.position.y - vision_buffer[i].pos.y;
			pos_diff.z = global_data.position.z - vision_buffer[i].pos.z;

			//turn pos_diff clockwise if yaw_e positive
			float_vect3 pos_diff_turned;
			//correct for yaw error
			turn_xy_plane(&pos_diff, yaw_e, &pos_diff_turned);

			//or don't correct for yaw error
//			turn_xy_plane(&pos_diff, 0, &pos_diff_turned);

			//Pack new vision_data package
			global_data.vision_data.time_captured
					= vision_buffer[i].time_captured;
			global_data.vision_data.comp_end = sys_time_clock_get_time_usec();

			//Set data from Vision directly
			global_data.vision_data.ang.x = pos->roll;
			global_data.vision_data.ang.y = pos->pitch;
			global_data.vision_data.ang.z = pos->yaw;

			global_data.vision_data.pos.x = pos->x;
			global_data.vision_data.pos.y = pos->y;
			global_data.vision_data.pos.z = pos->z;

			//Onlz use vision yaw because of Lorenz not trusting integration and prediction
			//	global_data.hack_yaw = pos->yaw;
			//global_data.vision_data.ang.z = pos.yaw;

			//		global_data.vision_data.pos.x = pos->x;
			//		global_data.vision_data.pos.y = pos->y;
			//		global_data.vision_data.pos.z = pos->z;


			//take directly the Vision position and speed
			//		global_data.position.x = pos->x;
			//		global_data.position.y = pos->y;
			//		global_data.position.z = pos->z;
			//////update velocity
			//
			//
			//		global_data.velocity.x = pos->vx;
			//		global_data.velocity.y = pos->vy;
			//		global_data.velocity.z = pos->vz;

			//		use predicted error
							global_data.vision_data.pos.x = vision_buffer[i].pos.x
									+ pos_diff_turned.x + pos_e.x;
							global_data.vision_data.pos.y = vision_buffer[i].pos.y
									+ pos_diff_turned.y + pos_e.y;
							global_data.vision_data.pos.z = vision_buffer[i].pos.z
									+ pos_diff_turned.z + pos_e.z;

			//Correct YAW
			global_data.attitude.z = global_data.attitude.z + yaw_e;
			//If yaw goes to infinity (no idea why) set it to setpoint, next time will be better
			if (global_data.attitude.z > 20 || global_data.attitude.z < -20)
			{
				global_data.attitude.z = global_data.yaw_pos_setpoint;
				debug_message_buffer("vision_buffer CRITICAL FAULT yaw was bigger than 20! prevented crash");
			}

			global_data.vision_data.new_data = 1;

			//TODO correct also all buffer data needed if we are going to have overlapping vision data
		}
		else
		{
			//rejected outlayer
			if (vision_buffer_reject_count++ % 16 == 0)
			{
				debug_message_buffer_sprintf("vision_buffer outlayer rejected %u",
						vision_buffer_reject_count);
			}
			//debug_message_buffer("vision_buffer outlayer rejected");
		}
		if (global_data.param[PARAM_SEND_SLOT_DEBUG_1] == 1)
		{

			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 202, global_data.attitude.z);

			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 210, pos.x);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 211, pos.y);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 212, pos.z);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 215, pos.yaw);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 212, pos.z);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 203, pos.r1);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 204, pos.confidence);
		}
		if (for_count)
		{
			debug_message_buffer_sprintf(
					"vision_buffer data found skipped %i data sets", for_count);
		}
	}
	else
	{
		//we didn't find it
		//		debug_message_buffer("vision_buffer data NOT found");
		if (for_count)
		{
			debug_message_buffer_sprintf(
					"vision_buffer data NOT found skipped %i data sets",
					for_count);
		}
	}
	vision_buffer_index_read = i;//skip all images that are older;
	//	if (for_count)
	//	{
	//		debug_message_buffer_sprintf("vision_buffer skipped %i data sets",
	//				for_count);
	//	}
}

#endif /* VISION_BUFFER_C_ */
