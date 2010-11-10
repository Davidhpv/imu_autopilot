/*=====================================================================
 
 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
 Please see our website at <http://pixhawk.ethz.ch>
 
 (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
 
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
 *   @brief Definition of the class remote_control.
 *
 *   @author
 *
 */

#include "remote_control.h"

#include "control_quadrotor_attitude.h"
#include "conf.h"
#include "ppm.h"
#include "inttypes.h"
#include "global_data.h"
// Include comm
#include "comm.h"
#include "mavlink.h"
#include "i2c_motor_mikrokopter.h"
#include "pid.h"
#include "radio_control.h"
#include "control_quadrotor_attitude.h"
#include "control_quadrotor_position.h"
#include "debug.h"
#include "math.h"
#include "sys_state.h"



inline void remote_control(void)
{

	if (global_data.state.mav_mode == (uint8_t) MAV_MODE_MANUAL || global_data.state.mav_mode
			== (uint8_t) MAV_MODE_GUIDED || global_data.state.mav_mode
			== (uint8_t) MAV_MODE_AUTO)
	{
		if (radio_control_status() == RADIO_CONTROL_ON)
		{

			//get remote controll values
			float gas_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
					global_data.param[PARAM_PPM_THROTTLE_CHANNEL]) - PPM_OFFSET);
			//	message_debug_send(MAVLINK_COMM_1, 12, gas_remote);
			float yaw_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
					global_data.param[PARAM_PPM_YAW_CHANNEL]) - PPM_CENTRE);
			//	message_debug_send(MAVLINK_COMM_1, 13, yaw_remote);
			float nick_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
					global_data.param[PARAM_PPM_NICK_CHANNEL]) - PPM_CENTRE);
			//	message_debug_send(MAVLINK_COMM_1, 12, gas_remote);
			float roll_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
					global_data.param[PARAM_PPM_ROLL_CHANNEL]) - PPM_CENTRE);
			//	message_debug_send(MAVLINK_COMM_1, 13, yaw_remote);
			//	if(radio_control_status()==RADIO_CONTROL_OFF){
			//		gas_remote=0.3;
			//		yaw_remote=0;
			//		nick_remote=0;
			//		roll_remote=0;
			//	}
			//MANUAL ATTITUDE CONTROL
			global_data.attitude_setpoint_remote.x = -roll_remote;
			global_data.attitude_setpoint_remote.y = -nick_remote;
			global_data.attitude_setpoint_remote.z = -5 * yaw_remote;// used as yaw speed!!! yaw offset1
			global_data.gas_remote = gas_remote;

			//MANUAL POSITION CONTROL
			//TODO


			//		Switch on MAV_STATE_ACTIVE
			if ((ppm_get_channel(global_data.param[PARAM_PPM_THROTTLE_CHANNEL])
					< PPM_LOW_TRIG) && (ppm_get_channel(
					global_data.param[PARAM_PPM_YAW_CHANNEL]) < PPM_LOW_TRIG))
			{


				//				//Reset YAW integration(only needed if no vision)
				//				global_data.attitude.z = 0;
				//				global_data.yaw_pos_setpoint = 0;

				//switch on motors
				global_data.state.status = MAV_STATE_ACTIVE;
//				global_data.state.fly = FLY_WAIT_MOTORS;
//				this will be done by setpoint
				debug_message_buffer("MAV_STATE_ACTIVE Motors started");
			}

			//		Switch off MAV_STATE_STANDBY
			if ((ppm_get_channel(global_data.param[PARAM_PPM_THROTTLE_CHANNEL])
					< PPM_LOW_TRIG) && (ppm_get_channel(
					global_data.param[PARAM_PPM_YAW_CHANNEL]) > PPM_HIGH_TRIG))
			{
				//switch off motors
				global_data.state.status = MAV_STATE_STANDBY;

				debug_message_buffer("MAV_STATE_STANDBY Motors off");

			}

			//			//Integrate YAW setpoint
			//			if (fabs(
			//					(ppm_get_channel(global_data.param[PARAM_PPM_GIER_CHANNEL])
			//							- PPM_CENTRE)) > 10)
			//			{
			//				global_data.yaw_pos_setpoint -= 0.02 * 0.03
			//						* (ppm_get_channel(
			//								global_data.param[PARAM_PPM_GIER_CHANNEL])
			//								- PPM_CENTRE);
			//			}
			//		Set PID VALUES

			float tune2 = 1 * (ppm_get_channel(
					global_data.param[PARAM_PPM_TUNE2_CHANNEL]) - 1109);
			float tune3 = 1 * (ppm_get_channel(
					global_data.param[PARAM_PPM_TUNE3_CHANNEL]) - 1109);
			if (tune2 < 0)
			{
				tune2 = 0;
			}
			if (tune2 > 1000)
			{
				tune2 = 1000;
			}

			if (tune3 < 0)
			{
				tune3 = 0;
			}
			if (tune3 > 1000)
			{
				tune3 = 1000;
			}

			//TUNING FOR TOBIS REMOTE CONTROL

			if (global_data.param[PARAM_TRIMCHAN] == 1)
			{
								global_data.param[PARAM_PID_ATT_P] = 0.1 * tune2;
								global_data.param[PARAM_PID_ATT_D] = 0.1 * tune3;
			}
			else if (global_data.param[PARAM_TRIMCHAN] == 2)
			{
								global_data.param[PARAM_PID_POS_P] = 0.01 * tune2;
								global_data.param[PARAM_PID_POS_D] = 0.01 * tune3;
			}
			//this is done at 10 Hz
			//			pid_set_parameters(&nick_controller,
			//					global_data.param[PARAM_PID_ATT_P],
			//					global_data.param[PARAM_PID_ATT_I],
			//					global_data.param[PARAM_PID_ATT_D],
			//					global_data.param[PARAM_PID_ATT_AWU]);
			//			pid_set_parameters(&roll_controller,
			//					global_data.param[PARAM_PID_ATT_P],
			//					global_data.param[PARAM_PID_ATT_I],
			//					global_data.param[PARAM_PID_ATT_D],
			//					global_data.param[PARAM_PID_ATT_AWU]);
			//
			//			pid_set_parameters(&x_axis_controller,
			//					global_data.param[PARAM_PID_POS_P],
			//					global_data.param[PARAM_PID_POS_I],
			//					global_data.param[PARAM_PID_POS_D],
			//					global_data.param[PARAM_PID_POS_AWU]);
			//			pid_set_parameters(&y_axis_controller,
			//					global_data.param[PARAM_PID_POS_P],
			//					global_data.param[PARAM_PID_POS_I],
			//					global_data.param[PARAM_PID_POS_D],
			//					global_data.param[PARAM_PID_POS_AWU]);

			//global_data.param_name[PARAM_MIX_REMOTE_WEIGHT] = 1;// add position only keep - tune3;
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 1;

		}
		else
		{
			//No Remote signal
			//		TODO: Wait a bit and start sinking
			//switch off motors
			//		global_data.status = MAV_STATE_STANDBY;

			//FOR NOW JUST TURN OFF MOTORS (WE HAVE A CABLE)
			//FIXME Emergency Landing
			sys_set_mode(MAV_MODE_LOCKED);
			global_data.state.status = MAV_STATE_STANDBY;

			debug_message_buffer("EMERGENCY SWITCH OFF. No remote signal");
		}

	}

}
