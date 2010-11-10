#include "control_quadrotor_attitude.h"
#include "conf.h"
#include "ppm.h"
#include "inttypes.h"
#include "global_data.h"
// Include comm
#include "comm.h"
#include "mavlink.h"
#include "debug.h"

#include "transformation.h"
#include "i2c_motor_mikrokopter.h"
#include "pid.h"
#include "radio_control.h"
#include "control_quadrotor_start_land.h"

#include <math.h>

#define CONTROL_PID_ATTITUDE_INTERVAL	5e-3
//defined in .h file
//PID_t yaw_speed_controller;
//PID_t nick_controller;
//PID_t roll_controller;
uint8_t controller_counter = 0;

inline void control_quadrotor_attitude_init()
{
	pid_init(&yaw_pos_controller, global_data.param[PARAM_PID_YAWPOS_P],
			global_data.param[PARAM_PID_YAWPOS_I],
			global_data.param[PARAM_PID_YAWPOS_D],
			global_data.param[PARAM_PID_YAWPOS_AWU], PID_MODE_DERIVATIV_SET,
			154);
	pid_init(&yaw_speed_controller, global_data.param[PARAM_PID_YAWSPEED_P],
			global_data.param[PARAM_PID_YAWSPEED_I],
			global_data.param[PARAM_PID_YAWSPEED_D],
			global_data.param[PARAM_PID_YAWSPEED_AWU], PID_MODE_DERIVATIV_CALC,
			155);
	pid_init(&nick_controller, 50, 0, 15, global_data.param[PARAM_PID_ATT_AWU],
			PID_MODE_DERIVATIV_SET, 156);
	pid_init(&roll_controller, 50, 0, 15, global_data.param[PARAM_PID_ATT_AWU],
			PID_MODE_DERIVATIV_SET, 157);
}

//void control_attitude(float roll, float rollRef, float rollSpeed, float rollSpeedRef, float nick, float nickRef, float nickSpeed, float nickSpeedRef){}
inline void control_quadrotor_attitude()
{
	float min_gas = 10;
	float max_gas = 255;
	//uint8_t stand_gas = 30;
	uint8_t motor_pwm[4];
	float motor_calc[4];
	//float global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
	//float global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
	float remote_control_weight_z = 1;
	//	float position_control_weight_z = 0;

	//Calculate setpoints

	//	Control Yaw POSTION before mixing speed!!!
	//TODO Check signs and -+180 degree
	float yaw_e = global_data.attitude.z - global_data.yaw_pos_setpoint;

	// don't turn around the wrong side (only works if yaw angle is between +- 180 degree)
	if (yaw_e > 3.14)
	{
		yaw_e -= 2 * 3.14;
	}
	if (yaw_e < -3.14)
	{
		yaw_e += 2 * 3.14;
	}

	float yaw_pos_corr = pid_calculate(&yaw_pos_controller,
			0, yaw_e, global_data.gyros_si.z,
			CONTROL_PID_ATTITUDE_INTERVAL);
	global_data.position_yaw_control_output = yaw_pos_corr;

	global_data.attitude_setpoint_pos.z = yaw_pos_corr;

	//limit control output
	if (global_data.attitude_setpoint_pos.z
			> global_data.param[PARAM_PID_YAWPOS_LIM])
	{
		global_data.attitude_setpoint_pos.z
				= global_data.param[PARAM_PID_YAWPOS_LIM];
		yaw_pos_controller.saturated=1;
	}
	if (global_data.attitude_setpoint_pos.z
			< -global_data.param[PARAM_PID_YAWPOS_LIM])
	{
		global_data.attitude_setpoint_pos.z
				= -global_data.param[PARAM_PID_YAWPOS_LIM];
		yaw_pos_controller.saturated=1;
	}

	//transform attitude setpoint from positioncontroller from navi to body frame on xy_plane
	navi2body_xy_plane(&global_data.attitude_setpoint_pos,
			global_data.attitude.z, &global_data.attitude_setpoint_pos_body); //yaw angle= global_data.attitude.z

	//now everything is in body frame
	global_data.attitude_setpoint.x
			= global_data.param[PARAM_MIX_REMOTE_WEIGHT]
					* global_data.attitude_setpoint_remote.x
					- global_data.param[PARAM_MIX_OFFSET_WEIGHT]
							* global_data.param[PARAM_ATT_OFFSET_X]
					+ global_data.param[PARAM_MIX_POSITION_WEIGHT]
							* global_data.attitude_setpoint_pos_body.x;
	global_data.attitude_setpoint.y
			= global_data.param[PARAM_MIX_REMOTE_WEIGHT]
					* global_data.attitude_setpoint_remote.y
					- global_data.param[PARAM_MIX_OFFSET_WEIGHT]
							* global_data.param[PARAM_ATT_OFFSET_Y]
					+ global_data.param[PARAM_MIX_POSITION_WEIGHT]
							* global_data.attitude_setpoint_pos_body.y;
	global_data.attitude_setpoint.z = remote_control_weight_z
			* global_data.attitude_setpoint_remote.z
			- global_data.param[PARAM_MIX_OFFSET_WEIGHT]
					* global_data.param[PARAM_ATT_OFFSET_Z]
			+ global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT]
					* global_data.attitude_setpoint_pos_body.z;

	/*Calculate Controllers*/

	//	Control Yaw Speed
	float yaw = pid_calculate(&yaw_speed_controller,
			global_data.attitude_setpoint.z, global_data.gyros_si.z, 0,
			CONTROL_PID_ATTITUDE_INTERVAL); //ATTENTION WE ARE CONTROLLING YAWspeed to YAW angle
	//Control Nick
	float nick = pid_calculate(&nick_controller,
			global_data.attitude_setpoint.y, global_data.attitude.y,
			global_data.gyros_si.y, CONTROL_PID_ATTITUDE_INTERVAL);
	//Control Roll
	float roll = pid_calculate(&roll_controller,
			global_data.attitude_setpoint.x, global_data.attitude.x,
			global_data.gyros_si.x, CONTROL_PID_ATTITUDE_INTERVAL);

	//compensation to keep force in z-direction
	float zcompensation;
	if (fabs(global_data.attitude.x) > 0.5)
	{
		zcompensation = 1.13949393;
	}
	else
	{
		zcompensation = 1 / cos(global_data.attitude.x);
	}
	if (fabs(global_data.attitude.y) > 0.5)
	{
		zcompensation *= 1.13949393;
	}
	else
	{
		zcompensation *= 1 / cos(global_data.attitude.y);
	}
	// use global_data.position_control_output.z and mix parameter global_data.param[PARAM_MIX_POSITION_Z_WEIGHT]
	// to compute thrust for Z position control
	//
	//	float motor_thrust = min_gas +
	//			( ( 1 - global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] ) * ( max_gas - min_gas ) * global_data.gas_remote * zcompensation )
	//		   + ( global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] * ( max_gas - min_gas ) * controlled_thrust * zcompensation );
	//calculate the basic thrust

	//CALCULATE HOVER THRUST, THEN SWITCH TO Z-CONTROLLER
	float motor_thrust = 0;


	// GUIDED AND AUTO MODES
	if (global_data.state.mav_mode == (uint8_t) MAV_MODE_GUIDED || global_data.state.mav_mode
			== (uint8_t) MAV_MODE_AUTO)
	{
		motor_thrust = quadrotor_start_land_motor_thrust();
	}
	else if (global_data.state.mav_mode == (uint8_t) MAV_MODE_MANUAL)
	{
		motor_thrust = global_data.gas_remote;

	}
	else if (global_data.state.mav_mode == (uint8_t)MAV_MODE_LOCKED)
	{
		// LOCKED MODE
		motor_thrust = 0;
	}
	else
	{
		// NO VALID MODE, LOCK DOWN TO ZERO
		motor_thrust = 0;
		// Message will flood buffer at 200 Hz, which is fine as it it critical
//		debug_message_buffer("ERROR: NO VALID MODE, THRUST LIMITED TO 0%");
	}
// OLD CODE from AMIR and ANDY
//	// GUIDED AND AUTO MODES
//	if (global_data.mode == (uint8_t)MAV_MODE_GUIDED || global_data.mode == (uint8_t)MAV_MODE_AUTO)
//	{
//
//		if (global_data.waiting_over == true)
//		{
//			if (global_data.ramp_up == true)
//			{
//				global_data.thrust_hover_offset += global_data.thrust_calibration_increment;
//				// check limit:
//				if (global_data.thrust_hover_offset > 0.65)
//				{
//					global_data.thrust_hover_offset = 0.65;
//					global_data.ramp_up = false;
//				}
//			}
//			motor_thrust = (global_data.thrust_hover_offset + global_data.position_control_output.z);
//			global_data.motor_thrust_actual=motor_thrust;
//
//			//Security: thrust never higher than remote control
//			if (motor_thrust > global_data.gas_remote)
//			{
//				motor_thrust = global_data.gas_remote;
//			}
//		}
//	}
//	else if (global_data.mode == (uint8_t)MAV_MODE_MANUAL)
//	{
////		if (global_data.waiting_over == true)
////				{
//				// MANUAL MODE
//				motor_thrust = global_data.gas_remote;
////				}
//	}
//	else if (global_data.mode == (uint8_t)MAV_MODE_LOCKED)
//	{
//		// LOCKED MODE
//		motor_thrust = 0;
//	}
//	else
//	{
//		// NO VALID MODE, LOCK DOWN TO ZERO
//		motor_thrust = 0;
//		// Message will flood buffer at 200 Hz, which is fine as it it critical
//		debug_message_buffer("ERROR: NO VALID MODE, THRUST LIMITED TO 0%");
//	}

	// Convertion to motor-step units
	motor_thrust *= zcompensation;
	motor_thrust *= (max_gas - min_gas);
	motor_thrust += min_gas;

	//limit control output

	//yawspeed
	if (yaw > global_data.param[PARAM_PID_YAWSPEED_LIM])
	{
		yaw = global_data.param[PARAM_PID_YAWSPEED_LIM];
		yaw_speed_controller.saturated = 1;
	}
	if (yaw < -global_data.param[PARAM_PID_YAWSPEED_LIM])
	{
		yaw = -global_data.param[PARAM_PID_YAWSPEED_LIM];
		yaw_speed_controller.saturated = 1;
	}

	if (nick > global_data.param[PARAM_PID_ATT_LIM])
	{
		nick = global_data.param[PARAM_PID_ATT_LIM];
		nick_controller.saturated = 1;
	}
	if (nick < -global_data.param[PARAM_PID_ATT_LIM])
	{
		nick = -global_data.param[PARAM_PID_ATT_LIM];
		nick_controller.saturated = 1;
	}


	if (roll > global_data.param[PARAM_PID_ATT_LIM])
	{
		roll = global_data.param[PARAM_PID_ATT_LIM];
		roll_controller.saturated = 1;
	}
	if (roll < -global_data.param[PARAM_PID_ATT_LIM])
	{
		roll = -global_data.param[PARAM_PID_ATT_LIM];
		roll_controller.saturated = 1;
	}

	// Emit motor values
	global_data.attitude_control_output.x = motor_thrust + roll;
	global_data.attitude_control_output.y = motor_thrust + nick;
	global_data.attitude_control_output.z = yaw;

	//add the yaw, nick and roll components to the basic thrust

	// RIGHT (MOTOR 3)
	motor_calc[0] = motor_thrust + yaw - roll;

	// LEFT (MOTOR 4)
	motor_calc[1] = motor_thrust + yaw + roll;

	// FRONT (MOTOR 1)
	motor_calc[2] = motor_thrust - yaw - nick;

	// BACK (MOTOR 2)
	motor_calc[3] = motor_thrust - yaw + nick;


	uint8_t i;
	for (i = 0; i < 4; i++)
	{
		//check for limits
		if (motor_calc[i] < min_gas)
		{
			motor_calc[i] = min_gas;
		}
		if (motor_calc[i] > max_gas)
		{
			motor_calc[i] = max_gas;
		}

	}

	//convert to byte
	motor_pwm[0] = (uint8_t) motor_calc[0];
	motor_pwm[1] = (uint8_t) motor_calc[1];
	motor_pwm[2] = (uint8_t) motor_calc[2];
	motor_pwm[3] = (uint8_t) motor_calc[3];
 //Disable for testing without motors
	if ((global_data.state.mav_mode == MAV_MODE_MANUAL || global_data.state.mav_mode
			== MAV_MODE_GUIDED) && (global_data.state.status == MAV_STATE_ACTIVE
			|| global_data.state.status == MAV_STATE_CRITICAL || global_data.state.status
			== MAV_STATE_EMERGENCY))
	{
		// Set MOTORS
		motor_i2c_set_pwm(MOT1_I2C_SLAVE_ADDRESS, motor_pwm[0]);
		motor_i2c_set_pwm(MOT2_I2C_SLAVE_ADDRESS, motor_pwm[1]);
		motor_i2c_set_pwm(MOT3_I2C_SLAVE_ADDRESS, motor_pwm[2]);
		motor_i2c_set_pwm(MOT4_I2C_SLAVE_ADDRESS, motor_pwm[3]);
	}
	else
	{
		if (global_data.state.mav_mode == MAV_MODE_TEST3
				&& global_data.param[PARAM_SW_VERSION] >= 0
				&& global_data.param[PARAM_SW_VERSION] <= 255)
		{
			//Testing motor trust with qgroundcontrol. PARAM_SW_VERSION is the thrust
//			motor_i2c_set_pwm(MOT1_I2C_SLAVE_ADDRESS,
//					global_data.param[PARAM_SW_VERSION]);
//			motor_i2c_set_pwm(MOT2_I2C_SLAVE_ADDRESS,
//					global_data.param[PARAM_SW_VERSION]);
//			motor_i2c_set_pwm(MOT3_I2C_SLAVE_ADDRESS,
//					global_data.param[PARAM_SW_VERSION]);
			motor_i2c_set_pwm(MOT4_I2C_SLAVE_ADDRESS,
					global_data.param[PARAM_SW_VERSION]);

		}
		else
		{
			// Set MOTORS to standby thrust
			motor_i2c_set_pwm(MOT1_I2C_SLAVE_ADDRESS, 0);
			motor_i2c_set_pwm(MOT2_I2C_SLAVE_ADDRESS, 0);
			motor_i2c_set_pwm(MOT3_I2C_SLAVE_ADDRESS, 0);
			motor_i2c_set_pwm(MOT4_I2C_SLAVE_ADDRESS, 0);
		}
	}

	//DEBUGGING
	if (controller_counter++ == 31)
	{
		controller_counter = 0;
		//run every 8th time

		if (global_data.param[PARAM_SEND_SLOT_DEBUG_3] == 1)
		{
//			debug_vect("att_sp", global_data.attitude_setpoint);
			//						mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 11, global_data.attitude_setpoint.x);
			//						mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 12, global_data.attitude_setpoint.y);
			//
			//			//		mavlink_msg_debug_send(MAVLINK_COMM_1, 6, gas_remote);
			//					mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 7, yaw);
			//					mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 8, nick);
			//					mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 9, roll);
			//			//		mavlink_msg_debug_send(MAVLINK_COMM_1, 10, ppm_get_channel(global_data.param[PARAM_PPM_GAS_CHANNEL]));
			//
			//			//
			//			//	message_debug_send(MAVLINK_COMM_1, 29, global_data.gyros_si.x);
			//			//	message_debug_send(MAVLINK_COMM_1, 28, global_data.gyros_si.y);
			//			//	message_debug_send(MAVLINK_COMM_1, 27, global_data.gyros_si.z);

			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 16,
					motor_pwm[0]);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 17,
					motor_pwm[1]);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 18,
					motor_pwm[2]);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 19,
					motor_pwm[3]);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 20,
					motor_thrust);
		}

	}

}
