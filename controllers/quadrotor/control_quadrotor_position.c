#include "control_quadrotor_position.h"
#include "conf.h"
#include "ppm.h"
#include "inttypes.h"
#include "global_data.h"
// Include comm
#include "comm.h"
#include "mavlink.h"
#include "debug.h"

#include "i2c_motor_mikrokopter.h"
#include "pid.h"
#include "radio_control.h"

#include "math.h"

// 20ms (50Hz) why was it 33.33e-3 ??
#define CONTROL_PID_POSITION_INTERVAL	0.020

uint8_t pos_controller_counter = 0;

inline void control_quadrotor_position_init()
{
	//	global_data.position_setpoint.x = .51;//.21;
	//	global_data.position_setpoint.y = .4485;// .1485;
	//	global_data.position_setpoint.z = -1;
	global_data.position_setpoint.x
			= global_data.param[PARAM_POSITION_SETPOINT_X];//.51;//.21;
	global_data.position_setpoint.y
			= global_data.param[PARAM_POSITION_SETPOINT_Y];//.4485;// .1485;
	global_data.position_setpoint.z
			= global_data.param[PARAM_POSITION_SETPOINT_Z];//-1;
	global_data.yaw_pos_setpoint
			= global_data.param[PARAM_POSITION_SETPOINT_YAW];

	pid_init(&x_axis_controller, 10, 0, 0,
			global_data.param[PARAM_PID_POS_AWU], PID_MODE_DERIVATIV_SET, 150);//150
	pid_init(&y_axis_controller, 10, 0, 0,
			global_data.param[PARAM_PID_POS_AWU], PID_MODE_DERIVATIV_SET, 151);//151
	pid_init(&z_axis_controller, 10, 0, 0,
			global_data.param[PARAM_PID_POS_Z_AWU], PID_MODE_DERIVATIV_SET, 152);

}

//void control_attitude(float roll, float rollRef, float rollSpeed, float rollSpeedRef, float nick, float nickRef, float nickSpeed, float nickSpeedRef){}
inline void control_quadrotor_position()
{

	//#define PPM_OFFSET 			1000
	//#define PPM_SCALE_FACTOR 	0.001f

	//#define YAW_WEIGHT 0.5

	//set the max and min angle to 20 degree (about 0.350 rad)
//	float max_angle = 0.2;//0.350;// 0.350; //now using parameters

	//float max_z_speed = 0.1;

	//get remote controll values
	//	float z_axis_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
	//			global_data.param[PARAM_PPM_GAS_CHANNEL]) - PPM_OFFSET);
	//
	//	float yaw_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
	//			global_data.param[PARAM_PPM_GIER_CHANNEL]) - 1500);
	//
	//	float x_axis_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
	//			global_data.param[PARAM_PPM_NICK_CHANNEL]) - 1500);
	//
	//	float y_axis_remote = PPM_SCALE_FACTOR * (ppm_get_channel(
	//			global_data.param[PARAM_PPM_ROLL_CHANNEL]) - 1500);
	//
	//	if(radio_control_status()==RADIO_CONTROL_OFF){
	//		z_axis_remote= -max_z_speed;
	//		x_axis_remote=0;
	//		y_axis_remote=0;
	//		yaw_remote=0;
	//	}
	/*Calculate Controllers*/

	//Control X axis
	float x_axis_correcture = pid_calculate(&x_axis_controller,
			global_data.position_setpoint.x, global_data.position.x,
			global_data.velocity.x - global_data.param[PARAM_VEL_OFFSET_X],
			CONTROL_PID_POSITION_INTERVAL);
	global_data.position_control_output.x = x_axis_correcture;
	//Control Y axis
	float y_axis_correcture = pid_calculate(&y_axis_controller,
			global_data.position_setpoint.y, global_data.position.y,
			global_data.velocity.y - global_data.param[PARAM_VEL_OFFSET_Y],
			CONTROL_PID_POSITION_INTERVAL);
	global_data.position_control_output.y = y_axis_correcture;

	//Control Position in Z direction
	float z_axis_correcture = pid_calculate(&z_axis_controller,
			global_data.position_setpoint.z, global_data.position.z,
			global_data.velocity.z, CONTROL_PID_POSITION_INTERVAL);
	global_data.position_control_output.z = -z_axis_correcture;

	// Limitation for z Control
	if (global_data.position_control_output.z
			> global_data.param[PARAM_PID_POS_Z_LIM])
	{
		global_data .position_control_output.z
				= global_data.param[PARAM_PID_POS_Z_LIM];

		//Inform Controller about saturation. TODO Check if this works and do it for all other controllers.
		z_axis_controller.saturated=1;

//		debug_message_buffer_sprintf(
//				"zcontrol output + limited would be*1000=%i", (uint32_t) 1000
//						* global_data.position_control_output.z);
	}
	if (global_data.position_control_output.z
			< 0)
	{
		global_data.position_control_output.z
				= 0;

		//Inform Controller about saturation. TODO Check if this works and do it for all other controllers.
		z_axis_controller.saturated=1;

//		debug_message_buffer_sprintf(
//				"zcontrol output - limited would be*1000=%i", (uint32_t) 1000
//						* global_data.position_control_output.z);
	}

	// Factor 0.05 to scale K_P, K_I to better human-readable values
	global_data.attitude_setpoint_pos.y = 0.05 * -x_axis_correcture; //y angle
	global_data.attitude_setpoint_pos.x = 0.05 * y_axis_correcture;

	//check for limits

	if (global_data.attitude_setpoint_pos.x > global_data.param[PARAM_PID_POS_LIM])
	{
		global_data.attitude_setpoint_pos.x = global_data.param[PARAM_PID_POS_LIM];
		y_axis_controller.saturated=1;
//		debug_message_buffer_sprintf(
//				"xcontrol output + limited would be*1000=%i", (uint32_t) 1000
//						* global_data.attitude_setpoint_pos.x);
	}
	else if (global_data.attitude_setpoint_pos.x < -global_data.param[PARAM_PID_POS_LIM])
	{
		global_data.attitude_setpoint_pos.x = -global_data.param[PARAM_PID_POS_LIM];
		y_axis_controller.saturated=1;
//		debug_message_buffer_sprintf(
//				"xcontrol output - limited would be*1000=%i", (uint32_t) 1000
//						* global_data.attitude_setpoint_pos.x);
	}

	if (global_data.attitude_setpoint_pos.y > global_data.param[PARAM_PID_POS_LIM])
	{
		global_data.attitude_setpoint_pos.y = global_data.param[PARAM_PID_POS_LIM];
		x_axis_controller.saturated=1;
//		debug_message_buffer_sprintf(
//				"ycontrol output + limited would be*1000=%i", (uint32_t) 1000
//						* global_data.attitude_setpoint_pos.y);
	}
	else if (global_data.attitude_setpoint_pos.y < -global_data.param[PARAM_PID_POS_LIM])
	{
		global_data.attitude_setpoint_pos.y = -global_data.param[PARAM_PID_POS_LIM];
		x_axis_controller.saturated=1;
//		debug_message_buffer_sprintf(
//				"ycontrol output - limited would be*1000=%i", (uint32_t) 1000
//						* global_data.attitude_setpoint_pos.y);
	}

	//DEBUGGING
	if (pos_controller_counter++ == 3)
	{
		pos_controller_counter = 0;
		//run every 8th time

		if (global_data.param[PARAM_SEND_SLOT_DEBUG_3] == 1)
		{
			debug_vect("att_sp_pos", global_data.attitude_setpoint_pos);
			//			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 60,
			//					global_data.attitude_setpoint_pos.x);
			//			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 61,
			//					global_data.attitude_setpoint_pos.y);
			//			//mavlink_msg_debug_send(MAVLINK_COMM_1, 62, z_axis_correcture);
		}
	}

	//TODO check z and yaw speed


}
