/*
 * mainfunctions.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#include "common_mainloop_functions.h"

#include "inttypes.h"
#include "mcu_init.h"
#include "conf.h"

// Include comm
#include "comm.h"
#include "mavlink.h"

// Include globals
#include "global_data.h"
#include "sensors.h"
#include "calibration.h"

#include "adc.h"
#include "led.h"
#include "ppm.h"
#include "pwm.h"
#include "sys_time.h"
#include "uart.h"
#include "dac.h"

#include "watchdog.h"
#include "control.h"

#include "compass.h"
#include "acceleration.h"
#include "gyros.h"
#include "motors.h"
#include "servos.h"
#include "radio_control.h"
#include "battery.h"
#include "shutter.h"
#include "infrared_distance.h"
#include "gps.h"

#include "altitude_speed.h"
#include "global_pos.h"

#include "simple_altitude_moving_average.h"
#include "attitude_compl_euler.h"
#include "altitude_kalman.h"
#include "lookup_sin_cos.h"
#include "least_square.h"

#include "control_quadrotor_attitude.h"
#include "control_fixed_wing_attitude.h"
#include "control_quadrotor_position.h"
#include "control_quadrotor_start_land.h"
#include "remote_control.h"
#include "position_kalman3.h"
#include "vision_buffer.h"

#include "debug.h"
#include "transformation.h"
#include "eeprom.h"
#include "params.h"
#include "attitude_observer.h"

#include "mmc_spi.h"
#include "dos.h"
#include "fat.h"

static uint32_t next_exec_time[NUM_OF_COUNTERS]; ///< Software counter for mainloop time control


// Integrating Position testing. Laurens
void position_integrate(float_vect3* att,float_vect3 *pos,float_vect3 *vel,float_vect3 *acc)
{
//	const float time = 0.005; //200Hz
//	static float_vect3 acc_offset;
	//static float_vect3 vel_offset;
	float_vect3 acc_nav;
	body2navi(acc, att, &acc_nav);
	debug_vect("acc_navi", acc_nav);
//
//	acc_nav.x = (acc_nav.x - global_data.param[PARAM_ACC_NAVI_OFFSET_X])/100;
//	acc_nav.y = (acc_nav.y - global_data.param[PARAM_ACC_NAVI_OFFSET_Y])/100;
//	acc_nav.z = (acc_nav.z - global_data.param[PARAM_ACC_NAVI_OFFSET_Z])/100;

//	float vel_lowpass = 0.01;
//	vel_offset.x = vel_offset.x * (1 - vel_lowpass) + vel->x * vel_lowpass;
//	vel_offset.y = vel_offset.y * (1 - vel_lowpass) + vel->y * vel_lowpass;
//	vel_offset.z = vel_offset.z * (1 - vel_lowpass) + vel->z * vel_lowpass;
//
//	acc_nav.x -= acc_offset.x;
//	acc_nav.y -= acc_offset.y;
//	acc_nav.z -= acc_offset.z;


//	acc_nav.x -= acc_offset.x;
//	acc_nav.y -= acc_offset.y;
//	acc_nav.z -= acc_offset.z;
//
//	if (abs(acc_nav.x) > 0.01)
//	{
//	vel->x += acc_nav.z * time;
//		vel->x*=global_data.param[PARAM_VEL_DAMP];
//
//		pos->x += vel->x * time + acc_nav.x * time * time;
//	}else{
//
//		float acc_lowpass = 0.01;
//		acc_offset.x = acc_offset.x * (1 - acc_lowpass) + acc_nav.x * acc_lowpass;
//		acc_offset.y = acc_offset.y * (1 - acc_lowpass) + acc_nav.y * acc_lowpass;
//		acc_offset.z = acc_offset.z * (1 - acc_lowpass) + acc_nav.z * acc_lowpass;
//
//	}
//
//	static uint8_t i = 0;
//	if (i++ > 50)
//	{
//		i = 0;
//		debug_vect("acc_navi", acc_nav);
//	}



}

void communication_send_raw_data(uint64_t loop_start_time)
{
	if (global_data.param[PARAM_SEND_SLOT_RAW_IMU] == 1)
	{

		mavlink_msg_raw_imu_send(
				global_data.param[PARAM_SEND_DEBUGCHAN],
				sys_time_clock_get_unix_offset() + loop_start_time,
				global_data.accel_raw.x, global_data.accel_raw.y,
				global_data.accel_raw.z, global_data.gyros_raw.x,
				global_data.gyros_raw.y, global_data.gyros_raw.z,
				global_data.magnet_raw.x,
				global_data.magnet_raw.y,
				global_data.magnet_raw.z);
	}
}

void communication_send_attitude_position(uint64_t loop_start_time)
{
	// ATTITUDE at 20 Hz
	if (global_data.param[PARAM_SEND_SLOT_ATTITUDE] == 1)
	{
		// Send attitude over both UART ports
		mavlink_msg_attitude_send(MAVLINK_COMM_0,
				sys_time_clock_get_unix_offset() + loop_start_time,
				global_data.attitude.x, global_data.attitude.y,
				global_data.attitude.z, global_data.gyros_si.x,
				global_data.gyros_si.y, global_data.gyros_si.z);
		mavlink_msg_attitude_send(MAVLINK_COMM_1,
				sys_time_clock_get_unix_offset() + loop_start_time,
				global_data.attitude.x, global_data.attitude.y,
				global_data.attitude.z, global_data.gyros_si.x,
				global_data.gyros_si.y, global_data.gyros_si.z);
	}

	if (global_data.param[PARAM_SEND_SLOT_DEBUG_5] == 1)
	{
		// Send current position and speed
		mavlink_msg_local_position_send(MAVLINK_COMM_0, sys_time_clock_get_unix_offset() + loop_start_time,
				global_data.position.x, global_data.position.y,
				global_data.position.z, global_data.velocity.x,
				global_data.velocity.y, global_data.velocity.z);
	}
}

uint8_t rc_to_255(int chan)
{
	return clamp(0, ((radio_control_get_channel(chan)+1.0f)/2.0f)*255.0f, 255);
}

void communication_send_remote_control(void)
{
	if (global_data.param[PARAM_SEND_SLOT_REMOTE_CONTROL] == 1)
	{
		if (global_data.param[PARAM_SEND_DEBUGCHAN] == 0)
		{
		mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
				radio_control_get_channel_raw(1),
				radio_control_get_channel_raw(2),
				radio_control_get_channel_raw(3),
				radio_control_get_channel_raw(4),
				radio_control_get_channel_raw(5),
				radio_control_get_channel_raw(6),
				radio_control_get_channel_raw(7),
				radio_control_get_channel_raw(8),
				rc_to_255(1),
				rc_to_255(2),
				rc_to_255(3),
				rc_to_255(4),
				rc_to_255(5),
				rc_to_255(6),
				rc_to_255(7),
				rc_to_255(8),
				((radio_control_status() > 0) ? 255 : 0));
				// Should be global_data.rc_rssi in the future
		}
		else
		{
		mavlink_msg_rc_channels_send(MAVLINK_COMM_1,
				radio_control_get_channel_raw(1),
				radio_control_get_channel_raw(2),
				radio_control_get_channel_raw(3),
				radio_control_get_channel_raw(4),
				radio_control_get_channel_raw(5),
				radio_control_get_channel_raw(6),
				radio_control_get_channel_raw(7),
				radio_control_get_channel_raw(8),
				rc_to_255(1),
				rc_to_255(2),
				rc_to_255(3),
				rc_to_255(4),
				rc_to_255(5),
				rc_to_255(6),
				rc_to_255(7),
				rc_to_255(8),
				((radio_control_status() > 0) ? 255 : 0));
		}
	}
}


void communication_send_controller_feedback(void)
{
	// Send controller outputs
	if (global_data.param[PARAM_SEND_SLOT_CONTROLLER_OUTPUT] == 1)
	{
		//send attitude setpoints
		debug_vect("att_s_pos", global_data.attitude_setpoint_pos);
		debug_vect("att_s_rm", global_data.attitude_setpoint_remote);
		debug_vect("att_s_off", global_data.attitude_setpoint_offset);
		debug_vect("att_s", global_data.attitude_setpoint);
	}

	// Send position outputs
	if (global_data.param[PARAM_SEND_SLOT_DEBUG_5] == 1)
	{
		debug_vect("pos", global_data.position);
		debug_vect("pos_sp", global_data.position_setpoint);
	}

	//Send PID Controller integrals
	if(global_data.param[PARAM_SEND_SLOT_DEBUG_2] == 1)
	{

		float_vect3 pid_int1, pid_int2, pid_int3;

		pid_int1.x=roll_controller.integral;
		pid_int1.y=nick_controller.integral;
		pid_int1.z=yaw_speed_controller.integral;

		pid_int2.x=x_axis_controller.integral;
		pid_int2.y=y_axis_controller.integral;
		pid_int2.z=z_axis_controller.integral;

		pid_int3.x=yaw_pos_controller.integral;
		pid_int3.y=0;
		pid_int3.z=0;

		debug_vect("int_att", pid_int1);
		debug_vect("int_pos", pid_int2);
		debug_vect("int_yawp", pid_int3);
	}
}

void handle_controller_timeouts(uint64_t loop_start_time)
{
	// Send position setpoint
	mavlink_msg_local_position_setpoint_send(MAVLINK_COMM_0,
			global_data.position_setpoint.x,
			global_data.position_setpoint.y,
			global_data.position_setpoint.z,
			global_data.yaw_pos_setpoint);

	// Position lock timeout - reenable once position control is not any more vision data dependent
	// Check for vision data / gps data timeout
	if (global_data.pos_last_valid == 0 || loop_start_time
			- global_data.pos_last_valid
			> (uint32_t) global_data.param[PARAM_POSITION_TIMEOUT])
	{
		global_data.state.vision_ok = 0;
		global_data.state.position_fix = 0;
	}
	else
	{
		global_data.state.vision_ok = 1;
		global_data.state.position_fix = 1;
	}

	// NO GPS YET
	global_data.state.gps_ok = 0;

	// UPDATE CONTROLLER STATES for QGroundcontrol widgets

	// ATTITUDE
	global_data.state.attitude_control_enabled = 1;

	// XY position
	if (global_data.param[PARAM_MIX_POSITION_WEIGHT] > 0)
	{
		global_data.state.position_xy_control_enabled = 1;
	}
	else
	{
		global_data.state.position_xy_control_enabled = 0;
	}

	// Z position
	if (global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] > 0)
	{
		global_data.state.position_z_control_enabled = 1;
	}
	else
	{
		global_data.state.position_z_control_enabled = 0;
	}

	// YAW position
	if (global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] > 0)
	{
		global_data.state.position_yaw_control_enabled = 1;
	}
	else
	{
		global_data.state.position_yaw_control_enabled = 0;
	}

	// Output control state
	mavlink_msg_control_status_send(MAVLINK_COMM_0,
			(global_data.state.vision_ok > 0) ? 3 : global_data.state.position_fix, // Send 3D fix if vision is available
					(global_data.state.vision_ok > 0) ? 3 : 0, // Send 3D fix for vision always
							global_data.state.gps_ok,
							global_data.state.attitude_control_enabled,
							global_data.state.position_xy_control_enabled,
							global_data.state.position_z_control_enabled,
							global_data.state.position_yaw_control_enabled
	);
	mavlink_msg_control_status_send(MAVLINK_COMM_1,
			(global_data.state.vision_ok > 0) ? 3 : global_data.state.position_fix, // Send 3D fix if vision is available
					(global_data.state.vision_ok > 0) ? 3 : 0, // Send 3D fix for vision always
							global_data.state.gps_ok,
							global_data.state.attitude_control_enabled,
							global_data.state.position_xy_control_enabled,
							global_data.state.position_z_control_enabled,
							global_data.state.position_yaw_control_enabled
	);
}


void sync_state_parameters(void){
	//Here all parameters that need to be checked often should be converted to uint8
	global_data.state.gps_mode=global_data.param[PARAM_GPS_MODE];

	if (global_data.state.gps_mode == 20)
	{
		global_data.state.uart0mode = UART_MODE_BYTE_FORWARD;
		global_data.state.uart1mode = UART_MODE_BYTE_FORWARD;
	}
}
uint8_t handle_reset_request(void)
{
	static int reset_countdown;
	if ((global_data.param[PARAM_IMU_RESET] == 1))
	{
		if (global_data.state.status == MAV_STATE_STANDBY)
		{
			if (reset_countdown == 0)
			{
				debug_message_buffer("RESET command received");// Won't come thru...
				reset_countdown = 7;
			}
			else if (reset_countdown == 1)
			{
				watchdog_wait_reset();
			}
			reset_countdown--;
			debug_message_buffer_sprintf("RESET count down: %u",
					reset_countdown);
		}
		else
		{
			reset_countdown = 0;
			global_data.param[PARAM_IMU_RESET] = 0;
			debug_message_buffer("RESET command REFUSED (I'm flying!)");
		}
	}
	return 0;
}

void handle_eeprom_write_request(void)
{
	//GET UPDATES FROM PARAMS
	//testing eeprom params

	if (global_data.state.status == MAV_STATE_STANDBY)
	{
		switch ((uint8_t) global_data.param[PARAM_IMU_RESET])
		{
		case 2:
			debug_message_buffer("start writing params to eeprom");
			param_write_all();
			global_data.param[PARAM_IMU_RESET] = 0;
			break;
		case 3:
			param_read_all();
			debug_message_buffer("start reading params from eeprom");
			global_data.param[PARAM_IMU_RESET] = 0;
			break;
		case 4:
			global_data.param[PARAM_IMU_RESET] = 0;
			debug_message_buffer("params reset to defaults");
			global_data_reset_param_defaults();
			break;
		case 5:
			global_data.param[PARAM_IMU_RESET] = 0;
			if (param_size_check())
			{
				debug_message_buffer("check true");
			}
			else
			{
				debug_message_buffer("check false");
			}

			break;
		case 6:
			global_data.param[PARAM_IMU_RESET] = 0;
			eeprom_check_start();
			break;
		default:
			break;
		}
	}
}



void adc_read(void)
{
	//Read out distance to ground from infrared sensor
	global_data.ground_distance_unfiltered = infrared_distance_get()-0.10;//TODO: offset calculation in multitracker

	// Read out system status
	global_data.battery_voltage = battery_get_value();

	float ground_distance_filter_weight = 0.15;
	global_data.ground_distance = (1 - ground_distance_filter_weight)
			* global_data.ground_distance
			+ ground_distance_filter_weight * global_data.ground_distance_unfiltered;

	if (global_data.ground_distance < 1 && global_data.ground_distance
			> 0)
	{
		global_data.state.ground_distance_ok = 1;
	}

	else
	{
		global_data.state.ground_distance_ok = 0;
	}

	//GET NEWEST DATA
	//Switch to Grounddistance sensor for z if we dont have vision
//	if (global_data.state.vision_ok == 0)
//	{
//		if (global_data.state.ground_distance_ok == 1)
//		{
//			global_data.position.z = -global_data.ground_distance;
//		}
//		else
//		{
//			global_data.position.z = -1;
//		}
//		global_data.velocity.z = 0;
//	}
}

void update_system_statemachine(uint64_t loop_start_time)
{
	// Update state machine, enable and disable controllers
	switch (global_data.state.mav_mode)
	{
	case MAV_MODE_MANUAL:
		global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
		global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
		global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
		global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
		break;
	case MAV_MODE_AUTO:
		// Same as guided mode, NO BREAK
	case MAV_MODE_TEST1:
		// Same as guided mode, NO BREAK
	case MAV_MODE_GUIDED:
		if (global_data.state.position_fix)
		{
			if (global_data.state.status == MAV_STATE_CRITICAL)
			{
				//get active again if we are in critical
				//if we are in Emergency we don't want to start again!!!!
				global_data.state.status = MAV_STATE_ACTIVE;
			}
		}
		else
		{
			if (global_data.state.status == MAV_STATE_ACTIVE)
			{
				global_data.state.status = MAV_STATE_CRITICAL;
				global_data.entry_critical = loop_start_time;
				debug_message_buffer("CRITICAL! POSITION LOST");
			}
			else if (global_data.state.status == MAV_STATE_CRITICAL && (loop_start_time - global_data.entry_critical
					> 3 * (uint32_t) global_data.param[PARAM_POSITION_TIMEOUT]))
			{
				//wait 3 times as long as waited for critical

				global_data.state.status = MAV_STATE_EMERGENCY;
				//Initiate Landing even if we didn't reach 0.7m
				global_data.state.fly=FLY_LANDING;
				debug_message_buffer("EMERGENCY! MAYDAY! MAYDAY! LANDING!");
			}
		}
		switch (global_data.state.status)
		{
		case MAV_STATE_ACTIVE:
			global_data.param[PARAM_MIX_POSITION_WEIGHT] = 1;
			global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;

		case MAV_STATE_CRITICAL:
			//global_data.param[PARAM_MIX_POSITION_WEIGHT] = 1; // estimate path without vision
			global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0; // try to hover on place
			global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;

		case MAV_STATE_EMERGENCY:
			global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
			global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 1;
			global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 1;
			break;
		default:
			global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
			global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
			global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
		}
		global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
		global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
		break;
		case MAV_MODE_TEST2:
			//allow other mix params to be set by hand
			global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 1;
			global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 1;
			break;
		case MAV_MODE_TEST3:
			//				break;

		case MAV_MODE_LOCKED:
			//				break;

		case MAV_MODE_READY:
			//				break;
		case MAV_MODE_UNINIT:
			//				break;
		default:
			global_data.param[PARAM_MIX_POSITION_WEIGHT] = 0;
			global_data.param[PARAM_MIX_POSITION_YAW_WEIGHT] = 0;
			global_data.param[PARAM_MIX_POSITION_Z_WEIGHT] = 0;
			global_data.param[PARAM_MIX_OFFSET_WEIGHT] = 0;
			global_data.param[PARAM_MIX_REMOTE_WEIGHT] = 0;
	}
}

void send_system_state(void)
{
	// Send heartbeat to announce presence of this system
	// Send over both communication links
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);

	// Send system status over both links
	mavlink_msg_sys_status_send(MAVLINK_COMM_1, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);
	mavlink_msg_sys_status_send(MAVLINK_COMM_0, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);

	// Send auxiliary status over both links
	mavlink_msg_aux_status_send(MAVLINK_COMM_1, global_data.cpu_usage,
			global_data.i2c0_err_count, global_data.i2c1_err_count,
			global_data.spi_err_count, global_data.spi_err_count,
			global_data.packet_drops);
	mavlink_msg_aux_status_send(MAVLINK_COMM_0, global_data.cpu_usage,
			global_data.i2c0_err_count, global_data.i2c1_err_count,
			global_data.spi_err_count, global_data.spi_err_count,
			global_data.packet_drops);

	//			mavlink_msg_raw_aux_send(MAVLINK_COMM_0, 0, 0, 0, 0,
	//					battery_get_value(), global_data.temperature,
	//					global_data.pressure_raw);
}

void fuse_vision_altitude_200hz(void)
{
	// Filter states
	static float_vect3 pos_est3;
	static float_vect3 vel_est3;

	// Just Vision Kalman Filter
	x_position_kalman3(&global_data.vision_data, &pos_est3, &vel_est3);
	y_position_kalman3(&global_data.vision_data, &pos_est3, &vel_est3);
	z_position_kalman3(&global_data.vision_data, &pos_est3, &vel_est3);

	global_data.position.x = pos_est3.x;
	global_data.position.y = pos_est3.y;
	global_data.position.z = pos_est3.z;

	global_data.velocity.x = vel_est3.x;
	global_data.velocity.y = vel_est3.y;
	global_data.velocity.z = vel_est3.z;

	if (global_data.vision_data.new_data)
	{
		uint32_t vision_delay = (uint32_t) (global_data.vision_data.comp_end
				- global_data.vision_data.time_captured);
		// Debug Time for Vision Processing
		mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 100,
				(float) vision_delay);
	}

	global_data.vision_data.new_data = 0;


	//Switch to Grounddistance sensor for z if we don't have vision
//	if (global_data.state.vision_ok == 0)
//	{
//		if (global_data.state.ground_distance_ok == 1)
//		{
//			global_data.position.z = -global_data.ground_distance;
//		}
//		else
//		{
//			global_data.position.z = -1;
//		}
//		global_data.velocity.z = 0;
//	}
}



void camera_shutter_handling(uint64_t loop_start_time)
{
	bool camera_triggered = shutter_loop();
	if (camera_triggered)
	{
		uint64_t usec = sys_time_clock_get_unix_offset() + loop_start_time;
		vision_buffer_buffer_camera_triggered(usec, loop_start_time,
				shutter_get_seq());

		// Emit timestamp of this image
		mavlink_msg_image_triggered_send(MAVLINK_COMM_0, usec,
				shutter_get_seq(), global_data.attitude.x,
				global_data.attitude.y);
		mavlink_msg_image_triggered_send(MAVLINK_COMM_1, usec,
				shutter_get_seq(), global_data.attitude.x,
				global_data.attitude.y);
	}
}

/**
* The average load will be typically much less,
* however in a time critical system only the peak load is relevant.
* As long as the peak load stays below 100%, the system will never
* fail any timing constraints.
*
* @param loop_start_time time this loop started
* @param loop_stop_time time this loop stopped
* @param min_mainloop_interval the shortest interval between two consecutive mainloop calls
*
* @return the max. cpu load, where 0 = 0% and 1000 = 100%
*/
uint16_t measure_peak_cpu_load(uint64_t loop_start_time,
		uint64_t loop_stop_time, uint64_t min_mainloop)
{
	// Time Measurement
	// this timer measures the maximum time
	// one mainloop cycle takes.
	// It should be never bigger than 1000/max_mainloop_rate milliseconds
	// else the next mainloop call is delayed

	//	last_mainloop_time = loop_stop_time - loop_start_time;
	//	if (time_elapsed > (loop_max_time / 10) * 6)
	//	{
	//	loop_max_time = last_mainloop_time;
	//	}



//	uint32_t last_mainloop_time = loop_stop_time - loop_start_time;
//	if (last_mainloop_time > loop_max_time)
//	{
//		loop_max_time = last_mainloop_time;
//	}
//	return loop_max_time/(min_mainloop_time/1000);
	return 0;
}
uint16_t measure_avg_cpu_load(uint64_t loop_start_time,
		uint64_t loop_stop_time, uint64_t min_mainloop)
{
//	static uint32_t last_time = 0;
//	// loop_max_time is in microseconds
//	const uint32_t lp_constant = 50; // 1-99
//	uint16_t curr_load = (loop_stop_time - loop_start_time);
//	if (curr_load > 100)
//	{
//		 curr_load /= (min_mainloop_time/1000);
//		 last_time = (((last_time * lp_constant)) + (curr_load * (100 - lp_constant))) / 100;
//	}
//	return last_time;
	return 0;
}

void us_run_init(void)
{
	// Initialize counters for mainloop
	uint8_t counter_id;
	for (counter_id = 0; counter_id < NUM_OF_COUNTERS; counter_id++)
	{
		next_exec_time[counter_id] = 0;
	}
}

/**
* @brief Check for periodic counter timeout
*
* This function can be called to check whether the counter associated with counter_id
* has expired. If expired it the counter is set to the current time plus parameter ms
* and TRUE is returned.
*
* @param us the interval to run the function with
* @param counter_id the id of the counter to use - use one per interval
* @param current_time the current system time
*/
uint8_t us_run_every(uint32_t us, counter_id_t counter_id,
		uint32_t current_time)
{
	if (next_exec_time[counter_id] <= current_time
			|| next_exec_time[counter_id] - current_time > us)
	{
		next_exec_time[counter_id] = current_time + us;
		return 1;
	}
	else
	{
		return 0;
	}
}
