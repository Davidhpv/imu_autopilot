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
* @brief Communication reception on both UARTS
*   @author Lorenz Meier
*   @author Laurens MacKay
*/

#include "arm7/led.h"

#include "communication.h"
#include "control.h"
#include "ppm.h"
#include "conf.h"
#include "shutter.h"
#include "comm.h"
#include "sys_time.h"
#include "gps.h"

#include "float_checks.h"
#include "debug.h"
#include "calibration.h"
#include "sys_state.h"

#include "vision_buffer.h"

#include "control_quadrotor_position.h"
#include "params.h"

static uint32_t m_parameter_i = 0;

void execute_action(uint8_t action)
{
	switch (action)
	{
	case MAV_ACTION_LAUNCH:
		if (global_data.state.mav_mode > (uint8_t)MAV_MODE_LOCKED)
		{
			global_data.state.status = (uint8_t)MAV_STATE_ACTIVE;
		}
		break;
	case MAV_ACTION_MOTORS_START:
		if (global_data.state.mav_mode > (uint8_t)MAV_MODE_LOCKED)
		{
			global_data.state.status = (uint8_t)MAV_STATE_ACTIVE;
		}
		break;
	case MAV_ACTION_MOTORS_STOP:
		global_data.state.status = (uint8_t)MAV_STATE_STANDBY;
		break;
	case MAV_ACTION_EMCY_KILL:
		global_data.state.status = (uint8_t)MAV_STATE_EMERGENCY;
		break;
	case MAV_ACTION_STORAGE_READ:
		param_read_all();
		debug_message_buffer("Started reading params from eeprom");
	break;
	case MAV_ACTION_STORAGE_WRITE:
		debug_message_buffer("Started writing params to eeprom");
		param_write_all();
	break;
	case MAV_ACTION_CALIBRATE_GYRO:
		start_gyro_calibration();
		break;
	case MAV_ACTION_CALIBRATE_RC:
		start_rc_calibration();
		break;
	case MAV_ACTION_CALIBRATE_MAG:
		start_mag_calibration();
		break;
	case MAV_ACTION_CALIBRATE_PRESSURE:
		start_pressure_calibration();
		break;
	default:
		// Should never be reached, ignore unknown commands
		debug_message_buffer("WARNING: Rejected unknown action");
		break;
	}
}

/** @addtogroup COMM */
//@{
/** @name Communication functions
*  abstraction layer for comm */
//@{
void handle_mavlink_message(mavlink_channel_t chan,
		mavlink_message_t* msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t len;
	switch (chan)
	{
	case MAVLINK_COMM_0:
	{
		if (msg->msgid != MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE)
		{
		// Copy to COMM 1
		len = mavlink_msg_to_send_buffer(buf, msg);
		for (int i = 0; i < len; i++)
			{
			uart1_transmit(buf[i]);
			}
		}
	}
		break;
	case MAVLINK_COMM_1:
	{
		if (msg->msgid != MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE)
		{
		// Copy to COMM 0
		len = mavlink_msg_to_send_buffer(buf, msg);
		for (int i = 0; i < len; i++)
			{
			uart0_transmit(buf[i]);
			}
		break;
		}
	}
	default:
		break;
	}


	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_SET_MODE:
	{
		mavlink_set_mode_t mode;
		mavlink_msg_set_mode_decode(msg, &mode);
		// Check if this system should change the mode
		if (mode.target == (uint8_t)global_data.param[PARAM_SYSTEM_ID])
		{
			sys_set_mode(mode.mode);

			// Emit current mode
			mavlink_msg_sys_status_send(MAVLINK_COMM_1, global_data.state.mav_mode, 0,
					global_data.state.status,
					global_data.cpu_usage, global_data.battery_voltage,
					global_data.motor_block, global_data.packet_drops);
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, global_data.state.mav_mode, 0,
					global_data.state.status,
					global_data.cpu_usage, global_data.battery_voltage,
					global_data.motor_block, global_data.packet_drops);

		}
	}
	break;
	case MAVLINK_MSG_ID_ACTION:
	{
		execute_action(mavlink_msg_action_get_action(msg));

		//Forwart actions from Xbee to Onboard Computer and vice versa
		if (chan == MAVLINK_COMM_1)
		{
			mavlink_send_uart(MAVLINK_COMM_0, msg);
		}
		else if (chan == MAVLINK_COMM_0)
		{
			mavlink_send_uart(MAVLINK_COMM_1, msg);
		}
	}
		break;
	case MAVLINK_MSG_ID_SYSTEM_TIME:
	{
		if (!sys_time_clock_get_unix_offset())
		{
			int64_t offset = ((int64_t) mavlink_msg_system_time_get_time_usec(
					msg)) - (int64_t) sys_time_clock_get_time_usec();
			sys_time_clock_set_unix_offset(offset);

			debug_message_buffer("UNIX offset updated");
		}
		else
		{

			//			debug_message_buffer("UNIX offset REFUSED");
		}
	}
	break;
	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
	{
		mavlink_request_data_stream_t stream;
		mavlink_msg_request_data_stream_decode(msg, &stream);
		switch (stream.req_stream_id)
		{
		case 0: // UNIMPLEMENTED
			break;
		case 1: // RAW SENSOR DATA
			global_data.param[PARAM_SEND_SLOT_RAW_IMU] = stream.start_stop;
			break;
		case 2: // EXTENDED SYSTEM STATUS
			global_data.param[PARAM_SEND_SLOT_ATTITUDE] = stream.start_stop;
			break;
		case 3: // REMOTE CONTROL CHANNELS
			global_data.param[PARAM_SEND_SLOT_REMOTE_CONTROL] = stream.start_stop;
			break;
		case 4: // RAW CONTROLLER
			//global_data.param[PARAM_SEND_SLOT_DEBUG_5] = stream.start_stop;
			//global_data.param[PARAM_SEND_SLOT_DEBUG_3] = stream.start_stop;
			global_data.param[PARAM_SEND_SLOT_CONTROLLER_OUTPUT] = stream.start_stop;
			break;
		case 5: // SENSOR FUSION

			//LOST IN GROUDNCONTROL
			//			global_data.param[PARAM_SEND_SLOT_DEBUG_5] = stream.start_stop;
			break;
		case 6: // POSITION
			global_data.param[PARAM_SEND_SLOT_DEBUG_5] = stream.start_stop;
			break;
		case 7: // EXTRA1
			global_data.param[PARAM_SEND_SLOT_DEBUG_2] = stream.start_stop;
			break;
		case 8: // EXTRA2
			global_data.param[PARAM_SEND_SLOT_DEBUG_4] = stream.start_stop;
			break;
		case 9: // EXTRA3
			global_data.param[PARAM_SEND_SLOT_DEBUG_6] = stream.start_stop;
			break;
		default:
			// Do nothing
			break;
		}
	}
	break;
	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	{
		mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			// Check if this message is for this system
			if ((uint8_t) set.target_system
					== (uint8_t) global_data.param[PARAM_SYSTEM_ID]
					                               && (uint8_t) set.target_component
					                               == (uint8_t) global_data.param[PARAM_COMPONENT_ID])
			{
				char* key = (char*) set.param_id;

				if (set.param_id[0] == '\0')
				{
					// Choose parameter based on index
					if (set.param_index < ONBOARD_PARAM_COUNT)
					{
						// Report back value
						mavlink_msg_param_value_send(chan,
								(int8_t*) global_data.param_name[set.param_index],
								global_data.param[set.param_index], ONBOARD_PARAM_COUNT, set.param_index);
					}
				}
				else
				{
					for (int i = 0; i < ONBOARD_PARAM_COUNT; i++)
					{
						bool match = true;
						for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
						{
							// Compare
							if (((char) (global_data.param_name[i][j]))
									!= (char) (key[j]))
							{
								match = false;
							}

							// End matching if null termination is reached
							if (((char) global_data.param_name[i][j]) == '\0')
							{
								break;
							}
						}

						// Check if matched
						if (match)
						{
								// Report back value
								mavlink_msg_param_value_send(chan,
										(int8_t*) global_data.param_name[i],
										global_data.param[i], ONBOARD_PARAM_COUNT, m_parameter_i);
						}
					}
				}
			}
	}
		break;
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	{
		// Start sending parameters
		m_parameter_i = 0;
	}
	break;
	case MAVLINK_MSG_ID_PARAM_SET:
	{
		mavlink_param_set_t set;
		mavlink_msg_param_set_decode(msg, &set);

		// Check if this message is for this system
		if ((uint8_t) set.target_system
				== (uint8_t) global_data.param[PARAM_SYSTEM_ID]
				                               && (uint8_t) set.target_component
				                               == (uint8_t) global_data.param[PARAM_COMPONENT_ID])
		{
			char* key = (char*) set.param_id;

			for (int i = 0; i < ONBOARD_PARAM_COUNT; i++)
			{
				bool match = true;
				for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
				{
					// Compare
					if (((char) (global_data.param_name[i][j]))
							!= (char) (key[j]))
					{
						match = false;
					}

					// End matching if null termination is reached
					if (((char) global_data.param_name[i][j]) == '\0')
					{
						break;
					}
				}

				// Check if matched
				if (match)
				{
					// Only write and emit changes if there is actually a difference
					// AND only write if new value is NOT "not-a-number"
					// AND is NOT infy
					if (global_data.param[i] != set.param_value
							&& !isnan(set.param_value)
							&& !isinf(set.param_value))
					{
						global_data.param[i] = set.param_value;
						// Report back new value
						mavlink_msg_param_value_send(MAVLINK_COMM_0,
								(int8_t*) global_data.param_name[i],
								global_data.param[i], ONBOARD_PARAM_COUNT, m_parameter_i);
						mavlink_msg_param_value_send(MAVLINK_COMM_1,
								(int8_t*) global_data.param_name[i],
								global_data.param[i], ONBOARD_PARAM_COUNT, m_parameter_i);

						debug_message_buffer_sprintf("Parameter received param id=%i",i);
					}
				}
			}
		}
	}
	break;
	case MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET:
	{
		mavlink_position_control_setpoint_set_t pos;
		mavlink_msg_position_control_setpoint_set_decode(msg, &pos);
		if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] == 1)
		{
			//			global_data.position_setpoint.x = pos.x;
			//			global_data.position_setpoint.y = pos.y;
			//			global_data.position_setpoint.z = pos.z;
			debug_message_buffer("Position setpoint updated. OLD?\n");
		}
		else
		{
			debug_message_buffer(
					"Position setpoint recieved but not updated. OLD?\n");
		}

		// Send back a message confirming the new position
		mavlink_msg_position_control_setpoint_send(MAVLINK_COMM_0, pos.id,
				pos.x, pos.y, pos.z, pos.yaw);
		mavlink_msg_position_control_setpoint_send(MAVLINK_COMM_1, pos.id,
				pos.x, pos.y, pos.z, pos.yaw);
	}
	break;
	case MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET:
	{
		mavlink_position_control_offset_set_t set;
		mavlink_msg_position_control_offset_set_decode(msg, &set);
		//global_data.attitude_setpoint_pos_body_offset.z = set.yaw;

		//Ball Tracking
		if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] == 1 && global_data.param[PARAM_POSITION_YAW_TRACKING]==1)
		{
			global_data.param[PARAM_POSITION_SETPOINT_YAW]
					= global_data.attitude.z + set.yaw;

			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 92, set.yaw);
		}
	}
		break;
	case MAVLINK_MSG_ID_SET_CAM_SHUTTER:
	{
		// Decode the desired shutter
		mavlink_set_cam_shutter_t cam;
		mavlink_msg_set_cam_shutter_decode(msg, &cam);
		shutter_set(cam.interval, cam.exposure);
		debug_message_buffer_sprintf("set_cam_shutter. interval %i",
				cam.interval);

	}
	break;
	case MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL:
	{
		uint8_t enable = mavlink_msg_image_trigger_control_get_enable(msg);
		shutter_control(enable);
		if (enable)
			{
			debug_message_buffer("CAM: Enabling hardware trigger");
			}
		else
		{
			debug_message_buffer("CAM: Disabling hardware trigger");
		}
	}
	break;
	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
	{
		mavlink_vision_position_estimate_t pos;
		mavlink_msg_vision_position_estimate_decode(msg, &pos);

		vision_buffer_handle_data(&pos);


	}
	break;
	case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		{
			mavlink_vicon_position_estimate_t pos;
			mavlink_msg_vicon_position_estimate_decode(msg, &pos);

			//Set data from Vicon directly
			global_data.vision_data.ang.x = pos.roll;
			global_data.vision_data.ang.y = pos.pitch;
			global_data.vision_data.ang.z = pos.yaw;

			global_data.vision_data.pos.x = pos.x;
			global_data.vision_data.pos.y = pos.y;
			global_data.vision_data.pos.z = pos.z;

			//Correct YAW
			global_data.attitude.z = pos.yaw;
			//If yaw goes to infy (no idea why) set it to setpoint, next time will be better
			if (global_data.attitude.z > 20 || global_data.attitude.z < -20)
			{
				global_data.attitude.z = global_data.yaw_pos_setpoint;
				debug_message_buffer("vicon CRITICAL FAULT yaw was bigger than 20! prevented crash");
			}

			global_data.vision_data.new_data = 1;

			// Update validity time
			global_data.pos_last_valid = sys_time_clock_get_time_usec();

		}
	break;
	case MAVLINK_MSG_ID_PING:
	{
		mavlink_ping_t ping;
		mavlink_msg_ping_decode(msg, &ping);
		if (ping.target_system == 0 && ping.target_component == 0)
		{
			// Respond to ping
			uint64_t r_timestamp = sys_time_clock_get_unix_time();
			mavlink_msg_ping_send(chan, ping.seq, msg->sysid, msg->compid, r_timestamp);
		}
	}
		break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET:
	{
		mavlink_local_position_setpoint_set_t sp;
		mavlink_msg_local_position_setpoint_set_decode(msg, &sp);
		if (sp.target_system == global_data.param[PARAM_SYSTEM_ID])
		{
			if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT] == 1)
			{
				if (sp.x >= global_data.position_setpoint_min.x && sp.y
						>= global_data.position_setpoint_min.y && sp.z
						>= global_data.position_setpoint_min.z && sp.x
						<= global_data.position_setpoint_max.x && sp.y
						<= global_data.position_setpoint_max.y && sp.z
						<= global_data.position_setpoint_max.z)
				{
					debug_message_buffer("Setpoint accepted and set.");
					global_data.param[PARAM_POSITION_SETPOINT_X] = sp.x;
					global_data.param[PARAM_POSITION_SETPOINT_Y] = sp.y;
					global_data.param[PARAM_POSITION_SETPOINT_Z] = sp.z;

					if (global_data.param[PARAM_POSITION_YAW_TRACKING] == 0)
					{
						// Only update yaw if we are not tracking ball.
						global_data.param[PARAM_POSITION_SETPOINT_YAW] = sp.yaw;
					}

					//check if we want to start or land
					if (global_data.state.status == MAV_STATE_ACTIVE
							|| global_data.state.status == MAV_STATE_CRITICAL)
					{
						if (sp.z > -0.1)
						{
							if (!(global_data.state.fly == FLY_GROUNDED
									|| global_data.state.fly == FLY_SINKING
									|| global_data.state.fly
											== FLY_WAIT_LANDING
									|| global_data.state.fly == FLY_LANDING
									|| global_data.state.fly == FLY_RAMP_DOWN))
							{
								//if setpoint is lower that ground iate landing
								global_data.state.fly = FLY_SINKING;
								global_data.param[PARAM_POSITION_SETPOINT_Z]
										= -0.7;//with lowpass
								debug_message_buffer(
										"Sinking for LANDING. (z-sp lower than 10cm)");
							}
							else if(!(global_data.state.fly == FLY_GROUNDED))
							{
								global_data.param[PARAM_POSITION_SETPOINT_Z]
										= -0.7;//with lowpass
							}
						}
						if (global_data.state.fly == FLY_GROUNDED && sp.z
								< -0.69)
						{
							//start if we were grounded and get a sp over 0.5m
							global_data.state.fly = FLY_WAIT_MOTORS;
							debug_message_buffer(
									"STARTING wait motors. (z-sp higher than 50cm)");
							//set point changed with lowpass, after 5s it will be ok.
						}
					}

					//SINK TO 0.7m if we are critical or emergency
					if (global_data.state.status == MAV_STATE_EMERGENCY
							|| global_data.state.status == MAV_STATE_CRITICAL)
					{
						global_data.param[PARAM_POSITION_SETPOINT_Z] = -0.7;//with lowpass
					}
				}
				else
				{
					debug_message_buffer("Setpoint refused. Out of range.");
				}
			}
			else
			{
				debug_message_buffer("Setpoint refused. Param setpoint accept=0.");
			}
		}
	}
		break;
	default:
		break;
	}
}

/**
* @brief Send low-priority messages at a maximum rate of xx Hertz
*
* This function sends messages at a lower rate to not exceed the wireless
* bandwidth. It sends one message each time it is called until the buffer is empty.
* Call this function with xx Hertz to increase/decrease the bandwidth.
*/
void communication_queued_send(void)
{
	//send parameters one by one
	//	for (int m_parameter_i = 0; m_parameter_i < ONBOARD_PARAM_COUNT; i++)
	if (m_parameter_i < ONBOARD_PARAM_COUNT)
	{
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
				(int8_t*) global_data.param_name[m_parameter_i],
				global_data.param[m_parameter_i], ONBOARD_PARAM_COUNT, m_parameter_i);
		mavlink_msg_param_value_send(MAVLINK_COMM_1,
				(int8_t*) global_data.param_name[m_parameter_i],
				global_data.param[m_parameter_i], ONBOARD_PARAM_COUNT, m_parameter_i);
		m_parameter_i++;
	}
}



void communication_init(void)
{
	if (global_data.param[PARAM_GPS_MODE] > 0)
	{
		global_data.state.uart1mode = UART_MODE_GPS;
	}
	else
	{
		global_data.state.uart1mode = UART_MODE_MAVLINK;
	}
}

/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void communication_receive(void)
{
	mavlink_message_t msg;
	mavlink_status_t status =
	{ 0 };
	status.packet_rx_drop_count = 0;

	// COMMUNICATION WITH ONBOARD COMPUTER

	while (uart0_char_available())
	{
		uint8_t c = uart0_get_char();


		if (global_data.state.uart0mode == UART_MODE_MAVLINK)
		{
			// Try to get a new message
			if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
			{
				// Handle message
				handle_mavlink_message(MAVLINK_COMM_0, &msg);
			}
		}
		else if (global_data.state.uart0mode == UART_MODE_BYTE_FORWARD)
		{
			uart1_transmit(c);
		}
		// And get the next one
	}

	// Update global packet drops counter
	global_data.packet_drops += status.packet_rx_drop_count;
	status.packet_rx_drop_count = 0;

	// COMMUNICATION WITH EXTERNAL COMPUTER

	while (uart1_char_available())
	{
		uint8_t c = uart1_get_char();

		// Check if this link is used for MAVLink or GPS
		if (global_data.state.uart1mode == UART_MODE_MAVLINK)
		{
			//uart0_transmit((unsigned char)c);
			// Try to get a new message
			if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status))
			{
				// Handle message
				handle_mavlink_message(MAVLINK_COMM_1, &msg);
			}
		}
		else if (global_data.state.uart1mode == UART_MODE_GPS)
		{
			if (global_data.state.gps_mode == 10)
			{
				static uint8_t gps_i = 0;
				static char gps_chars[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
				if (c == '$' || gps_i == MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN
						- 1)
				{
					gps_i = 0;
					char gps_chars_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					strncpy(gps_chars_buf, gps_chars,
							MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
					debug_message_buffer(gps_chars_buf);

				}
				gps_chars[gps_i++] = c;
			}
			if (gps_parse(c))
			{
				// New GPS data received
				//debug_message_buffer("RECEIVED NEW GPS DATA");
				parse_gps_msg();
				mavlink_msg_gps_raw_send(
						global_data.param[PARAM_SEND_DEBUGCHAN],
						sys_time_clock_get_unix_time(), gps_mode, gps_lat
								/ 1e7f, gps_lon / 1e7f, gps_alt / 100.0f, 0.0f,
						0.0f, gps_gspeed / 100.0f, gps_course / 10.0f);

				//				// Output satellite info
				//				for (int i = 0; i < gps_nb_channels; i++)
				//				{
				//					mavlink_msg_gps_status_send(global_data.param[PARAM_SEND_DEBUGCHAN], gps_numSV, gps_svinfos[i].svid, gps_satellite_used(gps_svinfos[i].qi), gps_svinfos[i].elev, ((gps_svinfos[i].azim/360.0f)*255.0f), gps_svinfos[i].cno);
				//				}
			}

		}
		else if (global_data.state.uart1mode == UART_MODE_BYTE_FORWARD)
		{
			uart0_transmit(c);
			led_toggle(LED_YELLOW);
		}
		// And get the next one
	}

	// Update global packet drops counter
	global_data.packet_drops += status.packet_rx_drop_count;
}

