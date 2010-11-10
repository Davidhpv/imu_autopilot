/*
 * mainloop.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#include "conf.h"

#if PX_VEHICLE_TYPE == PX_AIRFRAME_FIXED_WING

#include "mainloop_fixed_wing.h"
#include "mainloop_generic.h"
#include "common_mainloop_functions.h"

#include "inttypes.h"
#include "mcu_init.h"

// Include comm
#include "comm.h"
#include "mavlink.h"

// Include globals
#include "global_data.h"
#include "led.h"
#include "sys_time.h"

#include "servos.h"
#include "radio_control.h"
#include "sensors.h"

#include "communication.h"

#include "control_fixed_wing_attitude.h"
#include "remote_control.h"
#include "position_kalman3.h"

#include "debug.h"
#include "transformation.h"
#include "eeprom.h"
#include "params.h"
#include "attitude_observer.h"

// Static variables - scope is limited to this file
static const uint32_t min_mainloop_time = 5000;  ///< The minimum wait interval between two mainloop software timer calls, = 1/max rate, initialized to 1 sec = 1000000 microseconds
//static uint32_t loop_max_time = 0;               ///< The maximum time in microseconds one mainloop took
static uint64_t last_mainloop_idle = 0;				///< Starvation Prevention

/**
* @brief Initialize the whole system
*
* All functions that need to be called before the first mainloop iteration
* should be placed here.
*/
void main_init_fixed_wing(void)
{
	main_init_generic();
}

/**
* @brief This is the main loop
*
* It will be executed at maximum MCU speed (60 Mhz)
*/
void main_loop_fixed_wing(void)
{
	last_mainloop_idle = sys_time_clock_get_time_usec();
	debug_message_buffer("Starting main loop");
	while (1)
	{
		// Time Measurement
		uint64_t loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////


		if (global_data.mode == MAV_MODE_RC_TRAINING)
		{
			///////////////////////////////////////////////////////////////////////////
			/// RC INTERFACE HACK AT 50 Hz
			///////////////////////////////////////////////////////////////////////////
			if (us_run_every(20000, COUNTER8, loop_start_time))
			{
				// Write start byte
				uart1_transmit(0xFF);

				// Write channels 1-6
				for (int i = 1; i < 7; i++)
				{
					uart1_transmit((radio_control_get_channel(1)+1)*127);
				}
			}
			led_toggle(LED_GREEN);
			led_toggle(LED_RED);
			// Do not execute any of the functions below
			continue;
		}

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL 200 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(5000, COUNTER2, loop_start_time))
		{
			// Kalman Attitude filter, used on all systems
			gyro_read();
			sensors_read_acc();

			// Read out magnetometer at its default 50 Hz rate
			static uint8_t mag_count = 0;
			if (mag_count == 3)
			{
				sensors_read_mag();
				attitude_observer_correct_magnet(global_data.magnet_corrected);
				mag_count = 0;
			}
			else
			{
				mag_count++;
			}

			// Correction step of observer filter
			attitude_observer_correct_accel(global_data.accel_raw);

			// Write in roll and pitch
			static float_vect3 att; //if not static we have spikes in roll and pitch on bravo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			attitude_observer_get_angles(&att);
			global_data.attitude.x = att.x;
			global_data.attitude.y = att.y;
			if (global_data.param[PARAM_ATT_KAL_IYAW])
			{
				global_data.attitude.z += 0.005 * global_data.gyros_si.z;
			}
			else
			{
				global_data.attitude.z = att.z;
			}
			// Prediction step of observer
			attitude_observer_predict(global_data.gyros_si);

			control_fixed_wing_attitude();

			// ====================================================================

		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 50 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(20000, COUNTER3, loop_start_time))
		{
			// Read Analog-to-Digital converter
			adc_read();
			// Read remote control
			remote_control();
		}
		///////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(10000, COUNTER6, loop_start_time))
		{
			// Send the raw sensor/ADC values
			communication_send_raw_data(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////
		/// UNCRITICAL SLOW 5 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(200000, COUNTER8, loop_start_time))
		{
			// The onboard controllers go into failsafe mode once
			// position data is missing
			handle_controller_timeouts(loop_start_time);
			// Send buffered data such as debug text messages
			communication_queued_send();
			// Empty one message out of the buffer
			debug_message_send_one();

			// Toggle status led
			//led_toggle(LED_YELLOW);
			led_toggle(LED_RED); // just for green LED on alpha at the moment

			// Toggle active mode led
			if (global_data.mode == MAV_MODE_MANUAL || global_data.mode
					== MAV_MODE_GUIDED || global_data.mode == MAV_MODE_AUTO)
			{
				led_on(LED_GREEN);
			}
			else
			{
				led_off(LED_GREEN);
			}

			handle_eeprom_write_request();
			handle_reset_request();

			communication_send_controller_feedback();

			communication_send_remote_control();

			// Pressure sensor driver works, but not tested regarding stability
			sensors_pressure_bmp085_read_out();

			if (global_data.param[PARAM_POSITION_YAW_TRACKING] == 1)
			{
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
						90, global_data.param[PARAM_POSITION_SETPOINT_YAW]);
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
						91, global_data.yaw_pos_setpoint);
			}

		}
		///////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 1 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(1000000, COUNTER9, loop_start_time))
		{
			// Send system state, mode, battery voltage, etc.
			send_system_state();

			if (global_data.param[PARAM_GPS_MODE] >= 10)
			{
				//Send GPS information
				float_vect3 gps;
				gps.x = gps_utm_north / 100.0f;//m
				gps.y = gps_utm_east / 100.0f;//m
				gps.z = gps_utm_zone;// gps_week;
				debug_vect("GPS", gps);
			}

		}
		///////////////////////////////////////////////////////////////////////////




		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER7, loop_start_time))
		{
			led_toggle(LED_YELLOW);

			if (global_data.param[PARAM_GPS_MODE] >= 10)
			{
				//get thru all gps messages
				debug_message_send_one();
			}

			communication_send_attitude_position(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////




		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 200 Hz functions                                     //
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(5000, COUNTER5, loop_start_time))
		{
			if (global_data.status == MAV_STATE_STANDBY)
			{
				//Check if parameters should be written or read
				param_handler();
			}
		}
		///////////////////////////////////////////////////////////////////////////

		else {
			// All Tasks are fine and we have no starvation
			last_mainloop_idle = loop_start_time;
		}

		// Read out comm at max rate - takes only a few microseconds in worst case
		communication_receive();

		// MCU load measurement
		uint64_t loop_stop_time = sys_time_clock_get_time_usec();
		global_data.cpu_usage = measure_avg_cpu_load(loop_start_time, loop_stop_time, min_mainloop_time);
		global_data.cpu_peak = measure_peak_cpu_load(loop_start_time, loop_stop_time, min_mainloop_time);
		if (loop_start_time - last_mainloop_idle >= 5000)
		{
			debug_message_buffer(
					"CRITICAL WARNING! CPU LOAD TO HIGH. STARVATION!");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		}
		if (global_data.cpu_usage > 800)
		{
			// CPU load higher than 80%
			debug_message_buffer("CRITICAL WARNING! CPU LOAD HIGHER THAN 80%");
		}
	} // End while(1)

}

#endif
