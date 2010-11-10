/*
 * mainloop.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#include "conf.h"

#include "mainloop_generic.h"
#include "common_mainloop_functions.h"

#include "inttypes.h"
#include "mcu_init.h"

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
#include "communication.h"

#include "simple_altitude_moving_average.h"
#include "attitude_compl_euler.h"
#include "altitude_kalman.h"
#include "lookup_sin_cos.h"
#include "least_square.h"

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

// Static variables
// these variables are used during the whole
// code runtime in the mainloop
static const uint32_t min_mainloop_time = 5000;  ///< The minimum wait interval between two mainloop software timer calls, = 1/max rate, initialized to 1 sec = 1000000 microseconds
//static uint32_t loop_max_time = 0;               ///< The maximum time in microseconds one mainloop took
static uint64_t last_mainloop_idle = 0;				///< Starvation Prevention

/**
* @brief Initialize the whole system
*
* All functions that need to be called before the first mainloop iteration
* should be placed here.
*/
void main_init_generic(void)
{
	// Reset to safe values
	global_data_reset();

	// Load default eeprom parameters as fallback
	global_data_reset_param_defaults();

	// LOWLEVEL INIT, ONLY VERY BASIC SYSTEM FUNCTIONS
	hw_init();
	enableIRQ();
	led_init();
	led_on(LED_GREEN);
	sys_time_init();
	sys_time_periodic_init();
	sys_time_clock_init();
	ppm_init();
	pwm_init();

	// Lowlevel periphel support init
	adc_init();
	// FIXME SDCARD
//	MMC_IO_Init();
	spi_init();
	i2c_init();

	// Sensor init
	sensors_init();
	debug_message_buffer("Sensor initialized");

	// Shutter init
	shutter_init();
	shutter_control(0);

	// Debug output init
	debug_message_init();
	debug_message_buffer("Text message buffer initialized");

	// MEDIUM LEVEL INIT, INITIALIZE I2C, EEPROM, WAIT FOR MOTOR CONTROLLERS
	// Try to reach the EEPROM
	eeprom_check_start();

	// WAIT FOR 2 SECONDS FOR THE USER TO NOT TOUCH THE UNIT
	while (sys_time_clock_get_time_usec() < 2000000)
	{
	}

	// Do the auto-gyro calibration for 1 second
	// Get current temperature
	led_on(LED_RED);
	gyro_init();

//	uint8_t timeout = 3;
//	// Check for SD card
//	while (sys_time_clock_get_time_usec() < 2000000)
//	{
//		while (GetDriveInformation() != F_OK && timeout--)
//		  {
//		   debug_message_buffer("MMC/SD-Card not found ! retrying..");
//		  }
//	}
//
//	if (GetDriveInformation() == F_OK)
//	{
//		debug_message_buffer("MMC/SD-Card SUCCESS: FOUND");
//	}
//	else
//	{
//		debug_message_buffer("MMC/SD-Card FAILURE: NOT FOUND");
//	}
	//FIXME redo init because of SD driver decreasing speed
	//spi_init();
	led_off(LED_RED);

	// Stop trying to reach the EEPROM - if it has not been found by now, assume
	// there is no EEPROM mounted
	if (eeprom_check_ok())
	{
		param_read_all();
		debug_message_buffer("EEPROM detected - reading parameters from EEPROM");

		for (int i = 0; i < ONBOARD_PARAM_COUNT * 2 + 20; i++)
		{
			param_handler();
			//sleep 1 ms
			sys_time_wait(1000);
		}
	}
	else
	{
		debug_message_buffer("NO EEPROM - reading onboard parameters from FLASH");
	}

	//Magnet sensor
	hmc5843_init();
	acc_init();

	// Comm parameter init
	mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID]; // System ID, 1-255
	mavlink_system.compid = global_data.param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255

	// Comm init has to be
	// AFTER PARAM INIT
	comm_init(MAVLINK_COMM_0);
	comm_init(MAVLINK_COMM_1);

	// UART initialized, now initialize COMM peripherals
	communication_init();
	gps_init();

	us_run_init();

	servos_init();

	//position_kalman3_init();
	//	buzzer_init();

	// Calibration starts (this can take a few seconds)
	//	led_on(LED_GREEN);
	//	led_on(LED_RED);

	// Read out first time battery
	global_data.battery_voltage = battery_get_value();

	global_data.state.mav_mode = MAV_MODE_LOCKED;
	global_data.state.status = MAV_STATE_CALIBRATING;

	// Send first message heartbeat
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);
	// Send first global system status
	mavlink_msg_sys_status_send(MAVLINK_COMM_0, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);
	mavlink_msg_sys_status_send(MAVLINK_COMM_1, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);


	float_vect3 init_state_accel;
	init_state_accel.x = 0.0f;
	init_state_accel.y = 0.0f;
	init_state_accel.z = -1000.0f;
	float_vect3 init_state_magnet;
	init_state_magnet.x = 1.0f;
	init_state_magnet.y = 0.0f;
	init_state_magnet.z = 0.0f;


	//auto_calibration();


	attitude_observer_init(init_state_accel, init_state_magnet);

	debug_message_buffer("Attitude Filter initialized");
	led_on(LED_RED);

	// Send second message heartbeat
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
			global_data.param[PARAM_SYSTEM_TYPE], MAV_AUTOPILOT_PIXHAWK);
	// Send second global system status
	mavlink_msg_sys_status_send(MAVLINK_COMM_0, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);
	mavlink_msg_sys_status_send(MAVLINK_COMM_1, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);

	// Lowlevel services are now initialized
	// Send boot message
	mavlink_msg_boot_send(MAVLINK_COMM_0, global_data.param[PARAM_SW_VERSION]);
	//led_toggle(LED_RED);
	mavlink_msg_boot_send(MAVLINK_COMM_1, global_data.param[PARAM_SW_VERSION]);

	debug_message_buffer("System is initialized");

	// Calibration stopped
	led_off(LED_RED);

	global_data.state.mav_mode = MAV_MODE_READY;
	global_data.state.status = MAV_STATE_STANDBY;

	mavlink_msg_sys_status_send(MAVLINK_COMM_0, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);
	mavlink_msg_sys_status_send(MAVLINK_COMM_1, global_data.state.mav_mode, 0,
			global_data.state.status, global_data.battery_voltage,
			global_data.motor_block, global_data.packet_drops);

	debug_message_buffer("Checking if remote control is switched on:");
	if (radio_control_status() == RADIO_CONTROL_ON)
	{
		global_data.state.mav_mode = MAV_MODE_MANUAL;
		debug_message_buffer("RESULT: remote control switched ON");
		led_on(LED_GREEN);
	}
	else
	{
		global_data.state.mav_mode = MAV_MODE_LOCKED;
		debug_message_buffer("RESULT: remote control switched OFF");
		led_off(LED_GREEN);
	}
}


/**
* @brief This is the main loop
*
* It will be executed at maximum MCU speed (60 Mhz)
*/
void main_loop_generic(void)
{
	last_mainloop_idle = sys_time_clock_get_time_usec();
	debug_message_buffer("Starting main loop");
	while (1)
	{
		// Time Measurement
		uint64_t loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////


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
			attitude_observer_get_angles(&global_data.attitude);

			// Prediction step of observer
			attitude_observer_predict(global_data.gyros_si);


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
		/// CRITICAL FAST 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER4, loop_start_time))
		{
			update_system_statemachine(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////




		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(10000, COUNTER6, loop_start_time))
		{
			// Send the raw sensor/ADC values if enabled
			communication_send_raw_data(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// UNCRITICAL SLOW 5 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(200000, COUNTER8, loop_start_time))
		{
			// Send buffered data such as debug value messages
			communication_queued_send();
			// Empty one message out of the buffer
			debug_message_send_one();

			// Toggle status led
			//led_toggle(LED_YELLOW);
			led_toggle(LED_RED);

			// Toggle active mode led
			if (global_data.state.mav_mode == MAV_MODE_MANUAL || global_data.state.mav_mode
					== MAV_MODE_GUIDED || global_data.state.mav_mode == MAV_MODE_AUTO)
			{
				led_on(LED_GREEN);
			}
			else
			{
				led_off(LED_GREEN);
			}

			handle_eeprom_write_request();
			handle_reset_request();

			communication_send_remote_control();

			// Pressure sensor driver works, but not tested regarding stability
			sensors_pressure_bmp085_read_out();
		}
		///////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 1 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(1000000, COUNTER9, loop_start_time))
		{

		}
		///////////////////////////////////////////////////////////////////////////




		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER7, loop_start_time))
		{
			led_toggle(LED_YELLOW);
			communication_send_attitude_position(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////




		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 200 Hz functions                                     //
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(5000, COUNTER5, loop_start_time))
		{
			if (global_data.state.status == MAV_STATE_STANDBY)
			{
				//Check if parameters should be written or read
				param_handler();
			}
		}
		///////////////////////////////////////////////////////////////////////////

		else
		{
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

