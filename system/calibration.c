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
 * @brief Auto-calibration of remote control, magnetometer, etc.
 *   @author Lorenz Meier
 *   @author Laurens MacKay
 */

#include <stdbool.h>
#include <inttypes.h>
#include "calibration.h"
#include "comm.h"
#include "global_data.h"
#include "gyros.h"
#include "mavlink.h"
#include "sys_state.h"
#include "ppm.h"
#include "uart.h"
#include "range.h"
#include "remote_control.h"

static uint8_t calibration_prev_state;
static uint8_t calibration_prev_mode;

bool calibration_enter(void)
{
	// If not flying
	if (!sys_state_is_flying())
	{
		calibration_prev_state = sys_get_state();
		calibration_prev_mode = sys_get_mode();
		// Lock vehicle during calibration
		sys_set_mode(MAV_MODE_LOCKED);
		sys_set_state(MAV_STATE_CALIBRATING);
		debug_message_buffer("Starting calibration.");
		return 1;
	}
	else
	{
		//Can't calibrate during flight
		debug_message_buffer("Can't calibrate during flight!!!");
		return 0;
	}

}
void calibration_exit(void)
{
	// Go back to old state
	sys_set_mode(calibration_prev_state);
	sys_set_state(calibration_prev_mode);

	// Clear debug message buffers
	for (int i = 0; i < DEBUG_COUNT; i++)
	{
		debug_message_send_one();
	}

	// Clear UART buffers
	while (uart0_char_available())
	{uart0_get_char();}
	while (uart1_char_available())
	{uart1_get_char();}

	debug_message_buffer("Calibration finished. UART buffers cleared.");
}

void start_rc_calibration(void)
{
	if (calibration_enter())
	{


		// Calibration routine: Read out remote control and wait for all channels.
		// Read initial values.
		const uint8_t chan_count = 9;
		uint32_t chan_initial[chan_count];
		uint32_t chan_min[chan_count];
		uint32_t chan_max[chan_count];
		for (int i = 0; i < chan_count; i++)
		{
			chan_initial[i] = ppm_get_channel(i + 1);
			chan_min[i]=2000;
			chan_max[i]=1000;
		}

		bool abort_rc_cal = 0;

		while (!abort_rc_cal)
		{
			// Now save the min and max values for each channel
			for (int i = 0; i < chan_count; i++)
			{
				uint32_t ppm_value = (uint32_t) ppm_get_channel(i + 1);
				chan_min[i] = min(chan_min[i], ppm_value);
				chan_min[i] = min(chan_min[i], ppm_value);
			}
			// until motors stop conditions are met.
			if ((ppm_get_channel(global_data.param[PARAM_PPM_THROTTLE_CHANNEL])
					< PPM_LOW_TRIG) && (ppm_get_channel(
					global_data.param[PARAM_PPM_YAW_CHANNEL]) > PPM_HIGH_TRIG))
			{
				abort_rc_cal = 1;
			}
		}




		calibration_exit();
	}
}

void start_mag_calibration(void)
{
	// not done here

}

void start_pressure_calibration(void)
{
	if (calibration_enter())
	{
		// Calibration routine

		calibration_exit();
	}
}

void start_gyro_calibration(void)
{
	if (calibration_enter())
	{
		// Calibration routine
		gyro_calibrate();
		calibration_exit();
	}
}
