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
* @file Camera shutter triggering
*   @author Lorenz Meier
*
*/

#include "shutter.h"
#include "arm7/include/LPC21xx.h"
#include "arm7/sys_time.h"
#include "system/global_data.h"
#include "conf/conf.h"

static uint8_t m_shutter_active = 0;
static uint32_t m_last_shut = 0;
static uint32_t m_shutter_seq = 0;
static bool m_shutter_was_low = false; ///< Used to determine wether the shutter went off.

void shutter_init(void)
{
	// Set IO pin direction
#if (CAMERA_SHUTTER_PORT == 1)
	IO1DIR |= (1<<CAMERA_SHUTTER_PIN);  //pin 1.xx is configured as output
#endif
#if (CAMERA_SHUTTER_PORT == 0)
	IO0DIR |= (1<<CAMERA_SHUTTER_PIN);  //pin 0.xx is configured as output
#endif

	// set high = inactive
#if (CAMERA_SHUTTER_PORT == 1)
	IO1SET |= (1<<CAMERA_SHUTTER_PIN);  //pin 1.xx goes HIGH
#endif
#if (CAMERA_SHUTTER_PORT == 0)
	IO0SET |= (1<<CAMERA_SHUTTER_PIN);  //pin 0.xx goes LOW
#endif

	// Set flag
	m_last_shut = sys_time_clock_get_time_usec();
	m_shutter_active = 1;
}

void shutter_control(uint8_t enable)
{
	m_shutter_active = enable;
	if (!enable) m_shutter_seq = 0;
}

void shutter_set(uint32_t shutter_interval, uint32_t exposure)
{
	global_data.param[PARAM_CAM_INTERVAL] = shutter_interval;
	global_data.param[PARAM_CAM_EXP] = exposure;
}

uint32_t shutter_get_seq(void)
{
	return m_shutter_seq;
}

bool shutter_loop(void)
{	
	bool ret = false;

	if (m_shutter_active)
	{
		// Check wether the shutter pin should be currently down
		// shutter is active low (GND = active, VCC = disabled)
		uint32_t time = sys_time_clock_get_time_usec();
		if ((m_last_shut + ((uint32_t)global_data.param[PARAM_CAM_INTERVAL])) < time)
		{
			// Check wether the shutter transitioned right now to low
			if (m_shutter_was_low == false)
			{
				ret = true;
				m_shutter_seq++;
				m_shutter_was_low = true;
			}
			// Assert pin low (active)
#if (CAMERA_SHUTTER_PORT == 1)
			IO1CLR |= (1<<CAMERA_SHUTTER_PIN);  //pin 1.xx goes LOW
#endif
#if (CAMERA_SHUTTER_PORT == 0)
			IO0CLR |= (1<<CAMERA_SHUTTER_PIN);  //pin 0.xx goes LOW
#endif
			// Set last shutter time
			m_last_shut = time;
			//mavlink_msg_debug_send(MAVLINK_COMM_1, 230, time);
		}

		// raise shutter pin after exposure time
		// m_last_shut is now set to the latest
		if ((m_last_shut + ((uint32_t)global_data.param[PARAM_CAM_EXP])) < time)
		{
			m_shutter_was_low = false;
			// Assert pin high (inactive)
#if (CAMERA_SHUTTER_PORT == 1)
			IO1SET |= (1<<CAMERA_SHUTTER_PIN);  //pin 1.xx goes HIGH
#endif
#if (CAMERA_SHUTTER_PORT == 0)
			IO0SET |= (1<<CAMERA_SHUTTER_PIN);  //pin 0.xx goes HIGH
#endif
			//mavlink_msg_debug_send(MAVLINK_COMM_1, 231, time);
		}
	}
	return ret;
}

