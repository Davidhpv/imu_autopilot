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
* @brief System state representation
*   @author Lorenz Meier
*/

#include "sys_state.h"
#include "global_data.h"

/**
 * @brief Check if the system is currently in flight mode
 *
 * This function will return false for all states that allow
 * changes to critical system variables, such as calibrating sensors
 * or changing the system type.
 *
 * @return 0 if the system is not flying, 1 if it is flying or flight-ready
 */
uint8_t sys_state_is_flying(void)
{
	if (global_data.state.status == MAV_STATE_STANDBY || global_data.state.status == MAV_STATE_UNINIT)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
/**
 * @return Get the current mode of operation
 */
uint8_t sys_get_mode(void)
{
	return (uint8_t) global_data.state.mav_mode;
}

/**
 * @brief Set the current mode of operation
 * @param mode the new mode
 */
bool sys_set_mode(uint8_t mode)
{
	if (mode == MAV_MODE_AUTO)
	{
		global_data.state.mav_mode = MAV_MODE_AUTO;
		return true;
	}
	else if (mode == MAV_MODE_GUIDED)
	{
		global_data.state.mav_mode = MAV_MODE_GUIDED;
		return true;
	}
	else if (mode == MAV_MODE_LOCKED)
	{
		global_data.state.mav_mode = MAV_MODE_LOCKED;
		return true;
	}
	else if (mode == MAV_MODE_MANUAL)
	{
		global_data.state.mav_mode = MAV_MODE_MANUAL;
		return true;
	}
	else if (mode == MAV_MODE_READY)
	{
		global_data.state.mav_mode = MAV_MODE_READY;
		return true;
	}
	else if (mode == MAV_MODE_TEST1)
	{
		global_data.state.mav_mode = MAV_MODE_TEST1;
		return true;
	}
	else if (mode == MAV_MODE_TEST2)
	{
		global_data.state.mav_mode = MAV_MODE_TEST2;
		return true;
	}
	else if (mode == MAV_MODE_TEST3)
	{
		global_data.state.mav_mode = MAV_MODE_TEST3;
		return true;
	}
	else if (mode == MAV_MODE_RC_TRAINING)
	{
		// Only go into RC training if not flying
		if (! sys_state_is_flying())
		{
			global_data.state.mav_mode = MAV_MODE_RC_TRAINING;
			return true;
		}
		else
		{
			debug_message_buffer("WARNING: SYSTEM IS IN FLIGHT! Denied to switch to RC mode");
			return false;
		}
	}
	// UNINIT is not a mode that should be actively set
	// it will thus be rejected like any other invalid mode
	else
	{
		// No valid mode
		debug_message_buffer("WARNING: Attempted to set invalid mode");
		return false;
	}
}

/**
 * @brief Set the current system state
 * @param state the new system state
 */
bool sys_set_state(uint8_t state)
{
	if (state == MAV_STATE_ACTIVE)
	{
		global_data.state.status = MAV_STATE_ACTIVE;
		return true;
	}
	else if (state == MAV_STATE_BOOT)
	{
		global_data.state.status = MAV_STATE_BOOT;
		return true;
	}
	else if (state == MAV_STATE_CALIBRATING)
	{
		global_data.state.status = MAV_STATE_CALIBRATING;
		return true;
	}
	else if (state == MAV_STATE_CRITICAL)
	{
		global_data.state.status = MAV_STATE_CRITICAL;
		return true;
	}
	else if (state == MAV_STATE_EMERGENCY)
	{
		global_data.state.status = MAV_STATE_EMERGENCY;
		return true;
	}
	else if (state == MAV_STATE_POWEROFF)
	{
		global_data.state.status = MAV_STATE_POWEROFF;
		return true;
	}
	else if (state == MAV_STATE_STANDBY)
	{
		global_data.state.status = MAV_STATE_STANDBY;
		return true;
	}
	else if (state == MAV_STATE_UNINIT)
	{
		global_data.state.status = MAV_STATE_STANDBY;
		return true;
	}
	else
	{
		// UNINIT or invalid state, ignore value and return false
		debug_message_buffer("WARNING: Attempted to set invalid state");
		return false;
	}

	// FIXME Remove this once new interface is existent
	global_data.state.status = state;

}

/**
 * @return The current system state, following the MAV_STATE enum
 */
uint8_t sys_get_state(void)
{
	return (uint8_t) global_data.state.status;
}

/**
 * @param type the new system type
 */
void sys_set_type(enum MAV_TYPE type)
{
	global_data.state.type = type;
}

/**
 * @param nav_mode the new navigation mode
 */
void sys_set_nav_mode(enum MAV_NAV nav_mode)
{
	global_data.state.nav_mode = nav_mode;
}
