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

#ifndef SYS_STATE_H_
#define SYS_STATE_H_

#include <stdbool.h>
#include "debug.h"

/** @brief Check if the system is currently in flight mode */
uint8_t sys_state_is_flying(void);

/** @brief Set the current mode of operation */
bool sys_set_mode(uint8_t mode);

/** @brief Set the current system state */
bool sys_set_state(uint8_t state);

/** @brief Get the current system state */
uint8_t sys_get_state(void);

/** @return Get the current mode of operation */
uint8_t sys_get_mode(void);

/** @brief Set the current system type */
void sys_set_type(enum MAV_TYPE type);

/** @brief Set the current navigation mode */
void sys_set_nav_mode(enum MAV_NAV nav_mode);

#endif /* SYS_STATE_H_ */
