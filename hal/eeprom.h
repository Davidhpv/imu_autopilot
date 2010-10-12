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
 * @file the main file
 *   @author Martin Rutschmann
 *   @author Dominik Honegger
 *   @author Tobias Naegeli
 *   @author Joseph Kruijen
 *   @author Lorenz Meier
 *   @author Christian Dobler
 *
 *  Please refer to
 *
 *  http://pixhawk.ethz.ch/software/embedded/flight/
 *  and
 *  http://pixhawk.ethz.ch/api/flight/
 *
 *  For the full documentation of this code.
 *
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "conf.h"

#if defined (IMU_PIXHAWK_V200) || defined (IMU_PIXHAWK_V210)

/** @addtogroup HAL */
//@{
/** @name EEPROM functions
 *  EEPROM parameters */
//@{


static inline float eeprom_get_param(uint16_t param_id)
{
	return 0;
}

static inline uint8_t eeprom_write_param(uint16_t param_id)
{
	return 1;
}

//@}}

#endif

#endif /* EEPROM_H_ */
