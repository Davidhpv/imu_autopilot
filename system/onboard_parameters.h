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
 * @file Onboard paramaters / settings
 *   @author Lorenz Meier
 *   @author Laurens MacKay
 *   @author Tobias Naegeli
 */

#ifndef ONBOARD_PARAMETERS_H
#define ONBOARD_PARAMETERS_H

#include "global_data.h"
#include "conf.h"

uint8_t set_parameter(const char* param_id, float value)
{
	uint8_t result = 0;

//	for (int i = 0; i < )

	return result;
}

uint8_t commit_parameters(void)
{
	uint8_t result = 0;

	return result;
}

uint16_t get_parameter_count()
{
	return 0;
}

float get_parameter(uint16_t parameter_id)
{

}

uint16_t get_parameter_ids(uint16* ids, uint16_t len)
{
	// Check if enough space is available
	if (len < ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t get_parameter_names(char* data, uint8_t len)
{
	// Check if enough space is available
	if (len < ONBOARD_PARAM_COUNT * ONBOARD_PARAM_NAME_LENGTH)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

#endif /* ONBOARD_PARAMETERS.H */
