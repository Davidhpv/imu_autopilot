/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Martin Rutschmann
Contributing Authors (in alphabetical order):
  Dominik Honegger
  Tobias Naegeli
  Martin Rutschmann



(c) 2008, 2009 PIXHAWK PROJECT

This file is part of the PIXHAWK project

    mavlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mavlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mavlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

/**
* @file
*	@brief Get values from the radio control / futaba.
*
* This file contains functions to get values from the radio control.
**/

#ifndef RADIO_CONTROL_H_
#define RADIO_CONTROL_H_
#define RADIO_CONTROL_ON	1
#define RADIO_CONTROL_OFF	0
#include "inttypes.h"
#include "ppm.h"

/**
 * @brief Get one channel of the connected radio control
 *
 * @param channel radio control channel, in the range from 1-9
 * @return the normalized value in the range from -1 to 1 where 0 is the centered stick
 */
static inline float radio_control_get_channel(unsigned int channel)
{
	float normed;
	if(ppm_get_channel(channel)<1970)
	{
		int ms = ppm_get_channel(channel);
		normed = (ms-1500);
		normed=normed/500;
	}
	else
	{
		normed=0;
	}
	return normed;
}

/**
 * @brief Get one channel of the connected radio control
 *
 * @param channel radio control channel, in the range from 1-9
 * @return the raw ppm value
 */
static inline uint16_t radio_control_get_channel_raw(unsigned int channel)
{
	return ppm_get_channel(channel);
}

static inline uint8_t radio_control_status(void)
{
	if((ppm_get_channel(1) < 1970))
	{
		return RADIO_CONTROL_ON;
	}
	else
	{
		return RADIO_CONTROL_OFF;
	}
}
#endif /* RADIO_CONTROL_H_ */
