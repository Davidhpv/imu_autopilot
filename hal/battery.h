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
 *   @brief Battery voltage
 *
 *   @author Christian Dobler <pixhawk@student.ethz.ch>
 *
 */

/** @addtogroup hal */
/*@{*/

#ifndef BATTERY_H_
#define BATTERY_H_

#include "inttypes.h"
#include "adc.h"
#include "conf.h"

/**
 * @brief This function reads out the battery voltage
 * @return battery voltage in milli volts
 */
static inline uint16_t battery_get_value(void)
{
	uint16_t adc_value=adc_get_value(ADC_BAT_VDC_CHANNEL);
	uint16_t bat_act_mv = (uint16_t) (BAT_VOLT_SCALE*(float)adc_value);
	return (bat_act_mv);
}

#endif /* BATTERY_H_ */

/*@}*/
