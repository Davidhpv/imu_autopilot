/*======================================================================

PIXHAWK mavlib - The Micro Air Vehicle Platform Library
Please see our website at <http://pixhawk.ethz.ch>


Original Authors:
  Dominik Honegger
  Tobias Naegeli
  Martin Rutschmann
  Lorenz Meier <lm@student.ethz.ch>
Contributing Authors (in alphabetical order):



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

#ifndef _DAC_H_
#define _DAC_H_

#include "inttypes.h"

/**
 * @brief Initialisation of the Digital Analog converter (DAC)
 */
void dac_init (void);

/**
 * @brief Sets the value of the DAC to value/1024*V_analog
 * @param value has to be a 10 bit value (from 0 to 1023), the analog output of the DAC is calculated as value/1024*V_analog (typically 3.3V)
 */
void dac_set (uint16_t value);

#endif /* _DAC_H_ */
