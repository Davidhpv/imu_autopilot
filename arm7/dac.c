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

#include "LPC21xx.h"
#include "conf.h"
#include "dac.h"

#ifdef BOARD_PIXHAWK_V100

void dac_init (void)
{
	DAC_PINSEL|=DAC_PINSEL_VAL<<DAC_PINSEL_BIT;
	DACR = 1<<16; //set output in powermode and set output to 0V
}


void dac_set (uint16_t value)
{
	value&=DAC_VALUE_MASK;
	DACR = value<<6; // bits 0-5 are reserved
}

#endif
