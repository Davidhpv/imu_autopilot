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
*
*   @brief System watchdog
*   @author Laurens Mackay <pixhawk@student.ethz.ch>
*
*/

#include "LPC21xx.h"
#include "watchdog.h"


void watchdog_kick(void)
{
    // Errata 23xx WDT.1 No APB accesses allowed during feed sequence.
//    unsigned prev = VICIntEnable;
//    VICIntEnClr=prev;   // Disabling interrupts in the VIC
    WDFEED = 0xAA;
    WDFEED = 0x55;
//    VICIntEnable=prev;  // Enabling interrupts in the VIC
}

#define WDEN 0
#define WDRESET 1
void watchdog_init(void)
{
	// WDTC = PCLK/4;          // Watchdog timer constant.
	// Taking standard timout of 86 us?
    WDMOD = (1<<WDEN)|(1<<WDRESET);   // Reset processor if failure to kick.
    watchdog_kick();            // Activate watchdog.
}

void watchdog_wait_reset(void)
{
	watchdog_init();
	while(1);
}

