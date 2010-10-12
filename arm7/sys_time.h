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
*   @brief System time
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#ifndef SYS_TIME_H
#define SYS_TIME_H

#include "LPC21xx.h"
#include "armVIC.h"
#include "conf.h"
#include "inttypes.h"

#define SYS_TICS_OF_SEC(s)   (unsigned int)(s * PCLK)
#define SYS_TICS_OF_USEC(us) ((us)*(PCLK/1000000))
/*#define SYS_TICS_OF_USEC(us) SYS_TICS_OF_SEC((us) * 1e-6)*/
#define SYS_USEC_OF_TICS(tics) (unsigned int)((tics * 1e6 -0.5)/PCLK)
#define MAX_TC_VAL (0xFFFFFFFF-1)


void sys_time_init( void );


void sys_time_periodic_init(void);


int sys_time_periodic(void);


void sys_time_clock_init(void);

/** @brief Get the local onboard time (since powering on) */
uint64_t sys_time_clock_get_time_usec(void);

/** @brief Set the offset of the local onboard time to the UNIX epoch in usecs */
void sys_time_clock_set_unix_offset(int64_t offset);

/** @brief Get the current UNIX time offset in usecs */
int64_t sys_time_clock_get_unix_offset(void);

/** @brief Get the current UNIX time in usecs */
uint64_t sys_time_clock_get_unix_time(void);

/** @brief Convert a timestamp in UNIX epoch usecs to local onboard time */
uint64_t sys_time_clock_to_local_time(uint64_t unix_time);


static inline uint32_t sys_time_get_period_start_time(void){
	uint32_t time = T0MR2-PERIODIC_TASK_PERIOD;
	return time;
}

static inline void sys_time_wait(unsigned int usec){
	unsigned int now = T0TC;
	while (T0TC<now+SYS_TICS_OF_USEC(usec));
	return;
}


static inline uint32_t sys_time_get_timer_counter(void){
	return T0TC;
}

#endif /* SYS_TIME_H */
