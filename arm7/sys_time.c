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

#include "armVIC.h"
#include "sys_time.h"
#include "led.h"
#include "ppm.h"
#include "conf.h"
#include "pwm.h"
#include "i2c_devices/bmp085.h"
#include "inttypes.h"
#include <stdio.h>

#define START_TIMEOUT 0xFFFF;

void TIMER0_ISR ( void ) __attribute__((naked));

void SYS_TIME_PERIODIC_ISR( void );
void SYS_TIME_CLOCK_ISR( void );

void timer0_init( void );

uint8_t periodic_state;
uint32_t number_of_timer_overflow=0;
int64_t m_clock_offset=0;

void TIMER0_ISR ( void )  {
  ISR_ENTRY();
  //uint8_t buffer[200];	// string buffer for debug messages


  while (T0IR /*& TIMER0_IT_MASK*/) {

	if (T0IR&PPM_CAPTURE_TIMER_INTERRUPT) {
	  /*if capture on PPM interrupt start ppm isr*/

      PPM_ISR();
      /* clear interrupt */
      T0IR = PPM_CAPTURE_TIMER_INTERRUPT;
    }

	if (T0IR&TIR_MR1I) {
	  /*if	match1 interrupt start pwm isr */

      PWM_ISR();
      /* clear interrupt */
      T0IR = TIR_MR1I;
    }
	if (T0IR&TIR_MR2I) {


	  /*if	match2 interrupt start periodic isr */
	  SYS_TIME_PERIODIC_ISR();
      /* clear interrupt */
      T0IR = TIR_MR2I;
    }
	if (T0IR&TIR_MR3I) {
	  /*if	match3 interrupt start timer overflow isr */

		SYS_TIME_CLOCK_ISR();
      /* clear interrupt */
      T0IR = TIR_MR3I;
	}
	#if(FEATURE_SENSOR_PRESSURE_INTERUPT==FEATURE_SENSOR_PRESSURE_INTERUPT_DISABLED)
	if (T0IR&TIR_MR0I) {
	  /*if	match0 interrupt start pressure isr */
		bmp085_start_measurement_read();
		/* clear interrupt */
		T0IR = TIR_MR0I;
	}
	#endif
  }
  VICVectAddr = 0x00000000;

  ISR_EXIT();
}

void timer0_init( void ){

  /* setup Timer 0 to count forever  */
  /* reset & disable timer 0         */
  T0TCR = TCR_RESET;
  /* set the prescale divider        */
  T0PR = 0;
  /* disable match registers         */
  T0MCR = 0;
  /* disable compare registers       */
  T0CCR = 0;
  /* disable external match register */
  T0EMR = 0;
  /* enable timer 0                  */
  T0TCR = TCR_ENABLE;

  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0);
  /* on slot vic slot 1      */
  _VIC_CNTL(TIMER0_VIC_SLOT) = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  _VIC_ADDR(TIMER0_VIC_SLOT) = (unsigned int)TIMER0_ISR;

}

void sys_time_init(void)
{
	timer0_init();
}


void sys_time_periodic_init(void)
{
	/* enable match 2 interrupt */
	T0MCR |= TMCR_MR2_I;
	T0MR2 = START_TIME_PERIODIC;
	periodic_state=0;
}


void SYS_TIME_PERIODIC_ISR(void)
{
	periodic_state=1;
	T0MR2 += PERIODIC_TASK_PERIOD;
}



int sys_time_periodic(void)
{

	if ( periodic_state==1) {
		periodic_state=0;
		return 1;
	}
	else{
		return 0;
	}
}

void SYS_TIME_CLOCK_ISR(void)
{
	number_of_timer_overflow++;
}

void sys_time_clock_init(void)
{
	/* enable match 3 interrupt */
	T0MCR |= TMCR_MR3_I;
	T0MR3 = 0;
	m_clock_offset = 0;
	number_of_timer_overflow = 0;
}

uint64_t sys_time_clock_get_time_usec(void)
{
	return ((uint64_t)286331153llU)*number_of_timer_overflow + (uint32_t)SYS_USEC_OF_TICS(T0TC);
}

void sys_time_clock_set_unix_offset(int64_t offset)
{
	m_clock_offset = offset;
}

int64_t sys_time_clock_get_unix_offset(void)
{
	return m_clock_offset;
}

uint64_t sys_time_clock_get_unix_time(void)
{
	return sys_time_clock_get_time_usec() + m_clock_offset;
}

uint64_t sys_time_clock_to_local_time(uint64_t unix_time)
{
	return unix_time - m_clock_offset;
}
