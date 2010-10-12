#include "armVIC.h"
#include "sys_time.h"
#include "led.h"
#include "conf.h"

#define START_TIMEOUT 0xFFFF;

void timer0_init( void );


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

}

void sys_time_init( void ) {
	timer0_init();
}

int last_periodic_event;

void sys_time_periodic_init(void){
	last_periodic_event = T0TC;
}

int sys_time_periodic( void ) {
	unsigned int now = T0TC;
	unsigned int dif = now - last_periodic_event;
	if ( dif >= PERIODIC_TASK_PERIOD) {
		last_periodic_event += PERIODIC_TASK_PERIOD;
		return 1;
	}
	else{
		return 0;
	}
}

void sys_time_wait(unsigned int usec){
	unsigned int now = T0TC;
	while (T0TC<now+SYS_TICS_OF_USEC(usec));
	return;
}
