#include "armVIC.h"
#include "sys_time.h"
#include "led.h"
#include "ppm.h"
#include "conf.h"
#include "pwm.h"

#define START_TIMEOUT 0xFFFF;

void TIMER0_ISR ( void ) __attribute__((naked));

void timer0_init( void );

void TIMER0_ISR ( void )  {
  ISR_ENTRY();

  while (T0IR /*& TIMER0_IT_MASK*/) {

	if (T0IR&TIR_CR2I) {
	  /*if capture on PWM interrupt start ppm isr*/
      PPM_ISR();
      /* clear interrupt */
      T0IR = TIR_CR2I;
    }

	if (T0IR&TIR_MR1I) {
	  /*if	match1 interrupt start pwm isr */
      PWM_ISR();
      /* clear interrupt */
      T0IR = TIR_MR1I;
    }

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
