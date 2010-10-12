#include "pwm.h"
#include "conf.h"

#define START_TIMEOUT 0xFFFF;

int counter;
/* pause time between the pulses */
unsigned int pause_time;
unsigned int pwm_values[PWM_NB_CHANNELS];

void pwm_set_channel(unsigned int usec,unsigned int channel_nr){
	//catch false channel_nr inputs
	if(channel_nr>=PWM_NB_CHANNELS){
		return;
	}
	//catch false usec inputs
	if(usec<PWM_MIN_PULSE_USEC){
		usec=PWM_MIN_PULSE_USEC;
	}
	if(usec>PWM_MAX_PULSE_USEC){
		usec=PWM_MAX_PULSE_USEC;
	}
	//set value to channel
	pwm_values[channel_nr]=SYS_TICS_OF_USEC(usec);
}

int pwm_get_channel(unsigned int channel_nr){
	//catch false channel_nr inputs
	if(channel_nr>=PWM_NB_CHANNELS){
		return -1;
	}
	else{
		return SYS_USEC_OF_TICS(pwm_values[channel_nr]);
	}
}

void pwm_init ( void ) {
  /* P1.25-16 are used as GPIO */
  PINSEL2 &= ~(1<<3);
  /* select reset pin as GPIO output */
  PWM_RESET_IODIR |= 1<<PWM_RESET_PIN;
  /* set reset pin to ground */
  PWM_RESET_IOCLR = 1<<PWM_RESET_PIN;


  /* select clock pin as MAT0.1 output */
  IO0DIR |= 1<<PWM_CLOCK_PIN;
  PWM_CLOCK_PINSEL |= PWM_CLOCK_PINSEL_VAL << PWM_CLOCK_PINSEL_BIT;
  /* enable match 1 interrupt */
  T0MCR |= TMCR_MR1_I;  //#define TMCR_MR1_I  (1 << 3)
  /* set servo_clock low         */
  T0EMR &= ~TEMR_EM1;    // #define TEMR_EM1    (1 << 1)
  /* toggle servo_clock on match 1 */
  T0EMR |= TEMR_EMC1_2; //  #define TEMR_EMC1_2 (2 << 6)


  /* set first pulse in a while */
  T0MR1 = START_TIMEOUT;
  pause_time = PWM_PERIODE;
  counter=-1;

  /* Set all servos at their midpoints */
  /* compulsory for unaffected servos  */
  int i;
  for( i=0 ; i < PWM_NB_CHANNELS ; i++ )
    pwm_values[i] = SYS_TICS_OF_USEC(1500);
}

void PWM_ISR(void) {
	if(counter==-1){
		pause_time = PWM_PERIODE;
		//set next match in 100usec
		T0MR1 += SYS_TICS_OF_USEC(100);
		//set reset high (for 100USEC)
		PWM_RESET_IOSET = 1<<PWM_RESET_PIN;
		counter=0;
		return;
	}
	if(counter==0){
		//set next match when channel 0 pulse is done
		T0MR1 += pwm_values[counter]-SYS_TICS_OF_USEC(100);
		pause_time = pause_time - pwm_values[counter];
		//set reset low
		PWM_RESET_IOCLR = 1<<PWM_RESET_PIN;
		//set clock low
		T0EMR &= ~TEMR_EM1;
		counter++;
		return;
	}
	/* period pause */
	if(counter==PWM_NB_CHANNELS){
		//set next match when pause is done
		T0MR1 += pause_time;
		counter=-1;
		return;
	}

	//set next macht after the channel pulse is done
	T0MR1 += pwm_values[counter];
	//calculate new pause time (indepentent of pulses length)
	pause_time = pause_time - pwm_values[counter];
	//set clock low
	T0EMR &= ~TEMR_EM1;
	counter++;
}


