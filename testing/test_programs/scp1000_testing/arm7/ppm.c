#include "ppm.h"
#include "LPC21xx.h"
#include "sys_time.h"
#include "conf.h"
#include "watchdog.h"

int ppm_valid;

int ppm_pulses[PPM_NB_CHANNEL];


int ppm_get_channel(unsigned int nr){
	if(nr<1 || nr>PPM_NB_CHANNEL){
		return -1;
	}
	else{
		return SYS_USEC_OF_TICS(ppm_pulses[nr-1]);
	}
}


void ppm_init(void){
	/* select pin for capture */
	PINSEL0 |= 2 << 12;
	/* enable capture 0.2 on falling edge + trigger interrupt */
	T0CCR = TCCR_CR2_F | TCCR_CR2_I;
	int i;
	for(i=0;i<PPM_NB_CHANNEL;i++){
		ppm_pulses[i]=SYS_TICS_OF_USEC(1500);
	}
	ppm_valid = 0;
}

void PPM_ISR() {
   	static unsigned int state = PPM_NB_CHANNEL;
   	static unsigned int last;

    unsigned int now = T0CR2;
    unsigned int length = now - last;
    last = now;

    watchdog_set_time(WATCHDOG_CHANNEL_PPM);

    if (state == PPM_NB_CHANNEL) {
      	if (length > SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) && length < SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {
			state = 0;
      	}
      	else{
  			ppm_valid = 0;
      	}
    }
    else {
      	if (length > SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN) && length < SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN)) {
      		ppm_pulses[state] = (int)length;
			state++;
			if (state == PPM_NB_CHANNEL) {
	  			ppm_valid = 1;
			}
    	}
      	else{
			state = PPM_NB_CHANNEL;
  			ppm_valid = 0;
      	}
    }
}
