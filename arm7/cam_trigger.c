#include "cam_trigger.h"
#include "conf.h"
#include "LPC21xx.h"
#include "sys_time.h"


void cam_trigger_init(void){
//	/* set cam trigger pin as pwm output */
//	CAM_TRIGGER_PIN_PINSEL |= (CAM_TRIGGER_PIN_VAL << CAM_TRIGGER_PIN_BIT);
//
//	/* set the period and pulse lengths */
//	CAM_TRIGGER_PWM_CHANNEL_MR = SYS_TICS_OF_USEC(CAM_TRIGGER_PULSE_WIDTH_US);
//	PWMMR0 = SYS_TICS_OF_USEC(CAM_TRIGGER_PERIODE_US);
//
//	/* The speed of the pwm counter is the same as PCLK */
//	PWMPR = 0;
//
//	/* reset the pwm counter on PWMMR0 event */
//	PWMMCR |= PWMMCR_MR0R;
//
//	/* enable PWM 5 output */
//	PWMPCR |= PWMPCR_ENA5;
//
//	/* enable pwm counter */
//	PWMTCR |= PWMTCR_COUNTER_ENABLE;

}

