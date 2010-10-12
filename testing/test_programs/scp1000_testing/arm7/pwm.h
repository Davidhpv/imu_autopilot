#ifndef PWM_H
#define PWM_H

#include "LPC21xx.h"
#include "sys_time.h"
#include "conf.h"

/**
 * Initialization of the pwm generation. If you want to use pwm signals
 * you also have to initialize sys_time.
 */
void pwm_init(void);

/**
 * You can set the width of a channel with this function.
 * @param length_usec	pulse length in usec
 * @param channel_nr	channel number
 */
void pwm_set_channel(unsigned int length_usec, unsigned int channel_nr);

/**
 * Reads out the pwm length of the selected channel in usec.
 * @param channel_nr	channel number
 * @return 				pulse lengt of selected channel in usec
 */
int pwm_get_channel(unsigned int channel_nr);

/**
 * Interrupt routine for pwm generation. This function is called by
 * the timer interrupt.
 * @see sys_time.h
 */
void PWM_ISR(void);


#endif /* PWM_H */
