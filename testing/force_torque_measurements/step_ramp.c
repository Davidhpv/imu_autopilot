/*
 * single.c
 *
 *  Created on: 04.05.2009
 *      Author: Administrator
 */
#include "LPC21xx.h"
#include "conf.h"
#include "motors.h"
#include "step_ramp.h"
#include "led.h"
#include "servos.h"
#include "comm_temp.h"
#include "pwm.h"

double t_0;

void single_ramp_init(void){
	t_0 = T0TC;
}

void single_ramp_upper(double t_ramp, double ramp_size){
	/* This function increases the speed of the upper rotor constantly as ramp, starting
	from zero up to ramp_size (double between 0 and 1) in t_ramp seconds
	*/

	double t_now = T0TC;
	if(t_now - t_0 < t_ramp*PCLK){
		// updating the output
		double delta_t = t_now - t_0;
		double out = delta_t*ramp_size/(t_ramp*PCLK);
		/*
		char buffer[20];
		comm_int_to_string(1,buffer);
		comm_send_string(buffer);*/

		motors_set_upper((float)out);
	}
	else{led_off(LEDGREEN);
	motors_set_upper((float)ramp_size);;

	}

}

void single_ramp_lower(double t_ramp, double ramp_size){
	/* This function increases the speed of the upper rotor constantly as ramp, starting
	from zero up to ramp_size (double between 0 and 1) in t_ramp seconds
	*/

	double t_now = T0TC;
	if(t_now - t_0 < t_ramp*PCLK){
		// updating the output
		double delta_t = t_now - t_0;
		double out = delta_t*ramp_size/(t_ramp*PCLK);
		/*
		char buffer[20];
		comm_int_to_string(1,buffer);
		comm_send_string(buffer);*/

		motors_set_lower((float)out);
	}
	else{led_off(LEDRED);
	motors_set_lower((float)ramp_size);;

	}

}

void single_step_upper(double step_size){
	motors_set_upper((float)step_size);
}

void single_step_lower(double step_size){
	motors_set_lower((float)step_size);
}



