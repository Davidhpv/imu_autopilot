/*
 * least_square.c
 *
 *  Created on: Sep 3, 2009
 *      Author: mav
 */

#include "least_square.h"

void least_square(int press_raw, int dist, float *theta, float *p){
	float P11 = p[0];
	float P12 = p[1];
	float P21 = p[2];
	float P22 = p[3];

	float a = theta[0];
	float b = theta[1];

	// New pressure-sensor values: altitude(press_raw) = a*press_raw + b [mm]
	float a_new = a + ((P12 + P11*press_raw)*(-b + dist - a*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	float b_new = b + ((P22 + P21*press_raw)*(-b + dist - a*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	// Update least-square states
	float P11_new = P11*(1 - (press_raw*(P12 + P11*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1)) - (P21*(P12 + P11*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	float P12_new = P12*(1 - (press_raw*(P12 + P11*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1)) - (P22*(P12 + P11*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	float P21_new = P21*(1 - (P22 + P21*press_raw)/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1)) - (P11*press_raw*(P22 + P21*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	float P22_new = P22*(1 - (P22 + P21*press_raw)/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1)) - (P12*press_raw*(P22 + P21*press_raw))/(P22 + P12*press_raw + press_raw*(P21 + P11*press_raw) + 1);

	// Sample-Time shift
	theta[0]=a_new;
	theta[1]=b_new;

	p[0]=P11_new;
	p[1]=P12_new;
	p[2]=P21_new;
	p[3]=P22_new;
}


