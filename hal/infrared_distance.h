/*
 * infrared_distance.h
 *
 *  Created on: 29.06.2010
 *      Author: Laurens Mackay
 */

#ifndef INFRARED_DISTANCE_H_
#define INFRARED_DISTANCE_H_

//Infrared distance sensor

static inline float infrared_distance_get(void)
{

	uint16_t adc_value = adc_get_value(ADC_7_CHANNEL);
	float adc_volt = ((float) adc_value) / 310.0f;

	//Coefficients for polynomial
	// See http://pixhawk.ethz.ch/electronics/sensors
	float a = 0.147647523061757;
	float b = -1.194234306604760;
	float c = 3.561824135586408;
	float d = -4.855000409908190;
	float e = 2.960645777286909;

	//Calculate distance with polynomial
	float distance = (a * adc_volt * adc_volt * adc_volt * adc_volt + b
			* adc_volt * adc_volt * adc_volt + c * adc_volt * adc_volt + d
			* adc_volt + e);
	return distance;
}
#endif /* INFRARED_DISTANCE_H_ */
