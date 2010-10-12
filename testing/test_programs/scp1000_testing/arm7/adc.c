#include "adc.h"
#include "conf.h"
#include "LPC21xx.h"

void adc_init(void){
	//infrared
	PINSEL1 |= (1<<26); // set function P0.29 as AC0.2

	int CLKDIV = (int) (PCLK / 4500000);
	if(CLKDIV>0){
		CLKDIV=CLKDIV-1;
	}
	AD0CR = ( 1<<2 | 1<<16 | CLKDIV<<8 | 1<<21);

	//bat
	PINSEL0 |= (3<<30); // set function P0.15 as AC1.5

	AD1CR = ( 1<<5 | 1<<16 | CLKDIV<<8 | 1<<21);
}

/**
 * This function reads out the infrared sensor
 * @return distance from the infrared sensor in mm
 */
int adc_get_infrared_value(void){
	int adc_value=(AD0GDR>>6)&0x03FF;
	double dist_m = -0.06163 +(69.2/adc_value);
	int dist_mm = (int)(1000*dist_m);
	return (dist_mm);
}

/**
 * This function reads out the battery voltage
 * @return distance from the battery voltage in mv
 */
int adc_get_bat_value(void){
	int adc_value=((AD1GDR>>6)&0x03FF);
	/* max Value adc=3300 mV
	 * voltage divider for battery 100k and 15k
	 * 10 bit adc -> 1024 steps */
	int bat_mv = adc_value * 3300 * (100+15)/15 /1024;
	return (bat_mv);
}
