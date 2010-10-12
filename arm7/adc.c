/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
* @file
*   @brief Driver for LPC2000 series ADC
*
*   @author Martin Rutschmann <pixhawk@student.ethz.ch>
*
*/

#include "adc.h"
#include "conf.h"
#include "LPC21xx.h"


#if (FEATURE_ADC==FEATURE_ADC_PIXHAWK)
void adc_init(void){

	int CLKDIV = (int) (PCLK / 4500000);
	if(CLKDIV>0){
		CLKDIV=CLKDIV-1;
	}

	//set ADC3 (P0.4) as AD0.6
	PINSEL0 |= (3<<8);

	//set ADC5 (P0.29) as AD0.2
	PINSEL1 |= (3<<26);

	//set ADC6 (P0.28) as AD0.1
	PINSEL1 |= (3<<24);

	//set ADC7 (P0.12) as AD1.3
	PINSEL0 |= (3<<24);

	//set BAT_VDC (P0.15) as AD1.5
	PINSEL0 |= (3<<30);

	AD0CR = ( 1<<1 | 1<<2 | 1<<6 | 1<<16 | CLKDIV<<8 | 1<<21);
	AD1CR = ( 1<<3 | 1<<5 | 1<<16 | CLKDIV<<8 | 1<<21);
}

uint16_t adc_get_value(uint8_t channel){
	uint16_t adc_value=0;
	switch(channel){
	case ADC_BAT_VDC_CHANNEL:
		adc_value=(AD1DR5>>6)&0x03FF;
		break;
	case ADC_3_CHANNEL:
		adc_value=(AD0DR6>>6)&0x03FF;
		break;
	case ADC_5_CHANNEL:
		adc_value=(AD0DR2>>6)&0x03FF;
		break;
	case ADC_6_CHANNEL:
		adc_value=(AD0DR1>>6)&0x03FF;
		break;
	case ADC_7_CHANNEL:
		adc_value=(AD1DR3>>6)&0x03FF;
		break;
	}
	return adc_value;
}

#endif


#if defined(BOARD_PIXHAWK_V100) || defined(BOARD_TWOG_BOOZ)

void adc_init(void){

	int CLKDIV = (int) (PCLK / 4500000);
	if(CLKDIV>0){
		CLKDIV=CLKDIV-1;
	}

	//set BPlan_IO_0 (P0.6) as AD1.0
//	PINSEL0 |= (3<<12);

	//set BPlan_IO_2 (P0.10) as AD1.2
	PINSEL0 |= (3<<20);

	//set BAT_VDC (P0.13) as AD1.4
	PINSEL0 |= (3<<26);

	AD1CR = ( 1<<0 | 1<<2 | 1<<4 | 1<<16 | CLKDIV<<8 | 1<<21);
}

uint16_t adc_get_value(uint8_t channel){
	uint16_t adc_value=0;
	switch(channel){
	case ADC_BAT_VDC_CHANNEL:
		adc_value=(AD1DR4>>6)&0x03FF;
		break;
	case ADC_BPLANCD_IO_0_CHANNEL:
		adc_value=(AD1DR0>>6)&0x03FF;
		break;
	case ADC_BPLANCD_IO_2_CHANNEL:
		adc_value=(AD1DR2>>6)&0x03FF;
		break;
	}
	return adc_value;
}

#endif
