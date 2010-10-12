#include "LPC21xx.h"
#include "led.h"
#include "conf.h"


void led_init(void) {
	LED_YELLOW_PINSEL &= ~(LED_YELLOW_PINSEL_VAL<<LED_YELLOW_PINSEL_BIT);
	LED_RED_PINSEL &= ~(LED_RED_PINSEL_VAL<<LED_RED_PINSEL_BIT);
	LED_GREEN_PINSEL &= ~(LED_GREEN_PINSEL_VAL<<LED_GREEN_PINSEL_BIT);
	LED_DIR |= (1<<LED_YELLOW)|(1<<LED_GREEN)|(1<<LED_RED);
	led_off(LED_GREEN);
	led_off(LED_RED);
	led_off(LED_YELLOW);
}

void led_on(int led){
	LED_CLEAR = (1<<led);
}

void led_off(int led){
	LED_SET=(1<<led);
}

void led_toggle(int led){
	if((LED_PIN & (1<<led)) ){
			led_on(led);
		}
		else{
			led_off(led);
		}
}




