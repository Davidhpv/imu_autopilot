#include "LPC21xx.h"
#include "led.h"
#include "conf.h"

void led_init(void) {
	PINSEL2 &= ~(1<<3); //TODO Hack, this is for setting P16-P23 as GPIO ports
	LED_RED_DIR |= (1 << LED_RED);
	LED_GREEN_DIR |= (1 << LED_GREEN);
	led_off(LED_GREEN);
	led_off(LED_RED);
}

void led_on(int led) {
	switch (led) {
	case LED_RED: {
		LED_RED_CLEAR = (1 << led);
	}
	break;
	case LED_GREEN: {
		LED_GREEN_CLEAR = (1 << led);
	}
	break;
	}
}

void led_off(int led) {
	switch (led) {
	case LED_RED: {
		LED_RED_SET = (1 << led);
	}
	break;
	case LED_GREEN: {
		LED_GREEN_SET = (1 << led);
	}
	break;
	}
}

void led_toggle(int led) {
	switch (led) {
	case LED_RED: {
		if ((LED_RED_PIN & (1 << led))) {
			led_on(led);
		} else {
			led_off(led);
		}
	}
	break;
	case LED_GREEN: {
		if ((LED_GREEN_PIN & (1 << led))) {
			led_on(led);
		} else {
			led_off(led);
		}
	}
	break;
	}
}

