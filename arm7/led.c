#include "LPC21xx.h"
#include "led.h"
#include "conf.h"


#if defined(IMU_PIXHAWK_V210)
void led_init(void) {
	PINSEL2 &= ~(1<<3); //TODO Hack, this is for setting P16-P23 as GPIO ports
	LED_RED_DIR |= (1 << LED_RED);
	LED_GREEN_DIR |= (1 << LED_GREEN);
	LED_YELLOW_DIR |= (1 << LED_YELLOW);
	led_off(LED_GREEN);
	led_off(LED_RED);
	led_off(LED_YELLOW);
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
	case LED_YELLOW: {
			LED_YELLOW_CLEAR = (1 << led);
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
	case LED_YELLOW: {
			LED_YELLOW_SET = (1 << led);
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
	case LED_YELLOW: {
			if ((LED_YELLOW_PIN & (1 << led))) {
				led_on(led);
			} else {
				led_off(led);
			}
		}
		break;
	}
}

#endif

#if defined(IMU_PIXHAWK_V200)
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

#endif


#ifdef BOARD_TWOG_BOOZ
void led_init(void) {
//	LED_YELLOW_DIR |= (1 << LED_YELLOW);
	LED_RED_DIR |= (1 << LED_RED);
	LED_GREEN_DIR |= (1 << LED_GREEN);
	led_off(LED_GREEN);
	led_off(LED_RED);
//	led_off(LED_YELLOW);
}

void led_on(int led) {
	switch (led) {
	case LED_RED: {
		LED_RED_CLEAR = (1 << led);
	}
	break;
//	case LED_YELLOW: {
//		LED_YELLOW_CLEAR = (1 << led);
//	}
//	break;
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
//	case LED_YELLOW: {
//		LED_YELLOW_SET = (1 << led);
//	}
//	break;
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
//	case LED_YELLOW: {
//		if ((LED_YELLOW_PIN & (1 << led))) {
//			led_on(led);
//		} else {
//			led_off(led);
//		}
//	}
//	break;
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
#endif


