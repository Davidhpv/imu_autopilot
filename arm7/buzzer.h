/*
 * buzzer.h
 *
 *  Created on: Jul 9, 2009
 *      Author: mavteam
 */

#ifndef BUZZER_H_
#define BUZZER_H_

/*! LED pin. This is the pin on witch the led is connected. */
#define BUZZER_PIN_NR	18
/*! GPIO direction register for LED (IO0DIR or IO1DIR) */
#define BUZZER_DIR		IO1DIR
/*! GPIO clear register for LED (IO0CLR or IO1CLR) */
#define BUZZER_CLEAR	IO1CLR
/*! GPIO set register for LED (IO0SET or IO1SET) */
#define BUZZER_SET		IO1SET
/*! GPIO pin register for LED (IO0PIN or IO1PIN) */
#define BUZZER_PIN		IO1PIN

static inline void buzzer_init(void)
{
	BUZZER_DIR |= 1 << BUZZER_PIN_NR;
	BUZZER_CLEAR |= 1 << BUZZER_PIN_NR;
}

static inline void buzzer_on(void)
{
	BUZZER_SET |= 1 << BUZZER_PIN_NR;
}

static inline void buzzer_off(void)
{
	BUZZER_CLEAR |= 1 << BUZZER_PIN_NR;
}

static inline void buzzer_toggle(void)
{
	if ((BUZZER_PIN & (1 << BUZZER_PIN_NR)))
	{
		buzzer_off();
	}
	else
	{
		buzzer_on();
	}
}

#endif /* BUZZER_H_ */
