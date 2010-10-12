/**
* @file led.h
*
* @brief functions to use the leds
*
* This file contains different functions to use leds.
*
**/

#ifndef LED_H
#define LED_H

/** @addtogroup LED */
//@{
/** @name LED functions
 *  Pin settings for the YELLOW LED. */
//@{

/**
 * @brief initialization of the leds.
 * This function initializes the led GPIO port as outputs and turn
 * all leds of. After you have called this function you can use all
 * other led functions. *
 */
void led_init(void);

/**
 * @brief turn led on.
 * This function turns the led on. The leds are active low,
 * thats why the gpio port is set to low if led_on is called.
 */
void led_on(int led);

/**
 * @brief turn led off.
 * This function turns the led off. The leds are active low,
 * thats why the gpio port is set to high if led_off is called.
 */
void led_off(int led);

/**
 * @brief toggle led.
 * This function looks up if the led is on or off and then switches
 * the state.
 */
void led_toggle(int led);

//@}
//@}

#endif /* LED_H */
