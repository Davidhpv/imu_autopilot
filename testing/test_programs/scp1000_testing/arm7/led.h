#ifndef LED_H
#define LED_H

// define LED-Pins as outputs
void led_init(void);
void led_on(int led);
void led_off(int led);
void led_toggle(int led);

#endif /* LED_H */
