/*
    ARM IP interface

    leds.h:
           LED-driver testboard

*/

#ifndef LEDS_H
#define LEDS_H

//#include <config.h>

#define LED_RCK  16
#define LED_SCK  4
#define LED_SER  6

void LED_init(void);
void LED_put(unsigned short led_byte);

#endif /* LEDS_H */
