/*
    ARM IP interface

    leds.c:
           LED-driver testboard

    pin-info:
           P2 - RCK: Storage register clock
           P4 - SCK: Shift register clock
           P6 - SER: Data input

    opmerkingen:
                -pin 4 wordt door de SPI, de toetsen en de leds gebruikt. Pin 6
                 wordt door de SPI en de leds gebruikt.

*/

#include "leds.h"
#include "LPC214x.h"

void LED_init(void) 
/*
  Stelt gebruikte I/O pinnen in.
*/
{
     PINSEL0 &= (~(3<<LED_RCK*2)) & (~(3<<LED_SCK*2)) & (~(3<<LED_SER*2));
     IODIR0   |= (1<<LED_RCK)      | (1<<LED_SCK)      | (1<<LED_SER);
     IOCLR0   =  (1<<LED_RCK)      | (1<<LED_SCK)      | (1<<LED_SER);
}

void LED_put(unsigned short led_byte) 
/*
  Zet de meegegeven waarde op de LED's
*/
{
     unsigned short i;

		
     /* Data uitklokken (wordt in buffer van schuifregister geplaatst) */
     for(i=128; i>=1; i>>=1) {
          if(led_byte & i) {
               IOSET0 = 1<<LED_SER;
          }
          else {
               IOCLR0 = 1<<LED_SER;
          }
          IOSET0 = 1<<LED_SCK;
          IOCLR0 = 1<<LED_SCK;
     }
     /* Data vanuit buffer op uitgangen zetten */
     IOSET0 = 1<<LED_RCK;
     IOCLR0 = 1<<LED_RCK;
}
