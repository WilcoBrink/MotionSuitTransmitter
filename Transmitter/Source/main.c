/******************************************************************************\
 main.c
\******************************************************************************/

/******************************************************************************\
 Include files
\******************************************************************************/
#include "LPC214x.h"
#include "pll.h"
#include "MRF24J40.h"
#include "spi.h"
#include "delay.h"
#include "lcd.h"
#include "keys.h"
#include "leds.h"
#include "I2C.h"
#include "uart.h"
#include "mpu6050.h"

extern unsigned char received_data[42];

extern  void __enable_interrupts();
extern  void __disable_interrupts();

/******************************************************************************\
 main
\******************************************************************************/
extern int main()
{ 
	signed char test;
	int data;
	unsigned int key;
	
	/*   init libs   */
	PLL_init();
	//LED_init();
	InitKeys();
	i2c_init();
	UART_init();
	mpu6050_init();
	MRF24J40_init(0x0040);
	//LED_put(0x00);
	
	///////////////////// Zend continu een pakketje ///////////////////
	while(1)
 	{
 	if (key_0==ReadKeys())
 	{
 		while (0 != ReadKeys());
		 
 		
		//write_byte(0x68, 0x1C, 0x18);
		
		//if(key == key_0)
		//{
			test = read_byte(0x68, MPU6050_ACCEL_XOUT_H);		// Accelerometer x_as uitlezen
			UART_putchar(test);
			UART_put(" ");
			key = key_9;
		//}
		
		//test = read_byte(0x68, MPU6050_ACCEL_XOUT_H);		// Accelerometer x_as uitlezen
		//UART_putchar(test);
 		
		//data = read_axis();
 		//UART_putint(data);
 		//UART_put(" ");
 		
 		//delay_ms(20);
 		
 		}
	}

	return 0;									// don't ever come near this
}
