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

extern	char Received_Dataext[42];

extern  void __enable_interrupts(void);
extern  void __disable_interrupts(void);

/******************************************************************************\
 main
\******************************************************************************/
extern int main(void)
{ 

	char string[13];
	char *pString;
	pString=&string[0];
	/*   init libs   */
	PLL_init();
	SPI_init(0x0F);
	//LED_init();
	InitKeys();
	i2c_init();
	UART_init();
	mpu6050_init();

	MRF24J40_init(0xABBA);
	//MRF24J40_wake();
	delay_s(2);
	//LED_put(0x00);
	
	///////////////////// Zend continu een pakketje ///////////////////
	string[12] = '\0';
	while(1)
 	{
		string[0] = read_byte(0x68, MPU6050_ACCEL_XOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[1] = read_byte(0x68, MPU6050_ACCEL_XOUT_L);
		//delay_ms(50);
		string[2] = read_byte(0x68, MPU6050_ACCEL_YOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[3] = read_byte(0x68, MPU6050_ACCEL_YOUT_L);
		//delay_ms(50);
		string[4] = read_byte(0x68, MPU6050_ACCEL_ZOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[5] = read_byte(0x68, MPU6050_ACCEL_ZOUT_L);
		//delay_ms(50);
		string[6] = read_byte(0x68, MPU6050_GYRO_XOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[7] = read_byte(0x68, MPU6050_GYRO_XOUT_L);
		//delay_ms(50);
		string[8] = read_byte(0x68, MPU6050_GYRO_YOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[9] = read_byte(0x68, MPU6050_GYRO_YOUT_L);
		//delay_ms(50);
		string[10] = read_byte(0x68, MPU6050_GYRO_ZOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[11] = read_byte(0x68, MPU6050_GYRO_ZOUT_L);
		delay_ms(50);
		//string[0], string[2], string[4], string[6], string[8], string[10] = 0x30;
		//string[1], string[3], string[5], string[7], string[9], string[11] = 0x39;
		MRF24J40_send_string(pString,0xAABB);					// gaat dit wel goed?
		
 	}


	return 0;									// don't ever come near this
}
