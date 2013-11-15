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
void init_timer(void);
void TimerStartTwee(void);
void TimerResetTwee(void);
void TimerStopTwee(void);
extern  void __enable_interrupts(void);
extern  void __disable_interrupts(void);

/******************************************************************************\
 main
\******************************************************************************/
extern int main(void)
{ 

	char string[25];
	char *pString;
	pString=&string[0];
	/*   init libs   */
	PLL_init();
	SPI_init(0x0F);
	//LED_init();
	InitKeys();
	i2c_init();
	UART_init();
	mpu6050_init(0x68);
	mpu6050_init(0x69);
	init_timer();

	MRF24J40_init(0xABBA);
	//MRF24J40_wake();
	delay_s(2);
	//LED_put(0x00);
	
	///////////////////// Zend continu een pakketje ///////////////////
	string[24] = '\0';
	TimerStartTwee();
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

		string[12] = read_byte(0x69, MPU6050_ACCEL_XOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[13] = read_byte(0x69, MPU6050_ACCEL_XOUT_L);
		//delay_ms(50);
		string[14] = read_byte(0x69, MPU6050_ACCEL_YOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[15] = read_byte(0x69, MPU6050_ACCEL_YOUT_L);
		//delay_ms(50);
		string[16] = read_byte(0x69, MPU6050_ACCEL_ZOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[17] = read_byte(0x69, MPU6050_ACCEL_ZOUT_L);
		//delay_ms(50);
		string[18] = read_byte(0x69, MPU6050_GYRO_XOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[19] = read_byte(0x69, MPU6050_GYRO_XOUT_L);
		//delay_ms(50);
		string[20] = read_byte(0x69, MPU6050_GYRO_YOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[21] = read_byte(0x69, MPU6050_GYRO_YOUT_L);
		//delay_ms(50);
		string[22] = read_byte(0x69, MPU6050_GYRO_ZOUT_H);		// Accelerometer x_as uitlezen
		//delay_ms(50);
		string[23] = read_byte(0x69, MPU6050_GYRO_ZOUT_L);

		//string[0], string[2], string[4], string[6], string[8], string[10] = 0x30;
		//string[1], string[3], string[5], string[7], string[9], string[11] = 0x39;
		if (T1IR==1)
		{
			TimerResetTwee();
		MRF24J40_send_string(pString,0xAABB);
		// gaat dit wel goed?
		}
 	}


	return 0;									// don't ever come near this
}




void init_timer()
{
	T1TC = 0; //maak de timer zelf 0
	T1PC =0;// maak de prescaler zelf 0
	T1MR0=200; 	//instellen waarde timer en prescaler
	T1PR=60000; //Kloksnelheid
	T1MCR = 0x0003;
	T1EMR =0;// maak geen gebruik van  match acties..
	T1CCR =0;// we laden geen capture waardes in
	T1IR=1; // reset value interupt flag =1!!!
}

void TimerStartTwee(void){
// deze routine start de timer..
	T1TCR=0x01;
}

void TimerStopTwee(void){
	T1TCR=0x00; // stop timer
	T1TCR=0x10; // reset timer (moet nog in de interrupt routine voor ontvangst)
}

void TimerResetTwee(void)
{
	T1TC = 0;
	T1IR=1;
}
