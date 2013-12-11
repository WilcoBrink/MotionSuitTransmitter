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
#include "vic.h"
#include "math.h"

extern	char Received_Dataext[42];
void init_interrupt(void);
void mpu6050_interrupt(void);
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
	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;

	short ax = 0;
	short ay = 0;
	short az = 0;
	short gx = 0;
	short gy = 0;
	short gz = 0;

	/*char string[25];
	char *pString;
	pString=&string[0];*/
	/*   init libs   */
	PLL_init();
	SPI_init(0x0F);
	//LED_init();
	//InitKeys();
	i2cInit();
	UART_init();
	MRF24J40_init(0xABBA);
	//MRF24J40_wake();
	delay_s(2);
	
	//init_timer();
	init_interrupt();

	mpu6050_init(0x68);
	mpu6050_dmpInitialize(0x68);
	mpu6050_dmpEnable(0x68);
	delay_ms(10);
	__enable_interrupts();

	///////////////////// Zend continu een pakketje ///////////////////
	//string[24] = '\0';
	//TimerStartTwee();
	while(1)
 	{
		if(mpu6050_getQuaternionWait(0x68, &qw, &qx, &qy, &qz)) {
			__disable_interrupts();
			mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
			mpu6050_getRawData(0x68, &ax, &ay, &az, &gx, &gy, &gz);
			//UART_putint((int)qw*1000);
			//UART_putint((int)qx*1000);
			//UART_putint((int)qy*1000);
			//UART_putint((int)qz*1000);
			//UART_put("/n");
			//UART_putint((int)((roll*180)/M_PI));
			//UART_putint((int)((pitch*180)/M_PI));
			//UART_putint((int)((yaw*180)/M_PI));
			//UART_putint(1000);
			//UART_put("/n");
			__enable_interrupts();
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

void init_interrupt(void)
{
	__disable_interrupts();
	EnableVectorInt(14);
	PINSEL0 |= (3<<2);			// pin 1 op EINT0 mode
	EXTINT &= 0x0F;
	EXTMODE = 0x01;
	EXTPOLAR = 0x01;
	VicSetup((unsigned int)mpu6050_interrupt, 0, 14, 14);
}

void mpu6050_interrupt(void)
{
	mpu6050_mpuInterrupt = 0x0001;
	EXTINT &= 0x0F;
}
