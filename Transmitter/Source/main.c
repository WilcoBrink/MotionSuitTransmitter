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

#define USE_UART 1

extern	char Received_Dataext[42];
void init_interrupt(void);
void mpu6050_interrupt(void);
extern  void __enable_interrupts(void);
extern  void __disable_interrupts(void);
short *calibratie (void);

/******************************************************************************\
 main
\******************************************************************************/
extern int main(void)
{ 
	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;

	short ax = 0;
	short ay = 0;
	short az = 0;
	short gx = 0;
	short gy = 0;
	short gz = 0;

	short CalibratieAcc[3];
	short *pCalibratieAcc;
	#if USE_UART == 1
	int sendQ[4];
	unsigned short sendA[3];
	#endif

	int temp = 0;
	char string[23];
	string[23] = '\0';
	char *pString;
	pString=&string[0];
	/*   init libs   */
	PLL_init();
	SPI_init(0x0F);
	#if USE_UART == 1
	UART_init();
	#endif
	i2cInit();
	UART_init();
	MRF24J40_init(0xABBA);
	MRF24J40_wake();
	delay_s(2);
	init_interrupt();
	mpu6050_init(0x68);
	mpu6050_dmpInitialize(0x68);
	//mpu6050_calibrate(0x68);
	mpu6050_dmpEnable(0x68);
	delay_ms(10);
	__enable_interrupts();

	pCalibratieAcc=calibratie ();
	CalibratieAcc[0]=*pCalibratieAcc;
	pCalibratieAcc++;
	CalibratieAcc[1]=*pCalibratieAcc;
	pCalibratieAcc++;
	CalibratieAcc[2]=*pCalibratieAcc;

	///////////////////// Zend continu een pakketje ///////////////////
	while(1)
 	{
		if(mpu6050_getQuaternionWait(0x68, &qw, &qx, &qy, &qz)) {
			mpu6050_getRawData(0x68, &ax, &ay, &az, &gx, &gy, &gz);
			ax=ax-CalibratieAcc[0];
			ay=ay-CalibratieAcc[1];
			az=az-CalibratieAcc[2];
			int i,j;
			for(j = 0; j < 4; j++){
				switch(j){
					case 0:
						temp = (int)(qw * 10000000.0);
						break;
					case 1:
						temp = (int)(qx * 10000000.0);
						break;
					case 2:
						temp = (int)(qy * 10000000.0);
						break;
					case 3:
						temp = (int)(qz * 10000000.0);
						break;
				}
				for(i = 0; i < 4; i++){
					string[i+(j*4)] = temp>>(8*i);
				}
			}
			string[16] = ax>>8;
			string[17] = ax;
			string[18] = ay>>8;
			string[19] = ay;
			string[20] = az>>8;
			string[21] = az;

			#if USE_UART == 1
			sendQ[0] = (int)(qw * 10000000.0);
			sendQ[1] = (int)(qx * 10000000.0);
			sendQ[2] = (int)(qy * 10000000.0);
			sendQ[3] = (int)(qz * 10000000.0);
			sendA[0] = ax;
			sendA[1] = ay;
			sendA[2] = az;

			for (i=0;i<4;i++)			// Loop to send the 6 sensor values to the computer
			{
				UART_putint(sendQ[i]);
				UART_put(",");			// Comma used as split token in Processing
			}
			for (i=0;i<3;i++)			// Loop to send the 6 sensor values to the computer
			{
				UART_putint(sendA[i]);
				UART_put(",");			// Comma used as split token in Processing
			}
			UART_put("\n");				// New line used by Processing to recognize end of transmission
			#endif

			#if USE_UART != 1
			MRF24J40_send(pString, 22, 0xAABB);
			MRF24J40_receive();
			#endif
		}
		delay_ms(20);
 	}
	return 0;									// don't ever come near this
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

short *calibratie (void)
{
	short ax = 0;
	short ay = 0;
	short az = 0;
	short gx = 0;
	short gy = 0;
	short gz = 0;
	int x=0;
	int y=0;
	int z=0;
	int i=0;
	short CalibratieAcc[3];
	short *pCalibratieAcc;
	pCalibratieAcc=&CalibratieAcc[0];

	for(i=0;i<10;i++)
	{
	mpu6050_getRawData(0x68, &ax, &ay, &az, &gx, &gy, &gz);
	x=x+ax;
	y=y+ay;
	z=z+az;
	}
	CalibratieAcc[0]=x/10;
	CalibratieAcc[1]=y/10;
	CalibratieAcc[2]=(z/10)-16384;
return pCalibratieAcc;
}

