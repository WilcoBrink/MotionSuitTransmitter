#include "mpu6050.h"
#include "I2C.h"
#include "vic.h"
#include "LPC214x.h"
#include "uart.h"
#include "leds.h"

extern unsigned char received_data[42]="data";

// Vic setup
#define MPU6050_PRIOR			1
#define EINT1_INT_SLOT		15
#define MPU6050_EINT		0x01

void mpu6050_init()
{
	//Sets sample rate to 8000/1+7 = 1000Hz
    write_byte(0x68, MPU6050_SMPLRT_DIV, 0x9F);		//50Hz
    //Disable FSync, 256Hz DLPF
    write_byte(0x68, MPU6050_CONFIG, 0x00);
    //Disable gyro self tests, scale of 500 degrees/s
    write_byte(0x68, MPU6050_GYRO_CONFIG, 0x08);
    //Disable accel self tests, scale of +-2g, no DHPF
    write_byte(0x68, MPU6050_ACCEL_CONFIG, 0x00);
    //Freefall threshold of |0mg|
    write_byte(0x68, MPU6050_FF_THR, 0x00);
    //Freefall duration limit of 0
    write_byte(0x68, MPU6050_FF_DUR, 0x00);
    //Motion threshold of 0mg
    write_byte(0x68, MPU6050_MOT_THR, 0x00);
    //Motion duration of 0s
    write_byte(0x68, MPU6050_MOT_DUR, 0x00);
    //Zero motion threshold
    write_byte(0x68, MPU6050_ZRMOT_THR, 0x00);
    //Zero motion duration threshold
    write_byte(0x68, MPU6050_ZRMOT_DUR, 0x00);
    //Disable sensor output to FIFO buffer
    write_byte(0x68, MPU6050_FIFO_EN, 0x00); 
    write_byte(0x68, MPU6050_PWR_MGMT_1, 0x00);   
    //write_byte(0x68, MPU6050_SIGNAL_PATH_RESET, 0x07);  

	//PINSEL0 |= 0x20000000;
    /*EXTMODE = (1 << 1);				// EINT1 edge sensitive
	EXTPOLAR |= (1 << 1);				// EINT1 falling edge sensitive
	EXTINT = 0xf;*/
	//VicSetup((unsigned)mpu6050_interrupt,IRQ,EINT1_INT_SLOT,MPU6050_PRIOR);
	
	//write_byte(0x68, MPU6050_INT_PIN_CFG, 0x20);
	//write_byte(0x68, MPU6050_INT_ENABLE, 0x00);
}

void mpu6050_interrupt()
{
	int data;
	LED_put(0xFF);
	data = read_axis();
	UART_putint(data);
	EXTINT = 0x00000002;
	VICVectAddr = 0x00000000;
	LED_put(0x00);
}

void mpu6050_reset()
{
	write_byte(0x68, MPU6050_PWR_MGMT_1, 0x80);
}
