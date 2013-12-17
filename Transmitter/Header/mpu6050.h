/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony complementary filter for attitude estimation
    http://www.x-io.co.uk
*/

#include "mpu6050registers.h"

//defines to replace variables
#define SMPLRT_DIV 0x07		// 0x9F is 50 Hz

//i2c settings
#define MPU6050_I2CINIT 0 //init i2c

//enable the getattitude functions
//because we do not have a magnetometer, we have to start the chip always in the same position
//then to obtain your object attitude you have to apply the aerospace sequence
//0 disabled
//1 mahony filter
//2 dmp chip processor
#define MPU6050_GETATTITUDE 2

//definitions for raw data
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_2000
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2

#define MPU6050_GYRO_LSB_250 131.0
#define MPU6050_GYRO_LSB_500 65.5
#define MPU6050_GYRO_LSB_1000 32.8
#define MPU6050_GYRO_LSB_2000 16.4
#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_GGAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_GGAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2 16384.0
#define MPU6050_ACCEL_LSB_4 8192.0
#define MPU6050_ACCEL_LSB_8 4096.0
#define MPU6050_ACCEL_LSB_16 2048.0
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

#define MPU6050_CALIBRATEDACCGYRO 1 //set to 1 if is calibrated
#if MPU6050_CALIBRATEDACCGYRO == 1
#define MPU6050_AXOFFSET 0
#define MPU6050_AYOFFSET 0
#define MPU6050_AZOFFSET 0
#define MPU6050_AXGAIN 16384.0
#define MPU6050_AYGAIN 16384.0
#define MPU6050_AZGAIN 16384.0
#define MPU6050_GXOFFSET -88
#define MPU6050_GYOFFSET 69
#define MPU6050_GZOFFSET 52
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#endif

//definitions for attitude 1 function estimation
#if MPU6050_GETATTITUDE == 1
//setup timer0 overflow event and define madgwickAHRSsampleFreq equal to timer0 frequency
//timerfreq = (FCPU / prescaler) / timerscale
//     timerscale 8-bit = 256
// es. 61 = (16000000 / 1024) / 256
#define MPU6050_TIMER0INIT TCCR0B |=(1<<CS02)|(1<<CS00); \
		TIMSK0 |=(1<<TOIE0);
#define mpu6050_mahonysampleFreq 61.0f // sample frequency in Hz
#define mpu6050_mahonytwoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define mpu6050_mahonytwoKiDef (2.0f * 0.1f) // 2 * integral gain
#endif

#if MPU6050_GETATTITUDE == 2
//dmp definitions
//packet size
#define MPU6050_DMP_dmpPacketSize 42
//define INT0 rise edge interrupt
#define MPU6050_DMP_INT0SETUP EICRA |= (1<<ISC01) | (1<<ISC00)
//define enable and disable INT0 rise edge interrupt
#define MPU6050_DMP_INT0DISABLE EIMSK &= ~(1<<INT0)
#define MPU6050_DMP_INT0ENABLE EIMSK |= (1<<INT0)
extern volatile unsigned char mpu6050_mpuInterrupt;
#endif

//functions
void mpu6050_init(char address);
extern unsigned char mpu6050_testConnection(char address);

extern void mpu6050_getRawData(char address, short* ax, short* ay, short* az, short* gx, short* gy, short* gz);
extern void mpu6050_getConvData(char address, double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds);

extern void mpu6050_setSleepDisabled(char address);
extern void mpu6050_setSleepEnabled(char address);

#if MPU6050_GETATTITUDE == 1
extern void mpu6050_updateQuaternion(char address);
extern void mpu6050_getQuaternion(double *qw, double *qx, double *qy, double *qz);
extern void mpu6050_getRollPitchYaw(double *pitch, double *roll, double *yaw);
#endif

#if MPU6050_GETATTITUDE == 2
extern void mpu6050_setMemoryBank(char address, unsigned char bank, unsigned char prefetchEnabled, unsigned char userBank);
extern void mpu6050_setMemoryStartAddress(char address, unsigned char memAddress);
extern void mpu6050_readMemoryBlock(char address, unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char memAddress);
extern unsigned char mpu6050_writeMemoryBlock(char address, const unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char memAddress, unsigned char verify, unsigned char useProgMem);
extern unsigned char mpu6050_writeDMPConfigurationSet(char address, const unsigned char *data, unsigned short dataSize, unsigned char useProgMem);
extern unsigned short mpu6050_getFIFOCount(char address);
extern void mpu6050_getFIFOBytes(char address, unsigned char *data, unsigned char length);
extern unsigned char mpu6050_getIntStatus(char address);
extern void mpu6050_resetFIFO(char address);
extern char mpu6050_getXGyroOffset(char address);
extern void mpu6050_setXGyroOffset(char address, char offset);
extern char mpu6050_getYGyroOffset(char address);
extern void mpu6050_setYGyroOffset(char address, char offset);
extern char mpu6050_getZGyroOffset(char address);
extern void mpu6050_setZGyroOffset(char address, char offset);
//base dmp
extern unsigned char mpu6050_dmpInitialize(unsigned char address);
extern void mpu6050_dmpEnable(char address);
extern void mpu6050_dmpDisable(char address);
extern void mpu6050_getQuaternion(const unsigned char* packet, double *qw, double *qx, double *qy, double *qz);
extern void mpu6050_getRollPitchYaw(double qw, double qx, double qy, double qz, double *roll, double *pitch, double *yaw);
extern unsigned char mpu6050_getQuaternionWait(char address, double *qw, double *qx, double *qy, double *qz);
#endif
