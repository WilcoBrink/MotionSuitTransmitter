/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

/*
 * This file contains all the functions needed to process 6-axis orientation by the internal chip processor
 */

//to enable get roll, pitch and yaw function we include the math function

#include "mpu6050.h"
#include "mpu6050dmpcode.h"
#include "I2C.h"
#include "delay.h"
#include "math.h"

#if MPU6050_GETATTITUDE == 2

//#include <avr/pgmspace.h>

#define MPU6050_DMP_CODE_SIZE 1929
#define MPU6050_DMP_CONFIG_SIZE 192
#define MPU6050_DMP_UPDATES_SIZE 47

volatile unsigned char mpu6050_mpuInterrupt = 0;
unsigned char *dmpPacketBuffer;
unsigned short mpu6050_fifoCount = 0;
unsigned char mpu6050_mpuIntStatus = 0;
unsigned char mpu6050_fifoBuffer[64];

/*
 * initialize mpu6050 dmp
 */
unsigned char mpu6050_dmpInitialize(unsigned char address) {
	//setup interrupt
	//MPU6050_DMP_INT0SETUP;

	//reset
	i2cWriteBit(address, MPU6050_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
    delay_ms(30);//wait after reset

    //disable sleep mode
    mpu6050_setSleepDisabled(address);

    //set memorybank to 0
    mpu6050_setMemoryBank(address, 0, 0, 0);

    //get X/Y/Z gyro offsets
    char xgOffset = mpu6050_getXGyroOffset(address);
    char ygOffset = mpu6050_getYGyroOffset(address);
    char zgOffset = mpu6050_getZGyroOffset(address);

    //setting slave 0 address to 0x7F
    unsigned char temp = 0x7F;
	i2cWrite(address, MPU6050_I2C_SLV0_ADDR + 0*3, &temp, 1);
	//disabling I2C Master mode
	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, 0);
	//setting slave 0 address to 0x68 (self)
	i2cWrite(address, MPU6050_I2C_SLV0_ADDR + 0*3, &address, 1);
	//resetting I2C Master control
	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
	delay_ms(20);

    //load DMP code into memory banks
    if (mpu6050_writeMemoryBlock(address, mpu6050_dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, 0, 0) == 1) {
    	//unsigned char verifyBuffer[137];
    	//mpu6050_setMemoryBank(address, 7, 0, 0);
    	//mpu6050_setMemoryStartAddress(address, 0);
    	//i2cRead(address, MPU6050_MEM_R_W, verifyBuffer, 137);
        if (mpu6050_writeDMPConfigurationSet(address, mpu6050_dmpConfig, MPU6050_DMP_CONFIG_SIZE, 0)) {
        	//enable FIFO
        	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, 1);

        	//set clock source
        	i2cWriteBits(address, MPU6050_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_ZGYRO);

        	//set DMP and FIFO_OFLOW interrupts enabled
        	temp = 0x12;
        	i2cWrite(address, MPU6050_INT_ENABLE, &temp, 1);

            //set sample rate
        	temp = 4;
        	i2cWrite(address, MPU6050_SMPLRT_DIV, &temp, 1); // 1khz / (1 + 4) = 200 Hz

            //set external frame sync to TEMP_OUT_L[0]
            i2cWriteBits(address, MPU6050_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, MPU6050_EXT_SYNC_TEMP_OUT_L);

            //set DLPF bandwidth to 42Hz
            i2cWriteBits(address, MPU6050_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);

            //set gyro sensitivity to +/- 2000 deg/sec
            i2cWriteBits(address, MPU6050_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);

            //set DMP configuration bytes (function unknown)
            temp = 0x03;
            i2cWrite(address, MPU6050_DMP_CFG_1, &temp, 1);
            temp = 0x00;
            i2cWrite(address, MPU6050_DMP_CFG_2, &temp, 1);

            //clear OTP Bank flag
            i2cWriteBit(address, MPU6050_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, 0);

            //set X/Y/Z gyro offsets to previous values
            //xgOffset = 0;
            //ygOffset = 0;
            //zgOffset = 90;

            mpu6050_setXGyroOffset(address, xgOffset);
            mpu6050_setYGyroOffset(address, ygOffset);
            mpu6050_setZGyroOffset(address, zgOffset);

            //set X/Y/Z gyro user offsets to zero
            //i2cWrite(address, MPU6050_XG_OFFS_USRH, 0, 2);
            //i2cWrite(address, MPU6050_YG_OFFS_USRH, 0, 2);
            //i2cWrite(address, MPU6050_ZG_OFFS_USRH, 0, 2);

            //writing final memory update 1/7 (function unknown)
            unsigned char dmpUpdate[16], j;
            unsigned short pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //writing final memory update 2/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //reset FIFO
            i2cWriteBits(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1, 1);

            //reading FIFO count
            unsigned short fifoCount = mpu6050_getFIFOCount(address);
            unsigned char fifoBuffer[128];

            //current FIFO count
            i2cRead(address, MPU6050_FIFO_R_W, fifoBuffer, 2 /*fifoCount*/);

            //setting motion detection threshold to 2
            temp = 2;
            i2cWrite(address, MPU6050_MOT_THR, &temp, 1);

            //setting zero-motion detection threshold to 156
            temp = 156;
            i2cWrite(address, MPU6050_ZRMOT_THR, &temp, 1);

            //setting motion detection duration to 80
            temp = 80;
            i2cWrite(address, MPU6050_MOT_DUR, &temp, 1);

            //setting zero-motion detection duration to 0
            i2cWrite(address, MPU6050_ZRMOT_DUR, 0, 1);

            //reset FIFO
            i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);

            //enabling FIFO
            i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, 1);

            //enabling DMP
            mpu6050_dmpEnable(address);

            //resetting DMP
            i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);

            //waiting for FIFO count > 2
            //fifoCount = mpu6050_getFIFOCount(address);
            while ((fifoCount = mpu6050_getFIFOCount(address)) < 3);

            //writing final memory update 3/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //writing final memory update 4/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //writing final memory update 5/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //reading FIFO data..."));
            mpu6050_getFIFOBytes(address, fifoBuffer, fifoCount);

            //reading final memory update 6/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_readMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            //waiting for FIFO count > 2
            while ((fifoCount = mpu6050_getFIFOCount(address)) < 3);

            //reading FIFO data
            mpu6050_getFIFOBytes(address, fifoBuffer, fifoCount);

            //writing final memory update 7/7 (function unknown)
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = mpu6050_dmpUpdates[pos];
            mpu6050_writeMemoryBlock(address, dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], 0, 0);

            //disabling DMP (you turn it on later)
            mpu6050_dmpDisable(address);

            //resetting FIFO and clearing INT status one last time
            i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
        } else {
            return 2; // configuration block loading failed
        }
    } else {
        return 1; // main binary block loading failed
    }
    return 0; // success
}

/*
 * enable dmp
 */
void mpu6050_dmpEnable(char address) {
	//MPU6050_DMP_INT0ENABLE;
	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, 1);
}

/*
 * disable dmp
 */
void mpu6050_dmpDisable(char address) {
	//MPU6050_DMP_INT0DISABLE;
	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, 0);
}

/*
 * get quaternion from packet
 */
void mpu6050_getQuaternion(const unsigned char* packet, double *qw, double *qx, double *qy, double *qz) {
	if (packet == 0) packet = dmpPacketBuffer;
	char packetCopy[14];
	int i;
	for(i = 0; i<14;i++) {
	packetCopy[i] = packet[i];
	}
    short temp = (short)((packetCopy[0] << 8) + packetCopy[1]); /// 16384.0f;
    *qw = temp / 16384.0f;
    temp = (short)((packetCopy[4] << 8) + packetCopy[5]); /// 16384.0f;
    *qx = temp/16384.0f;
    temp = (short)((packetCopy[8] << 8) + packetCopy[9]); /// 16384.0f;
    *qy = temp/16384.0f;
    temp = (short)((packetCopy[12] << 8) + packetCopy[13]); /// 16384.0f;
    *qz = temp/16384.0f;
}

/*
 * get euler angles
 * aerospace sequence, to obtain sensor attitude:
 * 1. rotate around sensor Z plane by yaw
 * 2. rotate around sensor Y plane by pitch
 * 3. rotate around sensor X plane by roll
 */
void mpu6050_getRollPitchYaw(double qw, double qx, double qy, double qz, double *roll, double *pitch, double *yaw) {
	*yaw = atan2(2*qx*qy - 2*qw*qz, 2*qw*qw + 2*qx*qx - 1);
	*pitch = -asin(2*qx*qz + 2*qw*qy);
	*roll = atan2(2*qy*qz - 2*qw*qx, 2*qw*qw + 2*qz*qz - 1);
}

/*
 * get quaternion and wait
 */
unsigned char mpu6050_getQuaternionWait(char address, double *qw, double *qx, double *qy, double *qz) {
	while (!mpu6050_mpuInterrupt && mpu6050_fifoCount < MPU6050_DMP_dmpPacketSize);
	//reset interrupt
	mpu6050_mpuInterrupt = 0;

	//check for overflow
	mpu6050_mpuIntStatus = mpu6050_getIntStatus(address);
	mpu6050_fifoCount = mpu6050_getFIFOCount(address);
	if ((mpu6050_mpuIntStatus & 0x10) || mpu6050_fifoCount == 1024) {
		//reset
		mpu6050_resetFIFO(address);
	} else if (mpu6050_mpuIntStatus & 0x02) {
		//wait for correct available data length, should be a VERY short wait
		while (mpu6050_fifoCount < MPU6050_DMP_dmpPacketSize)
			mpu6050_fifoCount = mpu6050_getFIFOCount(address);
		//read a packet from FIFO
		mpu6050_getFIFOBytes(address, mpu6050_fifoBuffer, MPU6050_DMP_dmpPacketSize);
		mpu6050_fifoCount -= MPU6050_DMP_dmpPacketSize;
		//get quaternion
		mpu6050_getQuaternion(mpu6050_fifoBuffer, qw, qx, qy, qz);
		return 1;
	}

	return 0;
}

#endif
