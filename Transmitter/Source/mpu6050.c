/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#include "mpu6050.h"
#include "mpu6050registers.h"
#include "I2C.h"
#include "LPC214x.h"
#include "delay.h"

volatile unsigned char buffer[14];

void mpu6050_init(char address)
{
	#if MPU6050_I2CINIT == 1
	//init i2c
	i2c_init();
	delay_us(10);
	#endif
	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	i2cWriteBits(address, MPU6050_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	i2cWriteBits(address, MPU6050_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
	//Sets sample rate to 8000/1+7 = 1000Hz
	unsigned char divider = SMPLRT_DIV;
    i2cWrite(address, MPU6050_SMPLRT_DIV, &divider, 1);		//50Hz
    //set gyro range
    i2cWriteBits(address, MPU6050_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
    //set accel range
    i2cWriteBits(address, MPU6050_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);
    //set sleep disabled
    mpu6050_setSleepDisabled(address);
}

#if MPU6050_GETATTITUDE == 2
/*
 * set a chip memory bank
 */
void mpu6050_setMemoryBank(char address, unsigned char bank, unsigned char prefetchEnabled, unsigned char userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    i2cWrite(address, MPU6050_BANK_SEL, &bank, 1);
}

/*
 * set memory start address
 */
void mpu6050_setMemoryStartAddress(char address, unsigned char memAddress) {
	i2cWrite(address, MPU6050_MEM_START_ADDR, &memAddress, 1);
}

/*
 * read a memory block
 */
void mpu6050_readMemoryBlock(char address, unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char memAddress) {
	mpu6050_setMemoryBank(address, bank, 0, 0);
	mpu6050_setMemoryStartAddress(address, memAddress);
    unsigned char chunkSize;
    unsigned short i = 0;
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        i2cRead(address, MPU6050_MEM_R_W, data + i, chunkSize);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // unsigned char automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(address, bank, 0, 0);
            mpu6050_setMemoryStartAddress(address, memAddress);
        }
    }
}

/*
 * write a memory block
 */
unsigned char mpu6050_writeMemoryBlock(char address, const unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char memAddress, unsigned char verify, unsigned char useProgMem) {
	mpu6050_setMemoryBank(address, bank, 0, 0);
	mpu6050_setMemoryStartAddress(address, memAddress);
    unsigned char chunkSize;
    //unsigned char *verifyBuffer = 0;
    unsigned char progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
    unsigned char *pProgBuffer;
    pProgBuffer = &progBuffer[0];
    unsigned short i;
    unsigned char j;
    //if (verify) verifyBuffer = (unsigned char *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    //if (useProgMem) progBuffer = (unsigned char *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - memAddress) chunkSize = 256 - memAddress;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = 1; //pgm_read_byte(data + i + j);
        } else {
        	// write  the chunck of data as specified
        	for (j = 0; j < chunkSize; j++) {
        		progBuffer[j] = *data;
        		data++;
        	}
        }

        i2cWrite(address, MPU6050_MEM_R_W, pProgBuffer, chunkSize);

        /* // verify data if needed
        if (verify && verifyBuffer) {
        	mpu6050_setMemoryBank(address, bank, 0, 0);
            mpu6050_setMemoryStartAddress(address, memAddress);
            i2c_read_bytes(address, MPU6050_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return 0; // uh oh.
            }
        }*/

        // increase byte index by [chunkSize]
        i += chunkSize;

        // unsigned char automatically wraps to 0 at 256
        memAddress += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (memAddress == 0) bank++;
            mpu6050_setMemoryBank(address, bank, 0, 0);
            mpu6050_setMemoryStartAddress(address, memAddress);
        }
    }
    //if (verify) free(verifyBuffer);
    //if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * write a dmp configuration set
 */
unsigned char mpu6050_writeDMPConfigurationSet(char address, const unsigned char *data, unsigned short dataSize, unsigned char useProgMem) {
	unsigned char progBuffer[8];
	unsigned char *pProgBuffer;
	pProgBuffer = &progBuffer[0];
    unsigned char success, special;
    unsigned short i, j;
    //if (useProgMem) {
    //    progBuffer = (unsigned char *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    //}

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    unsigned char bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem) {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        } else {
            bank = *data; data++; i++;
            offset = *data; data++; i++;
            length = *data; data++; i++;
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            if (useProgMem) {
                //if (sizeof(progBuffer) < length) progBuffer = (unsigned char *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = 1; //pgm_read_byte(data + i + j);
            } else {
                for (j = 0; j < length; j++) {
                	progBuffer[j] = *data;
                	data++;
                }
            }
            success = mpu6050_writeMemoryBlock(address, pProgBuffer, length, bank, offset, 0, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = 1; //pgm_read_byte(data + i++);
            } else {
                special = *data; data++; i++;
            }
            if (special == 0x01) {
                // enable DMP-related interrupts

            	//write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled
            	//write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled
            	//write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled
            	unsigned char temp = 0x32;
            	i2cWrite(address, MPU6050_INT_ENABLE, &temp, 1);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }

        if (!success) {
            //if (useProgMem) free(progBuffer);
            return 0; // uh oh
        }
    }
    //if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * get the fifo count
 */
unsigned short mpu6050_getFIFOCount(char address) {
	unsigned char buffer_h = 0; i2cRead(address, MPU6050_FIFO_COUNTH, &buffer_h, 1);
	unsigned char buffer_l = 0; i2cRead(address, MPU6050_FIFO_COUNTL, &buffer_l, 1);
	unsigned short data = (buffer_h << 8) + buffer_l;
    return data;
}

/*
 * read fifo bytes
 */
void mpu6050_getFIFOBytes(char address, unsigned char *data, unsigned char length) {
	i2cRead(address, MPU6050_FIFO_R_W, data, length);
}

/*
 * get the interrupt status
 */
unsigned char mpu6050_getIntStatus(char address) {
	unsigned char data = 0; i2cRead(address, MPU6050_INT_STATUS, &data, 1);
    return data;
}

/*
 * reset fifo
 */
void mpu6050_resetFIFO(char address) {
	i2cWriteBit(address, MPU6050_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/*
 * get gyro offset X
 */
char mpu6050_getXGyroOffset(char address) {
	i2cReadBits(address, MPU6050_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (unsigned char *)buffer);
    return buffer[0];
}

/*
 * set gyro offset X
 */
void mpu6050_setXGyroOffset(char address, char offset) {
	i2cWriteBits(address, MPU6050_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Y
 */
char mpu6050_getYGyroOffset(char address) {
	i2cReadBits(address, MPU6050_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (unsigned char *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Y
 */
void mpu6050_setYGyroOffset(char address, char offset) {
	i2cWriteBits(address, MPU6050_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Z
 */
char mpu6050_getZGyroOffset(char address) {
	i2cReadBits(address, MPU6050_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (unsigned char *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Z
 */
void mpu6050_setZGyroOffset(char address, char offset) {
	i2cWriteBits(address, MPU6050_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
#endif

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled(char address) {
	i2cWriteBit(address, MPU6050_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled(char address) {
	i2cWriteBit(address, MPU6050_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/*
 * test connection to chip
 */
unsigned char mpu6050_testConnection(char address) {
	i2cReadBits(address, MPU6050_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (unsigned char *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

//can not accept many request if we alreay have getattitude requests
/*
 * get raw data
 */
void mpu6050_getRawData(char address, short* ax, short* ay, short* az, short* gx, short* gy, short* gz) {
	i2cRead(address, MPU6050_ACCEL_XOUT_H, (unsigned char *)buffer, 14);

	*ax = (((short)buffer[0]) << 8) | buffer[1];
	*ay = (((short)buffer[2]) << 8) | buffer[3];
	*az = (((short)buffer[4]) << 8) | buffer[5];
	*gx = (((short)buffer[8]) << 8) | buffer[9];
	*gy = (((short)buffer[10]) << 8) | buffer[11];
	*gz = (((short)buffer[12]) << 8) | buffer[13];
}

/*
 * get raw data converted to g and deg/sec values
 */
void mpu6050_getConvData(char address, double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds) {
	short ax = 0;
	short ay = 0;
	short az = 0;
	short gx = 0;
	short gy = 0;
	short gz = 0;
	mpu6050_getRawData(address, &ax, &ay, &az, &gx, &gy, &gz);

	#if MPU6050_CALIBRATEDACCGYRO == 1
    *axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
    *ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
    *azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
    *gxds = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
	*gyds = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
	*gzds = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
	#else
    *axg = (double)(ax)/MPU6050_AGAIN;
    *ayg = (double)(ay)/MPU6050_AGAIN;
    *azg = (double)(az)/MPU6050_AGAIN;
    *gxds = (double)(gx)/MPU6050_GGAIN;
	*gyds = (double)(gy)/MPU6050_GGAIN;
	*gzds = (double)(gz)/MPU6050_GGAIN;
	#endif
}

#if MPU6050_GETATTITUDE == 1
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
/*
 * Mahony update function (for 6DOF)
 */
void mpu6050_mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az) {
	float norm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		ax /= norm;
		ay /= norm;
		az /= norm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(mpu6050_mahonytwoKiDef > 0.0f) {
			integralFBx += mpu6050_mahonytwoKiDef * halfex * (1.0f / mpu6050_mahonysampleFreq);	// integral error scaled by Ki
			integralFBy += mpu6050_mahonytwoKiDef * halfey * (1.0f / mpu6050_mahonysampleFreq);
			integralFBz += mpu6050_mahonytwoKiDef * halfez * (1.0f / mpu6050_mahonysampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += mpu6050_mahonytwoKpDef * halfex;
		gy += mpu6050_mahonytwoKpDef * halfey;
		gz += mpu6050_mahonytwoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	gz *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;
}

/*
 * update quaternion
 */
void mpu6050_updateQuaternion(char address) {
	short ax = 0;
	short ay = 0;
	short az = 0;
	short gx = 0;
	short gy = 0;
	short gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxrs = 0;
	double gyrs = 0;
	double gzrs = 0;

	//get raw data
	while(1) {
		i2c_read_bit(address, MPU6050_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, (unsigned char *)buffer);
		if(buffer[0])
			break;
		delay_us(10);
	}

	i2c_read_bytes(address, MPU6050_ACCEL_XOUT_H, 14, (unsigned char *)buffer);
    ax = (((short)buffer[0]) << 8) | buffer[1];
    ay = (((short)buffer[2]) << 8) | buffer[3];
    az = (((short)buffer[4]) << 8) | buffer[5];
    gx = (((short)buffer[8]) << 8) | buffer[9];
    gy = (((short)buffer[10]) << 8) | buffer[11];
    gz = (((short)buffer[12]) << 8) | buffer[13];

	#if MPU6050_CALIBRATEDACCGYRO == 1
	axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
	gxrs = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN*0.01745329; //degree to radians
	gyrs = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN*0.01745329; //degree to radians
	gzrs = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN*0.01745329; //degree to radians
	#else
	axg = (double)(ax)/MPU6050_AGAIN;
	ayg = (double)(ay)/MPU6050_AGAIN;
	azg = (double)(az)/MPU6050_AGAIN;
	gxrs = (double)(gx)/MPU6050_GGAIN*0.01745329; //degree to radians
	gyrs = (double)(gy)/MPU6050_GGAIN*0.01745329; //degree to radians
	gzrs = (double)(gz)/MPU6050_GGAIN*0.01745329; //degree to radians
	#endif

    //compute data
    mpu6050_mahonyUpdate(gxrs, gyrs, gzrs, axg, ayg, azg);
}

/*
 * get quaternion
 */
void mpu6050_getQuaternion(double *qw, double *qx, double *qy, double *qz) {
	*qw = q0;
	*qx = q1;
	*qy = q2;
	*qz = q3;
}

/*
 * get euler angles
 * aerospace sequence, to obtain sensor attitude:
 * 1. rotate around sensor Z plane by yaw
 * 2. rotate around sensor Y plane by pitch
 * 3. rotate around sensor X plane by roll
 */
void mpu6050_getRollPitchYaw(double *roll, double *pitch, double *yaw) {
	*yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1);
	*pitch = -asin(2*q1*q3 + 2*q0*q2);
	*roll = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1);
}
#endif
