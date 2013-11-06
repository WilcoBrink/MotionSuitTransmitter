#include "LPC214x.h"
#include "I2C.h"
#include "mpu6050.h"

#define I2C_STA	0x20	// Startbit (set, test)
#define I2C_SIC	0x08	// Interuptbit (clear)
#define I2C_SI	0x08	// Interuptbit (test)
#define I2C_STO	0x10	// Stopbit (set)
#define I2C_STAC  0x20	// Startbit (clear)
#define I2C_AA	0x04	// Acknowledgebit (test)

unsigned int state;

int read_axis()
{
	unsigned char data_h, data_l;
	int size;
	data_h = read_byte(0x68, MPU6050_ACCEL_XOUT_H);
	data_l = read_byte(0x68, MPU6050_ACCEL_XOUT_L);
	size = (data_h<<8) + data_l;
	return size;
}

signed char testfunctie()
{
	signed char data;
	write_byte(0x68, MPU6050_SMPLRT_DIV, 0x07);
	data = read_byte(0x68, MPU6050_SMPLRT_DIV);
	return data;
}

signed char read_byte(char address, char reg)
{
    signed char meting;
	i2c_send_address(address<<1);
    //delay_us(500);
	i2c_write(reg);
    //delay_us(500);
	i2c_send_address((address<<1) | 0x01);
    //delay_us(500);
	meting = i2c_read();
	i2c_stop();
	return meting;
}

void write_byte(char address, char reg, char data)
{
	i2c_send_address(address<<1);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

void i2c_init()
{
   	I2C0CONCLR = 0xFF;                      // clr I2C0 status register
    
   	PINSEL0  |= (1<<6)|(1<<4);				// set P0.2 as SCL0 I2C0 and set P0.3 as SDA0 I2C0
    
    I2C0SCLH   = 150;      	// Set count for SCL High period          150 for 100 kHz I2C clock, High and Low time equal
	I2C0SCLL   = 150;      	// Set count for SCL Low period           150 for 100 kHz I2C clock
	I2C0CONSET = 0x40;      // Enable I2C0 interface and set it to Master mode
}

void i2c_send_address(unsigned char Addr_S)
{
   if(!(Addr_S & 0x01))               // test whether it is reading
   {
	   I2C0CONSET = I2C_STA;         // set STA - allow master to acknowlege slave;
   	   while(I2C0STAT != 0x08);
   	   I2C0DAT    = Addr_S;
   	   I2C0CONCLR = I2C_SIC|I2C_STAC;
   	   while(I2C0STAT != 0x18 && !(I2C0CONSET & I2C_SI));
   }
   else// looks like master reciever
   {
	   I2C0CONSET = I2C_STA;
	   I2C0CONCLR = I2C_SIC;
	   while(I2C0STAT != 0x10 && !(I2C0CONSET & I2C_SI)); 	// Startbitsetzen und auf Start warten >>start element setting and on start wait
	   I2C0DAT    = Addr_S;						// Partneradresse laden >> slave address load
	   I2C0CONCLR = I2C_SIC|I2C_STAC;					// I2C IR-bit löschen und senden starten>>I2C IR bit to delete and send start
	   while(I2C0STAT != 0x40 && !(I2C0CONSET & I2C_SI));	// auf ACK warten >>on ACK wait
   }
}

void i2c_write(unsigned char Data)
{
   I2C0DAT    = Data;                // Charge Data
   I2C0CONCLR = I2C_SIC;                 // SIC; Clear I2C interrupt bit to send the data
   while(I2C0STAT != 0x28 && !(I2C0CONSET & I2C_SI));     // wait till status available
}

void i2c_stop(void)
{
   I2C0CONCLR = I2C_SIC;
   I2C0CONSET = I2C_STO;
   while((I2C0CONSET & I2C_STO));          // wait for Stopped I2C bus
}

signed char i2c_read(void)
{
	/*I2C0CONCLR = SIC;
	while( ! ( I2C0CONSET & SI)) ; // wait till status available
	{
		state=I2C0STAT;
		if(state == 0x50)
		{
			return I2C0DAT;
		}
		else
		if(state== 0x58)
		{
		i2c_stop();
		}
	}
	return 0;*/

	signed char IIC_DATA=0x00;
	//ACK from the master
	I2C0CONCLR = I2C_SIC;	// Interrupt löschen und empfangen starten>>interrupts to delete and received start /
	while(I2C0STAT != 0x28 && !(I2C0CONSET & I2C_SI));// wenn fertg, ACK senden >>//wait the data
	IIC_DATA = I2C0DAT;	// Daten uebernehmen >>  //read data

	return IIC_DATA;
}

