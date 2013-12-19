/******************************************************************************
 * Includes
 *****************************************************************************/

#include "i2c.h"
#include <LPC214x.h>
#include "delay.h"

unsigned char i2cCheckStatus(void)
{
  unsigned char status = 0;

  /* wait for I2C Status changed */
  while( (I2C0CONSET & 0x08)  == 0);   /* while SI == 0 */

  /* Read I2C State */
  status = I2C0STAT;

  /* NOTE! SI flag is not cleared here */
  return status;
}

/******************************************************************************
 * Description:
 *    Reset the I2C module.
 *****************************************************************************/
void i2cInit(void)
{
	PINSEL0  |= (1<<6)|(1<<4);				// set P0.2 as SCL0 I2C0 and set P0.3 as SDA0 I2C0

	/* clear flags */
	I2C0CONCLR = 0x6c;

	I2C0SCLH   = 300;      	// Set count for SCL High period          150 for 400 kHz I2C clock, High and Low time equal
	I2C0SCLL   = 300;      	// Set count for SCL Low period           150 for 400 kHz I2C clock

	/* reset registers */
	I2C0ADR   = ( I2C0ADR   & ~I2C_REG_ADDR_MASK )   | I2C_REG_ADDR;		//set I2C0 address to 0
	I2C0CONSET = ( I2C0CONSET & ~I2C_REG_CONSET_MASK ) | I2C_REG_CONSET;	//enable I2C0 interface
}

/******************************************************************************
 * Description:
 *    Generates a start condition on I2C when bus is free.
 *    Master mode will also automatically be entered.
 *
 *    Note: After a stop condition, you may need a bus free time before you
 *          can generate a new start condition.
 *
 * Returns:
 *    I2C_CODE_OK or I2C status code
 *****************************************************************************/
signed char i2cStart(void)
{
  unsigned char status  = 0;
  signed char retCode = 0;

  /* issue a start condition */
  I2C0CONSET |= 0x20; /* STA = 1, set start flag */

  /* wait until START transmitted */
  while(1)
  {
    status = i2cCheckStatus();

    /* start transmitted */
    if((status == 0x08) || (status == 0x10))
    {
      retCode = I2C_CODE_OK;
      break;
    }
    /* error */
    else if(status != 0xf8)
    {
      retCode = (signed char) status;
      break;
    }
    else
    {
      /* clear SI flag */
      I2C0CONCLR = 0x08;
    }    
  }

  /* clear start flag */
  I2C0CONCLR = 0x20;

  return retCode;
}

/******************************************************************************
 * Description:
 *    Generates a start condition on I2C when bus is free.
 *    Master mode will also automatically be entered.
 *
 *    Note: After a stop condition, you may need a bus free time before you
 *          can generate a new start condition.
 *
 * Returns:
 *    I2C_CODE_OK or I2C status code
 *****************************************************************************/
signed char i2cRepeatStart(void)
{
  unsigned char status  = 0;
  signed char retCode = 0;

  /* issue a start condition */
  I2C0CONSET |= 0x20; /* STA = 1, set start flag */
  I2C0CONCLR = 0x08;  /* clear SI flag           */

  /* wait until START transmitted */
  while(1)
  {
    status = i2cCheckStatus();

    /* start transmitted */
    if((status == 0x08) || (status == 0x10))
    {
      retCode = I2C_CODE_OK;
      break;
    }
    /* error */
    else if(status != 0xf8)
    {
      retCode = (signed char) status;
      break;
    }
    else
    {
      /* clear SI flag */
      I2C0CONCLR = 0x08;
    }    
  }

  /* clear start flag */
  I2C0CONCLR = 0x20;

  return retCode;
}

/******************************************************************************
 * Description:
 *    Generates a stop condition in master mode or recovers from an error
 *    condition in slave mode.
 *
 *    Note: After this function is run, you may need a bus free time before
 *          you can generate a new start condition.
 *
 * Returns:
 *    I2C_CODE_OK
 *****************************************************************************/
signed char i2cStop(void)
{
  I2C0CONSET |= 0x10; /* STO = 1, set stop flag */
  I2C0CONCLR = 0x08;  /* clear SI flag          */

  /* wait for STOP detected (while STO = 1) */
  while((I2C0CONSET & 0x10) == 0x10 );

  return I2C_CODE_OK;
}

/******************************************************************************
 * Description:
 *    Sends a character on the I2C network
 *
 * Params:
 *    [in] data - the character to send
 *
 * Returns:
 *    I2C_CODE_OK   - successful
 *    I2C_CODE_BUSY - data register is not ready -> byte was not sent
 *****************************************************************************/
signed char i2cPutChar(unsigned char data)
{
  signed char retCode = 0;

  /* check if I2C Data register can be accessed */
  if((I2C0CONSET & 0x08) != 0)  /* if SI = 1 */
  {
    /* send data */
    I2C0DAT   = data;
    I2C0CONCLR = 0x08; /* clear SI flag */
    retCode    = I2C_CODE_OK;
  }
  else
  {
    /* data register not ready */
    retCode = I2C_CODE_BUSY;
  }

  return retCode;
}

/******************************************************************************
 * Description:
 *    Read a character. I2C master mode is used.
 *    This function is also used to prepare if the master shall generate
 *    acknowledge or not acknowledge.
 *
 * Params:
 *    [in]  mode  - I2C_MODE_ACK0 Set ACK=0. Slave sends next byte
 *                  I2C_MODE_ACK1 Set ACK=1. Slave sends last byte
 *                  I2C_MODE_READ Read data from data register
 *    [out] pData - a pointer to where the data shall be saved.
 *
 * Returns:
 *    I2C_CODE_OK    - successful
 *    I2C_CODE_EMPTY - no data is available
 *****************************************************************************/
signed char i2cGetChar(unsigned char  mode, unsigned char* pData)
{
  signed char retCode = I2C_CODE_OK;

  if(mode == I2C_MODE_ACK0)
  {
    /* the operation mode is changed from master transmit to master receive */
    /* set ACK=0 (informs slave to send next byte) */

    I2C0CONSET |= 0x04; /* AA=1          */
    I2C0CONCLR = 0x08;  /* clear SI flag */
  }
  else if(mode == I2C_MODE_ACK1)
  {
    /* set ACK=1 (informs slave to send last byte) */
    I2C0CONCLR = 0x04;
    I2C0CONCLR = 0x08; /* clear SI flag */
  }
  else if(mode == I2C_MODE_READ)
  {
    /* check if I2C Data register can be accessed */
    if((I2C0CONSET & 0x08) != 0)  /* SI = 1 */
    {
      /* read data  */
      *pData = (unsigned char) I2C0DAT;
    }
    else
    {
      /* No data available */
      retCode = I2C_CODE_EMPTY;
    }
  }

  return retCode;
}

/******************************************************************************
 * Description:
 *    Sends data on the I2C network
 *
 *    Note: After this function is run, you may need a bus free time before a
 *          new data transfer can be initiated.
 *
 * Params:
 *    [in] addr  - address
 *    [in] pData - data to transmit
 *    [in] len   - number of bytes to transmit
 *
 * Returns:
 *    I2C_CODE_OK    - successful
 *    I2C_CODE_ERROR - an error occured
 *****************************************************************************/
signed char i2cWrite(unsigned char addr, unsigned char reg, unsigned char* pData, unsigned short length)
{
  signed char retCode = 0;
  unsigned char status = 0;
  unsigned char i = 0;

  /* generate Start condition */
  retCode = i2cStart();

  /* Transmit address */
  if(retCode == I2C_CODE_OK)
  {
    /* write SLA+W */
    retCode = i2cPutChar(addr<<1);
    while(retCode == I2C_CODE_BUSY)
    {
      retCode = i2cPutChar(addr<<1);
    }
  }

  /* Transmit register */
  if(retCode == I2C_CODE_OK)
  {
	  /* write SLA+W */
      retCode = i2cPutChar(reg);
      while(retCode == I2C_CODE_BUSY)
      {
        retCode = i2cPutChar(reg);
      }
  }

  if(retCode == I2C_CODE_OK)
  {
    /* wait until address transmitted and transmit data */
    for(i = 0; i < length; i++)
    {
      /* wait until data transmitted */
      while(1)
      {
        /* get new status */
        status = i2cCheckStatus();

        /*
         * SLA+W transmitted, ACK received or
         * data byte transmitted, ACK received
         */
        if( (status == 0x18) || (status == 0x28) )
        {
          /* Data transmitted and ACK received */

          /* write data */
          retCode = i2cPutChar(*pData);
          while(retCode == I2C_CODE_BUSY)
          {
            retCode = i2cPutChar(*pData);
          }
          pData++;

          break;
        }
        /* no relevant status information */
        else if( status != 0xf8 )
        {
          /* error */
          i = length;
          retCode = I2C_CODE_ERROR;
          break;
        }
      }
    }
  }

  /* wait until data transmitted */
  while(1)
  {
    /* get new status */
    status = i2cCheckStatus();

    /*
     * SLA+W transmitted, ACK received or
     * data byte transmitted, ACK received
     */
    if( (status == 0x18) || (status == 0x28) )
    {
      /* data transmitted and ACK received */
      break;
    }
    /* no relevant status information */
    else if(status != 0xf8 )
    {
      /* error */
      i = length;
      retCode = I2C_CODE_ERROR;
      break;
    }
  }

  /* generate Stop condition */
  i2cStop();

  return retCode;
}

/******************************************************************************
 * Description:
 *    Sends data on the I2C network
 *
 *    Note: After this function is run, you may need a bus free time before a
 *          new data transfer can be initiated.
 *
 * Params:
 *    [in] addr  - address
 *    [in] reg	 - register to write
 *    [in] pData - data to transmit
 *
 * Returns:
 *    I2C_CODE_OK    - successful
 *    I2C_CODE_ERROR - an error occured
 *****************************************************************************/
signed char i2cWriteWord(unsigned char addr, unsigned char reg, signed short* pData)
{
  signed char retCode = 0;
  unsigned char status = 0;

  /* generate Start condition */
  retCode = i2cStart();

  /* Transmit address */
  if(retCode == I2C_CODE_OK)
  {
    /* write SLA+W */
    retCode = i2cPutChar(addr<<1);
    while(retCode == I2C_CODE_BUSY)
    {
      retCode = i2cPutChar(addr<<1);
    }
  }

  /* Transmit register */
  if(retCode == I2C_CODE_OK)
  {
	  /* write SLA+W */
      retCode = i2cPutChar(reg);
      while(retCode == I2C_CODE_BUSY)
      {
        retCode = i2cPutChar(reg);
      }
  }

  if(retCode == I2C_CODE_OK)
  {
      /* wait until data transmitted */
      while(1)
      {
        /* get new status */
        status = i2cCheckStatus();

        /*
         * SLA+W transmitted, ACK received or
         * data byte transmitted, ACK received
         */
        if( (status == 0x18) || (status == 0x28) )
        {
          /* Data transmitted and ACK received */

          /* write data */
          retCode = i2cPutChar((char)(*pData >> 8));
          while(retCode == I2C_CODE_BUSY)
          {
            retCode = i2cPutChar((char)*pData >> 8);
          }

          break;
        }
        /* no relevant status information */
        else if( status != 0xf8 )
        {
          /* error */
          retCode = I2C_CODE_ERROR;
          break;
        }
      }

      while(1)
            {
              /* get new status */
              status = i2cCheckStatus();

              /*
               * SLA+W transmitted, ACK received or
               * data byte transmitted, ACK received
               */
              if( (status == 0x18) || (status == 0x28) )
              {
                /* Data transmitted and ACK received */

                /* write data */
                retCode = i2cPutChar((char)*pData);
                while(retCode == I2C_CODE_BUSY)
                {
                  retCode = i2cPutChar((char)*pData);
                }

                break;
              }
              /* no relevant status information */
              else if( status != 0xf8 )
              {
                /* error */
                retCode = I2C_CODE_ERROR;
                break;
              }
            }
  }

  /* wait until data transmitted */
  while(1)
  {
    /* get new status */
    status = i2cCheckStatus();

    /*
     * SLA+W transmitted, ACK received or
     * data byte transmitted, ACK received
     */
    if( (status == 0x18) || (status == 0x28) )
    {
      /* data transmitted and ACK received */
      break;
    }
    /* no relevant status information */
    else if(status != 0xf8 )
    {
      /* error */
      retCode = I2C_CODE_ERROR;
      break;
    }
  }

  /* generate Stop condition */
  i2cStop();

  return retCode;
}

/******************************************************************************
 * Description:
 *    abc
 *
 * Params:
 *    [in]
 *
 * Returns:
 *
 *****************************************************************************/
void i2cWriteBits(unsigned char addr, unsigned char reg, unsigned char bitStart, unsigned char length, unsigned char data) {
    //      010 value to write
    // 76543210 bit numbers
    //      xxx args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		unsigned char b = 0;
		//b = i2c_read_byte(addr, reg);
		i2cRead(addr, reg, &b, 1);
		//if (b != 0) { //get current data
			unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			i2cWrite(addr, reg, &b, 1);
		//}
	}
}

/******************************************************************************
 * Description:
 *    abc
 *
 * Params:
 *    [in]
 *
 * Returns:
 *
 *****************************************************************************/
void i2cWriteBit(unsigned char addr, unsigned char reg, unsigned char bitNum, unsigned char data) {
	unsigned char b, c = 0;
	i2cRead(addr, reg, &b, 1);
	c = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	i2cWrite(addr, reg, &c, 1);
}

/******************************************************************************
 * Description:
 *    abc
 *
 * Params:
 *    [in]
 *
 * Returns:
 *
 *****************************************************************************/
signed char i2cWaitTransmit(void)
{
  unsigned char status = 0;

  /* wait until data transmitted */
  while(1)
  {
    /* get new status */
    status = i2cCheckStatus();

    /*
     * SLA+W transmitted, ACK received or
     * data byte transmitted, ACK received
     */
    if( (status == 0x18) || (status == 0x28) )
    {
      /* data transmitted and ACK received */
      return I2C_CODE_OK;
    }
    /* no relevant status information */
    else if(status != 0xf8 )
    {
      /* error */
      return I2C_CODE_ERROR;
    }
  }
}

signed char i2cWriteWithWait(unsigned char data)
{
  signed char retCode = 0;
 
  retCode = i2cPutChar(data);
  while(retCode == I2C_CODE_BUSY)
  {
    retCode = i2cPutChar(data);
  }

  if(retCode == I2C_CODE_OK)
    retCode = i2cWaitTransmit();

  return retCode;
}

signed char i2cMyWrite(unsigned char addr, unsigned char* pData, unsigned short length)
{
  signed char retCode = 0;
  unsigned char i = 0;

  do
  {
    /* generate Start condition */
    retCode = i2cStart();
    if(retCode != I2C_CODE_OK)
      break;

    /* write address */
    retCode = i2cWriteWithWait(addr);
    if(retCode != I2C_CODE_OK)
      break;

    for(i = 0; i <length; i++)
    {
      retCode = i2cWriteWithWait(*pData);
      if(retCode != I2C_CODE_OK)
        break;

      pData++;
    }

  } while(0);

  /* generate Stop condition */
  i2cStop();

  return retCode;
}


/******************************************************************************
 * Description:
 *    Read a specified number of bytes from the I2C network.
 *
 *    Note: After this function is run, you may need a bus free time before a
 *          new data transfer can be initiated.
 *
 * Params:
 *    [in] addr - address
 *    [in] pBuf - receive buffer
 *    [in] len  - number of bytes to receive
 *
 * Returns:
 *    I2C_CODE_OK or I2C status code
 *****************************************************************************/
signed char i2cRead(unsigned char addr, unsigned char reg, unsigned char* pBuf, unsigned short length)
{
  signed char retCode = 0;
  unsigned char status  = 0;
  unsigned char i = 0;

  /* Generate Start condition */
  retCode = i2cStart();

  /* Transmit address */
  if(retCode == I2C_CODE_OK )
  {
    /* write SLA+R */
    retCode = i2cPutChar(addr<<1);
    while(retCode == I2C_CODE_BUSY)
    {
      retCode = i2cPutChar(addr<<1);
    }
  }

  /* Transmit register */
    if(retCode == I2C_CODE_OK)
    {
  	  /* write SLA+W */
        retCode = i2cPutChar(reg);
        while(retCode == I2C_CODE_BUSY)
        {
          retCode = i2cPutChar(reg);
        }
    }

    delay_us(100);

    /* Generate Start condition */
    retCode = i2cRepeatStart();

    /* Transmit address */
    if(retCode == I2C_CODE_OK )
    {
    	/* write SLA+R */
    	retCode = i2cPutChar((addr<<1) | 0x01);
    	while(retCode == I2C_CODE_BUSY)
    	{
    		retCode = i2cPutChar((addr<<1) | 0x01);
    	}
    }


  if(retCode == I2C_CODE_OK )
  {
    /* wait until address transmitted and receive data */
    for(i = 1; i <= length; i++ )
    {
      /* wait until data transmitted */
      while(1)
      {
        /* get new status */
        status = i2cCheckStatus();

        /*
         * SLA+R transmitted, ACK received or
         * SLA+R transmitted, ACK not received
         * data byte received in master mode, ACK transmitted
         */
        if((status == 0x40 ) || (status == 0x48 ) || (status == 0x50 ))
        {
          /* data received */

          if(i == length)
          {
            /* Set generate NACK */
            retCode = i2cGetChar(I2C_MODE_ACK1, pBuf);
          }
          else
          {
            retCode = i2cGetChar(I2C_MODE_ACK0, pBuf);
          }

          /* Read data */
          retCode = i2cGetChar(I2C_MODE_READ, pBuf);
          while(retCode == I2C_CODE_EMPTY)
          {
            retCode = i2cGetChar(I2C_MODE_READ, pBuf);
          }
          pBuf++;

          break;
        }

        /* no relevant status information */
        else if(status != 0xf8 )
        {
          /* error */
          i = length;
          retCode = I2C_CODE_ERROR;
          break;
        }
      }
    }
  }

  /*--- Generate Stop condition ---*/
  i2cStop();

  return retCode;
}

// Read bits from chip register
char i2cReadBits(char addr, unsigned char reg, unsigned char bitStart, unsigned char length, unsigned char *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    char count = 0;
    if(length > 0) {
		unsigned char b = 0;
		if ((count = i2cRead(addr, reg, &b, 1)) != 0) {
			//b = count;
			unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}
