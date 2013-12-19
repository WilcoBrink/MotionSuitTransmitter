/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/

#define I2C_REG_CONSET      0x00000040 /* Control Set Register         */
#define I2C_REG_CONSET_MASK 0x0000007C /* Used bits                    */
//#define I2C_REG_DATA        0x00000000 /* Data register                */
//#define I2C_REG_DATA_MASK   0x000000FF /* Used bits                    */
#define I2C_REG_ADDR        0x00000000 /* Slave address register       */		//I2C adres ARM
#define I2C_REG_ADDR_MASK   0x000000FF /* Used bits                    */

/* modes */
#define I2C_MODE_ACK0 0
#define I2C_MODE_ACK1 1
#define I2C_MODE_READ 2

/* return codes */
#define I2C_CODE_OK   1
#define I2C_CODE_DATA 2
#define I2C_CODE_RTR  3

#define I2C_CODE_ERROR -1
#define I2C_CODE_FULL  -2
#define I2C_CODE_EMPTY -3
#define I2C_CODE_BUSY  -4


/******************************************************************************
 * Description:
 *    Checks the I2C status.
 *
 *  Returns:
 *      00h Bus error
 *      08h START condition transmitted
 *      10h Repeated START condition transmitted
 *      18h SLA + W transmitted, ACK received
 *      20h SLA + W transmitted, ACK not received
 *      28h Data byte transmitted, ACK received
 *      30h Data byte transmitted, ACK not received
 *      38h Arbitration lost
 *      40h SLA + R transmitted, ACK received
 *      48h SLA + R transmitted, ACK not received
 *      50h Data byte received in master mode, ACK transmitted
 *      58h Data byte received in master mode, ACK not transmitted
 *      60h SLA + W received, ACK transmitted
 *      68h Arbitration lost, SLA + W received, ACK transmitted
 *      70h General call address received, ACK transmitted
 *      78h Arbitration lost, general call addr received, ACK transmitted
 *      80h Data byte received with own SLA, ACK transmitted
 *      88h Data byte received with own SLA, ACK not transmitted
 *      90h Data byte received after general call, ACK transmitted
 *      98h Data byte received after general call, ACK not transmitted
 *      A0h STOP or repeated START condition received in slave mode
 *      A8h SLA + R received, ACK transmitted
 *      B0h Arbitration lost, SLA + R received, ACK transmitted
 *      B8h Data byte transmitted in slave mode, ACK received
 *      C0h Data byte transmitted in slave mode, ACK not received
 *      C8h Last byte transmitted in slave mode, ACK received
 *      F8h No relevant status information, SI=0
 *      FFh Channel error
 *****************************************************************************/

unsigned char i2cCheckStatus(void);
void i2cInit(void);
signed char i2cStart(void);
signed char i2cRepeatStart(void);
signed char i2cStop(void);
signed char i2cPutChar(unsigned char data);
signed char i2cGetChar(unsigned char  mode, unsigned char* pData);
signed char i2cWrite(unsigned char addr, unsigned char reg, unsigned char* pData, unsigned short length);
signed char i2cWriteWord(unsigned char addr, unsigned char reg, signed short* pData);
void i2cWriteBits(unsigned char addr, unsigned char reg, unsigned char bitStart, unsigned char length, unsigned char data);
void i2cWriteBit(unsigned char addr, unsigned char reg, unsigned char bitNum, unsigned char data);
signed char i2cWaitTransmit(void);
signed char i2cWriteWithWait(unsigned char data);
signed char i2cMyWrite(unsigned char addr, unsigned char* pData, unsigned short length);
signed char i2cRead(unsigned char addr, unsigned char reg, unsigned char* pBuf, unsigned short length);
char i2cReadBits(char addr, unsigned char reg, unsigned char bitStart, unsigned char length, unsigned char *data);
