#define STA  0x20
#define SIC  0x08
#define SI   0x08
#define STO  0x10
#define STAC 0x20
#define AA   0x04

#define I2EN        (1<<6)      // I2C I2C Enable bit

void delay_us(volatile unsigned int time);
void delay_ms(volatile unsigned int time);
void delay_s(volatile unsigned int time);

int read_axis(void);
signed char testfunctie(void);

void i2c_send_address(unsigned char Addr_S);
void i2c_init(void);
void i2c_write(unsigned char Data);
void i2c_stop(void);
signed char i2c_read(void);
void write_byte(char address, char reg, char data);
unsigned char read_byte(char address,char reg);

#define tekenbit   0x80

