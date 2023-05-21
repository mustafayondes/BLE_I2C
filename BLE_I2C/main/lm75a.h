#ifndef lm75a_h
#define lm75a_h

#define LM75A_ADDRESS 0x48 // I2C Sensors address read in datasheets
#define SDA_GPIO	21	//I2C pins
#define SCL_GPIO	22


float lm75a_read();
void i2c_init();
#endif
