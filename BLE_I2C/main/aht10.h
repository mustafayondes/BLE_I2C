#ifndef aht10_h
#define aht10_h

#define AHT10_ADDRESS 0x38
#define SDA_GPIO	21	//I2C pins
#define SCL_GPIO	22


#define AHT10_INIT_CMD 0xE1
#define AHT10_MEASURE_CMD 0xAC
#define AHT10_CALIBRATION_ENABLE 0x08

float aht10_read();
void i2c_init();
#endif
