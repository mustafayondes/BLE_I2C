#include"driver/i2c.h" // Adding library for I2C
#include"lm75a.h"


void i2c_init(){
	  i2c_config_t i2c_config = { //I2C initialization
	        .mode = I2C_MODE_MASTER,
	        .sda_io_num = SDA_GPIO,
	        .scl_io_num = SCL_GPIO,
	        .sda_pullup_en = GPIO_PULLUP_ENABLE,
	        .scl_pullup_en = GPIO_PULLUP_ENABLE,
	        .master.clk_speed = 50000};
	    i2c_param_config(I2C_NUM_0, &i2c_config);
	    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

float lm75a_read(){

	    i2c_init();
    	uint8_t raw[2];
        i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

        //Write value to address of LM75A to I2C bus to determine which address is tried to reach
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (LM75A_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_handle, 0x00, true); // LM75A sıcaklık kaydı adresi (0x00)
        i2c_master_stop(cmd_handle);
        i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_handle);

        cmd_handle = i2c_cmd_link_create();
        // I2C read işlemi başlatılıyor
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (LM75A_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd_handle, (uint8_t *)&raw, 2, I2C_MASTER_ACK); // read data and register raw
        i2c_master_stop(cmd_handle);
        i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_handle);


    bool isNeg = false;
    if (raw[0] & 0x80) //It is condition to set temperature is negative or positive
    {				   // if raw[0] & 1000 0000, it is positive
        isNeg = true;
        raw[0] = raw[0] & 0x7f;
    }
    int16_t data = (raw[0] << 8 | raw[1]) >> 5;
    float temperature = (data * 0.125) * (isNeg? -1 : 1);
    return temperature;
    }
