#include"driver/i2c.h" // Adding library for I2C

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
float aht10_read(){
	i2c_init();

    // AHT10 initialization.Write value to address of LM75A to I2C bus to determine which address is tried to reach

    uint8_t init_cmd[3] = {AHT10_INIT_CMD, 0x08, 0x00};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT10_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, init_cmd, 3, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(50));
        // AHT10 measurement command
        uint8_t measure_cmd[3] = {AHT10_MEASURE_CMD, 0x33, 0x00};
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AHT10_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, measure_cmd, 3, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        vTaskDelay(pdMS_TO_TICKS(80)); // Measurement takes up to 75ms

        // Read AHT10 data
        uint8_t raw_data[6];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (AHT10_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, raw_data, 6, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        uint32_t raw_humidity = ((uint32_t)(raw_data[1]) << 12) | ((uint32_t)(raw_data[2]) << 4) | ((uint32_t)(raw_data[3]) >> 4);

        float humidity = (raw_humidity * 100.0) / 0x100000;

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds

		return humidity;

}
