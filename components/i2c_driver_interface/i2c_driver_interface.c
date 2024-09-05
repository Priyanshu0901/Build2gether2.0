#include <stdio.h>
#include "i2c_driver_interface.h"


void initialize_i2c_ports(void)
{
    ESP_LOGI("I2C_Interface", "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PORT_0_SDA,
        .scl_io_num = I2C_PORT_0_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_0_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_0_HOST, I2C_MODE_MASTER, 0, 0, 0));
}
