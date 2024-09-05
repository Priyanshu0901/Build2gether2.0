
idf_component_register(SRCS "oled_display_driver.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "esp_lcd" "i2c_driver_interface" "display_ui" "esp_timer" "esp_lvgl_port")
