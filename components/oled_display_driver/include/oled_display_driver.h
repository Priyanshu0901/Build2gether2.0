#include <stdio.h>
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#define LCD_RST -1

#define LCD_PIXEL_CLOCK_HZ (400 * 1000) // 400MHz
#define LCD_I2C_ADDR 0x3C

#define LCD_H_RES 128
#define LCD_V_RES 64
#define LVGL_LCD_BUF_SIZE (LCD_H_RES * LCD_V_RES)

// Bit number used to represent command and parameter
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

extern void initialize_display(void);