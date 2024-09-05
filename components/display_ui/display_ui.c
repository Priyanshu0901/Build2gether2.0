#include <stdio.h>
#include "display_ui.h"
#include "pcf8563_interface.h"
#include "bme680.h"

lv_obj_t *disp;
lv_obj_t *screen_0;
lv_obj_t *time_label, *date_label,*temp_label,*hum_label,*gas_label,*press_label;

static lv_style_t style10 , style14;

void update_lvgl_time_label()
{
    struct tm time_now = get_current_time();
    char buf[9];
    strftime(buf, sizeof(buf), "%H:%M:%S", &time_now);
    lv_label_set_text(time_label, buf);
    strftime(buf, sizeof(buf), "%d-%m-%y", &time_now);
    lv_label_set_text(date_label, buf);
}

void update_lvgl_temp_sensor(){
    struct Result _bme_res = getBME680data();
    char buf[12];
    snprintf(buf, sizeof(buf), "T: %.2f", _bme_res.temperature);
    lv_label_set_text(temp_label, buf);
    snprintf(buf, sizeof(buf), "H: %.2f", _bme_res.humidity);
    lv_label_set_text(hum_label, buf);
    snprintf(buf, sizeof(buf), "G: %lu", _bme_res.gas);
    lv_label_set_text(gas_label, buf);
    snprintf(buf, sizeof(buf), "P: %.2f", _bme_res.pressure);
    lv_label_set_text(press_label, buf);
}

void display_ui(lv_obj_t *config_disp)
{
    disp = lv_tileview_create(config_disp);
    lv_obj_align(disp, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_style_init(&style10);
    lv_style_set_text_font(&style10, &lv_font_montserrat_10);

    lv_style_init(&style14);
    lv_style_set_text_font(&style14, &lv_font_montserrat_14);

    screen_0 = lv_tileview_add_tile(disp, 0, 0, LV_DIR_HOR);

    time_label = lv_label_create(screen_0);
    lv_obj_add_style(time_label, &style10, 0);
    lv_label_set_long_mode(time_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(time_label, "00:00:00");
    lv_obj_set_width(time_label, 45);
    lv_obj_align(time_label, LV_ALIGN_TOP_RIGHT, 0, 0);

    date_label = lv_label_create(screen_0);
    lv_obj_add_style(date_label, &style10, 0);
    lv_label_set_long_mode(date_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(date_label, "00-00-00");
    lv_obj_set_width(date_label, 45);
    lv_obj_align(date_label, LV_ALIGN_TOP_LEFT, 0, 0);

    temp_label = lv_label_create(screen_0);
    lv_obj_add_style(temp_label, &style10, 0);
    lv_label_set_long_mode(temp_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(temp_label, "T: ");
    lv_obj_set_width(temp_label, 64);
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 0, 20);

    hum_label = lv_label_create(screen_0);
    lv_obj_add_style(hum_label, &style10, 0);
    lv_label_set_long_mode(hum_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(hum_label, "H: ");
    lv_obj_set_width(hum_label, 64);
    lv_obj_align(hum_label, LV_ALIGN_TOP_LEFT, 64, 20);

    gas_label = lv_label_create(screen_0);
    lv_obj_add_style(gas_label, &style10, 0);
    lv_label_set_long_mode(gas_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(gas_label, "G: ");
    lv_obj_set_width(gas_label, 64);
    lv_obj_align(gas_label, LV_ALIGN_TOP_LEFT, 0, 35);

    press_label = lv_label_create(screen_0);
    lv_obj_add_style(press_label, &style10, 0);
    lv_label_set_long_mode(press_label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(press_label, "H: ");
    lv_obj_set_width(press_label, 64);
    lv_obj_align(press_label, LV_ALIGN_TOP_LEFT, 64, 35);

    lv_timer_create(update_lvgl_time_label, 500, NULL); // Call `my_task` every second
    lv_timer_create(update_lvgl_temp_sensor, 1000, NULL); // Call `my_task` every second
}
