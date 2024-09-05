#include "wifi_sta.h"
#include "i2c_driver_interface.h"
#include "pcf8563_interface.h"
#include "sntp_task.h"
#include "camera_interface.h"
#include "oled_display_driver.h"
#include "bme680.h"


void initialize_drivers()
{
    initialize_i2c_ports();
    wifi_init_sta();
    initialize_camera();
    init_pcf8563_interface();
    start_sntp_client();
    init_bme680(false);
}

void start_tasks()
{
    initialize_display();
}

void app_main(void)
{
    initialize_drivers();
    start_tasks();
}
