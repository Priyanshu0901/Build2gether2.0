#include "pcf8563_interface.h"

i2c_dev_t dev_handle;
struct tm time_pcf;

void init_pcf8563_interface()
{
    ESP_ERROR_CHECK(pcf8563_init_desc(&dev_handle, 0, 4, 5, false));
    ESP_ERROR_CHECK(pcf8563_reset(&dev_handle));
}

// Function to get current time (consider thread safety if needed)
struct tm get_current_time()
{
    pcf8563_get_time(&dev_handle, &time_pcf);
    return time_pcf;
}

void update_pcf8586_date(time_t* sntp_time){
    struct tm timeinfo;
    setenv("TZ", "UTC-05:30", 1);
    tzset();
    localtime_r(sntp_time, &timeinfo);
    pcf8563_set_time(&dev_handle, localtime(sntp_time));
}
