#ifndef SNTP_TASK_H
#define SNTP_TASK_H

#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

#include "wifi_sta.h"

void start_sntp_client(void);

#endif //SNTP_TASK_H