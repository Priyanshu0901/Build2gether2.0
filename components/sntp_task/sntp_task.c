#include <stdio.h>
#include "sntp_task.h"         // Assuming this is a custom header for SNTP-related tasks
#include "pcf8563_interface.h" // Assuming this interacts with a PCF8563 RTC chip

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

static const char *TAG = "sntp";

// Global time variable (consider thread safety if needed)
static time_t current_time;

void callback_notification_for_time_sync(struct timeval *tv)
{
    struct tm timeinfo;
    time(&current_time);
    localtime_r(&current_time, &timeinfo);

    // Update PCF8563 RTC chip (thread-safe update function)
    update_pcf8586_date(&current_time);
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

// Function to obtain time from SNTP server(s)
static void obtain_time(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("time.windows.com");
    config.start = true;                                  // start SNTP service explicitly (after connecting)
    config.server_from_dhcp = false;                      // Accept NTP offers from DHCP
    config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;       // Refresh configuration on IP acquisition
    config.sync_cb = callback_notification_for_time_sync; // Callback for time sync events
    esp_netif_sntp_init(&config);

    // Wait for time to be set (with timeout and logging)
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    if (retry == retry_count)
    {
        ESP_LOGE(TAG, "Failed to synchronize time with NTP after %d retries", retry_count);
    }

    time(&current_time);
    localtime_r(&current_time, &timeinfo);

    // Update PCF8563 RTC chip (thread-safe update function)
    update_pcf8586_date(&current_time);
}

// Function to start SNTP client and time update task
void start_sntp_client(void)
{
    if (current_time == 0)
    {
        obtain_time();
    }
}