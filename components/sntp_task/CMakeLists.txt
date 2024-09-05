idf_component_register(SRCS "sntp_task.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "pcf8563" "esp_netif" "wifi_sta")
