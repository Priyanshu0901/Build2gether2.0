
idf_component_register(SRCS "wifi_sta.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_wifi" "nvs_flash" "freertos" "esp_event" "lwip")
