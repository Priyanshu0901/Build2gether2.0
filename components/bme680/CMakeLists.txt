idf_component_register(SRCS  "bme680.c" "priv_include/bme68x.c"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "priv_include"
                    REQUIRES "driver" "freertos" "esp_timer")
