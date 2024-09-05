
idf_component_register(SRCS "pcf8563.c" "i2cdev.c" "pcf8563_interface.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "driver" "freertos"
                       PRIV_INCLUDE_DIRS "private_include")
