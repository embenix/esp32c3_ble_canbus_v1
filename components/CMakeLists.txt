set(component_srcs "src/leds_rmt_ws2812.c" "src/canbus_task.c" "src/sht3x.c" "src/esp_wrapper.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
