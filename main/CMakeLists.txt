idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES esp_h264 esp_driver_cam esp_driver_i2c esp_driver_isp esp_psram sensor_init
)
