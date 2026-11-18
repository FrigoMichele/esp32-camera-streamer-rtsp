idf_component_register(SRCS "mipi_camera.c"
                       INCLUDE_DIRS "include"
                       REQUIRES hal
                       PRIV_REQUIRES esp_driver_cam esp_driver_isp esp_mm esp_psram esp_timer)
