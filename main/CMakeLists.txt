idf_component_register(SRCS "main.c" "h264_encoder.c" "rtsp_connection.c"
                       PRIV_REQUIRES mipi_camera esp_driver_i2c esp_hw_support esp_timer esp_eth esp_event esp_netif)
