#include <inttypes.h>
#include "driver/i2c_master.h"
#include "esp_cam_sensor.h"
#include "esp_check.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_sccb_i2c.h"
#include "ov5647.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mipi_camera.h"
#include "h264_encoder.h"
#include "rtsp_connection.h"

static const char *TAG = "camera";

// Power the CSI PHY and configure the OV5647 without starting its stream.
static esp_err_t prepare_ov5647(esp_cam_sensor_device_t **sensor,
                               mipi_camera_config_t *cfg, uint8_t *fps)
{
    // Function EV Board wiring from test/rtsp-streamer.
    const esp_ldo_channel_config_t phy_power = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    esp_ldo_channel_handle_t phy_ldo;
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&phy_power, &phy_ldo),
                        TAG, "Could not power CSI PHY");

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 7,
        .scl_io_num = 8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "Could not start camera I2C");

    const sccb_i2c_config_t sccb_cfg = {
        .scl_speed_hz = 10000,
        .device_address = OV5647_SCCB_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    };
    esp_cam_sensor_config_t sensor_cfg = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };
    ESP_RETURN_ON_ERROR(sccb_new_i2c_io(bus, &sccb_cfg, &sensor_cfg.sccb_handle),
                        TAG, "Could not create SCCB interface");
    *sensor = ov5647_detect(&sensor_cfg);
    ESP_RETURN_ON_FALSE(*sensor != NULL, ESP_ERR_NOT_FOUND, TAG, "OV5647 not detected at 0x36");

    esp_cam_sensor_format_array_t formats = {0};
    ESP_RETURN_ON_ERROR(esp_cam_sensor_query_format(*sensor, &formats),
                        TAG, "Could not list OV5647 formats");
    for (uint32_t i = 0; i < formats.count; ++i) {
        const esp_cam_sensor_format_t *format = &formats.format_array[i];
        if (format->format != ESP_CAM_SENSOR_PIXFORMAT_RAW8 ||
            format->width != 800 || format->height != 640) {
            continue;
        }
        ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(*sensor, format),
                            TAG, "Could not set OV5647 format");
        *cfg = (mipi_camera_config_t) {
            .width = format->width,
            .height = format->height,
            .lane_count = format->mipi_info.lane_num,
            .lane_bit_rate_mbps = 200, // ESP-IDF RAW8 800x640 example setting.
            .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
            .has_line_start_packet = format->mipi_info.line_sync_en,
            .has_line_end_packet = format->mipi_info.line_sync_en,
        };
        *fps = format->fps;
        ESP_LOGI(TAG, "OV5647 detected: %s", format->name);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "OV5647 RAW8 800x640 mode not found");
    return ESP_ERR_NOT_SUPPORTED;
}

typedef struct {
    TaskHandle_t main_task;
    uint32_t width;
    uint32_t height;
    uint8_t fps;
} encoder_startup_t;

// Sole consumer of raw camera frames; CSI callbacks never run encoder code.
static void encoder_task(void *arg)
{
    encoder_startup_t *startup = arg;
    esp_cam_sensor_device_t *sensor = NULL;
    mipi_camera_config_t cfg = {0};
    uint8_t fps;
    ESP_ERROR_CHECK(prepare_ov5647(&sensor, &cfg, &fps));
    ESP_ERROR_CHECK(mipi_camera_init(&cfg));
    ESP_ERROR_CHECK(h264_encoder_init(cfg.width, cfg.height, fps));

    // Start streaming only after CSI and the ISP are ready.
    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));
    startup->width = cfg.width;
    startup->height = cfg.height;
    startup->fps = fps;
    xTaskNotifyGive(startup->main_task);
    while (true) {
        esp_err_t err = h264_encoder_process_next(pdMS_TO_TICKS(2000));
        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "No frame received; check sensor streaming and CSI settings");
            continue;
        }
        ESP_ERROR_CHECK(err);
    }
}

void app_main(void)
{
    encoder_startup_t startup = {.main_task = xTaskGetCurrentTaskHandle()};
    BaseType_t created = xTaskCreate(encoder_task, "h264_encoder", 8192, &startup,
                                     tskIDLE_PRIORITY + 1, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    // Wait for output pool and camera startup.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_err_t err = rtsp_connection_start(startup.width, startup.height, startup.fps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTSP unavailable: %s", esp_err_to_name(err));
    }

    int64_t previous_timestamp_us = -1;
    while (true) {
        h264_frame_t frame;
        ESP_ERROR_CHECK(h264_encoder_get_frame(&frame, portMAX_DELAY));
        if (previous_timestamp_us >= 0) {
            ESP_LOGI(TAG, "H.264 frame %" PRIu32 ": %zu bytes, encode %" PRId64
                     " us, %" PRId64 " us since previous frame", frame.sequence, frame.size,
                     frame.encode_us, frame.timestamp_us - previous_timestamp_us);
        } else {
            ESP_LOGI(TAG, "H.264 frame %" PRIu32 ": %zu bytes, encode %" PRId64 " us (first frame)",
                     frame.sequence, frame.size, frame.encode_us);
        }
        previous_timestamp_us = frame.timestamp_us;
        if (!rtsp_connection_offer(&frame)) {
            ESP_ERROR_CHECK(h264_encoder_release_frame(&frame));
        }
    }
}
