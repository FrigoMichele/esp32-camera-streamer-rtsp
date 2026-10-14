/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 OV5647 camera capture, hardware H.264 encoding and RTSP output.
 */

#include <inttypes.h>
#include <stdint.h>

#include "camera.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "rtsp.h"
#include "sdkconfig.h"

static const char *TAG = "camera_rtsp";

void app_main(void)
{
    const uint32_t width = CONFIG_H264_ENCODER_WIDTH;
    const uint32_t height = CONFIG_H264_ENCODER_HEIGHT;

    camera_t *camera = NULL;
    encoder_t *encoder = NULL;
    rtsp_t *rtsp = NULL;

    camera_config_t camera_config = {
        .width = width,
        .height = height,
        .format_name = "MIPI_2lane_24Minput_RAW8_800x640_50fps",
    };
    ESP_ERROR_CHECK(camera_create(&camera_config, &camera));

    const size_t frame_size = camera_frame_size(camera);
    encoder_config_t encoder_config = {
        .width = width,
        .height = height,
        .fps = CONFIG_H264_ENCODER_FPS,
        .bitrate = CONFIG_H264_ENCODER_BITRATE,
        .gop_size = CONFIG_H264_ENCODER_GOP_SIZE,
        .qp_value = CONFIG_H264_ENCODER_QP_VALUE,
    };
    ESP_ERROR_CHECK(encoder_create(&encoder_config, frame_size, &encoder));

    rtsp_config_t rtsp_config = {
        .width = width,
        .height = height,
        .fps = CONFIG_H264_ENCODER_FPS,
        .max_frame_size = frame_size,
        .network = {
            .server_port = 8554,
            .server_address = "192.168.2.5",
            .server_uri = "rtsp://192.168.2.5:8554/live",
            .netmask = "255.255.255.0",
            .gateway = "192.168.2.1",
            .connect_timeout_ms = 10000U,
        },
    };
    ESP_ERROR_CHECK(rtsp_create(&rtsp_config, &rtsp));
    ESP_ERROR_CHECK(camera_start(camera));

    ESP_LOGI(TAG, "capturing OV5647 %" PRIu32 "x%" PRIu32 " frames",
             width, height);

    const uint32_t frame_limit = CONFIG_H264_ENCODER_FRAME_COUNT;
    const uint32_t frame_timeout_ms = 1000U;
    uint32_t frame_number = 0;
    while (frame_limit == 0 || frame_number < frame_limit) {
        esp_err_t wait_result = camera_wait_frame(camera, frame_timeout_ms);
        if (wait_result == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "camera frame timeout after %" PRIu32 " ms",
                     frame_timeout_ms);
            break;
        }
        ESP_ERROR_CHECK(wait_result);

        encoder_frame_t encoded_frame = {0};
        ESP_ERROR_CHECK(encoder_encode(encoder,
                                       camera_frame(camera),
                                       camera_frame_size(camera),
                                       &encoded_frame));
        ESP_ERROR_CHECK(rtsp_publish(rtsp,
                                     encoded_frame.data,
                                     encoded_frame.length,
                                     encoded_frame.pts));
        ++frame_number;
    }

    rtsp_destroy(rtsp);
    encoder_destroy(encoder);
    camera_destroy(camera);
    ESP_LOGI(TAG, "capture complete");
}
