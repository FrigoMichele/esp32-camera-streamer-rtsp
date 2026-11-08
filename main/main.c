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
#include "esp_timer.h"
#include "rtsp.h"
#include "sdkconfig.h"

static const char *TAG = "camera_rtsp";

/* BEGIN identifies a blocked call even if it never returns. Start the clock
 * after that log so its UART overhead is not charged to the operation.
 * Durations are wall time: hardware work, lock waits and task scheduling.
 * Per-frame logging itself adds overhead and can reduce streaming FPS. */
#define TIME_STEP(label, operation) do {                                  \
    ESP_LOGI(TAG, "BEGIN %s", label);                                     \
    const int64_t step_start_us = esp_timer_get_time();                    \
    operation;                                                           \
    const int64_t step_elapsed_us = esp_timer_get_time() - step_start_us;  \
    ESP_LOGI(TAG, "END %s: %" PRId64 " us", label, step_elapsed_us);       \
} while (0)

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
        .format_name = "MIPI_2lane_24Minput_RAW10_1920x1080_30fps",
    };
    //- Create camera device reader
    TIME_STEP("camera_create", ESP_ERROR_CHECK(camera_create(&camera_config, &camera)));

    const size_t frame_size = camera_frame_size(camera);
    encoder_config_t encoder_config = {
        .width = width,
        .height = height,
        .fps = CONFIG_H264_ENCODER_FPS,
        .bitrate = CONFIG_H264_ENCODER_BITRATE,
        .gop_size = CONFIG_H264_ENCODER_GOP_SIZE,
        .qp_value = CONFIG_H264_ENCODER_QP_VALUE,
    };

    //- Create encoder device
    TIME_STEP("encoder_create",
              ESP_ERROR_CHECK(encoder_create(&encoder_config, frame_size, &encoder)));

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
        },
    };
    //- Create rtsp sender
    TIME_STEP("rtsp_create", ESP_ERROR_CHECK(rtsp_create(&rtsp_config, &rtsp)));

    // Start camera
    TIME_STEP("camera_start", ESP_ERROR_CHECK(camera_start(camera)));

    ESP_LOGI(TAG, "capturing OV5647 %" PRIu32 "x%" PRIu32 " frames",
             width, height);

    const uint32_t frame_limit = CONFIG_H264_ENCODER_FRAME_COUNT;
    uint32_t frame_number = 0;
    while (frame_limit == 0 || frame_number < frame_limit) {
        ESP_LOGI(TAG, "FRAME %" PRIu32, frame_number);

        /* 1. camera_frame() waits for a CSI completion notification.
         * WARNING: this does not lock the live YUV buffer; CSI can overwrite
         * it during encoding. RTSP synchronization below does not fix that. */
        /* Keep capture outside encoder_encode() so capture waiting is not
         * incorrectly included in the encoder timing. */
        const uint8_t *input_frame = NULL;
        TIME_STEP("camera_frame (wait for CSI)",
                  input_frame = camera_frame(camera));
        ESP_ERROR_CHECK(input_frame != NULL ? ESP_OK : ESP_ERR_INVALID_STATE);

        encoder_frame_t encoded_frame = {0};
        TIME_STEP("encoder_encode",
                  ESP_ERROR_CHECK(encoder_encode(encoder, input_frame,
                                                 frame_size, &encoded_frame)));
        /* 2. Encoding has returned with H.264 bytes in the encoder's reusable
         * output buffer. Publish locks the shared RTSP buffer and waits for
         * DMA to transfer those bytes before returning. Only then is it safe
         * for the next loop iteration to reuse the encoder output buffer. */
        /* This duration includes mutex contention AND the DMA completion wait;
         * it does not measure network transmission by the RTSP worker. */
        TIME_STEP("rtsp_publish (mutex + DMA)",
                  ESP_ERROR_CHECK(rtsp_publish(rtsp,
                                               encoded_frame.data,
                                               encoded_frame.length,
                                               encoded_frame.pts)));
        ++frame_number;
    }

    /* No further publications: stop RTSP callbacks before freeing the DMA
     * and shared buffer, then release encoder and capture resources. */
    TIME_STEP("rtsp_destroy", rtsp_destroy(rtsp));
    TIME_STEP("encoder_destroy", encoder_destroy(encoder));
    TIME_STEP("camera_destroy", camera_destroy(camera));
    ESP_LOGI(TAG, "capture complete");
}
