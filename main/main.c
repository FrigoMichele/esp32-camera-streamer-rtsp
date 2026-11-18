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
        .qp_min = CONFIG_H264_ENCODER_QP_MIN,
        .qp_max = CONFIG_H264_ENCODER_QP_MAX,
    };

    //- Create encoder device
    TIME_STEP("encoder_create",
              ESP_ERROR_CHECK(encoder_create(&encoder_config, frame_size, &encoder)));

    rtsp_config_t rtsp_config = {
        .width = width,
        .height = height,
        .send_poll_hz = CONFIG_RTSP_SEND_POLL_HZ,
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
    int64_t report_start_us = esp_timer_get_time();
    uint32_t report_frames = 0;
    int64_t capture_total_us = 0;
    int64_t encode_total_us = 0;
    int64_t publish_total_us = 0;
    uint64_t encoded_bytes = 0;
    size_t max_encoded_size = 0;
    ESP_LOGI(TAG, "open %s; capture continues without a viewer",
             rtsp_config.network.server_uri);
    while (frame_limit == 0 || frame_number < frame_limit) {
        /* One task owns capture -> encode -> publish. The next capture cannot
         * overwrite the YUV input until encoding has finished. */
        const int64_t capture_start_us = esp_timer_get_time();
        const uint8_t *input_frame = camera_frame(camera);
        ESP_ERROR_CHECK(input_frame != NULL ? ESP_OK : ESP_FAIL);

        const int64_t capture_end_us = esp_timer_get_time();
        encoder_frame_t encoded_frame = {0};
        ESP_ERROR_CHECK(encoder_encode(encoder, input_frame,
                                       frame_size, &encoded_frame));
        const int64_t encode_end_us = esp_timer_get_time();
        const rtsp_frame_timing_t timing = {
            .frame_id = frame_number + 1,
            .capture_start_us = capture_start_us,
            .raw_ready_us = capture_end_us,
            .encode_end_us = encode_end_us,
        };
        ESP_ERROR_CHECK(rtsp_publish(rtsp, encoded_frame.data,
                                     encoded_frame.length, encoded_frame.pts,
                                     &timing));
        const int64_t now_us = esp_timer_get_time();
        ++frame_number;
        ++report_frames;
        capture_total_us += capture_end_us - capture_start_us;
        encode_total_us += encode_end_us - capture_end_us;
        publish_total_us += now_us - encode_end_us;
        encoded_bytes += encoded_frame.length;
        if (encoded_frame.length > max_encoded_size) {
            max_encoded_size = encoded_frame.length;
        }

        /* Report only at bucket boundaries, outside the timed operations. */
        if (now_us - report_start_us >= 1000000) {
            const double divisor = 1000.0 * report_frames;
            ESP_LOGI(TAG, "pipeline: %.1f fps, n=%" PRIu32
                     ", avg ms capture=%.2f encode=%.2f publish=%.2f total=%.2f"
                     ", encoded=%.0f kbit/s max_AU=%zu B",
                     report_frames * 1000000.0 / (now_us - report_start_us),
                     report_frames, capture_total_us / divisor,
                     encode_total_us / divisor, publish_total_us / divisor,
                     (capture_total_us + encode_total_us + publish_total_us) / divisor,
                     encoded_bytes * 8000.0 / (now_us - report_start_us),
                     max_encoded_size);
            rtsp_report_timing(rtsp);
            report_start_us = now_us;
            report_frames = 0;
            capture_total_us = encode_total_us = publish_total_us = 0;
            encoded_bytes = 0;
            max_encoded_size = 0;
        }
    }

    /* No further publications: stop callbacks before freeing their buffers. */
    TIME_STEP("rtsp_destroy", rtsp_destroy(rtsp));
    TIME_STEP("encoder_destroy", encoder_destroy(encoder));
    TIME_STEP("camera_destroy", camera_destroy(camera));
    ESP_LOGI(TAG, "capture complete");
}
