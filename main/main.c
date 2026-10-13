/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal ESP32-P4 H.264 hardware encoder example.
 * Based on ESP-IDF examples/peripherals/h264.
 */

#include <inttypes.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_h264_alloc.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "h264_encoder";

static void check_encoder_result(esp_h264_err_t result, const char *operation)
{
    if (result != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "%s failed: %d", operation, (int)result);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

static uint8_t *allocate_frame_buffer(uint32_t size, uint32_t *actual_size)
{
    uint8_t *buffer = esp_h264_aligned_malloc(
        16, 1, size, actual_size, ESP_H264_MEM_SPIRAM);

    ESP_ERROR_CHECK(buffer != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    return buffer;
}

/*
 * Generate a neutral O_UYY_E_VYY frame.
 * Replace this function with the camera's YUV frame producer.
 *
 * The ESP32-P4 hardware encoder expects alternating chroma rows:
 *   U Y Y, U Y Y, ...
 *   V Y Y, V Y Y, ...
 */
static void fill_source_frame(uint8_t *buffer, uint32_t frame_number)
{
    const uint8_t luma = (uint8_t)(16 + (frame_number * 17U) % 220U);

    for (uint32_t row = 0; row < CONFIG_H264_ENCODER_HEIGHT; row += 2) {
        const uint8_t chroma = 128;

        for (uint32_t pixel = 0; pixel < CONFIG_H264_ENCODER_WIDTH; pixel += 2) {
            *buffer++ = chroma;
            *buffer++ = luma;
            *buffer++ = luma;
        }

        for (uint32_t pixel = 0; pixel < CONFIG_H264_ENCODER_WIDTH; pixel += 2) {
            *buffer++ = chroma;
            *buffer++ = luma;
            *buffer++ = luma;
        }
    }
}

static void handle_encoded_frame(const esp_h264_enc_out_frame_t *frame,
                                 uint32_t frame_number)
{
    ESP_LOGI(TAG, "frame=%" PRIu32 ", type=%d, bytes=%" PRIu32 ", pts=%" PRIu32,
             frame_number, frame->frame_type, frame->length, frame->pts);

    /* Send frame->raw_data.buffer and frame->length to the RTSP sink here. */
}

void app_main(void)
{
    const uint32_t frame_size =
        (uint32_t)CONFIG_H264_ENCODER_WIDTH * CONFIG_H264_ENCODER_HEIGHT * 3U / 2U;

    esp_h264_enc_in_frame_t source_frame = {0};
    esp_h264_enc_out_frame_t encoded_frame = {0};
    esp_h264_enc_handle_t encoder = NULL;

    esp_h264_enc_cfg_hw_t encoder_config = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = CONFIG_H264_ENCODER_GOP_SIZE,
        .fps = CONFIG_H264_ENCODER_FPS,
        .res = {
            .width = CONFIG_H264_ENCODER_WIDTH,
            .height = CONFIG_H264_ENCODER_HEIGHT,
        },
        .rc = {
            .bitrate = CONFIG_H264_ENCODER_BITRATE,
            .qp_min = CONFIG_H264_ENCODER_QP_VALUE,
            .qp_max = CONFIG_H264_ENCODER_QP_VALUE,
        },
    };

    ESP_ERROR_CHECK((CONFIG_H264_ENCODER_WIDTH % 2 == 0 &&
                     CONFIG_H264_ENCODER_HEIGHT % 2 == 0)
                        ? ESP_OK
                        : ESP_ERR_INVALID_ARG);

    source_frame.raw_data.buffer =
        allocate_frame_buffer(frame_size, &source_frame.raw_data.len);
    encoded_frame.raw_data.buffer =
        allocate_frame_buffer(frame_size, &encoded_frame.raw_data.len);

    check_encoder_result(
        esp_h264_enc_hw_new(&encoder_config, &encoder), "create encoder");
    check_encoder_result(esp_h264_enc_open(encoder), "open encoder");

    ESP_LOGI(TAG, "encoding %dx%d at %d fps",
             CONFIG_H264_ENCODER_WIDTH,
             CONFIG_H264_ENCODER_HEIGHT,
             CONFIG_H264_ENCODER_FPS);

    for (uint32_t frame_number = 0;
         frame_number < CONFIG_H264_ENCODER_FRAME_COUNT;
         ++frame_number) {
        fill_source_frame(source_frame.raw_data.buffer, frame_number);
        source_frame.pts = frame_number * (1000U / CONFIG_H264_ENCODER_FPS);

        check_encoder_result(
            esp_h264_enc_process(encoder, &source_frame, &encoded_frame),
            "encode frame");
        handle_encoded_frame(&encoded_frame, frame_number);
    }

    check_encoder_result(esp_h264_enc_close(encoder), "close encoder");
    check_encoder_result(esp_h264_enc_del(encoder), "delete encoder");

    esp_h264_free(source_frame.raw_data.buffer);
    esp_h264_free(encoded_frame.raw_data.buffer);

    ESP_LOGI(TAG, "encoding complete");
}
