#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    const uint8_t *data;
    size_t size;
    uint32_t sequence;
    int64_t timestamp_us;  // Timestamp of the source camera frame.
    int64_t encode_us;     // Elapsed time inside esp_h264_enc_process().
    bool is_idr;           // The encoder includes SPS/PPS with each IDR.
} h264_frame_t;

// Initialize once for the camera's native, unswapped ESP32-P4 ISP YUV420 output.
esp_err_t h264_encoder_init(uint32_t width, uint32_t height, uint8_t fps);
// Single producer: reserve an output slot, encode one camera frame, and publish it.
// ESP_ERR_TIMEOUT means no camera frame arrived. Other errors stop the pipeline.
esp_err_t h264_encoder_process_next(TickType_t camera_timeout);
// Single consumer: release each acquired output exactly once after using data.
esp_err_t h264_encoder_get_frame(h264_frame_t *frame, TickType_t timeout);
esp_err_t h264_encoder_release_frame(const h264_frame_t *frame);
