#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/color_types.h"

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t lane_bit_rate_mbps;
    uint8_t lane_count;
    color_raw_element_order_t bayer_order;
    bool has_line_start_packet;
    bool has_line_end_packet;
} mipi_camera_config_t;

typedef struct {
    const uint8_t *data;  // YUV420 in native ISP layout; valid until release.
    size_t size;
    uint32_t sequence;
    int64_t timestamp_us;
} mipi_frame_t;

// Initialize once, with the RAW8 sensor configured but not yet streaming.
esp_err_t mipi_camera_init(const mipi_camera_config_t *config);

// Wait up to timeout ticks for a frame; zero polls without blocking.
esp_err_t mipi_camera_get_frame(mipi_frame_t *frame, TickType_t timeout);

// Release each acquired frame exactly once; do not access its data afterward.
esp_err_t mipi_camera_release_frame(const mipi_frame_t *frame);
