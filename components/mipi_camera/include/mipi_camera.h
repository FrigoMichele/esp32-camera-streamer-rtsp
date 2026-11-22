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

typedef enum {
    MIPI_LAYOUT_UNKNOWN = 0,
    MIPI_LAYOUT_P4_ISP_YUV420,  // Native packed UYY/VYY, no CSI byte swap.
} mipi_frame_layout_t;

typedef struct {
    const uint8_t *data;  // Valid until release.
    mipi_frame_layout_t layout;
    size_t size;           // Captured bytes, excluding bottom padding.
    size_t capacity;       // Includes initialized padding for the hardware encoder.
    uint32_t sequence;
    int64_t timestamp_us;
} mipi_frame_t;

// Packed YUV420 capacity with height rounded up to a 16-line macroblock.
static inline size_t mipi_camera_padded_frame_size(uint32_t width, uint32_t height)
{
    return (size_t)width * ((height + 15U) & ~15U) * 3 / 2;
}

// Initialize once, with the RAW10 sensor configured but not yet streaming.
esp_err_t mipi_camera_init(const mipi_camera_config_t *config);

// Wait up to timeout ticks for a frame; zero polls without blocking.
esp_err_t mipi_camera_get_frame(mipi_frame_t *frame, TickType_t timeout);

// Release each acquired frame exactly once; do not access its data afterward.
esp_err_t mipi_camera_release_frame(const mipi_frame_t *frame);
