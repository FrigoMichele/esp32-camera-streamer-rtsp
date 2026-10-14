/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque hardware H.264 encoder. */
typedef struct encoder encoder_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t bitrate;
    uint32_t gop_size;
    uint32_t qp_value;
} encoder_config_t;

typedef struct {
    const uint8_t *data;
    size_t length;
    uint32_t pts;
} encoder_frame_t;

/**
 * Create and open a hardware H.264 encoder.
 *
 * The encoder allocates its output buffer. The buffer is reused for every
 * call to encoder_encode(), so the returned frame must be consumed before
 * the next encode call.
 */
esp_err_t encoder_create(const encoder_config_t *config,
                         size_t input_frame_size,
                         encoder_t **out_encoder);

/** Encode one YUV420 frame. */
esp_err_t encoder_encode(encoder_t *encoder,
                         const uint8_t *input,
                         size_t input_size,
                         encoder_frame_t *output);

/** Close and release the encoder. */
void encoder_destroy(encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
