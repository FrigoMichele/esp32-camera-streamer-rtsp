/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque camera and ISP pipeline. */
typedef struct camera camera_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    const char *format_name;
} camera_config_t;

/**
 * Create and configure the OV5647, CSI receiver and ISP pipeline.
 *
 * The caller supplies the sensor dimensions and format name. The ISP output
 * is the packed YUV420 format expected by the hardware H.264 encoder.
 */
esp_err_t camera_create(const camera_config_t *config, camera_t **out_camera);

/** Start asynchronous CSI capture. */
esp_err_t camera_start(camera_t *camera);

/**
 * Return the live YUV buffer without waiting for a complete frame.
 *
 * The pointer remains owned by the camera and is valid until the camera is
 * destroyed. CSI continuously overwrites it, even while the caller reads it.
 * This unsynchronized intermediate test can return incomplete/torn frames.
 */
const uint8_t *camera_frame(const camera_t *camera);

/** Return the size, in bytes, of one captured frame. */
size_t camera_frame_size(const camera_t *camera);

/** Stop capture and release all camera and ISP resources. */
void camera_destroy(camera_t *camera);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */
