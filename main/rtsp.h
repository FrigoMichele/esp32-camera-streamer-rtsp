/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RTSP_H
#define RTSP_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque Ethernet-backed RTSP server. */
typedef struct rtsp rtsp_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    size_t max_frame_size;
} rtsp_config_t;

/** Start Ethernet and an H.264 RTSP server. */
esp_err_t rtsp_create(const rtsp_config_t *config, rtsp_t **out_rtsp);

/**
 * Publish one encoded access unit.
 *
 * The data is copied before this function returns, so the caller may reuse
 * its encoder buffer immediately afterwards.
 */
esp_err_t rtsp_publish(rtsp_t *rtsp,
                       const uint8_t *data,
                       size_t length,
                       uint32_t pts);

/** Stop the server and release Ethernet and RTSP resources. */
void rtsp_destroy(rtsp_t *rtsp);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_H */
