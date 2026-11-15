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

/**
 * Ethernet and RTSP endpoint configuration.
 *
 * The string pointers must remain valid while the RTSP handle is active.
 */
typedef struct {
    uint16_t server_port;
    const char *server_address;
    const char *server_uri;
    const char *netmask;
    const char *gateway;
} rtsp_network_config_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    size_t max_frame_size;
    rtsp_network_config_t network;
} rtsp_config_t;

/** Start Ethernet and RTSP immediately, without waiting for link/IP readiness. */
esp_err_t rtsp_create(const rtsp_config_t *config, rtsp_t **out_rtsp);

/**
 * Publish one encoded access unit.
 *
 * Copy into a latest-frame buffer without waiting for a viewer. A mutex
 * protects bytes and metadata during copies; the caller may reuse its encoder
 * buffer after return. Callbacks consume the newest unread frame at most once.
 * Unread frames may be overwritten. Losing H.264 reference frames can cause
 * decoding errors until the next keyframe; this is not a lossless queue.
 * Use one publishing task and stop it before destroying the server.
 */
esp_err_t rtsp_publish(rtsp_t *rtsp,
                       const uint8_t *data,
                       size_t length,
                       uint32_t pts);

/** Stop the server and release resources; caller must first stop publishing. */
void rtsp_destroy(rtsp_t *rtsp);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_H */
