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
 * The data is transferred by AXI-GDMA before this function returns, so the
 * caller may reuse its encoder buffer immediately afterwards. A mutex protects
 * the shared frame bytes, length and PTS during publication and RTSP transfers.
 * Calls block until DMA completes; DMA errors are returned without a CPU-copy
 * fallback. Buffers must satisfy the IDF DMA driver's memory/alignment rules.
 * Callbacks return zero bytes before the first publish and may repeat the
 * latest access unit; this is a
 * latest-frame buffer, not a queue.
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
