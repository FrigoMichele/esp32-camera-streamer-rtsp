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
    /* Server send-callback polling rate (1..1000 Hz), NOT encoded video FPS.
     * The H.264 SPS and per-frame PTS carry the actual video timing. */
    uint32_t send_poll_hz;
    size_t max_frame_size;
    rtsp_network_config_t network;
} rtsp_config_t;

/* Diagnostics only: monotonic esp_timer timestamps for this access unit.
 * The first timestamp is a capture request, NOT sensor exposure time. */
typedef struct {
    uint32_t frame_id;
    int64_t capture_start_us;
    int64_t raw_ready_us;
    int64_t encode_end_us;
} rtsp_frame_timing_t;

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
 * timing is copied with the frame for diagnostics; it never affects RTP PTS.
 * Use one publishing task and stop it before destroying the server.
 */
esp_err_t rtsp_publish(rtsp_t *rtsp,
                       const uint8_t *data,
                       size_t length,
                       uint32_t pts,
                       const rtsp_frame_timing_t *timing);

/** Report/reset handoff diagnostics from the publishing task (~once/second). */
void rtsp_report_timing(rtsp_t *rtsp);

/** Stop the server and release resources; caller must first stop publishing. */
void rtsp_destroy(rtsp_t *rtsp);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_H */
