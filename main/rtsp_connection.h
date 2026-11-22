#pragma once

#include "h264_encoder.h"

// Start a video-only RTSP server task on the Function EV Board's Ethernet port.
esp_err_t rtsp_connection_start(uint32_t width, uint32_t height, uint8_t fps);

// Never blocks. If true, ownership transferred; otherwise the caller must release.
bool rtsp_connection_offer(const h264_frame_t *frame);
