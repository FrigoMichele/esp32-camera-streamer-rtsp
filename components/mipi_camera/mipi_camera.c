#include "mipi_camera.h"

#include <string.h>
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr_types.h"
#include "esp_heap_caps.h"
#include "driver/isp.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define FRAME_COUNT 3

static const char *TAG = "MIPI_CAMERA";

static esp_cam_ctlr_handle_t s_camera;
static isp_proc_handle_t s_isp;
static QueueHandle_t s_free_queue;  // Buffer pointers available to CSI.
static QueueHandle_t s_ready_queue; // Completed frames waiting for the camera task.

static size_t s_frame_size;
static size_t s_frame_capacity;
static uint32_t s_sequence;
static mipi_frame_layout_t s_layout;

// Supply a free DMA buffer without blocking.
static bool IRAM_ATTR camera_next_buffer(esp_cam_ctlr_handle_t camera,
                                        esp_cam_ctlr_trans_t *trans, void *arg)
{
    (void)camera;
    (void)arg;
    uint8_t *buffer;
    BaseType_t woken = pdFALSE;

    // CSI asks for the first buffer in task context and subsequent buffers in the ISR.
    BaseType_t ready = xPortInIsrContext()
        ? xQueueReceiveFromISR(s_free_queue, &buffer, &woken)
        : xQueueReceive(s_free_queue, &buffer, 0);
    if (ready == pdTRUE) {
        trans->buffer = buffer;
        trans->buflen = s_frame_size;
        // ESP-IDF v5.5.2 uses this value for cache invalidation on completion.
        trans->received_size = s_frame_size;
    }
    // If no buffer is free, CSI uses its own discard buffer.
    return woken == pdTRUE;
}

// Timestamp and publish a completed frame from the CSI ISR.
static bool IRAM_ATTR camera_frame_done(esp_cam_ctlr_handle_t camera,
                                       esp_cam_ctlr_trans_t *trans, void *arg)
{
    (void)camera;
    (void)arg;
    // CSI does not report completions for its private discard buffer.
    uint8_t *buffer = trans->buffer;
    mipi_frame_t frame = {
        .data = buffer,
        .layout = s_layout,
        .size = trans->received_size,
        .capacity = s_frame_capacity,
        .sequence = ++s_sequence,
        .timestamp_us = esp_timer_get_time(),
    };
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_ready_queue, &frame, &woken) != pdTRUE) {
        // If the frame cannot be queued, recycle its buffer.
        xQueueSendFromISR(s_free_queue, &buffer, &woken);
    }
    return woken == pdTRUE;
}

// Allocate the frame pool and arm CSI before the sensor starts streaming.
esp_err_t mipi_camera_init(
    const mipi_camera_config_t *config)
{
    if (!config || !config->width ||
        !config->height || !config->lane_count ||
        !config->lane_bit_rate_mbps) {
        ESP_LOGE(TAG, "Camera config requires width, height, lane_count, and lane_bit_rate_mbps");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_camera) {
        return ESP_ERR_INVALID_STATE;
    }

    if ((config->width | config->height) & 1U) {
        ESP_LOGE(TAG, "YUV420 requires even width and height");
        return ESP_ERR_INVALID_ARG;
    }

    // YUV420 uses 12 bits per pixel.
    s_frame_size = (size_t)config->width * config->height * 3 / 2;
    s_frame_capacity = mipi_camera_padded_frame_size(config->width, config->height);

    s_free_queue = xQueueCreate(
        FRAME_COUNT, sizeof(uint8_t *));

    s_ready_queue = xQueueCreate(
        FRAME_COUNT, sizeof(mipi_frame_t));

    if (!s_free_queue || !s_ready_queue) {
        return ESP_ERR_NO_MEM;
    }

    // Allocate aligned PSRAM buffers and mark them all free.
    for (uint8_t i = 0; i < FRAME_COUNT; i++) {
        uint8_t *buffer = heap_caps_aligned_alloc(
            64,
            s_frame_capacity,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (!buffer) {
            return ESP_ERR_NO_MEM;
        }

        // CSI writes only the visible rows. Initialize the encoder's bottom
        // padding once, before DMA starts; 0x80 is neutral gray in packed YUV420.
        if (s_frame_capacity > s_frame_size) {
            memset(buffer + s_frame_size, 0x80, s_frame_capacity - s_frame_size);
            esp_err_t err = esp_cache_msync(buffer, s_frame_capacity, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            if (err != ESP_OK) {
                heap_caps_free(buffer);
                return err;
            }
        }

        xQueueSend(s_free_queue, &buffer, 0);
    }

    // Configure CSI geometry, lane settings, and pixel formats.
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = config->width,
        .v_res = config->height,
        .lane_bit_rate_mbps =
            config->lane_bit_rate_mbps,
        .input_data_color_type =
            CAM_CTLR_COLOR_RAW10,
        .output_data_color_type =
            CAM_CTLR_COLOR_YUV420,
        .data_lane_num = config->lane_count,
        .byte_swap_en = false,
        .queue_items = FRAME_COUNT,
    };

    esp_err_t err = esp_cam_new_csi_ctlr(
        &csi_cfg, &s_camera);

    if (err != ESP_OK) {
        return err;
    }

    // Register buffer handoff callbacks before enabling the controller.
    const esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = camera_next_buffer,
        .on_trans_finished = camera_frame_done,
    };
    err = esp_cam_ctlr_register_event_callbacks(s_camera, &callbacks, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_cam_ctlr_enable(s_camera);
    if (err != ESP_OK) {
        return err;
    }

    // The ISP converts the sensor's RAW10 Bayer data to YUV420.
    const esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 160000000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW10,
        .output_data_color_type = ISP_COLOR_YUV420,
        .has_line_start_packet = config->has_line_start_packet,
        .has_line_end_packet = config->has_line_end_packet,
        .h_res = config->width,
        .v_res = config->height,
        .bayer_order = config->bayer_order,
    };
    err = esp_isp_new_processor(&isp_cfg, &s_isp);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_isp_enable(s_isp);
    if (err != ESP_OK) {
        return err;
    }

    // Only advertise the layout verified for this ISP/CSI configuration.
    s_layout = (isp_cfg.output_data_color_type == ISP_COLOR_YUV420 &&
                csi_cfg.output_data_color_type == CAM_CTLR_COLOR_YUV420 && !csi_cfg.byte_swap_en)
               ? MIPI_LAYOUT_P4_ISP_YUV420 : MIPI_LAYOUT_UNKNOWN;
    err = esp_cam_ctlr_start(s_camera);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized");
    return ESP_OK;
}

// Take a completed frame from the ready queue without copying its pixels.
esp_err_t mipi_camera_get_frame(
    mipi_frame_t *frame,
    TickType_t timeout)
{
    if (!frame || !s_ready_queue) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueReceive(s_ready_queue, frame,
                      timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

// Return the frame's buffer to the free pool.
esp_err_t mipi_camera_release_frame(
    const mipi_frame_t *frame)
{
    if (!frame || !s_free_queue || !frame->data) {
        return ESP_ERR_INVALID_ARG;
    }

    // Pixel data is read-only to callers but writable by CSI DMA.
    uint8_t *buffer = (uint8_t *)frame->data;
    return xQueueSend(s_free_queue, &buffer, 0)
        == pdTRUE ? ESP_OK : ESP_FAIL;
}
