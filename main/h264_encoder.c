#include "h264_encoder.h"

#include "esp_check.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "mipi_camera.h"

#define OUTPUT_COUNT 2

static const char *TAG = "h264";
static esp_h264_enc_handle_t s_encoder;
static uint32_t s_frame_bytes;
static QueueHandle_t s_free;
static QueueHandle_t s_ready;

static void delete_output_pool(void)
{
    if (s_free) {
        uint8_t *slot;
        while (xQueueReceive(s_free, &slot, 0) == pdTRUE) {
            heap_caps_free(slot);
        }
        vQueueDelete(s_free);
        s_free = NULL;
    }
    if (s_ready) {
        vQueueDelete(s_ready);
        s_ready = NULL;
    }
}

static esp_err_t create_output_pool(void)
{
    s_free = xQueueCreate(OUTPUT_COUNT, sizeof(uint8_t *));
    s_ready = xQueueCreate(OUTPUT_COUNT, sizeof(h264_frame_t));
    ESP_RETURN_ON_FALSE(s_free && s_ready, ESP_ERR_NO_MEM, TAG,
                        "Could not create output queues");

    for (unsigned i = 0; i < OUTPUT_COUNT; ++i) {
        uint8_t *slot = heap_caps_aligned_alloc(64, s_frame_bytes,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_RETURN_ON_FALSE(slot, ESP_ERR_NO_MEM, TAG,
                            "Could not allocate output slot %u", i);
        if (xQueueSend(s_free, &slot, 0) != pdTRUE) {
            heap_caps_free(slot);
            ESP_LOGE(TAG, "Could not enqueue output slot %u", i);
            return ESP_ERR_INVALID_STATE;
        }
    }
    return ESP_OK;
}

esp_err_t h264_encoder_init(uint32_t width, uint32_t height, uint8_t fps)
{
    ESP_RETURN_ON_FALSE(!s_encoder, ESP_ERR_INVALID_STATE, TAG, "Already initialized");

    // The HW reads whole 16x16 macroblocks. Never invent padding for a CSI frame.
    ESP_RETURN_ON_FALSE(width >= 80 && width <= 1920 && height >= 80 && height <= 2032 &&
                        (width % 16) == 0 && (height % 16) == 0 && fps,
                        ESP_ERR_NOT_SUPPORTED, TAG, "Unsupported camera geometry or frame rate");

    const esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .res = {.width = width, .height = height},
        .fps = fps,
        .gop = fps,
        .rc = {.bitrate = 4000000, .qp_min = 20, .qp_max = 40},
    };
    esp_h264_err_t rc = esp_h264_enc_hw_new(&cfg, &s_encoder);
    ESP_RETURN_ON_FALSE(rc == ESP_H264_ERR_OK, ESP_FAIL, TAG, "HW encoder creation failed: %d", rc);

    // esp_h264 recommends output capacity >= the macroblock-aligned raw frame size.
    s_frame_bytes = width * height * 3 / 2;
    esp_err_t err = create_output_pool();
    if (err != ESP_OK) {
        delete_output_pool();
        esp_h264_enc_del(s_encoder);
        s_encoder = NULL;
        return err;
    }

    rc = esp_h264_enc_open(s_encoder);
    if (rc != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "HW encoder open failed: %d", rc);
        delete_output_pool();
        esp_h264_enc_del(s_encoder);
        s_encoder = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t return_slot(const uint8_t *slot)
{
    ESP_RETURN_ON_FALSE(xQueueSend(s_free, &slot, 0) == pdTRUE,
                        ESP_ERR_INVALID_STATE, TAG, "Could not return output slot");
    return ESP_OK;
}

static esp_err_t encode_frame(const mipi_frame_t *raw, uint8_t *slot, h264_frame_t *encoded)
{
    if (raw->layout != MIPI_LAYOUT_P4_ISP_YUV420 || raw->size != s_frame_bytes ||
        !raw->data || ((uintptr_t)raw->data % 16) != 0) {
        ESP_LOGE(TAG, "Incompatible camera frame: expected native P4 ISP YUV420, "
                 "%lu bytes, 16-byte aligned", (unsigned long)s_frame_bytes);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // esp_h264's input API isn't const-qualified; DMA only reads this buffer.
    esp_h264_enc_in_frame_t in = {
        .raw_data = {.buffer = (uint8_t *)raw->data, .len = raw->size},
        .pts = (uint32_t)(raw->timestamp_us / 1000),
    };
    esp_h264_enc_out_frame_t out = {
        .raw_data = {.buffer = slot, .len = s_frame_bytes},
    };

    // Measure only the blocking encoder call, not queue waits or logging.
    int64_t start_us = esp_timer_get_time();
    esp_h264_err_t rc = esp_h264_enc_process(s_encoder, &in, &out);
    int64_t encode_us = esp_timer_get_time() - start_us;

    if (rc != ESP_H264_ERR_OK || !out.length || out.length > s_frame_bytes) {
        ESP_LOGE(TAG, "HW encode failed: %d (%lu bytes)", rc, (unsigned long)out.length);

        // Quiesce DMA before either buffer can be reused. Do not recycle on close failure.
        ESP_ERROR_CHECK(esp_h264_enc_close(s_encoder) == ESP_H264_ERR_OK ? ESP_OK : ESP_FAIL);
        return ESP_FAIL;
    }

    *encoded = (h264_frame_t) {
        .data = slot,
        .size = out.length,
        .sequence = raw->sequence,
        .timestamp_us = raw->timestamp_us,
        .encode_us = encode_us,
        .is_idr = out.frame_type == ESP_H264_FRAME_TYPE_IDR,
    };
    return ESP_OK;
}

esp_err_t h264_encoder_process_next(TickType_t camera_timeout)
{
    // Wait for output space without holding a camera buffer or blocking CSI DMA.
    uint8_t *slot;
    ESP_RETURN_ON_FALSE(xQueueReceive(s_free, &slot, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "Output pool unavailable");

    mipi_frame_t raw;
    esp_err_t err = mipi_camera_get_frame(&raw, camera_timeout);
    if (err != ESP_OK) {
        // The caller logs normal camera timeouts.
        if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Could not acquire camera frame: %s", esp_err_to_name(err));
        }
        esp_err_t slot_err = return_slot(slot);
        return slot_err == ESP_OK ? err : slot_err;
    }

    h264_frame_t encoded;
    err = encode_frame(&raw, slot, &encoded);
    esp_err_t release_err = mipi_camera_release_frame(&raw);
    if (release_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not release camera frame: %s", esp_err_to_name(release_err));
        return_slot(slot);
        return release_err;
    }
    if (err != ESP_OK) {
        return_slot(slot);
        return err;
    }

    if (xQueueSend(s_ready, &encoded, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Could not publish encoded frame");
        return_slot(slot);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t h264_encoder_get_frame(h264_frame_t *frame, TickType_t timeout)
{
    return xQueueReceive(s_ready, frame, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t h264_encoder_release_frame(const h264_frame_t *frame)
{
    return return_slot(frame->data);
}
