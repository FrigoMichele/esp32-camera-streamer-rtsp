/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 hardware H.264 encoder.
 */

#include "encoder.h"

#include <stdbool.h>
#include <stdlib.h>

#include "esp_h264_alloc.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_log.h"

static const char *TAG = "encoder";

struct encoder {
    esp_h264_enc_handle_t handle;
    uint8_t *output_buffer;
    uint32_t output_buffer_size;
    uint32_t fps;
    uint32_t frame_number;
    bool opened;
};

/** Convert an H.264 component error to the application error type. */
static esp_err_t h264_error_to_esp(esp_h264_err_t result)
{
    switch (result) {
    case ESP_H264_ERR_ARG:
        return ESP_ERR_INVALID_ARG;
    case ESP_H264_ERR_MEM:
        return ESP_ERR_NO_MEM;
    case ESP_H264_ERR_TIMEOUT:
        return ESP_ERR_TIMEOUT;
    case ESP_H264_ERR_OVERFLOW:
        return ESP_ERR_INVALID_SIZE;
    case ESP_H264_ERR_OK:
        return ESP_OK;
    default:
        return ESP_FAIL;
    }
}

/** Log an H.264 operation failure. */
static void log_h264_error(esp_h264_err_t result, const char *operation)
{
    if (result != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "%s failed: %d", operation, (int)result);
    }
}

esp_err_t encoder_create(const encoder_config_t *config,
                         size_t input_frame_size,
                         encoder_t **out_encoder)
{
    if (config == NULL || out_encoder == NULL || input_frame_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_encoder = NULL;

    if (config->width < 80 || config->width > UINT16_MAX ||
        config->height < 80 || config->height > UINT16_MAX ||
        config->fps == 0 || config->fps > UINT8_MAX ||
        config->gop_size == 0 || config->gop_size > UINT8_MAX ||
        config->qp_value > 51 || input_frame_size > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_t *encoder = calloc(1, sizeof(*encoder));
    if (encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    encoder->output_buffer = esp_h264_aligned_malloc(
        16, 1, (uint32_t)input_frame_size, &encoder->output_buffer_size,
        ESP_H264_MEM_SPIRAM);
    if (encoder->output_buffer == NULL) {
        free(encoder);
        return ESP_ERR_NO_MEM;
    }
    encoder->fps = config->fps;

    esp_h264_enc_cfg_hw_t hw_config = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = (uint8_t)config->gop_size,
        .fps = (uint8_t)config->fps,
        .res = {
            .width = (uint16_t)config->width,
            .height = (uint16_t)config->height,
        },
        .rc = {
            .bitrate = config->bitrate,
            .qp_min = (uint8_t)config->qp_value,
            .qp_max = (uint8_t)config->qp_value,
        },
    };

    esp_h264_err_t h264_result = esp_h264_enc_hw_new(
        &hw_config, &encoder->handle);
    if (h264_result != ESP_H264_ERR_OK) {
        log_h264_error(h264_result, "create encoder");
        esp_h264_free(encoder->output_buffer);
        free(encoder);
        return h264_error_to_esp(h264_result);
    }

    h264_result = esp_h264_enc_open(encoder->handle);
    if (h264_result != ESP_H264_ERR_OK) {
        log_h264_error(h264_result, "open encoder");
        (void)esp_h264_enc_del(encoder->handle);
        esp_h264_free(encoder->output_buffer);
        free(encoder);
        return h264_error_to_esp(h264_result);
    }
    encoder->opened = true;

    *out_encoder = encoder;
    return ESP_OK;
}

esp_err_t encoder_encode(encoder_t *encoder,
                         const uint8_t *input,
                         size_t input_size,
                         encoder_frame_t *output)
{
    if (encoder == NULL || !encoder->opened || input == NULL ||
        input_size == 0 || input_size > UINT32_MAX || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_h264_enc_in_frame_t input_frame = {
        .raw_data = {
            .buffer = (uint8_t *)input,
            .len = (uint32_t)input_size,
        },
        .pts = (uint32_t)(((uint64_t)encoder->frame_number * 1000U) /
                          encoder->fps),
    };
    esp_h264_enc_out_frame_t output_frame = {
        .raw_data = {
            .buffer = encoder->output_buffer,
            .len = encoder->output_buffer_size,
        },
    };

    esp_h264_err_t h264_result = esp_h264_enc_process(
        encoder->handle, &input_frame, &output_frame);
    if (h264_result != ESP_H264_ERR_OK) {
        log_h264_error(h264_result, "encode frame");
        return h264_error_to_esp(h264_result);
    }

    output->data = output_frame.raw_data.buffer;
    output->length = output_frame.length;
    output->pts = output_frame.pts;
    ++encoder->frame_number;
    return ESP_OK;
}

void encoder_destroy(encoder_t *encoder)
{
    if (encoder == NULL) {
        return;
    }

    if (encoder->handle != NULL) {
        if (encoder->opened) {
            esp_h264_err_t result = esp_h264_enc_close(encoder->handle);
            if (result != ESP_H264_ERR_OK) {
                log_h264_error(result, "close encoder");
            }
            encoder->opened = false;
        }

        esp_h264_err_t result = esp_h264_enc_del(encoder->handle);
        if (result != ESP_H264_ERR_OK) {
            log_h264_error(result, "delete encoder");
        }
        encoder->handle = NULL;
    }

    if (encoder->output_buffer != NULL) {
        esp_h264_free(encoder->output_buffer);
    }
    free(encoder);
}
