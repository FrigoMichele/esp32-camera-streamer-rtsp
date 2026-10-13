/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 OV5647 MIPI-CSI camera capture and hardware H.264 encoder.
 * The camera path is based on ESP-IDF examples/peripherals/camera.
 */

#include <inttypes.h>
#include <stdint.h>

#include "driver/isp.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_err.h"
#include "esp_h264_alloc.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "example_sensor_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "camera_encoder";

typedef struct {
    esp_cam_ctlr_trans_t transaction;
    SemaphoreHandle_t frame_ready;
} camera_context_t;

static void check_encoder_result(esp_h264_err_t result, const char *operation)
{
    if (result != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "%s failed: %d", operation, (int)result);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

static uint8_t *allocate_frame_buffer(uint32_t size, uint32_t *actual_size)
{
    uint8_t *buffer = esp_h264_aligned_malloc(
        16, 1, size, actual_size, ESP_H264_MEM_SPIRAM);

    ESP_ERROR_CHECK(buffer != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    return buffer;
}

static bool IRAM_ATTR camera_get_new_buffer(esp_cam_ctlr_handle_t handle,
                                            esp_cam_ctlr_trans_t *transaction,
                                            void *user_data)
{
    const camera_context_t *camera_context = user_data;

    transaction->buffer = camera_context->transaction.buffer;
    transaction->buflen = camera_context->transaction.buflen;
    return false;
}

static bool IRAM_ATTR camera_transaction_finished(esp_cam_ctlr_handle_t handle,
                                                  esp_cam_ctlr_trans_t *transaction,
                                                  void *user_data)
{
    camera_context_t *camera_context = user_data;
    BaseType_t higher_priority_task_woken = pdFALSE;

    xSemaphoreGiveFromISR(camera_context->frame_ready,
                           &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void handle_encoded_frame(const esp_h264_enc_out_frame_t *frame,
                                 uint32_t frame_number)
{
    ESP_LOGI(TAG, "frame=%" PRIu32 ", type=%d, bytes=%" PRIu32 ", pts=%" PRIu32,
             frame_number, frame->frame_type, frame->length, frame->pts);

    /* Send frame->raw_data.buffer and frame->length to the RTSP sink here. */
}

void app_main(void)
{
    const uint32_t width = CONFIG_H264_ENCODER_WIDTH;
    const uint32_t height = CONFIG_H264_ENCODER_HEIGHT;
    const uint32_t frame_size = width * height * 3U / 2U;
    const uint32_t frame_limit = CONFIG_H264_ENCODER_FRAME_COUNT;

    ESP_ERROR_CHECK((width == 800 && height == 640) ? ESP_OK : ESP_ERR_INVALID_ARG);

    esp_ldo_channel_handle_t mipi_ldo = NULL;
    esp_cam_ctlr_handle_t camera = NULL;
    isp_proc_handle_t isp = NULL;
    example_sensor_handle_t sensor = {0};
    esp_h264_enc_handle_t encoder = NULL;
    SemaphoreHandle_t frame_ready = xSemaphoreCreateBinary();
    uint8_t *camera_frame = NULL;
    uint8_t *encoded_data = NULL;
    uint32_t camera_frame_size = 0;
    uint32_t encoded_buffer_size = 0;

    ESP_ERROR_CHECK(frame_ready ? ESP_OK : ESP_ERR_NO_MEM);

    // Power the MIPI CSI PHY.
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = CONFIG_CAMERA_MIPI_LDO_CHANNEL,
        .voltage_mv = CONFIG_CAMERA_MIPI_LDO_VOLTAGE,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &mipi_ldo));

    camera_frame = allocate_frame_buffer(frame_size, &camera_frame_size);
    encoded_data = allocate_frame_buffer(frame_size, &encoded_buffer_size);

    camera_context_t camera_context = {
        .transaction = {
            .buffer = camera_frame,
            .buflen = camera_frame_size,
        },
        .frame_ready = frame_ready,
    };

    // Configure the OV5647 over I2C/SCCB.
    example_sensor_config_t sensor_config = {
        .i2c_port_num = I2C_NUM_0,
        .i2c_sda_io_num = CONFIG_CAMERA_I2C_SDA,
        .i2c_scl_io_num = CONFIG_CAMERA_I2C_SCL,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name = "MIPI_2lane_24Minput_RAW8_800x640_50fps",
    };
    example_sensor_init(&sensor_config, &sensor);

    // Capture RAW8 from the board's MIPI camera port.
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = width,
        .v_res = height,
        .lane_bit_rate_mbps = CONFIG_CAMERA_MIPI_LANE_BITRATE,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_YUV420,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_config, &camera));

    esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = camera_get_new_buffer,
        .on_trans_finished = camera_transaction_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(
        camera, &callbacks, &camera_context));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(camera));

    // Convert the Bayer image to YUV420 for the H.264 encoder.
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_YUV420,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = width,
        .v_res = height,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp));
    ESP_ERROR_CHECK(esp_isp_enable(isp));

    // Configure the ESP32-P4 hardware H.264 encoder.
    esp_h264_enc_cfg_hw_t encoder_config = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = CONFIG_H264_ENCODER_GOP_SIZE,
        .fps = CONFIG_H264_ENCODER_FPS,
        .res = {
            .width = width,
            .height = height,
        },
        .rc = {
            .bitrate = CONFIG_H264_ENCODER_BITRATE,
            .qp_min = CONFIG_H264_ENCODER_QP_VALUE,
            .qp_max = CONFIG_H264_ENCODER_QP_VALUE,
        },
    };
    esp_h264_enc_in_frame_t input = {
        .raw_data = {
            .buffer = camera_frame,
            .len = frame_size,
        },
    };
    esp_h264_enc_out_frame_t output = {
        .raw_data = {
            .buffer = encoded_data,
            .len = encoded_buffer_size,
        },
    };

    check_encoder_result(esp_h264_enc_hw_new(&encoder_config, &encoder), "create encoder");
    check_encoder_result(esp_h264_enc_open(encoder), "open encoder");

    ESP_ERROR_CHECK(esp_cam_ctlr_start(camera));
    ESP_ERROR_CHECK(esp_cam_ctlr_receive(
        camera, &camera_context.transaction, ESP_CAM_CTLR_MAX_DELAY));
    ESP_LOGI(TAG, "capturing OV5647 %" PRIu32 "x%" PRIu32 " frames", width, height);

    uint32_t frame_number = 0;
    // Capture and encode frames until the configured limit is reached.
    while (frame_limit == 0 || frame_number < frame_limit) {
        ESP_ERROR_CHECK(xSemaphoreTake(frame_ready, portMAX_DELAY) == pdTRUE
                            ? ESP_OK
                            : ESP_ERR_TIMEOUT);

        input.pts = frame_number * (1000U / CONFIG_H264_ENCODER_FPS);
        check_encoder_result(esp_h264_enc_process(encoder, &input, &output), "encode frame");
        handle_encoded_frame(&output, frame_number);
        ++frame_number;
    }

    check_encoder_result(esp_h264_enc_close(encoder), "close encoder");
    check_encoder_result(esp_h264_enc_del(encoder), "delete encoder");
    ESP_ERROR_CHECK(esp_cam_ctlr_stop(camera));
    ESP_ERROR_CHECK(esp_cam_ctlr_disable(camera));
    ESP_ERROR_CHECK(esp_cam_ctlr_del(camera));
    ESP_ERROR_CHECK(esp_isp_disable(isp));
    ESP_ERROR_CHECK(esp_isp_del_processor(isp));
    example_sensor_deinit(sensor);
    ESP_ERROR_CHECK(esp_ldo_release_channel(mipi_ldo));

    vSemaphoreDelete(frame_ready);
    esp_h264_free(camera_frame);
    esp_h264_free(encoded_data);
    ESP_LOGI(TAG, "capture complete");
}
