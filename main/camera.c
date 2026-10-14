/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * OV5647 MIPI-CSI capture and ESP32-P4 ISP pipeline.
 */

#include "camera.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_attr.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_h264_alloc.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "camera";

#define CAMERA_TRANSACTION_TIMEOUT_MS 1000U
#define CAMERA_SCCB_FREQUENCY_HZ 10000U
#define CAMERA_CSI_LANE_COUNT 2

struct camera {
    esp_cam_ctlr_handle_t controller;
    isp_proc_handle_t isp;
    esp_ldo_channel_handle_t mipi_ldo;
    esp_cam_sensor_device_t *sensor;
    i2c_master_bus_handle_t i2c_bus;
    esp_sccb_io_handle_t sccb_io;
    SemaphoreHandle_t frame_ready;

    esp_cam_ctlr_trans_t transaction;
    uint8_t *frame_buffer;
    size_t frame_size;
    uint32_t frame_buffer_size;

    bool controller_enabled;
    bool controller_started;
    bool isp_enabled;
};

/** Return one point of the camera gamma correction curve. */
static uint32_t isp_gamma_curve(uint32_t value)
{
    return (uint32_t)(pow((double)value / 256.0, 0.72) * 256.0);
}

/** Configure and enable all ISP corrections used by the camera pipeline. */
static esp_err_t configure_isp(isp_proc_handle_t isp)
{
    esp_isp_bf_config_t bf_config = {
        .denoising_level = 5,
        .padding_mode = ISP_BF_EDGE_PADDING_MODE_SRND_DATA,
        .bf_template = {
            {1, 2, 1},
            {2, 4, 2},
            {1, 2, 1},
        },
    };
    esp_err_t result = esp_isp_bf_configure(isp, &bf_config);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_bf_enable(isp);
    if (result != ESP_OK) {
        return result;
    }

    esp_isp_demosaic_config_t demosaic_config = {
        .grad_ratio = {
            .integer = 2,
            .decimal = 5,
        },
    };
    result = esp_isp_demosaic_configure(isp, &demosaic_config);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_demosaic_enable(isp);
    if (result != ESP_OK) {
        return result;
    }

    isp_gamma_curve_points_t gamma_points = {};
    result = esp_isp_gamma_fill_curve_points(isp_gamma_curve, &gamma_points);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_gamma_configure(isp, COLOR_COMPONENT_R, &gamma_points);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_gamma_configure(isp, COLOR_COMPONENT_G, &gamma_points);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_gamma_configure(isp, COLOR_COMPONENT_B, &gamma_points);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_gamma_enable(isp);
    if (result != ESP_OK) {
        return result;
    }

    /* OV5647 ISP calibration matrix from esp_cam_sensor's default profile. */
    esp_isp_ccm_config_t ccm_config = {
        .matrix = {
            { 2.0000f, -0.5459f, -0.4541f },
            {-0.4751f,  1.7696f, -0.2945f },
            {-0.2002f, -0.7998f,  2.0000f },
        },
        .saturation = true,
    };
    result = esp_isp_ccm_configure(isp, &ccm_config);
    if (result != ESP_OK) {
        return result;
    }

    result = esp_isp_ccm_enable(isp);
    if (result != ESP_OK) {
        return result;
    }

    esp_isp_color_config_t color_config = {
        .color_contrast = {
            .integer = 1,
            .decimal = 0,
        },
        .color_saturation = {
            .integer = 1,
            .decimal = 0,
        },
        .color_hue = 0,
        .color_brightness = 0,
    };
    result = esp_isp_color_configure(isp, &color_config);
    if (result != ESP_OK) {
        return result;
    }

    return esp_isp_color_enable(isp);
}

/** Give the CSI controller the camera module's reusable frame buffer. */
static bool IRAM_ATTR camera_get_new_buffer(esp_cam_ctlr_handle_t handle,
                                            esp_cam_ctlr_trans_t *transaction,
                                            void *user_data)
{
    (void)handle;

    const camera_t *camera = user_data;
    if (camera == NULL || transaction == NULL) {
        return false;
    }

    transaction->buffer = camera->transaction.buffer;
    transaction->buflen = camera->transaction.buflen;
    return false;
}

/** Notify the application task that one frame has been captured. */
static bool IRAM_ATTR camera_transaction_finished(esp_cam_ctlr_handle_t handle,
                                                  esp_cam_ctlr_trans_t *transaction,
                                                  void *user_data)
{
    (void)handle;
    (void)transaction;

    camera_t *camera = user_data;
    if (camera == NULL || camera->frame_ready == NULL) {
        return false;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(camera->frame_ready, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

/** Initialize the sensor, detect it over SCCB and select its format. */
static esp_err_t camera_sensor_init(camera_t *camera,
                                    const camera_config_t *config)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CONFIG_CAMERA_I2C_SDA,
        .scl_io_num = CONFIG_CAMERA_I2C_SCL,
        .i2c_port = I2C_NUM_0,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t result = i2c_new_master_bus(&i2c_bus_config, &camera->i2c_bus);
    if (result != ESP_OK) {
        return result;
    }

    esp_cam_sensor_config_t sensor_config = {
        .sccb_handle = NULL,
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
        .xclk_freq_hz = 0,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    for (esp_cam_sensor_detect_fn_t *detector =
             &__esp_cam_sensor_detect_fn_array_start;
         detector < &__esp_cam_sensor_detect_fn_array_end;
         ++detector) {
        if (detector->port != ESP_CAM_SENSOR_MIPI_CSI) {
            continue;
        }

        sccb_i2c_config_t sccb_config = {
            .scl_speed_hz = CAMERA_SCCB_FREQUENCY_HZ,
            .device_address = detector->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        result = sccb_new_i2c_io(camera->i2c_bus, &sccb_config,
                                 &sensor_config.sccb_handle);
        if (result != ESP_OK) {
            return result;
        }

        camera->sensor = detector->detect(&sensor_config);
        if (camera->sensor != NULL) {
            camera->sccb_io = sensor_config.sccb_handle;
            break;
        }

        result = esp_sccb_del_i2c_io(sensor_config.sccb_handle);
        sensor_config.sccb_handle = NULL;
        if (result != ESP_OK) {
            return result;
        }
    }

    if (camera->sensor == NULL) {
        ESP_LOGE(TAG, "failed to detect camera sensor");
        return ESP_ERR_NOT_FOUND;
    }

    const char *sensor_name = esp_cam_sensor_get_name(camera->sensor);
    if (sensor_name == NULL) {
        sensor_name = "unknown";
    }

    esp_cam_sensor_id_t sensor_id = {0};
    esp_err_t id_result = esp_cam_sensor_ioctl(
        camera->sensor, ESP_CAM_SENSOR_IOC_G_CHIP_ID, &sensor_id);
    if (id_result == ESP_OK) {
        ESP_LOGI(TAG,
                 "sensor detected directly: name=%s MID=0x%02x%02x "
                 "PID=0x%04x version=0x%02x",
                 sensor_name, (unsigned)sensor_id.midh,
                 (unsigned)sensor_id.midl, (unsigned)sensor_id.pid,
                 (unsigned)sensor_id.ver);
    } else {
        ESP_LOGW(TAG, "could not read the sensor chip ID: %s",
                 esp_err_to_name(id_result));
    }

    esp_cam_sensor_capability_t capability = {0};
    const bool capability_valid =
        esp_cam_sensor_get_capability(camera->sensor, &capability) == ESP_OK;
    if (capability_valid) {
        ESP_LOGI(TAG,
                 "sensor capabilities: RAW=%d RGB565=%d YUV=%d JPEG=%d",
                 (int)capability.fmt_raw, (int)capability.fmt_rgb565,
                 (int)capability.fmt_yuv, (int)capability.fmt_jpeg);
    }

    esp_cam_sensor_format_array_t formats = {0};
    result = esp_cam_sensor_query_format(camera->sensor, &formats);
    if (result != ESP_OK || formats.format_array == NULL || formats.count == 0) {
        ESP_LOGE(TAG, "failed to query sensor formats: %s",
                 esp_err_to_name(result));
        return result != ESP_OK ? result : ESP_ERR_NOT_FOUND;
    }

    const esp_cam_sensor_format_t *selected_format = NULL;
    for (uint32_t index = 0; index < formats.count; ++index) {
        const esp_cam_sensor_format_t *format = &formats.format_array[index];
        ESP_LOGI(TAG, "sensor format[%" PRIu32 "]: %s", index,
                 format->name != NULL ? format->name : "unknown");
        if (format->name != NULL &&
            strcmp(format->name, config->format_name) == 0) {
            selected_format = format;
        }
    }

    if (selected_format == NULL) {
        ESP_LOGE(TAG, "unsupported sensor format: %s", config->format_name);
        return ESP_ERR_INVALID_ARG;
    }

    result = esp_cam_sensor_set_format(camera->sensor, selected_format);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "failed to set sensor format %s: %s",
                 selected_format->name, esp_err_to_name(result));
        return result;
    }

    esp_cam_sensor_format_t active_format = {0};
    const bool active_format_valid =
        esp_cam_sensor_get_format(camera->sensor, &active_format) == ESP_OK;
    if (active_format_valid) {
        ESP_LOGI(TAG,
                 "sensor format in use: %s, %ux%u @ %u fps, "
                 "interface=%d lanes=%" PRIu32,
                 active_format.name != NULL ? active_format.name : "unknown",
                 (unsigned)active_format.width, (unsigned)active_format.height,
                 (unsigned)active_format.fps, (int)active_format.port,
                 active_format.mipi_info.lane_num);
    }

    int stream_enabled = 1;
    result = esp_cam_sensor_ioctl(camera->sensor,
                                  ESP_CAM_SENSOR_IOC_S_STREAM,
                                  &stream_enabled);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "failed to start sensor stream: %s",
                 esp_err_to_name(result));
    }
    return result;
}

/** Stop the sensor and release its SCCB and I2C resources. */
static void camera_sensor_deinit(camera_t *camera)
{
    if (camera == NULL) {
        return;
    }

    if (camera->sensor != NULL) {
        int stream_enabled = 0;
        esp_err_t result = esp_cam_sensor_ioctl(
            camera->sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enabled);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "failed to stop sensor stream: %s",
                     esp_err_to_name(result));
        }

        result = esp_cam_sensor_del_dev(camera->sensor);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "failed to delete sensor device: %s",
                     esp_err_to_name(result));
        }
        camera->sensor = NULL;
    }

    if (camera->sccb_io != NULL) {
        esp_err_t result = esp_sccb_del_i2c_io(camera->sccb_io);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "failed to delete SCCB interface: %s",
                     esp_err_to_name(result));
        }
        camera->sccb_io = NULL;
    }

    if (camera->i2c_bus != NULL) {
        esp_err_t result = i2c_del_master_bus(camera->i2c_bus);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "failed to delete I2C bus: %s",
                     esp_err_to_name(result));
        }
        camera->i2c_bus = NULL;
    }
}

/** Release initialized camera resources without freeing the camera object. */
static void camera_release_partial(camera_t *camera)
{
    if (camera == NULL) {
        return;
    }

    if (camera->controller_started) {
        esp_err_t result = esp_cam_ctlr_stop(camera->controller);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "camera stop failed during cleanup: %s",
                     esp_err_to_name(result));
        }
        camera->controller_started = false;
    }

    if (camera->controller_enabled) {
        esp_err_t result = esp_cam_ctlr_disable(camera->controller);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "camera disable failed during cleanup: %s",
                     esp_err_to_name(result));
        }
        camera->controller_enabled = false;
    }

    if (camera->controller != NULL) {
        esp_err_t result = esp_cam_ctlr_del(camera->controller);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "camera delete failed during cleanup: %s",
                     esp_err_to_name(result));
        }
        camera->controller = NULL;
    }

    if (camera->isp_enabled) {
        esp_err_t result = esp_isp_disable(camera->isp);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "ISP disable failed during cleanup: %s",
                     esp_err_to_name(result));
        }
        camera->isp_enabled = false;
    }

    if (camera->isp != NULL) {
        esp_err_t result = esp_isp_del_processor(camera->isp);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "ISP delete failed during cleanup: %s",
                     esp_err_to_name(result));
        }
        camera->isp = NULL;
    }

    camera_sensor_deinit(camera);

    if (camera->mipi_ldo != NULL) {
        esp_err_t result = esp_ldo_release_channel(camera->mipi_ldo);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "MIPI LDO release failed: %s", esp_err_to_name(result));
        }
        camera->mipi_ldo = NULL;
    }

    if (camera->frame_ready != NULL) {
        vSemaphoreDelete(camera->frame_ready);
        camera->frame_ready = NULL;
    }

    if (camera->frame_buffer != NULL) {
        esp_h264_free(camera->frame_buffer);
        camera->frame_buffer = NULL;
    }
}

esp_err_t camera_create(const camera_config_t *config, camera_t **out_camera)
{
    if (config == NULL || out_camera == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_camera = NULL;

    if (config->width == 0 || config->height == 0 ||
        (config->width & 1U) != 0 || (config->height & 1U) != 0 ||
        config->format_name == NULL || config->format_name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    /* YUV420 needs even dimensions; reject arithmetic overflow generically. */
    const uint64_t frame_size_u64 =
        ((uint64_t)config->width * (uint64_t)config->height * 3U) / 2U;
    if (frame_size_u64 == 0 || frame_size_u64 > UINT32_MAX ||
        frame_size_u64 > SIZE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t frame_size = (size_t)frame_size_u64;

    camera_t *camera = calloc(1, sizeof(*camera));
    if (camera == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_ldo_channel_config_t ldo_config = {
        .chan_id = CONFIG_CAMERA_MIPI_LDO_CHANNEL,
        .voltage_mv = CONFIG_CAMERA_MIPI_LDO_VOLTAGE,
    };
    esp_err_t result = esp_ldo_acquire_channel(&ldo_config, &camera->mipi_ldo);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    camera->frame_buffer = esp_h264_aligned_malloc(
        16, 1, (uint32_t)frame_size, &camera->frame_buffer_size,
        ESP_H264_MEM_SPIRAM);
    if (camera->frame_buffer == NULL) {
        camera_release_partial(camera);
        free(camera);
        return ESP_ERR_NO_MEM;
    }
    camera->frame_size = frame_size;

    camera->frame_ready = xSemaphoreCreateBinary();
    if (camera->frame_ready == NULL) {
        camera_release_partial(camera);
        free(camera);
        return ESP_ERR_NO_MEM;
    }

    result = camera_sensor_init(camera, config);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = config->width,
        .v_res = config->height,
        .lane_bit_rate_mbps = CONFIG_CAMERA_MIPI_LANE_BITRATE,
        .input_data_color_type = CAM_CTLR_COLOR_RAW10,
        .output_data_color_type = CAM_CTLR_COLOR_YUV420,
        .data_lane_num = CAMERA_CSI_LANE_COUNT,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    result = esp_cam_new_csi_ctlr(&csi_config, &camera->controller);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = camera_get_new_buffer,
        .on_trans_finished = camera_transaction_finished,
    };
    camera->transaction.buffer = camera->frame_buffer;
    camera->transaction.buflen = camera->frame_buffer_size;
    result = esp_cam_ctlr_register_event_callbacks(
        camera->controller, &callbacks, camera);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    result = esp_cam_ctlr_enable(camera->controller);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }
    camera->controller_enabled = true;

    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW10,
        .output_data_color_type = ISP_COLOR_YUV420,
        .yuv_range = ISP_COLOR_RANGE_FULL,
        .yuv_std = ISP_YUV_CONV_STD_BT601,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = config->width,
        .v_res = config->height,
        .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    result = esp_isp_new_processor(&isp_config, &camera->isp);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    result = esp_isp_enable(camera->isp);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }
    camera->isp_enabled = true;

    result = configure_isp(camera->isp);
    if (result != ESP_OK) {
        camera_release_partial(camera);
        free(camera);
        return result;
    }

    *out_camera = camera;
    return ESP_OK;
}

esp_err_t camera_start(camera_t *camera)
{
    if (camera == NULL || camera->controller == NULL ||
        !camera->controller_enabled) {
        return ESP_ERR_INVALID_ARG;
    }
    if (camera->controller_started) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = esp_cam_ctlr_start(camera->controller);
    if (result != ESP_OK) {
        return result;
    }
    camera->controller_started = true;

    result = esp_cam_ctlr_receive(camera->controller, &camera->transaction,
                                  CAMERA_TRANSACTION_TIMEOUT_MS);
    if (result != ESP_OK) {
        (void)esp_cam_ctlr_stop(camera->controller);
        camera->controller_started = false;
        return result;
    }

    return ESP_OK;
}

/** Convert milliseconds to a bounded, finite FreeRTOS timeout. */
static TickType_t camera_timeout_ticks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * configTICK_RATE_HZ + 999U) /
                     1000U;
    if (ticks >= (uint64_t)portMAX_DELAY) {
        /* Never pass portMAX_DELAY: FreeRTOS may interpret it as forever. */
        ticks = (uint64_t)portMAX_DELAY - 1U;
    }
    return (TickType_t)ticks;
}

esp_err_t camera_wait_frame(camera_t *camera, uint32_t timeout_ms)
{
    if (camera == NULL || camera->frame_ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return xSemaphoreTake(camera->frame_ready,
                          camera_timeout_ticks(timeout_ms)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

const uint8_t *camera_frame(const camera_t *camera)
{
    return camera != NULL ? camera->frame_buffer : NULL;
}

size_t camera_frame_size(const camera_t *camera)
{
    return camera != NULL ? camera->frame_size : 0;
}

void camera_destroy(camera_t *camera)
{
    if (camera == NULL) {
        return;
    }

    camera_release_partial(camera);
    free(camera);
}
