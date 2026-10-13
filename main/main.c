/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 OV5647 MIPI-CSI camera capture and hardware H.264 encoder.
 * The camera path is based on ESP-IDF examples/peripherals/camera.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/isp.h"
#include "esp_attr.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_h264_alloc.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rtsp.h"
#include "example_sensor_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "media_lib_adapter.h"
#include "sdkconfig.h"

static const char *TAG = "camera_encoder";

#define RTSP_SERVER_PORT 8554
#define RTSP_SERVER_ADDRESS "192.168.2.5"
#define RTSP_SERVER_URI "rtsp://" RTSP_SERVER_ADDRESS ":8554/live"

/* ESP32-P4 Function EV Board Ethernet (IP101, RMII). */
#define ETHERNET_PHY_ADDR 1
#define ETHERNET_PHY_RESET_GPIO 51
#define ETHERNET_MDC_GPIO 31
#define ETHERNET_MDIO_GPIO 52
#define ETHERNET_RMII_CLK_GPIO 50
#define ETHERNET_RMII_TX_EN_GPIO 49
#define ETHERNET_RMII_TXD0_GPIO 34
#define ETHERNET_RMII_TXD1_GPIO 35
#define ETHERNET_RMII_CRS_DV_GPIO 28
#define ETHERNET_RMII_RXD0_GPIO 29
#define ETHERNET_RMII_RXD1_GPIO 30
#define ETHERNET_NETMASK "255.255.255.0"
#define ETHERNET_GATEWAY "192.168.2.1"
#define ETHERNET_CONNECT_TIMEOUT_MS 10000

typedef struct {
    esp_cam_ctlr_trans_t transaction;
    SemaphoreHandle_t frame_ready;
} camera_context_t;

typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t frame_ready;
    uint8_t *latest_frame;
    size_t frame_capacity;
    size_t frame_length;
    uint32_t frame_pts;
    esp_rtsp_handle_t server;
    esp_rtsp_video_info_t video_info;
    esp_rtsp_data_cb_t callbacks;
    esp_rtsp_config_t config;
} rtsp_context_t;

static rtsp_context_t s_rtsp;
static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static SemaphoreHandle_t s_eth_ip_ready;

static esp_err_t configure_ethernet_static_ip(void)
{
    esp_err_t result = esp_netif_dhcpc_stop(s_eth_netif);
    if (result != ESP_OK && result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return result;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr = esp_ip4addr_aton(RTSP_SERVER_ADDRESS);
    ip_info.netmask.addr = esp_ip4addr_aton(ETHERNET_NETMASK);
    ip_info.gw.addr = esp_ip4addr_aton(ETHERNET_GATEWAY);
    return esp_netif_set_ip_info(s_eth_netif, &ip_info);
}

static void ethernet_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == ETH_EVENT) {
        switch (event_id) {
        case ETHERNET_EVENT_CONNECTED: {
            esp_err_t result = configure_ethernet_static_ip();
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure Ethernet static IP: %s",
                         esp_err_to_name(result));
            } else {
                ESP_LOGI(TAG, "Ethernet link up");
            }
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet link down");
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&event->ip_info.ip));
        if (s_eth_ip_ready != NULL) {
            xSemaphoreGive(s_eth_ip_ready);
        }
    }
}

static void start_ethernet(void)
{
    esp_err_t result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(result);
    }

    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(result);
    }

    s_eth_ip_ready = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_eth_ip_ready != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETHERNET_PHY_ADDR;
    phy_config.reset_gpio_num = ETHERNET_PHY_RESET_GPIO;

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = ETHERNET_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = ETHERNET_MDIO_GPIO;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio =
        (emac_rmii_clock_gpio_t)ETHERNET_RMII_CLK_GPIO;
    emac_config.emac_dataif_gpio.rmii.tx_en_num = ETHERNET_RMII_TX_EN_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd0_num = ETHERNET_RMII_TXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd1_num = ETHERNET_RMII_TXD1_GPIO;
    emac_config.emac_dataif_gpio.rmii.crs_dv_num = ETHERNET_RMII_CRS_DV_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd0_num = ETHERNET_RMII_RXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd1_num = ETHERNET_RMII_RXD1_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    ESP_ERROR_CHECK(mac != NULL && phy != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_config);
    ESP_ERROR_CHECK(s_eth_netif != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(glue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               ethernet_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               ethernet_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    if (xSemaphoreTake(s_eth_ip_ready,
                       pdMS_TO_TICKS(ETHERNET_CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Ethernet did not obtain %s within %d ms",
                 RTSP_SERVER_ADDRESS, ETHERNET_CONNECT_TIMEOUT_MS);
        ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
    }
}

static uint32_t isp_gamma_curve(uint32_t value)
{
    return (uint32_t)(pow((double)value / 256.0, 0.72) * 256.0);
}

static void configure_isp(isp_proc_handle_t isp)
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
    ESP_ERROR_CHECK(esp_isp_bf_configure(isp, &bf_config));
    ESP_ERROR_CHECK(esp_isp_bf_enable(isp));

    esp_isp_demosaic_config_t demosaic_config = {
        .grad_ratio = {
            .integer = 2,
            .decimal = 5,
        },
    };
    ESP_ERROR_CHECK(esp_isp_demosaic_configure(isp, &demosaic_config));
    ESP_ERROR_CHECK(esp_isp_demosaic_enable(isp));

    isp_gamma_curve_points_t gamma_points = {};
    ESP_ERROR_CHECK(esp_isp_gamma_fill_curve_points(isp_gamma_curve, &gamma_points));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp, COLOR_COMPONENT_R, &gamma_points));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp, COLOR_COMPONENT_G, &gamma_points));
    ESP_ERROR_CHECK(esp_isp_gamma_configure(isp, COLOR_COMPONENT_B, &gamma_points));
    ESP_ERROR_CHECK(esp_isp_gamma_enable(isp));

    /* OV5647 ISP calibration matrix from esp_cam_sensor's default profile. */
    esp_isp_ccm_config_t ccm_config = {
        .matrix = {
            { 2.0000f, -0.5459f, -0.4541f },
            {-0.4751f,  1.7696f, -0.2945f },
            {-0.2002f, -0.7998f,  2.0000f },
        },
        .saturation = true,
    };
    ESP_ERROR_CHECK(esp_isp_ccm_configure(isp, &ccm_config));
    ESP_ERROR_CHECK(esp_isp_ccm_enable(isp));

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
    ESP_ERROR_CHECK(esp_isp_color_configure(isp, &color_config));
    ESP_ERROR_CHECK(esp_isp_color_enable(isp));
}

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

static int rtsp_state_handler(esp_rtsp_state_t state, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "RTSP state: %d", (int)state);
    return ESP_OK;
}

static int rtsp_send_video(unsigned char *data, unsigned int *length,
                           uint32_t *pts, void *ctx)
{
    rtsp_context_t *stream = ctx;

    if (stream == NULL || data == NULL || length == NULL || pts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The RTSP task asks for a frame when the client is in PLAY state. */
    if (xSemaphoreTake(stream->frame_ready, pdMS_TO_TICKS(1000)) != pdTRUE) {
        *length = 0;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(stream->mutex, portMAX_DELAY) != pdTRUE) {
        *length = 0;
        return ESP_FAIL;
    }

    if (stream->frame_length > stream->frame_capacity) {
        xSemaphoreGive(stream->mutex);
        *length = 0;
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(data, stream->latest_frame, stream->frame_length);
    *length = (unsigned int)stream->frame_length;
    *pts = stream->frame_pts;
    xSemaphoreGive(stream->mutex);
    return ESP_OK;
}

static void rtsp_publish_video(const uint8_t *data, size_t length, uint32_t pts)
{
    if (data == NULL || length == 0 || length > s_rtsp.frame_capacity) {
        ESP_LOGW(TAG, "Dropping H.264 frame of %zu bytes", length);
        return;
    }

    if (xSemaphoreTake(s_rtsp.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    memcpy(s_rtsp.latest_frame, data, length);
    s_rtsp.frame_length = length;
    s_rtsp.frame_pts = pts;
    xSemaphoreGive(s_rtsp.mutex);

    /* A binary semaphore is enough: the callback always copies the newest frame. */
    xSemaphoreGive(s_rtsp.frame_ready);
}

static void stop_rtsp_server(void)
{
    if (s_rtsp.server != NULL) {
        int result = esp_rtsp_server_stop(s_rtsp.server);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "RTSP server stop failed: %d", result);
        }
        s_rtsp.server = NULL;
    }

    if (s_rtsp.frame_ready != NULL) {
        vSemaphoreDelete(s_rtsp.frame_ready);
    }
    if (s_rtsp.mutex != NULL) {
        vSemaphoreDelete(s_rtsp.mutex);
    }
    free(s_rtsp.latest_frame);
    memset(&s_rtsp, 0, sizeof(s_rtsp));
}

static void start_rtsp_server(uint32_t max_frame_size, uint32_t width, uint32_t height)
{
    ESP_ERROR_CHECK(media_lib_add_default_adapter());

    s_rtsp.latest_frame = malloc(max_frame_size);
    s_rtsp.frame_capacity = max_frame_size;
    s_rtsp.mutex = xSemaphoreCreateMutex();
    s_rtsp.frame_ready = xSemaphoreCreateBinary();
    if (s_rtsp.latest_frame == NULL || s_rtsp.mutex == NULL || s_rtsp.frame_ready == NULL) {
        stop_rtsp_server();
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    s_rtsp.video_info = (esp_rtsp_video_info_t) {
        .vcodec = RTSP_VCODEC_H264,
        .width = (int)width,
        .height = (int)height,
        .fps = CONFIG_H264_ENCODER_FPS,
        .len = (int)max_frame_size,
    };
    s_rtsp.callbacks = (esp_rtsp_data_cb_t) {
        .send_video = rtsp_send_video,
    };
    s_rtsp.config = (esp_rtsp_config_t) {
        .ctx = &s_rtsp,
        .video_enable = true,
        .audio_enable = false,
        .uri = RTSP_SERVER_URI,
        .local_port = RTSP_SERVER_PORT,
        .local_addr = RTSP_SERVER_ADDRESS,
        .stack_size = 4096,
        .task_prio = 5,
        .mode = RTSP_SERVER,
        .video_info = &s_rtsp.video_info,
        .data_cb = &s_rtsp.callbacks,
        .state = rtsp_state_handler,
        .trans = RTSP_TRANSPORT_TCP,
    };

    s_rtsp.server = esp_rtsp_server_start(&s_rtsp.config);
    if (s_rtsp.server == NULL) {
        stop_rtsp_server();
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    ESP_LOGI(TAG, "RTSP server listening on %s", RTSP_SERVER_URI);
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

    rtsp_publish_video(frame->raw_data.buffer, frame->length, frame->pts);
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
        .yuv_range = ISP_COLOR_RANGE_FULL,
        .yuv_std = ISP_YUV_CONV_STD_BT601,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = width,
        .v_res = height,
        .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp));
    ESP_ERROR_CHECK(esp_isp_enable(isp));
    configure_isp(isp);

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
    start_ethernet();
    start_rtsp_server(encoded_buffer_size, width, height);

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

    stop_rtsp_server();
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
