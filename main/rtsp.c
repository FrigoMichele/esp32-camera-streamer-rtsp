/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ethernet configuration and H.264 RTSP server.
 */

#include "rtsp.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_eth.h"
#include "esp_eth_netif_glue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rtsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "media_lib_adapter.h"

static const char *TAG = "rtsp";

#define RTSP_SERVER_PORT 8554
#define RTSP_SERVER_ADDRESS "192.168.2.5"
#define RTSP_SERVER_URI "rtsp://" RTSP_SERVER_ADDRESS ":8554/live"
#define RTSP_SERVER_NETMASK "255.255.255.0"
#define RTSP_SERVER_GATEWAY "192.168.2.1"
#define RTSP_CONNECT_TIMEOUT_MS 10000

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

struct rtsp {
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

    esp_eth_handle_t eth_handle;
    esp_eth_mac_t *eth_mac;
    esp_eth_phy_t *eth_phy;
    esp_eth_netif_glue_handle_t eth_glue;
    esp_netif_t *eth_netif;
    SemaphoreHandle_t eth_ip_ready;
    bool eth_event_registered;
    bool ip_event_registered;
    bool eth_started;
};

static esp_err_t configure_ethernet_static_ip(rtsp_t *rtsp)
{
    esp_err_t result = esp_netif_dhcpc_stop(rtsp->eth_netif);
    if (result != ESP_OK && result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return result;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr = esp_ip4addr_aton(RTSP_SERVER_ADDRESS);
    ip_info.netmask.addr = esp_ip4addr_aton(RTSP_SERVER_NETMASK);
    ip_info.gw.addr = esp_ip4addr_aton(RTSP_SERVER_GATEWAY);
    return esp_netif_set_ip_info(rtsp->eth_netif, &ip_info);
}

static void ethernet_event_handler(void *arg,
                                   esp_event_base_t event_base,
                                   int32_t event_id,
                                   void *event_data)
{
    rtsp_t *rtsp = arg;
    if (rtsp == NULL) {
        return;
    }

    if (event_base == ETH_EVENT) {
        switch (event_id) {
        case ETHERNET_EVENT_CONNECTED: {
            esp_err_t result = configure_ethernet_static_ip(rtsp);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "failed to configure Ethernet static IP: %s",
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
        if (event != NULL) {
            ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&event->ip_info.ip));
        }
        if (rtsp->eth_ip_ready != NULL) {
            xSemaphoreGive(rtsp->eth_ip_ready);
        }
    }
}

static void stop_ethernet(rtsp_t *rtsp)
{
    if (rtsp == NULL) {
        return;
    }

    if (rtsp->ip_event_registered) {
        (void)esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                           ethernet_event_handler);
        rtsp->ip_event_registered = false;
    }
    if (rtsp->eth_event_registered) {
        (void)esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID,
                                           ethernet_event_handler);
        rtsp->eth_event_registered = false;
    }

    if (rtsp->eth_started) {
        esp_err_t result = esp_eth_stop(rtsp->eth_handle);
        if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Ethernet stop failed: %s", esp_err_to_name(result));
        }
        rtsp->eth_started = false;
    }

    if (rtsp->eth_glue != NULL) {
        esp_err_t result = esp_eth_del_netif_glue(rtsp->eth_glue);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Ethernet netif glue delete failed: %s",
                     esp_err_to_name(result));
        }
        rtsp->eth_glue = NULL;
    }

    if (rtsp->eth_handle != NULL) {
        esp_err_t result = esp_eth_driver_uninstall(rtsp->eth_handle);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Ethernet driver uninstall failed: %s",
                     esp_err_to_name(result));
        }
        rtsp->eth_handle = NULL;
    }

    if (rtsp->eth_phy != NULL) {
        (void)rtsp->eth_phy->del(rtsp->eth_phy);
        rtsp->eth_phy = NULL;
    }
    if (rtsp->eth_mac != NULL) {
        (void)rtsp->eth_mac->del(rtsp->eth_mac);
        rtsp->eth_mac = NULL;
    }

    if (rtsp->eth_netif != NULL) {
        esp_netif_destroy(rtsp->eth_netif);
        rtsp->eth_netif = NULL;
    }

    if (rtsp->eth_ip_ready != NULL) {
        vSemaphoreDelete(rtsp->eth_ip_ready);
        rtsp->eth_ip_ready = NULL;
    }
}

static esp_err_t start_ethernet(rtsp_t *rtsp)
{
    esp_err_t result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }

    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }

    rtsp->eth_ip_ready = xSemaphoreCreateBinary();
    if (rtsp->eth_ip_ready == NULL) {
        return ESP_ERR_NO_MEM;
    }

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

    rtsp->eth_mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    rtsp->eth_phy = esp_eth_phy_new_ip101(&phy_config);
    if (rtsp->eth_mac == NULL || rtsp->eth_phy == NULL) {
        stop_ethernet(rtsp);
        return ESP_ERR_NO_MEM;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(rtsp->eth_mac,
                                                      rtsp->eth_phy);
    result = esp_eth_driver_install(&eth_config, &rtsp->eth_handle);
    if (result != ESP_OK) {
        stop_ethernet(rtsp);
        return result;
    }

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    rtsp->eth_netif = esp_netif_new(&netif_config);
    if (rtsp->eth_netif == NULL) {
        stop_ethernet(rtsp);
        return ESP_ERR_NO_MEM;
    }

    rtsp->eth_glue = esp_eth_new_netif_glue(rtsp->eth_handle);
    if (rtsp->eth_glue == NULL) {
        stop_ethernet(rtsp);
        return ESP_ERR_NO_MEM;
    }

    result = esp_netif_attach(rtsp->eth_netif, rtsp->eth_glue);
    if (result != ESP_OK) {
        stop_ethernet(rtsp);
        return result;
    }

    result = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                        ethernet_event_handler, rtsp);
    if (result != ESP_OK) {
        stop_ethernet(rtsp);
        return result;
    }
    rtsp->eth_event_registered = true;

    result = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                        ethernet_event_handler, rtsp);
    if (result != ESP_OK) {
        stop_ethernet(rtsp);
        return result;
    }
    rtsp->ip_event_registered = true;

    result = esp_eth_start(rtsp->eth_handle);
    if (result != ESP_OK) {
        stop_ethernet(rtsp);
        return result;
    }
    rtsp->eth_started = true;

    if (xSemaphoreTake(rtsp->eth_ip_ready,
                       pdMS_TO_TICKS(RTSP_CONNECT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Ethernet did not obtain %s within %d ms",
                 RTSP_SERVER_ADDRESS, RTSP_CONNECT_TIMEOUT_MS);
        stop_ethernet(rtsp);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static int rtsp_state_handler(esp_rtsp_state_t state, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "RTSP state: %d", (int)state);
    return ESP_OK;
}

static int rtsp_send_video(unsigned char *data,
                           unsigned int *length,
                           uint32_t *pts,
                           void *ctx)
{
    rtsp_t *rtsp = ctx;
    if (rtsp == NULL || data == NULL || length == NULL || pts == NULL ||
        rtsp->frame_ready == NULL || rtsp->mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The RTSP task asks for a frame when the client is in PLAY state. */
    if (xSemaphoreTake(rtsp->frame_ready, pdMS_TO_TICKS(1000)) != pdTRUE) {
        *length = 0;
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(rtsp->mutex, portMAX_DELAY) != pdTRUE) {
        *length = 0;
        return ESP_FAIL;
    }

    if (rtsp->frame_length > rtsp->frame_capacity ||
        rtsp->latest_frame == NULL) {
        xSemaphoreGive(rtsp->mutex);
        *length = 0;
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(data, rtsp->latest_frame, rtsp->frame_length);
    *length = (unsigned int)rtsp->frame_length;
    *pts = rtsp->frame_pts;
    xSemaphoreGive(rtsp->mutex);
    return ESP_OK;
}

static void stop_rtsp_server(rtsp_t *rtsp)
{
    if (rtsp == NULL) {
        return;
    }

    if (rtsp->server != NULL) {
        int result = esp_rtsp_server_stop(rtsp->server);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "RTSP server stop failed: %d", result);
        }
        rtsp->server = NULL;
    }

    if (rtsp->frame_ready != NULL) {
        vSemaphoreDelete(rtsp->frame_ready);
        rtsp->frame_ready = NULL;
    }
    if (rtsp->mutex != NULL) {
        vSemaphoreDelete(rtsp->mutex);
        rtsp->mutex = NULL;
    }
    free(rtsp->latest_frame);
    rtsp->latest_frame = NULL;
}

esp_err_t rtsp_create(const rtsp_config_t *config, rtsp_t **out_rtsp)
{
    if (config == NULL || out_rtsp == NULL || config->width == 0 ||
        config->height == 0 || config->fps == 0 ||
        config->max_frame_size == 0 || config->max_frame_size > UINT_MAX ||
        config->max_frame_size > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_rtsp = NULL;

    rtsp_t *rtsp = calloc(1, sizeof(*rtsp));
    if (rtsp == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = media_lib_add_default_adapter();
    if (result != ESP_OK) {
        free(rtsp);
        return result;
    }

    rtsp->latest_frame = malloc(config->max_frame_size);
    rtsp->frame_capacity = config->max_frame_size;
    rtsp->mutex = xSemaphoreCreateMutex();
    rtsp->frame_ready = xSemaphoreCreateBinary();
    if (rtsp->latest_frame == NULL || rtsp->mutex == NULL ||
        rtsp->frame_ready == NULL) {
        stop_rtsp_server(rtsp);
        free(rtsp);
        return ESP_ERR_NO_MEM;
    }

    rtsp->video_info = (esp_rtsp_video_info_t) {
        .vcodec = RTSP_VCODEC_H264,
        .width = (int)config->width,
        .height = (int)config->height,
        .fps = (int)config->fps,
        .len = (int)config->max_frame_size,
    };
    rtsp->callbacks = (esp_rtsp_data_cb_t) {
        .send_video = rtsp_send_video,
    };
    rtsp->config = (esp_rtsp_config_t) {
        .ctx = rtsp,
        .video_enable = true,
        .audio_enable = false,
        .uri = RTSP_SERVER_URI,
        .local_port = RTSP_SERVER_PORT,
        .local_addr = RTSP_SERVER_ADDRESS,
        .stack_size = 4096,
        .task_prio = 5,
        .mode = RTSP_SERVER,
        .video_info = &rtsp->video_info,
        .data_cb = &rtsp->callbacks,
        .state = rtsp_state_handler,
        .trans = RTSP_TRANSPORT_TCP,
    };

    result = start_ethernet(rtsp);
    if (result != ESP_OK) {
        stop_rtsp_server(rtsp);
        free(rtsp);
        return result;
    }

    rtsp->server = esp_rtsp_server_start(&rtsp->config);
    if (rtsp->server == NULL) {
        stop_ethernet(rtsp);
        stop_rtsp_server(rtsp);
        free(rtsp);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTSP server listening on %s", RTSP_SERVER_URI);
    *out_rtsp = rtsp;
    return ESP_OK;
}

esp_err_t rtsp_publish(rtsp_t *rtsp,
                       const uint8_t *data,
                       size_t length,
                       uint32_t pts)
{
    if (rtsp == NULL || data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > rtsp->frame_capacity || rtsp->latest_frame == NULL ||
        rtsp->mutex == NULL || rtsp->frame_ready == NULL) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(rtsp->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    memcpy(rtsp->latest_frame, data, length);
    rtsp->frame_length = length;
    rtsp->frame_pts = pts;
    xSemaphoreGive(rtsp->mutex);

    /* A binary semaphore is enough: the callback always copies the newest frame. */
    (void)xSemaphoreGive(rtsp->frame_ready);
    return ESP_OK;
}

void rtsp_destroy(rtsp_t *rtsp)
{
    if (rtsp == NULL) {
        return;
    }

    stop_rtsp_server(rtsp);
    stop_ethernet(rtsp);
    free(rtsp);
}
