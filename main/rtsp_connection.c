#include "rtsp_connection.h"

#include <stdatomic.h>
#include <string.h>
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_netif_glue.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_rtsp.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mipi_camera.h"
#include "port/media_lib_adapter.h"
#include "sdkconfig.h"

#define RTSP_PORT 8554
#define PHY_RESET_GPIO 51

static const char *TAG = "rtsp";

typedef struct {
    h264_frame_t frame;
    uint32_t session;
    uint32_t order;  // Encoded frame order, not CSI sequence (which may skip).
} stream_frame_t;

static QueueHandle_t s_pending;  // One slot, so RTSP cannot own the entire encoder pool.
static atomic_uint s_session;    // Odd while playing; changes on each new PLAY.
static uint32_t s_offer_order;   // Written only by app_main.
static esp_rtsp_video_info_t s_video_info;

bool rtsp_connection_offer(const h264_frame_t *frame)
{
    uint32_t session = atomic_load(&s_session);
    if (!(session & 1U)) {
        return false;
    }
    stream_frame_t item = {.frame = *frame, .session = session, .order = s_offer_order++};
    return xQueueSend(s_pending, &item, 0) == pdTRUE;
}

static int on_rtsp_state(esp_rtsp_state_t state, void *ctx)
{
    (void)ctx;
    if (state == RTSP_STATE_PLAY) {
        atomic_fetch_and(&s_session, ~1U);
        atomic_fetch_add(&s_session, 3U);
        ESP_LOGI(TAG, "RTSP playback started");
    } else if (state == RTSP_STATE_TEARDOWN) {
        atomic_fetch_and(&s_session, ~1U);
        ESP_LOGI(TAG, "RTSP playback ended");
    }
    return ESP_OK;
}

static int supply_video(unsigned char *data, unsigned int *len, uint32_t *pts, void *ctx)
{
    (void)ctx;
    // esp_media_protocols calls this from a periodic timer. Never wait for a frame here.
    unsigned int capacity = *len;
    *len = 0;
    stream_frame_t item;
    if (xQueueReceive(s_pending, &item, 0) != pdTRUE) {
        return ESP_OK;
    }

    static uint32_t session_seen;
    static uint32_t next_order;
    static int64_t first_timestamp_us;
    static bool synchronized;

    uint32_t session = atomic_load(&s_session);
    if (session != session_seen) {
        session_seen = session;
        first_timestamp_us = -1;
        synchronized = false;
    }
    if ((session & 1U) && item.session == session) {
        if (item.order != next_order || item.frame.size > capacity) {
            synchronized = false;
        }
        // Start and recover at an IDR (with SPS/PPS), not a dependent P frame.
        if ((synchronized || item.frame.is_idr) && item.frame.size <= capacity) {
            if (first_timestamp_us < 0) {
                first_timestamp_us = item.frame.timestamp_us;
            }
            // The library needs data in its own RTP staging buffer. Only encoded data is copied.
            memcpy(data, item.frame.data, item.frame.size);
            *len = item.frame.size;
            *pts = (uint32_t)((item.frame.timestamp_us - first_timestamp_us) / 1000);
            synchronized = true;
        }
        next_order = item.order + 1;
    }
    h264_encoder_release_frame(&item.frame);
    return ESP_OK;
}

static esp_err_t start_ethernet(esp_netif_t **netif)
{
    esp_netif_ip_info_t static_ip = {0};
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(CONFIG_CAMERA_ETH_STATIC_IP, &static_ip.ip),
                        TAG, "Invalid static Ethernet IP");
    ESP_RETURN_ON_FALSE(static_ip.ip.addr != 0, ESP_ERR_INVALID_ARG, TAG,
                        "Static Ethernet IP cannot be 0.0.0.0");
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(CONFIG_CAMERA_ETH_STATIC_NETMASK, &static_ip.netmask),
                        TAG, "Invalid Ethernet subnet mask");
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(CONFIG_CAMERA_ETH_STATIC_GATEWAY, &static_ip.gw),
                        TAG, "Invalid Ethernet gateway");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop creation failed");

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = PHY_RESET_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    ESP_RETURN_ON_FALSE(mac, ESP_ERR_NO_MEM, TAG, "Could not create Ethernet MAC");
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);
    if (!phy) {
        mac->del(mac);
        ESP_LOGE(TAG, "Could not create IP101 PHY");
        return ESP_ERR_NO_MEM;
    }
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = NULL;
    esp_err_t err = esp_eth_driver_install(&eth_cfg, &eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet install failed: %s", esp_err_to_name(err));
        phy->del(phy);
        mac->del(mac);
        return err;
    }

    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base.flags &= ~ESP_NETIF_DHCP_CLIENT;
    base.ip_info = &static_ip;
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    cfg.base = &base;
    *netif = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(*netif, ESP_ERR_NO_MEM, TAG, "Could not create Ethernet netif");
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth);
    ESP_RETURN_ON_FALSE(glue, ESP_ERR_NO_MEM, TAG, "Could not create Ethernet glue");
    ESP_RETURN_ON_ERROR(esp_netif_attach(*netif, glue), TAG, "Ethernet netif attach failed");
    ESP_RETURN_ON_ERROR(esp_eth_start(eth), TAG, "Ethernet start failed");
    return ESP_OK;
}

static void connection_task(void *arg)
{
    (void)arg;
    esp_netif_t *netif;
    esp_err_t err = start_ethernet(&netif);
    if (err != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }
    err = media_lib_add_default_adapter();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Media adapter init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Waiting for Ethernet link (static IP: %s)", CONFIG_CAMERA_ETH_STATIC_IP);

#if CONFIG_CAMERA_RTSP_RTP_TCP
    const esp_rtsp_transport_t preferred_transport = RTSP_TRANSPORT_TCP;
    const char *transport_name = "tcp";
#else
    const esp_rtsp_transport_t preferred_transport = RTSP_TRANSPORT_UDP;
    const char *transport_name = "udp";
#endif

    char address[16];
    esp_rtsp_data_cb_t callbacks = {.send_video = supply_video};
    esp_rtsp_config_t cfg = {
        .mode = RTSP_SERVER,
        .local_addr = address,
        .local_port = RTSP_PORT,
        .video_enable = true,
        .video_info = &s_video_info,
        .data_cb = &callbacks,
        .state = on_rtsp_state,
        .stack_size = 6144,
        .task_prio = tskIDLE_PRIORITY + 2,
        .trans = preferred_transport, // Library client default; server uses SETUP.
    };

    esp_rtsp_handle_t server = NULL;
    uint32_t bound_ip = 0;
    uint32_t last_ip = 0;
    while (true) {
        esp_netif_ip_info_t ip = {0};
        if (esp_netif_is_netif_up(netif)) {
            esp_netif_get_ip_info(netif, &ip);
        }
        if (ip.ip.addr != last_ip) {
            last_ip = ip.ip.addr;
            if (last_ip) {
                ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&ip.ip));
            } else {
                ESP_LOGW(TAG, "Ethernet link down");
            }
        }
        if (server && bound_ip != ip.ip.addr) {
            atomic_fetch_and(&s_session, ~1U);
            if (esp_rtsp_server_stop(server) != ESP_OK) {
                ESP_LOGE(TAG, "Could not stop RTSP server");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            server = NULL;
            atomic_fetch_and(&s_session, ~1U);
            stream_frame_t item;
            if (xQueueReceive(s_pending, &item, 0) == pdTRUE) {
                h264_encoder_release_frame(&item.frame);
            }
            ESP_LOGI(TAG, "RTSP server stopped; Ethernet address changed or link went down");
        }
        if (!server && ip.ip.addr) {
            esp_ip4addr_ntoa(&ip.ip, address, sizeof(address));
            server = esp_rtsp_server_start(&cfg);
            if (server) {
                bound_ip = ip.ip.addr;
                ESP_LOGI(TAG, "RTSP URI: rtsp://%s:%d/live", address, RTSP_PORT);
                ESP_LOGI(TAG, "ffplay -rtsp_transport %s rtsp://%s:%d/live",
                         transport_name, address, RTSP_PORT);
            } else {
                ESP_LOGE(TAG, "Could not start RTSP server on %s:%d", address, RTSP_PORT);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t rtsp_connection_start(uint32_t width, uint32_t height, uint8_t fps)
{
    ESP_RETURN_ON_FALSE(!s_pending, ESP_ERR_INVALID_STATE, TAG, "RTSP task already started");
    s_pending = xQueueCreate(1, sizeof(stream_frame_t));
    ESP_RETURN_ON_FALSE(s_pending, ESP_ERR_NO_MEM, TAG, "Could not allocate RTSP queue");
    s_video_info = (esp_rtsp_video_info_t) {
        .vcodec = RTSP_VCODEC_H264,
        .width = width,
        .height = height,
        .fps = fps,
        .len = mipi_camera_padded_frame_size(width, height), // Matches the encoder output pool.
    };

    if (xTaskCreate(connection_task, "rtsp_connection", 6144, NULL,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        vQueueDelete(s_pending);
        s_pending = NULL;
        ESP_LOGE(TAG, "Could not create RTSP connection task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
