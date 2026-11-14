/*
 * wifi_ota.c — 可复用的 ESP32-P4 无线下载（OTA）模块实现
 *
 * 依赖：
 *   - espressif/esp_hosted（主机侧，P4 通过 SDIO 使用 C5 的 Wi-Fi）
 *   - espressif/esp_wifi_remote（把远程 Wi-Fi 暴露为标准 esp_wifi API）
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/inet.h"

#include "wifi_ota.h"

#define WIFI_OTA_SSID      CONFIG_WIFI_OTA_SSID
#define WIFI_OTA_PASS      CONFIG_WIFI_OTA_PASSWORD
#define WIFI_OTA_MAX_RETRY CONFIG_WIFI_OTA_MAX_RETRY
#define WIFI_OTA_TCP_PORT  CONFIG_WIFI_OTA_TCP_PORT

static const char *TAG = "wifi_ota";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_OTA_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_OTA_SSID,
            .password = WIFI_OTA_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_OTA_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_OTA_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void ota_update_task(void *arg)
{
    char *url = (char *)arg;
    ESP_LOGI(TAG, "Starting OTA from: %s", url);

    esp_http_client_config_t http_config = {
        .url = url,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_config);
    if (client == NULL) {
        ESP_LOGE(TAG, "http client init failed");
        free(url);
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "http open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length <= 0) {
        ESP_LOGE(TAG, "fetch headers failed, content_length=%d", content_length);
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "no OTA update partition found");
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Writing to OTA partition %s (size %u)", update_partition->label,
             (unsigned)update_partition->size);

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota begin failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }

    char buf[1024];
    int total = 0;
    int last_pct = -1;
    while (1) {
        int len = esp_http_client_read(client, buf, sizeof(buf));
        if (len < 0) {
            ESP_LOGE(TAG, "http read failed");
            break;
        }
        if (len == 0) {
            break;
        }
        err = esp_ota_write(ota_handle, buf, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota write failed: %s", esp_err_to_name(err));
            break;
        }
        total += len;
        int pct = (total * 100) / content_length;
        if (pct != last_pct && pct % 10 == 0) {
            ESP_LOGI(TAG, "OTA progress: %d%% (%d/%d)", pct, total, content_length);
            last_pct = pct;
        }
    }

    bool ok = false;
    if (total == content_length) {
        err = esp_ota_end(ota_handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(update_partition);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "OTA succeeded, rebooting...");
                ok = true;
            } else {
                ESP_LOGE(TAG, "set boot partition failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "ota end failed: %s", esp_err_to_name(err));
        }
    } else {
        esp_ota_abort(ota_handle);
        ESP_LOGE(TAG, "OTA incomplete: %d/%d bytes", total, content_length);
    }

    esp_http_client_cleanup(client);
    free(url);

    if (ok) {
        ESP_LOGI(TAG, "Restarting to apply new firmware...");
        esp_restart();
    }

    vTaskDelete(NULL);
}

static void tcp_server_task(void *arg)
{
    char rx_buf[128];
    int opt = 1;

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket create failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(WIFI_OTA_TCP_PORT),
    };
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "listen failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP echo server listening on port %d", WIFI_OTA_TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept failed: errno %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "client connected: %s:%d",
                 inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        int len;
        while ((len = recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0)) > 0) {
            rx_buf[len] = 0;
            while (len > 0 && (rx_buf[len - 1] == '\n' || rx_buf[len - 1] == '\r')) {
                rx_buf[--len] = 0;
            }
            ESP_LOGI(TAG, "recv %d bytes: %s", len, rx_buf);

            if (strncmp(rx_buf, "ota ", 4) == 0 || strncmp(rx_buf, "OTA ", 4) == 0) {
                const char *url = rx_buf + 4;
                while (*url == ' ') url++;
                if (*url != 0) {
                    char *url_copy = strdup(url);
                    if (url_copy) {
                        send(client_sock, "OTA started, watch serial log\n", 31, 0);
                        xTaskCreate(&ota_update_task, "ota_update", 8192, url_copy, 5, NULL);
                    } else {
                        send(client_sock, "OTA command rejected (no memory)\n", 33, 0);
                    }
                }
            } else {
                send(client_sock, rx_buf, len, 0);
            }
        }

        ESP_LOGI(TAG, "client disconnected");
        close(client_sock);
    }
}

esp_err_t wifi_ota_start(void)
{
    /* 初始化 NVS（幂等，若已初始化则跳过） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "P4 host starting: Wi-Fi via C5 co-processor (SDIO)");
    wifi_init_sta();

    xTaskCreate(&tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    return ESP_OK;
}
