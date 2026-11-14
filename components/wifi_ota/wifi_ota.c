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
#include <strings.h>
#include <stdarg.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/inet.h"

#include "wifi_ota.h"

/* 账号键名与 tools/p4_tool.py 保持一致（CONFIG_ESP_WIFI_SSID/PASSWORD） */
#ifdef CONFIG_ESP_WIFI_SSID
#define WIFI_OTA_SSID      CONFIG_ESP_WIFI_SSID
#else
#define WIFI_OTA_SSID      "myssid"
#endif
#ifdef CONFIG_ESP_WIFI_PASSWORD
#define WIFI_OTA_PASS      CONFIG_ESP_WIFI_PASSWORD
#else
#define WIFI_OTA_PASS      "mypassword"
#endif
#ifdef CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_OTA_MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY
#else
#define WIFI_OTA_MAX_RETRY 5
#endif
#define WIFI_OTA_TCP_PORT  CONFIG_WIFI_OTA_TCP_PORT

/* TCP 服务参数：
 * - backlog 不能是 1：工具每条消息都新建连接，队列一满新 SYN 会被静默丢弃，
 *   现象就是「ping 通、TCP 连接一直 timed out」。
 * - 用 select 轮询代替阻塞 recv：旧连接半死（PC 休眠 / 热点抖动）时不会卡住 accept。 */
#define WIFI_OTA_TCP_BACKLOG     4
#define WIFI_OTA_TCP_SELECT_MS   200
#define WIFI_OTA_TCP_TASK_STACK  6144
#define WIFI_OTA_TCP_TASK_PRIO   10

static const char *TAG = "wifi_ota";

#define WIFI_CRED_NVS_NAMESPACE "wifi_cfg"
#define WIFI_CRED_NVS_SSID      "ssid"
#define WIFI_CRED_NVS_PASS      "pass"

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* 实际使用的账号：默认取 Kconfig，可在运行时被 NVS 配置覆盖 */
static char s_ssid[33] = WIFI_OTA_SSID;
static char s_pass[65] = WIFI_OTA_PASS;
static bool s_wifi_inited = false;

/* 连接诊断：最近一次断开原因，用于打印可读提示 */
static volatile int s_last_reason = 0;
static volatile bool s_sta_connected = false;

/* ── 无线串口：当前客户端 socket 与接收回调 ── */
static SemaphoreHandle_t s_tx_lock = NULL;
static int s_client_sock = -1;
static wifi_ota_rx_cb_t s_rx_cb = NULL;

/* 推送式 OTA 进行中：此时日志镜像必须让路，否则二进制流会被日志插花 */
static volatile bool s_push_active = false;

bool wifi_ota_push_active(void)
{
    return s_push_active;
}

/* 登记新客户端连接；旧连接在同一把锁内关闭，避免 wifi_ota_send() 正在 send 的
 * socket 被提前关掉，或被复用的 fd 被误关。 */
static void client_sock_swap(int sock)
{
    int old;
    if (s_tx_lock != NULL) xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    old = s_client_sock;
    s_client_sock = sock;
    if (old >= 0 && old != sock) close(old);
    if (s_tx_lock != NULL) xSemaphoreGive(s_tx_lock);

    if (old >= 0 && old != sock) {
        ESP_LOGW(TAG, "旧客户端连接已被新连接替换 (fd=%d)", old);
    }
}

/* 断开指定客户端；只清理仍然登记着的那个 fd，避免误关后来的新连接 */
static void client_sock_drop(int sock)
{
    if (s_tx_lock != NULL) xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    if (s_client_sock == sock) s_client_sock = -1;
    close(sock);
    if (s_tx_lock != NULL) xSemaphoreGive(s_tx_lock);
}

bool wifi_ota_client_connected(void)
{
    return s_client_sock >= 0;
}

void wifi_ota_set_rx_callback(wifi_ota_rx_cb_t cb)
{
    s_rx_cb = cb;
}

/* 在 tx 锁保护下把整块数据发完；返回 false 表示对端已断 */
static bool sock_send_all(int sock, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t off = 0;
    bool ok = true;

    if (s_tx_lock != NULL) xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    while (off < len) {
        int n = send(sock, p + off, len - off, 0);
        if (n <= 0) {
            ok = false;
            break;
        }
        off += (size_t)n;
    }
    if (s_tx_lock != NULL) xSemaphoreGive(s_tx_lock);
    return ok;
}

int wifi_ota_send(const void *data, size_t len)
{
    if (data == NULL || len == 0 || s_tx_lock == NULL) return -1;
    /* 推送固件期间禁止日志镜像混入，否则 socket 上是二进制+文本的混合流 */
    if (s_push_active) return -1;

    int result = -1;
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    int sock = s_client_sock;
    if (sock >= 0) {
        size_t off = 0;
        result = 0;
        while (off < len) {
            int n = send(sock, (const uint8_t *)data + off, len - off, 0);
            if (n <= 0) {
                result = -1;
                break;
            }
            off += (size_t)n;
        }
        if (result == 0) result = (int)off;
    }
    xSemaphoreGive(s_tx_lock);
    return result;
}

/* 把断开原因码翻译成可操作的中文提示 */
static const char *wifi_reason_hint(int reason)
{
    switch (reason) {
    case 201: return "未找到该 SSID：名称写错、热点未开启，或热点只在 2.4G/5G 某一频段";
    case 202: return "认证失败：密码错误或加密方式不匹配";
    case 203: return "AP 拒绝关联：可能达到最大连接数或被 MAC 过滤";
    case 204: return "握手超时：通常是密码错误";
    case 15:  return "四次握手超时：通常是密码错误";
    case 2:   return "认证过期";
    case 200: return "信号太弱或 AP 主动断开";
    case 205: return "连接失败：AP 不稳定或距离太远";
    case 0:   return "连接超时：AP 没有回应";
    default:  return "未知原因，见下方扫描结果";
    }
}

/* 扫描附近热点并打印：直接看出 SSID 是否可见、在哪个频段 */
static void wifi_scan_report(void)
{
    wifi_scan_config_t scan_cfg = { .show_hidden = true };
    if (esp_wifi_scan_start(&scan_cfg, true) != ESP_OK) {
        ESP_LOGW(TAG, "扫描失败（Wi-Fi 可能未启动）");
        return;
    }

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num == 0) {
        ESP_LOGW(TAG, "扫描不到任何热点：检查 C5 天线/协处理器是否正常");
        return;
    }

    uint16_t want = (ap_num > 8) ? 8 : ap_num;
    wifi_ap_record_t *recs = calloc(want, sizeof(wifi_ap_record_t));
    if (recs == NULL) return;

    if (esp_wifi_scan_get_ap_records(&want, recs) == ESP_OK) {
        ESP_LOGW(TAG, "附近可见 %u 个热点，前 %u 个：", ap_num, want);
        for (int i = 0; i < want; i++) {
            ESP_LOGW(TAG, "  [%d] \"%s\" RSSI=%d dBm ch=%d auth=%d",
                     i + 1, (const char *)recs[i].ssid, recs[i].rssi,
                     recs[i].primary, recs[i].authmode);
        }
    }
    free(recs);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)event_data;
        s_last_reason = ev->reason;
        s_sta_connected = false;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_sta_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* 把当前 s_ssid / s_pass 写进 Wi-Fi 配置 */
static esp_err_t wifi_apply_config(void)
{
    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, s_pass, sizeof(wifi_config.sta.password));
    /* 不限制加密方式：WPA2 / WPA3 / 开放热点都能连 */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    return esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
}

/* NVS 里保存过账号就优先用它，否则用 Kconfig 里的默认值 */
static void wifi_credentials_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(WIFI_CRED_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;

    char ssid[33] = {0};
    size_t len = sizeof(ssid);
    if (nvs_get_str(handle, WIFI_CRED_NVS_SSID, ssid, &len) == ESP_OK && ssid[0] != 0) {
        char pass[65] = {0};
        len = sizeof(pass);
        if (nvs_get_str(handle, WIFI_CRED_NVS_PASS, pass, &len) != ESP_OK) {
            pass[0] = 0;
        }
        strlcpy(s_ssid, ssid, sizeof(s_ssid));
        strlcpy(s_pass, pass, sizeof(s_pass));
        ESP_LOGI(TAG, "使用 NVS 里保存的 Wi-Fi 账号");
    }
    nvs_close(handle);
}

/* 连接监督任务：断线自动重连，失败时打印原因并周期扫描附近热点 */
static void wifi_supervisor_task(void *arg)
{
    int attempt = 0;
    bool up = false;

    while (1) {
        if (up) {
            if (!s_sta_connected) {
                up = false;
                attempt = 0;
                ESP_LOGW(TAG, "Wi-Fi 掉线 (reason=%d: %s)，开始重连",
                         s_last_reason, wifi_reason_hint(s_last_reason));
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "发起连接失败: %s", esp_err_to_name(err));
        }

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                               pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
        if (bits & WIFI_CONNECTED_BIT) {
            up = true;
            attempt = 0;
            ESP_LOGI(TAG, "已连接到热点 SSID:\"%s\"", s_ssid);
            continue;
        }

        attempt++;
        ESP_LOGW(TAG, "第 %d 次连接失败 (reason=%d: %s)", attempt,
                 s_last_reason, wifi_reason_hint(s_last_reason));
        if (WIFI_OTA_MAX_RETRY > 0 && attempt % WIFI_OTA_MAX_RETRY == 0) {
            wifi_scan_report();
            ESP_LOGW(TAG, "可串口输入: wifi 你的热点名 你的密码   （保存后立即重连）");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(wifi_apply_config());
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 关掉省电：否则热点侧周期休眠，表现为 ping 丢包、TCP 握手超时 */
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "关闭 Wi-Fi 省电失败: %s", esp_err_to_name(ps_err));
    }

#if defined(CONFIG_SLAVE_SOC_WIFI_SUPPORT_5G)
    /* C5 支持 5GHz：设成 AUTO，手机热点开在 5G 也能连 */
    if (esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO) != ESP_OK) {
        ESP_LOGW(TAG, "设置 2.4G+5G 自动频段失败，继续使用默认频段");
    }
#endif

    s_wifi_inited = true;

    ESP_LOGI(TAG, "即将连接 Wi-Fi: SSID=\"%s\"", s_ssid);
    ESP_LOGI(TAG, "改账号: 串口输入 \"wifi 热点名 密码\"，或 idf.py menuconfig -> WiFi OTA Helper");

    xTaskCreate(wifi_supervisor_task, "wifi_sup", 4096, NULL, 5, NULL);
}

#define OTA_IMAGE_MAGIC 0xE9   /* ESP 应用镜像首字节 */

/* 回包给 PC（推送/HTTP OTA 用，绕过日志镜像限制） */
static void ota_reply(int sock, const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) sock_send_all(sock, buf, (size_t)n);
}

static void ota_update_task(void *arg)
{
    char *url = (char *)arg;
    ESP_LOGI(TAG, "HTTP OTA 开始: %s", url);

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
        ESP_LOGE(TAG, "http open failed: %s（电脑防火墙是否放行 8000 端口？）",
                 esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }

    int content_length = (int)esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP status=%d content_length=%d", status, content_length);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP 状态异常 (status=%d)：URL 或端口不对", status);
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }
    /* content_length < 0 表示 chunked/未知长度：按流式读到 EOF 处理 */
    bool length_known = (content_length > 0);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "no OTA update partition found（分区表是否双 OTA？）");
        esp_http_client_cleanup(client);
        free(url);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "目标分区 %s，容量 %u 字节", update_partition->label,
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

    static char buf[4096];
    int total = 0;
    int last_pct = -1;
    bool magic_ok = false;
    while (1) {
        int len = esp_http_client_read(client, buf, sizeof(buf));
        if (len < 0) {
            ESP_LOGE(TAG, "http read failed（连接被中断）");
            break;
        }
        if (len == 0) break;

        if (!magic_ok) {
            magic_ok = (((uint8_t)buf[0]) == OTA_IMAGE_MAGIC);
            if (!magic_ok) {
                ESP_LOGE(TAG, "固件首字节 0x%02X != 0xE9，不是应用镜像（别选 bootloader/合并包）",
                         (uint8_t)buf[0]);
                esp_ota_abort(ota_handle);
                esp_http_client_cleanup(client);
                free(url);
                vTaskDelete(NULL);
                return;
            }
        }

        err = esp_ota_write(ota_handle, buf, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota write failed: %s", esp_err_to_name(err));
            break;
        }
        total += len;
        if (length_known) {
            int pct = (total * 100) / content_length;
            if (pct != last_pct && pct % 10 == 0) {
                ESP_LOGI(TAG, "OTA progress: %d%% (%d/%d)", pct, total, content_length);
                last_pct = pct;
            }
        } else if (total % (256 * 1024) < (int)sizeof(buf)) {
            ESP_LOGI(TAG, "OTA progress: %d 字节已写入", total);
        }
    }

    bool complete = length_known ? (total == content_length) : (total > 0);
    bool ok = false;
    if (complete) {
        err = esp_ota_end(ota_handle);   /* 这里会校验镜像 */
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(update_partition);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "OTA succeeded, rebooting... (%d 字节已校验)", total);
                ok = true;
            } else {
                ESP_LOGE(TAG, "set boot partition failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "ota end failed: %s（镜像校验不通过，可能只下载了一部分）",
                     esp_err_to_name(err));
        }
    } else {
        esp_ota_abort(ota_handle);
        ESP_LOGE(TAG, "OTA incomplete: %d/%d bytes", total, content_length);
    }

    esp_http_client_cleanup(client);
    free(url);

    if (ok) {
        ESP_LOGI(TAG, "Restarting to apply new firmware...");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }

    vTaskDelete(NULL);
}

/* 推送式 OTA：PC 通过同一条 TCP 连接把固件二进制流发过来。
 * 优点：不依赖 HTTP 服务器、不受电脑防火墙入站规则影响。 */
static void ota_push_handle(int client_sock, uint32_t size)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ota_reply(client_sock, "ERR no_ota_partition\n");
        return;
    }
    if (size == 0 || size > update_partition->size) {
        ota_reply(client_sock, "ERR bad_size %u (partition %u)\n",
                  (unsigned)size, (unsigned)update_partition->size);
        return;
    }

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &handle);
    if (err != ESP_OK) {
        ota_reply(client_sock, "ERR ota_begin %s\n", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "推送 OTA：分区 %s，预计 %u 字节",
             update_partition->label, (unsigned)size);
    ota_reply(client_sock, "READY %u\n", (unsigned)size);

    /* 推送期间放宽接收超时，避免大包偶尔变慢被判超时 */
    struct timeval rcv_timeout = { .tv_sec = 30, .tv_usec = 0 };
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));

    static uint8_t buf[2048];
    uint32_t remaining = size;
    uint32_t written = 0;
    int last_pct = -1;
    bool failed = false;

    while (remaining > 0) {
        int want = (remaining > sizeof(buf)) ? (int)sizeof(buf) : (int)remaining;
        int n = recv(client_sock, buf, want, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "推送中断：recv=%d errno=%d（已收 %u/%u）",
                     n, errno, (unsigned)written, (unsigned)size);
            failed = true;
            break;
        }
        if (written == 0 && buf[0] != OTA_IMAGE_MAGIC) {
            ESP_LOGE(TAG, "首字节 0x%02X != 0xE9，不是应用镜像", buf[0]);
            failed = true;
            break;
        }
        err = esp_ota_write(handle, buf, n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ota write failed: %s", esp_err_to_name(err));
            failed = true;
            break;
        }
        written += (uint32_t)n;
        remaining -= (uint32_t)n;

        int pct = (int)((uint64_t)written * 100 / size);
        if (pct / 10 != last_pct / 10 || pct == 100) {
            ESP_LOGI(TAG, "推送进度 %d%% (%u/%u)", pct, (unsigned)written, (unsigned)size);
            last_pct = pct;
        }
    }

    if (failed) {
        esp_ota_abort(handle);
        ota_reply(client_sock, "ERR aborted after %u/%u bytes\n",
                  (unsigned)written, (unsigned)size);
        return;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota end failed: %s（镜像校验不通过）", esp_err_to_name(err));
        ota_reply(client_sock, "ERR ota_end %s\n", esp_err_to_name(err));
        return;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set boot partition failed: %s", esp_err_to_name(err));
        ota_reply(client_sock, "ERR set_boot %s\n", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "OTA succeeded, rebooting... (%u 字节已校验)", (unsigned)written);
    ota_reply(client_sock, "OK %u\n", (unsigned)written);
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Restarting to apply new firmware...");
    esp_restart();
}

/* 打开监听端口；失败时重试而不是退出任务，否则一次 bind 失败就再也连不上 */
static int tcp_listen_open(void)
{
    int opt = 1;

    while (1) {
        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "socket create failed: errno %d，3 秒后重试", errno);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in server_addr = {
            .sin_family = AF_INET,
            .sin_addr.s_addr = htonl(INADDR_ANY),
            .sin_port = htons(WIFI_OTA_TCP_PORT),
        };
        if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "bind failed (port %d): errno %d，3 秒后重试",
                     WIFI_OTA_TCP_PORT, errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        if (listen(listen_sock, WIFI_OTA_TCP_BACKLOG) < 0) {
            ESP_LOGE(TAG, "listen failed: errno %d，3 秒后重试", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        return listen_sock;
    }
}

/* 处理一条已去掉换行的命令 */
static void tcp_handle_line(int client_sock, char *line, int len)
{
    ESP_LOGI(TAG, "recv %d bytes: %s", len, line);

    /* 连通性自测：PC 端发 hello p4 / ping，必须立刻有回包 */
    if (strncasecmp(line, "hello", 5) == 0) {
        char reply[128];
        int n = snprintf(reply, sizeof(reply),
                         "\r\n[P4] hello p4 ✅ alive, uptime=%llu s, free_heap=%u B\r\n",
                         (unsigned long long)(esp_timer_get_time() / 1000000ULL),
                         (unsigned)esp_get_free_heap_size());
        if (n > 0) send(client_sock, reply, (size_t)n, 0);
        return;
    }
    if (strcasecmp(line, "ping") == 0) {
        send(client_sock, "pong\r\n", 6, 0);
        return;
    }
    if (strcasecmp(line, "status") == 0) {
        char reply[128];
        int n = snprintf(reply, sizeof(reply),
                         "\r\n[P4] wifi=%s port=%d free_heap=%u B\r\n",
                         s_sta_connected ? "connected" : "disconnected",
                         WIFI_OTA_TCP_PORT, (unsigned)esp_get_free_heap_size());
        if (n > 0) send(client_sock, reply, (size_t)n, 0);
        return;
    }
    if (strcasecmp(line, "reboot") == 0) {
        send(client_sock, "rebooting\r\n", 11, 0);
        ESP_LOGW(TAG, "收到 reboot 命令，重启中...");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
        return;
    }

    if (strncmp(line, "ota-push ", 9) == 0) {
        uint32_t size = (uint32_t)strtoul(line + 9, NULL, 10);
        s_push_active = true;
        ota_push_handle(client_sock, size);
        s_push_active = false;   /* 成功时已重启，走到这里说明失败 */
        return;
    }

    if (strncmp(line, "ota ", 4) == 0 || strncmp(line, "OTA ", 4) == 0) {
        const char *url = line + 4;
        while (*url == ' ') url++;
        if (*url == 0) return;

        char *url_copy = strdup(url);
        if (url_copy == NULL) {
            send(client_sock, "OTA command rejected (no memory)\n", 33, 0);
            return;
        }
        send(client_sock, "OTA started, watch serial log\n", 31, 0);
        if (xTaskCreate(&ota_update_task, "ota_update", 8192, url_copy, 5, NULL) != pdPASS) {
            free(url_copy);
            ESP_LOGE(TAG, "OTA 任务创建失败");
        }
        return;
    }

    if (s_rx_cb != NULL) {
        /* 无线串口：交给业务侧处理（例如注入串口屏事件） */
        s_rx_cb(line, (size_t)len);
    } else {
        /* 回显带换行：PC 端接收区才能一行一条，不会粘成一片 */
        send(client_sock, line, len, 0);
        send(client_sock, "\r\n", 2, 0);
    }
}

static void tcp_server_task(void *arg)
{
    char rx_buf[512];

    int listen_sock = tcp_listen_open();
    ESP_LOGI(TAG, "TCP server listening on port %d （hello p4 / status / reboot / ota-push）",
             WIFI_OTA_TCP_PORT);

    int client_sock = -1;

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_sock, &rfds);
        int max_fd = listen_sock;
        if (client_sock >= 0) {
            FD_SET(client_sock, &rfds);
            if (client_sock > max_fd) max_fd = client_sock;
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = WIFI_OTA_TCP_SELECT_MS * 1000 };
        int sel = select(max_fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            ESP_LOGE(TAG, "select failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (sel == 0) continue;

        /* 1) 新连接：直接接管。工具端每条消息都新建连接，旧连接可能是半死状态，
         *    若继续等它的 recv 就会把所有新连接都堵在监听队列外面。 */
        if (FD_ISSET(listen_sock, &rfds)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
            if (new_sock < 0) {
                ESP_LOGW(TAG, "accept failed: errno %d", errno);
            } else {
                /* 发送超时：镜像日志时客户端不读也不会卡死任务 */
                struct timeval send_timeout = { .tv_sec = 1, .tv_usec = 0 };
                setsockopt(new_sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
                /* 半开连接靠 TCP keepalive 回收 */
                int keepalive = 1;
                setsockopt(new_sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
                int idle = 15, intvl = 5, cnt = 3;
                setsockopt(new_sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
                setsockopt(new_sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
                setsockopt(new_sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

                client_sock_swap(new_sock);
                client_sock = new_sock;
                ESP_LOGI(TAG, "client connected: %s:%d",
                         inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                continue;
            }
        }

        /* 2) 客户端数据 */
        if (client_sock >= 0 && FD_ISSET(client_sock, &rfds)) {
            int len = recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0);
            if (len <= 0) {
                ESP_LOGI(TAG, "client disconnected (%d)", len);
                client_sock_drop(client_sock);
                client_sock = -1;
                continue;
            }

            rx_buf[len] = 0;
            int text_len = len;
            while (text_len > 0 && (rx_buf[text_len - 1] == '\n' || rx_buf[text_len - 1] == '\r')) {
                rx_buf[--text_len] = 0;
            }
            if (text_len == 0) continue;
            tcp_handle_line(client_sock, rx_buf, text_len);
        }
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
    {
        const esp_partition_t *run = esp_ota_get_running_partition();
        ESP_LOGI(TAG, "运行分区: %s @0x%lx（无线升级成功后会切到另一个 ota_x）",
                 run ? run->label : "?", run ? (unsigned long)run->address : 0UL);
    }
    wifi_credentials_from_nvs();
    wifi_init_sta();

    if (s_tx_lock == NULL) {
        s_tx_lock = xSemaphoreCreateMutex();
        if (s_tx_lock == NULL) {
            ESP_LOGE(TAG, "tx mutex create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreate(&tcp_server_task, "tcp_server", WIFI_OTA_TCP_TASK_STACK,
                    NULL, WIFI_OTA_TCP_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "TCP 服务任务创建失败");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t wifi_ota_set_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (strlen(ssid) > 32) return ESP_ERR_INVALID_ARG;
    if (password != NULL && strlen(password) > 64) return ESP_ERR_INVALID_ARG;
    if (password != NULL && password[0] != '\0' && strlen(password) < 8) {
        ESP_LOGW(TAG, "WPA/WPA2 密码至少 8 位");
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy(s_ssid, ssid, sizeof(s_ssid));
    strlcpy(s_pass, password ? password : "", sizeof(s_pass));

    nvs_handle_t handle;
    if (nvs_open(WIFI_CRED_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, WIFI_CRED_NVS_SSID, s_ssid);
        nvs_set_str(handle, WIFI_CRED_NVS_PASS, s_pass);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "新账号已保存到 NVS，下次上电自动使用");
    }

    if (!s_wifi_inited) return ESP_OK;

    ESP_LOGW(TAG, "切换 Wi-Fi 账号 -> SSID:\"%s\"", s_ssid);
    wifi_apply_config();
    esp_wifi_disconnect();   /* 由监督任务按新账号重连 */
    return ESP_OK;
}

esp_err_t wifi_ota_get_credentials(char *ssid, size_t ssid_len,
                                   char *password, size_t pass_len)
{
    if (ssid == NULL || ssid_len == 0) return ESP_ERR_INVALID_ARG;
    strlcpy(ssid, s_ssid, ssid_len);
    if (password != NULL && pass_len > 0) {
        strlcpy(password, s_pass, pass_len);
    }
    return ESP_OK;
}

esp_err_t wifi_ota_clear_credentials(void)
{
    nvs_handle_t handle;
    if (nvs_open(WIFI_CRED_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_key(handle, WIFI_CRED_NVS_SSID);
        nvs_erase_key(handle, WIFI_CRED_NVS_PASS);
        nvs_commit(handle);
        nvs_close(handle);
    }
    strlcpy(s_ssid, WIFI_OTA_SSID, sizeof(s_ssid));
    strlcpy(s_pass, WIFI_OTA_PASS, sizeof(s_pass));
    ESP_LOGI(TAG, "已清除 NVS 账号，恢复 Kconfig 默认: \"%s\"", s_ssid);
    return ESP_OK;
}
