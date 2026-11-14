#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>

#include "wireless.h"
#include "wifi_ota.h"
#include "config.h"

#include "esp_heap_caps.h"

#include "esp_hosted.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "WIRELESS";

#define WIRELESS_TASK_STACK  8192
#define WIRELESS_TASK_PRIO   4

#define WIRELESS_LOG_TASK_STACK 4096
#define WIRELESS_LOG_TASK_PRIO  3
#define WIRELESS_LOG_BUF_SIZE   8192   /* 无线串口发送环形缓冲 */
#define WIRELESS_LOG_LINE_SIZE  256    /* 单条日志最大截断长度 */

static SemaphoreHandle_t s_lock = NULL;
static bool s_started = false;
static bool s_connected = false;
static char s_ip[16] = "0.0.0.0";

/* ── 控制台日志镜像到无线串口 ── */
static SemaphoreHandle_t s_log_lock = NULL;
static char s_log_buf[WIRELESS_LOG_BUF_SIZE];
static size_t s_log_head = 0;
static size_t s_log_tail = 0;
static volatile uint32_t s_log_dropped = 0;
static vprintf_like_t s_console_vprintf = NULL;

/* ── 日志环形缓冲：写入方是 vprintf 钩子，读取方是无线串口发送任务 ── */

/* 非阻塞写入；缓冲满时丢弃最旧数据，保证不拖慢打印日志的任务 */
static void wireless_log_push(const char *text, size_t len)
{
    if (s_log_lock == NULL || len == 0) return;
    if (xSemaphoreTake(s_log_lock, 0) != pdTRUE) {
        s_log_dropped++;
        return;
    }
    for (size_t i = 0; i < len; i++) {
        size_t next = (s_log_head + 1) % WIRELESS_LOG_BUF_SIZE;
        if (next == s_log_tail) {
            s_log_tail = (s_log_tail + 1) % WIRELESS_LOG_BUF_SIZE;
        }
        s_log_buf[s_log_head] = text[i];
        s_log_head = next;
    }
    xSemaphoreGive(s_log_lock);
}

static size_t wireless_log_pop(char *out, size_t cap)
{
    size_t n = 0;
    if (s_log_lock == NULL) return 0;
    if (xSemaphoreTake(s_log_lock, portMAX_DELAY) != pdTRUE) return 0;
    while (n < cap && s_log_tail != s_log_head) {
        out[n++] = s_log_buf[s_log_tail];
        s_log_tail = (s_log_tail + 1) % WIRELESS_LOG_BUF_SIZE;
    }
    xSemaphoreGive(s_log_lock);
    return n;
}

static void wireless_log_clear(void)
{
    char tmp[WIRELESS_LOG_LINE_SIZE];
    while (wireless_log_pop(tmp, sizeof(tmp)) > 0) {
    }
}

/*
 * 接管 ESP_LOG 的输出：先按原样打印到 USB 串口（保持电脑串口监视器不变），
 * 再把同一份内容压入环形缓冲，由无线串口任务发给 TCP 客户端。
 */
static int wireless_console_vprintf(const char *fmt, va_list args)
{
    char line[WIRELESS_LOG_LINE_SIZE];
    va_list copy;

    va_copy(copy, args);
    int len = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (len > 0) {
        size_t n = (size_t)len;
        if (n >= sizeof(line)) n = sizeof(line) - 1;
        wireless_log_push(line, n);
    }

    return s_console_vprintf ? s_console_vprintf(fmt, args) : vprintf(fmt, args);
}

static void wireless_console_attach(void)
{
    if (s_console_vprintf != NULL || s_log_lock == NULL) return;
    s_console_vprintf = esp_log_set_vprintf(wireless_console_vprintf);
}

/* 无线串口发送任务：把日志缓冲推给 TCP 客户端 */
static void wireless_serial_task(void *arg)
{
    static const char banner[] = "\r\n=== wireless serial connected: mirroring console log ===\r\n";
    char out[512];
    bool was_connected = false;

    while (1) {
        /* 推送 OTA 期间 socket 上跑的是固件二进制，绝不能混入日志 */
        if (wifi_ota_push_active()) {
            wireless_log_clear();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        bool connected = wifi_ota_client_connected();

        if (connected && !was_connected) {
            wifi_ota_send(banner, sizeof(banner) - 1);
        }
        was_connected = connected;

        if (connected) {
            size_t n;
            while ((n = wireless_log_pop(out, sizeof(out))) > 0) {
                if (wifi_ota_send(out, n) < 0) break;
            }
        } else {
            /* 无客户端时不留积压，避免下次连接时刷出一堆旧日志 */
            wireless_log_clear();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void wireless_refresh_state(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    bool connected = false;
    char ip[16] = "0.0.0.0";

    if (netif != NULL &&
        esp_netif_get_ip_info(netif, &info) == ESP_OK &&
        info.ip.addr != 0) {
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
        connected = true;
    }

    if (s_lock != NULL) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_connected = connected;
    strlcpy(s_ip, ip, sizeof(s_ip));
    if (s_lock != NULL) xSemaphoreGive(s_lock);
}

/* 串口控制台：换热点时不用重新编译
 *   wifi                      查看当前账号与连接状态
 *   wifi <热点名> <密码>      切换并立即重连（保存到 NVS）
 *   wifi reset                清除 NVS 账号，恢复 Kconfig 默认
 * 若底盘调试控制台已占用 stdin（CHASSIS_DEBUG_ENABLE=1），则不创建本任务。
 */
#if !CHASSIS_DEBUG_ENABLE
static void wireless_console_task(void *arg)
{
    char line[96];
    ESP_LOGI(TAG, "串口命令: wifi <热点名> <密码> | wifi | wifi reset");

    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        line[strcspn(line, "\r\n")] = 0;
        if (strncmp(line, "wifi", 4) != 0) continue;

        char *arg1 = line + 4;
        while (*arg1 == ' ') arg1++;

        if (*arg1 == '\0' || strcmp(arg1, "status") == 0) {
            char ssid[33] = {0};
            char pass[65] = {0};
            char ip[16] = {0};
            wifi_ota_get_credentials(ssid, sizeof(ssid), pass, sizeof(pass));
            ESP_LOGI(TAG, "SSID=\"%s\" 密码长度=%u 状态=%s", ssid, (unsigned)strlen(pass),
                     wireless_is_connected() ? "已连接" : "未连接");
            if (wireless_get_ip(ip, sizeof(ip))) ESP_LOGI(TAG, "IP=%s", ip);
            continue;
        }

        if (strcmp(arg1, "reset") == 0) {
            wifi_ota_clear_credentials();
            continue;
        }

        char *pass = strchr(arg1, ' ');
        if (pass != NULL) {
            *pass++ = '\0';
            while (*pass == ' ') pass++;
        }
        esp_err_t err = wifi_ota_set_credentials(arg1, (pass != NULL) ? pass : "");
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "已切换 Wi-Fi 账号，正在重连...");
        } else {
            ESP_LOGE(TAG, "设置失败(%s)，格式: wifi 热点名 密码", esp_err_to_name(err));
        }
    }
}
#endif

/* 先确认 C5 协处理器在线：未烧录 C5 固件时直接跳过，避免 esp_wifi_init 触发 abort */
static bool wireless_coprocessor_ready(void)
{
    esp_hosted_coprocessor_fwver_t ver;
    if (esp_hosted_get_coprocessor_fwversion(&ver) != 0) {
        ESP_LOGE(TAG, "未检测到 C5 协处理器，请先烧录 C5 固件，跳过无线调试");
        return false;
    }
    ESP_LOGI(TAG, "C5 协处理器固件 %" PRIu32 ".%" PRIu32 ".%" PRIu32,
             ver.major1, ver.minor1, ver.patch1);
    return true;
}

static void wireless_task(void *arg)
{
    ESP_LOGI(TAG, "内存: 内部空闲=%u B, 内部DMA空闲=%u B, PSRAM空闲=%u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (!wireless_coprocessor_ready()) {
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = wifi_ota_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "无线调试启动失败: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    bool announced = false;
    while (1) {
        wireless_refresh_state();
        if (!announced && wireless_is_connected()) {
            char ip[16];
            if (wireless_get_ip(ip, sizeof(ip))) {
                ESP_LOGI(TAG, "无线调试就绪: IP=%s TCP端口=%d", ip, CONFIG_WIFI_OTA_TCP_PORT);
            }
            announced = true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t wireless_start(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) return ESP_ERR_NO_MEM;
    }
    if (s_started) return ESP_OK;

    if (s_log_lock == NULL) {
        s_log_lock = xSemaphoreCreateMutex();
        if (s_log_lock == NULL) return ESP_ERR_NO_MEM;
    }

    /* 本地串口日志镜像到无线串口：先接入钩子，再起发送任务 */
    wireless_console_attach();

    if (xTaskCreate(wireless_serial_task, "wl_serial", WIRELESS_LOG_TASK_STACK,
                    NULL, WIRELESS_LOG_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "无线串口任务创建失败");
        return ESP_FAIL;
    }

    if (xTaskCreate(wireless_task, "wireless", WIRELESS_TASK_STACK, NULL,
                    WIRELESS_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "无线调试任务创建失败");
        return ESP_FAIL;
    }

#if !CHASSIS_DEBUG_ENABLE
    xTaskCreate(wireless_console_task, "wl_console", 4096, NULL, 3, NULL);
#endif

    s_started = true;
    return ESP_OK;
}

bool wireless_is_connected(void)
{
    bool connected = false;
    if (s_lock != NULL) xSemaphoreTake(s_lock, portMAX_DELAY);
    connected = s_connected;
    if (s_lock != NULL) xSemaphoreGive(s_lock);
    return connected;
}

bool wireless_get_ip(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0) return false;

    bool connected = false;
    if (s_lock != NULL) xSemaphoreTake(s_lock, portMAX_DELAY);
    connected = s_connected;
    if (connected) strlcpy(buf, s_ip, buf_len);
    if (s_lock != NULL) xSemaphoreGive(s_lock);
    return connected;
}
