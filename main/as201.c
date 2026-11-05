#include "as201.h"
#include "config.h"
#include <string.h>
#include <inttypes.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "AS201";

/* ==================== 一、硬件配置区 ==================== */
#define AS201_TX_GPIO       39  // 接陀螺仪RX
#define AS201_RX_GPIO       40  // 接陀螺仪TX
#define AS201_UART_PORT     UART_NUM_1 // 使用串口1
#define AS201_BAUD_RATE     115200
#define AS201_BUF_SIZE      512
#define AS201_HEAD0         0xFA
#define AS201_HEAD1         0xFB
#define AS201_TAIL0         0xFC
#define AS201_TAIL1         0xFD
#define AS201_CMD_REPORT    0x00

/* ==================== 二、全局变量区 ==================== */
static uint8_t s_buf[AS201_BUF_SIZE];
static size_t  s_len = 0;
static TaskHandle_t s_task = NULL;
static as201_data_t s_last_data; 
static volatile bool s_data_valid = false;
static SemaphoreHandle_t s_data_mutex = NULL; 

/* ==================== 三、协议解析区 ==================== */
// 读取16位有符号整数（小端）
static int16_t rd_s16(const uint8_t *d, int off) {
    return (int16_t)((uint16_t)d[off] | ((uint16_t)d[off + 1] << 8));
}

// 将原始字节乘以手册给定的系数
static void as201_parse_payload(const uint8_t *d, size_t dlen, as201_data_t *out) {
    out->mag_level = (d[0] >> 2) & 0x03;
    out->ax = rd_s16(d, 1) * 0.00478515625f; out->ay = rd_s16(d, 3) * 0.00478515625f; out->az = rd_s16(d, 5) * 0.00478515625f;
    out->gx = rd_s16(d, 7) * 0.0625f; out->gy = rd_s16(d, 9) * 0.0625f; out->gz = rd_s16(d, 11) * 0.0625f;
    out->roll  = rd_s16(d, 13) * 0.0054931640625f; out->pitch = rd_s16(d, 15) * 0.0054931640625f; out->yaw   = rd_s16(d, 17) * 0.0054931640625f;
    out->mx = rd_s16(d, 19) * 0.006103515625f; out->my = rd_s16(d, 21) * 0.006103515625f; out->mz = rd_s16(d, 23) * 0.006103515625f;
    out->temp_c = rd_s16(d, 33) * 0.01f;
}

// 状态机：寻找完整的一帧
static esp_err_t as201_process(as201_data_t *out) {
    size_t start = 0;
    while (start + 1 < s_len) {
        if (s_buf[start] == AS201_HEAD0 && s_buf[start + 1] == AS201_HEAD1) break;
        start++;
    }
    if (start > 0) { memmove(s_buf, s_buf + start, s_len - start); s_len -= start; }
    if (s_len < 3) return ESP_ERR_NOT_FOUND;

    uint8_t len = s_buf[2];
    if (len < 3 || (size_t)len + 5 > AS201_BUF_SIZE) {
        memmove(s_buf, s_buf + 2, s_len - 2); s_len -= 2; return ESP_ERR_INVALID_STATE;
    }
    size_t total = (size_t)len + 5;
    if (s_len < total) return ESP_ERR_NOT_FOUND;

    uint8_t cmd = s_buf[3];
    size_t data_len = (size_t)len - 2;
    uint8_t sum = cmd;
    for (size_t i = 0; i < data_len; i++) sum += s_buf[4 + i];

    bool check_ok = (s_buf[4 + data_len] == sum);
    bool tail_ok  = (s_buf[total - 2] == AS201_TAIL0) && (s_buf[total - 1] == AS201_TAIL1);

    esp_err_t ret = ESP_ERR_INVALID_CRC;
    if (check_ok && tail_ok) {
        if (cmd == AS201_CMD_REPORT && data_len >= 35) {
            as201_parse_payload(s_buf + 4, data_len, out);
            ret = ESP_OK;
        } else ret = ESP_ERR_INVALID_STATE;
    }
    memmove(s_buf, s_buf + total, s_len - total);
    s_len -= total;
    return ret;
}

/* ==================== 四、生命周期与任务区 ==================== */
esp_err_t as201_init(void) {
    if (s_data_mutex == NULL) {
        s_data_mutex = xSemaphoreCreateMutex();
        if (s_data_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    uart_config_t cfg = {
        .baud_rate = AS201_BAUD_RATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(AS201_UART_PORT, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(AS201_UART_PORT, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(AS201_UART_PORT, AS201_TX_GPIO, AS201_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    s_len = 0;
    ESP_LOGI(TAG, "陀螺仪 UART 初始化完成");
    return ESP_OK;
}

esp_err_t as201_read(as201_data_t *out, int timeout_ms) {
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    TickType_t t_start = xTaskGetTickCount();
    TickType_t t_wait  = pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        esp_err_t err = as201_process(out);
        if (err == ESP_OK) return ESP_OK;
        if (err == ESP_ERR_INVALID_CRC || err == ESP_ERR_INVALID_STATE) continue;
        if ((xTaskGetTickCount() - t_start) >= t_wait) return ESP_ERR_TIMEOUT;
        if (s_len >= AS201_BUF_SIZE) s_len = 0;
        int n = uart_read_bytes(AS201_UART_PORT, s_buf + s_len, AS201_BUF_SIZE - s_len, pdMS_TO_TICKS(20));
        if (n > 0) s_len += (size_t)n;
    }
}

static void as201_task(void *arg) {
    as201_data_t d;
    while (1) {
        if (as201_read(&d, 500) == ESP_OK) {
            if (s_data_mutex) {
                xSemaphoreTake(s_data_mutex, portMAX_DELAY);
                s_last_data = d; s_data_valid = true;
                xSemaphoreGive(s_data_mutex);
            }
        } else vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t as201_start(void) {
    if (s_task != NULL) return ESP_OK;
    BaseType_t r = xTaskCreate(as201_task, "as201_task", 4096, NULL, 6, &s_task);
    return (r == pdPASS) ? ESP_OK : ESP_FAIL;
}

bool as201_get_latest(as201_data_t *out) {
    if (out == NULL || s_data_mutex == NULL || !s_data_valid) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    *out = s_last_data;
    xSemaphoreGive(s_data_mutex);
    return true;
}