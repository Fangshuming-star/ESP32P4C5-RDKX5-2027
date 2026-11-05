#include "qr_reader.h"
#include "config.h"
#include "mission.h"
#include "screen.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "QR_READER";

/* ==================== 一、硬件配置区 ==================== */
#define QR_UART_PORT        UART_NUM_4
#define QR_TX_GPIO          42
#define QR_RX_GPIO          43
#define QR_BAUD_RATE        9600
#define QR_MAX_DATA_LEN     100
#define QR_READ_CHUNK_SIZE  64
#define QR_FRAME_IDLE_MS    100

/* ==================== 二、数据接收与解析区 ==================== */
static void qr_process_frame(uint8_t *frame, size_t length)
{
    if (length == 0) return;
    frame[length] = '\0';
#if QR_LOG_ENABLE
    ESP_LOGI(TAG, "收到二维码数据 (%u bytes)", (unsigned)length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, frame, length, ESP_LOG_INFO);
#endif

    if (memchr(frame, '\0', length) != NULL) {
        ESP_LOGW(TAG, "二维码数据包含 NUL 字节，已忽略");
        return;
    }

    g_qr_received = true;
    mission_notify_qr_complete();

    screen_send_str("GOTO_MISSION");
    vTaskDelay(pdMS_TO_TICKS(200));

    char cmd[QR_MAX_DATA_LEN + 3];
    int cmd_len = snprintf(cmd, sizeof(cmd), "T:%s", (char *)frame);
    if (cmd_len < 0 || (size_t)cmd_len >= sizeof(cmd)) {
        ESP_LOGE(TAG, "二维码数据无法编码为屏幕命令");
        return;
    }
    screen_send_str(cmd);
    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t *plus = memchr(frame, '+', length);
    if (plus == NULL) return;

    size_t first_len = (size_t)(plus - frame);
    size_t second_len = length - first_len - 1;
    if (first_len > 0) {
        char current_cmd[16];
        snprintf(current_cmd, sizeof(current_cmd), "C:%c", frame[0]);
        screen_send_str(current_cmd);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (second_len > 0) {
        char next_cmd[16];
        snprintf(next_cmd, sizeof(next_cmd), "N:%c", plus[1]);
        screen_send_str(next_cmd);
    }
}

static void qr_rx_task(void *arg)
{
    uint8_t chunk[QR_READ_CHUNK_SIZE];
    uint8_t frame[QR_MAX_DATA_LEN + 1];
    size_t frame_len = 0;
    bool discard_frame = false;

    while (1) {
        int n = uart_read_bytes(QR_UART_PORT, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(QR_FRAME_IDLE_MS));
        if (n < 0) {
            ESP_LOGE(TAG, "二维码 UART 接收失败");
            vTaskDelay(pdMS_TO_TICKS(QR_FRAME_IDLE_MS));
            continue;
        }
        if (n == 0) {
            if (discard_frame) {
                ESP_LOGW(TAG, "二维码数据超过 %d 字节，已丢弃", QR_MAX_DATA_LEN);
            } else if (frame_len > 0) {
                qr_process_frame(frame, frame_len);
            }
            frame_len = 0;
            discard_frame = false;
            continue;
        }

        for (int i = 0; i < n; i++) {
            uint8_t byte = chunk[i];
            if (byte == '\r' || byte == '\n') {
                if (discard_frame) {
                    ESP_LOGW(TAG, "二维码数据超过 %d 字节，已丢弃", QR_MAX_DATA_LEN);
                } else if (frame_len > 0) {
                    qr_process_frame(frame, frame_len);
                }
                frame_len = 0;
                discard_frame = false;
                continue;
            }

            if (discard_frame) continue;
            if (frame_len >= QR_MAX_DATA_LEN) {
                frame_len = 0;
                discard_frame = true;
                continue;
            }
            frame[frame_len++] = byte;
        }
    }
}

/* ==================== 三、初始化区 ==================== */
void qr_reader_init(void) {
    uart_config_t cfg = {
        .baud_rate  = QR_BAUD_RATE, .data_bits  = UART_DATA_8_BITS, .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1, .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(QR_UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(QR_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(QR_UART_PORT, QR_TX_GPIO, QR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (xTaskCreate(qr_rx_task, "qr_rx", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "二维码接收任务创建失败");
        abort();
    }
    ESP_LOGI(TAG, "二维码 UART4 初始化完成 (TX=%d, RX=%d, 9600 8N1)",
             QR_TX_GPIO, QR_RX_GPIO);
}
