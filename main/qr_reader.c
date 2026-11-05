#include "qr_reader.h"
#include "config.h"
#include "mission.h"
#include "screen.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "QR_READER";

/* ==================== 一、硬件配置区 ==================== */
#define QR_UART_PORT        UART_NUM_4
#define QR_TX_GPIO          42
#define QR_RX_GPIO          43
#define QR_BAUD_RATE        9600
#define QR_BUF_SIZE         1024 

/* ==================== 二、数据接收与解析区 ==================== */
static void qr_rx_task(void *arg) {
    uint8_t buf[QR_BUF_SIZE];
    while (1) {
        int n = uart_read_bytes(QR_UART_PORT, buf, QR_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;
        
        buf[n] = '\0'; 
        for (int i = 0; i < n; i++) {
            if (buf[i] == '\r' || buf[i] == '\n') { buf[i] = '\0'; break; }
        }

#if QR_LOG_ENABLE
        ESP_LOGI(TAG, "收到二维码数据: %s", (char *)buf);
#endif

        if (strlen((char *)buf) > 0) {
            g_qr_received = true; 
            mission_notify_qr_complete();

            screen_send_str("GOTO_MISSION");
            vTaskDelay(pdMS_TO_TICKS(200));

            char cmd[256]; 
            snprintf(cmd, sizeof(cmd), "T:%.100s", (char *)buf); 
            screen_send_str(cmd);
            vTaskDelay(pdMS_TO_TICKS(200));

            char *plus = strchr((char *)buf, '+');
            if (plus != NULL) {
                *plus = '\0';
                char *part1 = (char *)buf;
                char *part2 = plus + 1;

                if (strlen(part1) > 0) {
                    char c_cmd[16];
                    snprintf(c_cmd, sizeof(c_cmd), "C:%c", part1[0]);
                    screen_send_str(c_cmd);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                if (strlen(part2) > 0) {
                    char n_cmd[16];
                    snprintf(n_cmd, sizeof(n_cmd), "N:%c", part2[0]);
                    screen_send_str(n_cmd);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
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
    
    xTaskCreate(qr_rx_task, "qr_rx", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "二维码 UART4 初始化完成 (TX=%d, RX=%d, 9600)", QR_TX_GPIO, QR_RX_GPIO);
}