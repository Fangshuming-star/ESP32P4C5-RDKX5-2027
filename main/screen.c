#include "screen.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "SCREEN";

/* ==================== 一、全局队列区 ==================== */
static QueueHandle_t s_screen_queue = NULL;

/* ==================== 二、串口接收与解析区 ==================== */
static void screen_rx_task(void *arg) {
    uart_port_t port = (uart_port_t)SCREEN_UART_PORT_NUM;
    uint8_t buf[1024]; size_t len = 0; uint8_t tmp[128];
    while (1) {
        int n = uart_read_bytes(port, tmp, sizeof(tmp), pdMS_TO_TICKS(50));
        if (n <= 0) continue;
        if (len + n > sizeof(buf)) len = 0; 
        memcpy(buf + len, tmp, n); len += n;
        while (len >= 3) {
            int end_idx = -1;
            for (size_t i = 0; i + 2 < len; i++) {
                if (buf[i] == 0xFF && buf[i+1] == 0xFF && buf[i+2] == 0xFF) { end_idx = i; break; }
            }
            if (end_idx < 0) break;
            screen_msg_t msg = {0};

            if (end_idx == 83 && memcmp(buf, "POINTS,5", 8) == 0) {
                msg.type = SCREEN_EVT_POINTS;
                for (int v = 0; v < 15; v++) memcpy(&msg.data.points.vals[v], buf + 9 + v * 5, 4);
            } else if (end_idx > 0) {
                char text[128] = {0};
                memcpy(text, buf, (end_idx < 127) ? end_idx : 127);
                if (strcmp(text, "START") == 0) msg.type = SCREEN_EVT_START;
                else if (strcmp(text, "STOP") == 0) msg.type = SCREEN_EVT_STOP;
                else if (strcmp(text, "ENTER_MISSION") == 0) msg.type = SCREEN_EVT_ENTER_MISSION;
                else if (strcmp(text, "LEAVE_MISSION") == 0) msg.type = SCREEN_EVT_LEAVE_MISSION;
                else if (strcmp(text, "SWITCH_VIEW1") == 0) msg.type = SCREEN_EVT_SWITCH_VIEW1;
                else if (strcmp(text, "SWITCH_VIEW2") == 0) msg.type = SCREEN_EVT_SWITCH_VIEW2;
                else if (strncmp(text, "PARAM,", 6) == 0) {
                    msg.type = SCREEN_EVT_PARAM;
                    int p[10] = {0};
                    sscanf(text + 6, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8], &p[9]);
                    msg.data.param.carL = p[0]; msg.data.param.carW = p[1]; msg.data.param.spdMove = p[2]; msg.data.param.spdRot = p[3];
                    msg.data.param.grabT = p[4]; msg.data.param.rawGap = p[5]; msg.data.param.tempY = p[6]; msg.data.param.obsRad = p[7];
                    msg.data.param.safeRad = p[8]; msg.data.param.zoneSel = p[9];
                }
                else if (strncmp(text, "encal", 5) == 0) { msg.type = SCREEN_EVT_ENCAL; strncpy(msg.data.text, text, sizeof(msg.data.text)-1); }
                else if (strncmp(text, "cal", 3) == 0)   { msg.type = SCREEN_EVT_CAL; strncpy(msg.data.text, text, sizeof(msg.data.text)-1); }
                else if (strncmp(text, "disen", 5) == 0) { msg.type = SCREEN_EVT_DISEN; }
                else if (strncmp(text, "UAR", 3) == 0)   { msg.type = SCREEN_EVT_UAR; strncpy(msg.data.text, text, sizeof(msg.data.text)-1); }
            }
            if (msg.type != SCREEN_EVT_NONE) xQueueSend(s_screen_queue, &msg, 0);
            size_t consumed = end_idx + 3;
            memmove(buf, buf + consumed, len - consumed); len -= consumed;
        }
    }
}

/* ==================== 三、生命周期与发送区 ==================== */
void screen_init(void) {
    s_screen_queue = xQueueCreate(10, sizeof(screen_msg_t));
    uart_config_t cfg = {
        .baud_rate  = SCREEN_BAUD_RATE, .data_bits  = UART_DATA_8_BITS, .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1, .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install((uart_port_t)SCREEN_UART_PORT_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config((uart_port_t)SCREEN_UART_PORT_NUM, &cfg);
    uart_set_pin((uart_port_t)SCREEN_UART_PORT_NUM, SCREEN_TX_GPIO, SCREEN_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(screen_rx_task, "screen_rx", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "串口屏通信初始化完成");
}

void screen_send_str(const char *msg) {
    if (!msg) return;
    uart_write_bytes((uart_port_t)SCREEN_UART_PORT_NUM, msg, strlen(msg));
    const uint8_t end[3] = {0xFF, 0xFF, 0xFF};
    uart_write_bytes((uart_port_t)SCREEN_UART_PORT_NUM, end, 3);
    vTaskDelay(pdMS_TO_TICKS(30)); 
}

bool screen_get_msg(screen_msg_t *msg, int timeout_ms) {
    return xQueueReceive(s_screen_queue, msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}