#include "limit_switch.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LIMIT_SW";

/* ==================== 一、全局变量区 ==================== */
static const int s_sw_gpio[3] = { LIMIT_SW1_GPIO, LIMIT_SW2_GPIO, LIMIT_SW3_GPIO };
static bool s_last_state[3] = { false, false, false };

/* ==================== 二、轮询状态机区 ==================== */
static void limit_switch_task(void *arg) {
    while (1) {
        for (int i = 0; i < 3; i++) {
            bool current_state = limit_switch_read(i + 1);
            if (current_state != s_last_state[i]) {
                s_last_state[i] = current_state;
#if LIMIT_SW_LOG_ENABLE
                if (current_state) ESP_LOGW(TAG, "🛑 [SW%d] 触发", i + 1);
                else ESP_LOGI(TAG, "✅ [SW%d] 释放", i + 1);
#endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ==================== 三、生命周期与读取区 ==================== */
void limit_switch_init(void) {
    for (int i = 0; i < 3; i++) s_last_state[i] = limit_switch_read(i + 1);
    xTaskCreate(limit_switch_task, "limit_sw", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "限位开关初始化完成");
}

bool limit_switch_read(int sw_id) {
    if (sw_id < 1 || sw_id > 3) return false;
    return (gpio_get_level(s_sw_gpio[sw_id - 1]) == LIMIT_SW_ACTIVE_LEVEL);
}