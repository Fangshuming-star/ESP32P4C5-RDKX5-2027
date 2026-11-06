#include "servo.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>
#include <ctype.h>

static const char *TAG = "SERVO";

/* ==================== 一、硬件与通道配置区 ==================== */
#define SERVO_GPIO_1    26
#define SERVO_GPIO_2    27
#define SERVO_GPIO_3    32
#define SERVO_GPIO_4    33

// 使用 LEDC_CHANNEL_4~7 避开底盘
static const ledc_channel_t s_servo_ch[4] = { LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7 };

/* ==================== 二、初始化区 ==================== */
void servo_init(void) {
    ledc_timer_config_t tcfg = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&tcfg);

    gpio_config_t io_conf = { .pin_bit_mask = (1ULL << SERVO_GPIO_1) | (1ULL << SERVO_GPIO_2) | (1ULL << SERVO_GPIO_3) | (1ULL << SERVO_GPIO_4), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io_conf);
    gpio_set_level(SERVO_GPIO_1, 0); gpio_set_level(SERVO_GPIO_2, 0); gpio_set_level(SERVO_GPIO_3, 0); gpio_set_level(SERVO_GPIO_4, 0);

#if SERVO_1_ENABLE
    ledc_channel_config_t ccfg1 = { .gpio_num = SERVO_GPIO_1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = s_servo_ch[0], .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&ccfg1); servo_set_angle(1, 90.0f);
#endif
#if SERVO_2_ENABLE
    ledc_channel_config_t ccfg2 = { .gpio_num = SERVO_GPIO_2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = s_servo_ch[1], .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&ccfg2); servo_set_angle(2, 90.0f);
#endif
    ESP_LOGI(TAG, "舵机初始化完成");
}

/* ==================== 三、角度控制区 ==================== */
void servo_set_angle(int servo_id, float angle_deg) {
    if (servo_id < 1 || servo_id > 4) return;
#if !SERVO_1_ENABLE
    if (servo_id == 1) return;
#endif
#if !SERVO_2_ENABLE
    if (servo_id == 2) return;
#endif
#if !SERVO_3_ENABLE
    if (servo_id == 3) return;
#endif
#if !SERVO_4_ENABLE
    if (servo_id == 4) return;
#endif
    float min_angle = 0.0f, max_angle = 180.0f;
    if (servo_id == 1) { min_angle = SERVO_A_MIN_ANGLE; max_angle = SERVO_A_MAX_ANGLE; }
    else if (servo_id == 2) { min_angle = SERVO_B_MIN_ANGLE; max_angle = SERVO_B_MAX_ANGLE; }
    if (angle_deg < min_angle) angle_deg = min_angle;
    if (angle_deg > max_angle) angle_deg = max_angle;
    
    float pulse_ms = 0.5f + (angle_deg / 180.0f) * 2.0f;
    uint32_t duty = (uint32_t)((pulse_ms / 20.0f) * 16384);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_servo_ch[servo_id - 1], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_servo_ch[servo_id - 1]);
}

/* ==================== 四、串口解析区 ==================== */
bool servo_parse_cmd(char *line) {
    if (line == NULL || strlen(line) < 3) return false;
    char op = line[0];
    if (op >= 'a' && op <= 'd') op -= 32; 
    if (op >= 'A' && op <= 'D') {
        int servo_id = op - 'A' + 1;
        float angle = 0;
        if (sscanf(line + 1, "%f", &angle) == 1) {
            servo_set_angle(servo_id, angle);
            return true;
        }
    }
    return false;
}