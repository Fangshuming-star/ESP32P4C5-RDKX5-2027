#include "motor_independent.h"
#include "config.h"
#include "limit_switch.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IND_MOTOR";

/* ==================== 一、全局状态与硬件配置区 ==================== */
static const int s_step_gpio[3] = { IND_MOTOR_1_STEP_GPIO, IND_MOTOR_2_STEP_GPIO, IND_MOTOR_3_STEP_GPIO };
static const int s_dir_gpio[3]  = { IND_MOTOR_1_DIR_GPIO,  IND_MOTOR_2_DIR_GPIO,  IND_MOTOR_3_DIR_GPIO  };

static const int s_cal_dir[3] = { CAL_DIR_1, CAL_DIR_2, CAL_DIR_3 };
static const int s_cal_sw[3]  = { CAL_SW_1,  CAL_SW_2,  CAL_SW_3  };
static const int s_cal_speed[3] = { SPEED_CAL_1, SPEED_CAL_2, SPEED_CAL_3 };
static const int s_move_speed[3] = { SPEED_MOVE_1, SPEED_MOVE_2, SPEED_MOVE_3 };

static volatile bool s_run_flag[3] = { false, false, false };
static volatile uint64_t s_next_pulse_us[3] = { 0, 0, 0 };
static bool s_pulse_level[3] = { false, false, false };
static bool s_dir_positive[3] = { true, true, true };
static esp_timer_handle_t s_pulse_timer = NULL;
static uint32_t s_current_speed[3] = { SPEED_MOVE_1, SPEED_MOVE_2, SPEED_MOVE_3 };
static TaskHandle_t s_cal_handle[3] = { NULL, NULL, NULL };

static int32_t s_current_steps[3] = { 0, 0, 0 };
static int32_t s_target_steps[3] = { 0, 0, 0 };
static bool s_keep_enabled = false;
static bool s_count_up[3] = { true, true, true };

/* ==================== 二、机械换算与中断处理区 ==================== */
static float get_steps_per_unit(int id) {
    if (id == 0) {
        return (MOTOR_STEPS_PER_REV * Z_MICROSTEPS * Z_PLATFORM_TEETH) / (Z_MOTOR_TEETH * 360.0f);
    } else if (id == 1) {
        float circumference = X_GEAR_TEETH * X_MODULE * M_PI;
        return (MOTOR_STEPS_PER_REV * X_MICROSTEPS) / circumference;
    } else {
        return (MOTOR_STEPS_PER_REV * Y_MICROSTEPS) / Y_SCREW_LEAD_MM;
    }
}

static void IRAM_ATTR limit_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    if (gpio_get_level(gpio_num) == 0) {
        int idx = -1;
        if (gpio_num == LIMIT_SW1_GPIO) idx = 0;
        else if (gpio_num == LIMIT_SW2_GPIO) idx = 1;
        else if (gpio_num == LIMIT_SW3_GPIO) idx = 2;
        
        if (idx >= 0 && s_run_flag[idx]) {
            s_run_flag[idx] = false;
            gpio_set_level(s_step_gpio[idx], 0);
            s_pulse_level[idx] = false;
            if (s_cal_handle[idx] != NULL) {
                s_current_steps[idx] = 0;
                s_target_steps[idx] = 0;
            } else {
                s_target_steps[idx] = s_current_steps[idx];
            }
        }
    }
}

/* ==================== 三、脉冲定时器生成区 ==================== */
static void pulse_timer_cb(void *arg) {
    uint64_t now = esp_timer_get_time();
    for (int i = 0; i < 3; i++) {
        if (s_run_flag[i]) {
            if (now >= s_next_pulse_us[i]) {
                if (!s_pulse_level[i]) {
                    s_pulse_level[i] = true;
                    gpio_set_level(s_step_gpio[i], 1);
                    if (s_count_up[i]) s_current_steps[i]++;
                    else s_current_steps[i]--;
                    if (s_target_steps[i] != 0 || s_cal_handle[i] == NULL) { 
                        if (s_current_steps[i] == s_target_steps[i] && s_cal_handle[i] == NULL) {
                            s_run_flag[i] = false;
                            gpio_set_level(s_step_gpio[i], 0);
                            s_pulse_level[i] = false;
                        }
                    }
                    uint32_t half_period_us = 1000000 / (s_current_speed[i] * 2);
                    s_next_pulse_us[i] = now + half_period_us;
                } else {
                    s_pulse_level[i] = false;
                    gpio_set_level(s_step_gpio[i], 0);
                    uint32_t half_period_us = 1000000 / (s_current_speed[i] * 2);
                    s_next_pulse_us[i] = now + half_period_us;
                }
            }
        }
    }
}

/* ==================== 四、生命周期与使能控制区 ==================== */
void motor_enable_all(void) { gpio_set_level(IND_MOTOR_EN_GPIO, 0); s_keep_enabled = true; }
void motor_disable_all(void) {
    for (int i = 0; i < 3; i++) { s_run_flag[i] = false; gpio_set_level(s_step_gpio[i], 0); s_pulse_level[i] = false; }
    gpio_set_level(IND_MOTOR_EN_GPIO, 1); s_keep_enabled = false;
}

void motor_independent_run(int motor_id, bool run, bool dir_positive) {
    if (motor_id < 1 || motor_id > 3) return;
    int idx = motor_id - 1;
    gpio_set_level(s_dir_gpio[idx], dir_positive ? 1 : 0);
    s_dir_positive[idx] = dir_positive;
    
    if (run) {
        s_run_flag[idx] = true;
        s_next_pulse_us[idx] = esp_timer_get_time();
        gpio_set_level(IND_MOTOR_EN_GPIO, 0);
    } else {
        s_run_flag[idx] = false;
        gpio_set_level(s_step_gpio[idx], 0);
        s_pulse_level[idx] = false;
        if (!s_keep_enabled) {
            bool all_stopped = true;
            for(int i = 0; i < 3; i++) {
                if (s_run_flag[i]) all_stopped = false;
            }
            if (all_stopped) gpio_set_level(IND_MOTOR_EN_GPIO, 1);
        }
    }
}

void motor_independent_stop_all(void) { for (int i = 0; i < 3; i++) motor_independent_run(i + 1, false, false); }

/* ==================== 五、绝对坐标移动区 ==================== */
void motor_move_to_absolute(float x_mm, float y_mm, float z_deg) {
    float targets[3] = { z_deg, x_mm, y_mm };
    float max_travel[3] = { Z_MAX_TRAVEL_DEG, X_MAX_TRAVEL_MM, Y_MAX_TRAVEL_MM };
    
    for (int i = 0; i < 3; i++) {
        float target = targets[i];
        if (i == 0) {
            if (target > max_travel[i]) target = max_travel[i];
            if (target < -max_travel[i]) target = -max_travel[i]; 
        } else {
            if (target > max_travel[i]) target = max_travel[i];
            if (target < 0.0f) target = 0.0f; 
        }
        
        float steps_per_unit = get_steps_per_unit(i);
        int32_t target_step_count = (int32_t)(target * steps_per_unit);
        if (target_step_count == s_current_steps[i]) continue;

        bool dir_positive;
        if (target_step_count > s_current_steps[i]) {
            dir_positive = (s_cal_dir[i] == 0) ? 1 : 0;
            s_count_up[i] = true;
        } else {
            dir_positive = s_cal_dir[i];
            s_count_up[i] = false;
        }
        s_target_steps[i] = target_step_count;
        s_current_speed[i] = s_move_speed[i]; 
        motor_independent_run(i + 1, true, dir_positive);
    }
}

/* ==================== 六、校准任务区 ==================== */
static void calibrate_worker_task(void *arg) {
    int motor_id = (int)(intptr_t)arg;
    if (motor_id < 1 || motor_id > 3) { vTaskDelete(NULL); return; }
    int idx = motor_id - 1;
    
    vTaskDelay(pdMS_TO_TICKS(500));
    int target_gpio = (s_cal_sw[idx] == 1) ? LIMIT_SW1_GPIO : ((s_cal_sw[idx] == 2) ? LIMIT_SW2_GPIO : LIMIT_SW3_GPIO);
    if (gpio_get_level(target_gpio) == LIMIT_SW_ACTIVE_LEVEL) {
        s_current_steps[idx] = 0; s_target_steps[idx] = 0; s_cal_handle[idx] = NULL;
        vTaskDelete(NULL); return;
    }

    s_current_speed[idx] = s_cal_speed[idx];
    s_count_up[idx] = (s_cal_dir[idx] == 1); 
    s_target_steps[idx] = (s_cal_dir[idx] == 1) ? 99999999 : -99999999; 
    motor_independent_run(motor_id, true, s_cal_dir[idx] == 1);
    
    bool hit_limit = false;
    while (1) {
        if (g_stop_requested) break;
        if (!s_run_flag[idx]) { hit_limit = true; break; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (hit_limit) vTaskDelay(pdMS_TO_TICKS(1000));
    
    motor_independent_run(motor_id, false, s_cal_dir[idx] == 1);
    s_current_speed[idx] = s_move_speed[idx]; 
    s_cal_handle[idx] = NULL;

    if (!s_keep_enabled) {
        bool all_stopped = true;
        for(int i = 0; i < 3; i++) {
            if (s_run_flag[i]) all_stopped = false;
        }
        if (all_stopped) gpio_set_level(IND_MOTOR_EN_GPIO, 1);
    }
    vTaskDelete(NULL);
}

void motor_calibrate_all_async(bool keep_enabled) {
    s_keep_enabled = keep_enabled;
    if (s_keep_enabled) motor_enable_all();
    for (int i = 1; i <= 3; i++) {
        if (s_cal_handle[i-1] == NULL) {
            xTaskCreate(calibrate_worker_task, "cal_worker", 4096, (void*)(intptr_t)i, 4, &s_cal_handle[i-1]);
        }
    }
}

void motor_calibrate_single_async(int motor_id, bool keep_enabled) {
    if (motor_id < 1 || motor_id > 3) return;
    if (s_cal_handle[motor_id-1] != NULL) return;
    s_keep_enabled = keep_enabled;
    if (s_keep_enabled) motor_enable_all();
    xTaskCreate(calibrate_worker_task, "cal_worker", 4096, (void*)(intptr_t)motor_id, 4, &s_cal_handle[motor_id-1]);
}

/* ==================== 七、串口调试控制台区 ==================== */
#if ARM_SERIAL_DEBUG_ENABLE
static void serial_test_task(void *arg) {
    char line[64];
    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            line[strcspn(line, "\r\n")] = 0; 
            if (servo_parse_cmd(line)) continue;

            if (strcmp(line, "stop") == 0) { g_stop_requested = true; motor_independent_stop_all(); }
            else if (strcmp(line, "disen") == 0) { motor_disable_all(); }
            else if (strcmp(line, "encal") == 0) { g_stop_requested = false; int c; while ((c = getchar()) != '\n' && c != EOF); motor_calibrate_all_async(true); }
            else if (strncmp(line, "encal ", 6) == 0) { g_stop_requested = false; int id = line[6] - '0'; motor_calibrate_single_async(id, true); }
            else if (strcmp(line, "cal") == 0) { g_stop_requested = false; int c; while ((c = getchar()) != '\n' && c != EOF); motor_calibrate_all_async(false); }
            else if (strncmp(line, "cal ", 4) == 0) { g_stop_requested = false; int id = line[4] - '0'; motor_calibrate_single_async(id, false); }
            else if (strchr(line, ',') != NULL) {
                float x, y, z;
                if (sscanf(line, "%f,%f,%f", &x, &y, &z) == 3) motor_move_to_absolute(x, y, z);
            } else if (strlen(line) >= 3 && line[1] == '+') {
                int motor_id = line[0] - '0';
                int dir_val = line[2] - '0';
                if (motor_id >= 1 && motor_id <= 3 && (dir_val == 0 || dir_val == 1)) {
                    s_cal_handle[motor_id-1] = NULL; 
                    s_target_steps[motor_id-1] = (dir_val == 1) ? 99999999 : -99999999;
                    s_count_up[motor_id-1] = (dir_val == 1);
                    s_current_speed[motor_id-1] = s_move_speed[motor_id-1];
                    motor_independent_run(motor_id, true, dir_val == 1);
                }
            }
        } else vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif

/* ==================== 八、硬件初始化区 ==================== */
void motor_independent_init(void) {
    gpio_config_t en_conf = { .pin_bit_mask = (1ULL << IND_MOTOR_EN_GPIO), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&en_conf);
    gpio_set_level(IND_MOTOR_EN_GPIO, 1); 

    uint64_t pin_mask = 0;
    for (int i = 0; i < 3; i++) { pin_mask |= (1ULL << s_step_gpio[i]); pin_mask |= (1ULL << s_dir_gpio[i]); }
    gpio_config_t io_conf = { .pin_bit_mask = pin_mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io_conf);
    for (int i = 0; i < 3; i++) { gpio_set_level(s_step_gpio[i], 0); gpio_set_level(s_dir_gpio[i], 0); }

    gpio_config_t isr_conf = {
        .pin_bit_mask = (1ULL << LIMIT_SW1_GPIO) | (1ULL << LIMIT_SW2_GPIO) | (1ULL << LIMIT_SW3_GPIO),
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&isr_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LIMIT_SW1_GPIO, limit_isr_handler, (void*)LIMIT_SW1_GPIO);
    gpio_isr_handler_add(LIMIT_SW2_GPIO, limit_isr_handler, (void*)LIMIT_SW2_GPIO);
    gpio_isr_handler_add(LIMIT_SW3_GPIO, limit_isr_handler, (void*)LIMIT_SW3_GPIO);

    esp_timer_create_args_t t_args = { .callback = &pulse_timer_cb, .name = "ind_pulse" };
    esp_timer_create(&t_args, &s_pulse_timer);
    esp_timer_start_periodic(s_pulse_timer, 50); 
    
#if ARM_SERIAL_DEBUG_ENABLE
    xTaskCreate(serial_test_task, "ind_serial", 4096, NULL, 4, NULL);
#endif
    ESP_LOGI(TAG, "机械臂三轴初始化完成");
}