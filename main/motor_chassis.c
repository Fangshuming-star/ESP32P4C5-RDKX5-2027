#include "motor_chassis.h"
#include "chassis_pid.h"
#include "config.h"
#include "as201.h" 
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

_Static_assert(pdMS_TO_TICKS(CHASSIS_SPEED_CONTROL_PERIOD_MS) > 0,
               "CHASSIS_SPEED_CONTROL_PERIOD_MS is below the FreeRTOS tick resolution");
_Static_assert(pdMS_TO_TICKS(STRAIGHT_YAW_CORRECTION_PERIOD_MS) > 0,
               "STRAIGHT_YAW_CORRECTION_PERIOD_MS is below the FreeRTOS tick resolution");

static const char *TAG = "CHASSIS";

/* ============================================================
 * 一、引脚定义与使能极性
 * 调试：
 * 1. 如果轮子完全没反应，检查 CHASSIS_PIN_EN 是否有效。
 * 2. CHASSIS_EN_ACTIVE_LEVEL 为 0 表示低电平锁死。
 * ============================================================ */
#define CHASSIS_PIN_EN          34      
#define CHASSIS_EN_ACTIVE_LEVEL 0

#define CHASSIS_M1_STEP_GPIO    13      
#define CHASSIS_M1_DIR_GPIO     28      
#define CHASSIS_M2_STEP_GPIO    2       
#define CHASSIS_M2_DIR_GPIO     8       
#define CHASSIS_M3_STEP_GPIO    3       
#define CHASSIS_M3_DIR_GPIO     12      
#define CHASSIS_M4_STEP_GPIO    11      
#define CHASSIS_M4_DIR_GPIO     21      

/* ============================================================
 * 二、运动方向与绕角配置
 * 调试：
 * 1. 车动反了：改 DIR_FWD_Mx 的 0/1。
 * 2. 绕左前，你希望 1,4 转。如果实际是 2,3 转，把 ROT_LT_EN 的 1,4 改为 1，2,3 改为 0。
 * 3. 绕角方向反向：改 ROT_LT_DIR 的 0/1。
 * ============================================================ */
#define DIR_FWD_M1      0
#define DIR_FWD_M2      0
#define DIR_FWD_M3      1
#define DIR_FWD_M4      1

#define DIR_STL_M1      1
#define DIR_STL_M2      0
#define DIR_STL_M3      0
#define DIR_STL_M4      1

#define DIR_RTL_M1      1
#define DIR_RTL_M2      0
#define DIR_RTL_M3      1
#define DIR_RTL_M4      0

// 绕角旋转 发脉冲开关
#define ROT_LT_EN       { 1, 0, 0, 1 } // 绕左前 (1和4转，2和3停)
#define ROT_RT_EN       { 0, 1, 1, 0 } // 绕右前 (2和3转，1和4停)
#define ROT_LB_EN       { 1, 0, 0, 1 } // 绕左后 (1和4转)
#define ROT_RB_EN       { 0, 1, 1, 0 } // 绕右后 (2和3转)

// 绕角旋转方向 0/1：前绕按前进方向驱动对应轮组，后绕将对应轮组反转
#define ROT_LT_DIR      { 0, 0, 1, 1 } // 左前：1、4前进，2、3停止
#define ROT_RT_DIR      { 1, 0, 1, 1 } // 右前：2、3前进，1、4停止
#define ROT_LB_DIR      { 1, 0, 1, 0 } // 左后：1、4反转
#define ROT_RB_DIR      { 0, 1, 0, 1 } // 右后：2、3反转

#define INVERT_BACKWARD 1   
#define INVERT_ST_RIGHT 1   
#define INVERT_RT_RIGHT 1   

static const int s_step_gpio[CHASSIS_MOTOR_NUM] = { CHASSIS_M1_STEP_GPIO, CHASSIS_M2_STEP_GPIO, CHASSIS_M3_STEP_GPIO, CHASSIS_M4_STEP_GPIO };
static const int s_dir_gpio[CHASSIS_MOTOR_NUM]  = { CHASSIS_M1_DIR_GPIO,  CHASSIS_M2_DIR_GPIO,  CHASSIS_M3_DIR_GPIO,  CHASSIS_M4_DIR_GPIO };

static uint8_t s_running[CHASSIS_MOTOR_NUM]; 
static uint8_t s_current_dir[CHASSIS_MOTOR_NUM]; 
static uint32_t s_current_speed[CHASSIS_MOTOR_NUM]; 
static uint32_t s_target_speed[CHASSIS_MOTOR_NUM];  
static uint64_t s_next_pulse_us[CHASSIS_MOTOR_NUM]; 
static uint8_t s_pulse_level[CHASSIS_MOTOR_NUM];

static esp_timer_handle_t s_pulse_timer = NULL;
static SemaphoreHandle_t s_move_done_sem = NULL;
static volatile bool s_distance_tracking;
static volatile bool s_distance_cancelled;
static volatile uint32_t s_distance_target_steps;
static volatile uint32_t s_distance_steps[CHASSIS_MOTOR_NUM];
static volatile uint8_t s_distance_motor_mask;
static volatile bool s_distance_final_pulse[CHASSIS_MOTOR_NUM];
static volatile uint32_t s_distance_cruise_hz[CHASSIS_MOTOR_NUM];

static void finish_distance_motor(int motor);

/* ============================================================
 * 脉冲生成回调 (由软件定时器每100us触发一次)
 * ============================================================ */
static void pulse_timer_cb(void *arg) {
    uint64_t now = esp_timer_get_time();
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (s_distance_tracking && (s_distance_motor_mask & (1U << i)) &&
            s_distance_final_pulse[i]) {
            s_pulse_level[i] = 0;
            gpio_set_level(s_step_gpio[i], 0);
            finish_distance_motor(i);
            continue;
        }

        if (s_running[i] && s_current_speed[i] > 0) {
            if (now >= s_next_pulse_us[i]) {
                s_pulse_level[i] = !s_pulse_level[i];
                gpio_set_level(s_step_gpio[i], s_pulse_level[i]);
                if (s_distance_tracking && (s_distance_motor_mask & (1U << i)) &&
                    s_pulse_level[i]) {
                    s_distance_steps[i]++;
                    if (s_distance_steps[i] >= s_distance_target_steps) {
                        s_distance_final_pulse[i] = true;
                    }
                }
                uint32_t safe_speed = s_current_speed[i];
                uint32_t half_period_us = 1000000 / (safe_speed * 2);
                s_next_pulse_us[i] = now + half_period_us;
            }
        }
    }
}

static void finish_distance_motor(int motor) {
    s_distance_final_pulse[motor] = false;
    s_running[motor] = 0;
    s_current_speed[motor] = 0;
    s_target_speed[motor] = 0;

    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if ((s_distance_motor_mask & (1U << i)) && s_running[i]) return;
    }

    s_distance_tracking = false;
    if (s_move_done_sem) xSemaphoreGive(s_move_done_sem);
}

/* ============================================================
 * 平滑调速任务 (解决启动打滑)
 * ============================================================ */
static void speed_control_task(void *arg) {
    while (1) {
        uint32_t distance_motor_count = 0;
        uint32_t distance_speed_sum = 0;
        uint32_t distance_target_sum = 0;

        if (s_distance_tracking) {
            for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
                if (!(s_distance_motor_mask & (1U << i))) continue;

                if (s_distance_steps[i] >= s_distance_target_steps) {
                    s_target_speed[i] = 0;
                    continue;
                }

                uint64_t remaining_steps = s_distance_target_steps - s_distance_steps[i];
                float accel_hz_per_sec = (float)DECEL_HZ_PER_STEP * 1000.0f /
                                         CHASSIS_SPEED_CONTROL_PERIOD_MS;
                uint32_t braking_hz = (uint32_t)sqrtf(
                    2.0f * accel_hz_per_sec * (float)remaining_steps);
                uint32_t target_hz = s_distance_cruise_hz[i];
                if (target_hz > braking_hz) target_hz = braking_hz;
                if (target_hz < 1) target_hz = 1;
                s_target_speed[i] = target_hz;

                distance_speed_sum += s_current_speed[i];
                distance_target_sum += target_hz;
                distance_motor_count++;
            }
        }

        uint32_t average_current_hz = distance_motor_count ?
            distance_speed_sum / distance_motor_count : 0;
        uint32_t average_target_hz = distance_motor_count ?
            distance_target_sum / distance_motor_count : 0;
        int32_t base_speed_delta = 0;
        if (average_current_hz < average_target_hz) {
            uint32_t delta = average_target_hz - average_current_hz;
            base_speed_delta = delta < ACCEL_HZ_PER_STEP ?
                (int32_t)delta : ACCEL_HZ_PER_STEP;
        } else if (average_current_hz > average_target_hz) {
            uint32_t delta = average_current_hz - average_target_hz;
            base_speed_delta = -(int32_t)(delta < DECEL_HZ_PER_STEP ?
                delta : DECEL_HZ_PER_STEP);
        }

        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            if (s_distance_tracking && (s_distance_motor_mask & (1U << i))) {
                if (s_distance_steps[i] >= s_distance_target_steps) {
                    s_target_speed[i] = 0;
                    s_current_speed[i] = 0;
                    s_running[i] = 0;
                    s_pulse_level[i] = 0;
                    gpio_set_level(s_step_gpio[i], 0);
                    continue;
                }

                int32_t current_offset = (int32_t)s_current_speed[i] -
                                         (int32_t)average_current_hz;
                int32_t target_offset = (int32_t)s_target_speed[i] -
                                        (int32_t)average_target_hz;
                int32_t correction_delta = target_offset - current_offset;
                if (correction_delta > (int32_t)STRAIGHT_YAW_SPEED_STEP_HZ) {
                    correction_delta = STRAIGHT_YAW_SPEED_STEP_HZ;
                } else if (correction_delta < -(int32_t)STRAIGHT_YAW_SPEED_STEP_HZ) {
                    correction_delta = -(int32_t)STRAIGHT_YAW_SPEED_STEP_HZ;
                }

                int32_t next_speed = (int32_t)s_current_speed[i] +
                                     base_speed_delta + correction_delta;
                if (next_speed < 1) next_speed = 1;
                if (next_speed > (int32_t)s_target_speed[i]) {
                    next_speed = (int32_t)s_target_speed[i];
                }
                s_current_speed[i] = (uint32_t)next_speed;
                s_running[i] = 1;
                continue;
            }

            if (s_target_speed[i] == 0) {
                if (s_current_speed[i] > DECEL_HZ_PER_STEP) {
                    s_current_speed[i] -= DECEL_HZ_PER_STEP;
                } else {
                    s_current_speed[i] = 0;
                    s_running[i] = 0;
                    s_pulse_level[i] = 0;
                    gpio_set_level(s_step_gpio[i], 0);
                }
            } else if (s_current_speed[i] < s_target_speed[i]) {
                s_current_speed[i] += ACCEL_HZ_PER_STEP;
                if (s_current_speed[i] > s_target_speed[i]) s_current_speed[i] = s_target_speed[i];
                s_running[i] = 1;
            } else if (s_current_speed[i] > s_target_speed[i]) {
                if (s_current_speed[i] > DECEL_HZ_PER_STEP) s_current_speed[i] -= DECEL_HZ_PER_STEP;
                else s_current_speed[i] = 0;
                if (s_current_speed[i] < s_target_speed[i]) s_current_speed[i] = s_target_speed[i];
                if (s_current_speed[i] == 0) s_running[i] = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_SPEED_CONTROL_PERIOD_MS));
    }
}

esp_err_t chassis_init(void) {
    (void)TAG; // 消除未使用警告
    gpio_config_t en_cfg = { .pin_bit_mask = (1ULL << CHASSIS_PIN_EN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&en_cfg);
    gpio_set_level(CHASSIS_PIN_EN, CHASSIS_EN_ACTIVE_LEVEL ? 0 : 1);

    uint64_t out_mask = 0;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        out_mask |= (1ULL << s_step_gpio[i]);
        out_mask |= (1ULL << s_dir_gpio[i]);
    }
    gpio_config_t io_cfg = { .pin_bit_mask = out_mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io_cfg);
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        gpio_set_level(s_step_gpio[i], 0);
        gpio_set_level(s_dir_gpio[i], 0);
        s_running[i] = 0; s_pulse_level[i] = 0;
        s_current_speed[i] = 0; s_target_speed[i] = 0;
    }

    esp_timer_create_args_t p_args = { .callback = &pulse_timer_cb, .name = "chassis_pulse" };
    esp_timer_create(&p_args, &s_pulse_timer);
    esp_timer_start_periodic(s_pulse_timer, 100);

    s_move_done_sem = xSemaphoreCreateBinary();
    xTaskCreate(speed_control_task, "chassis_speed", 2048, NULL, 4, NULL);
    chassis_enable(1);
    return ESP_OK;
}

esp_err_t chassis_enable(bool enable) { return gpio_set_level(CHASSIS_PIN_EN, enable ? CHASSIS_EN_ACTIVE_LEVEL : !CHASSIS_EN_ACTIVE_LEVEL); }

esp_err_t chassis_stop(void) {
    if (s_distance_tracking) {
        s_distance_cancelled = true;
        s_distance_tracking = false;
        s_distance_motor_mask = 0;
        if (s_move_done_sem) xSemaphoreGive(s_move_done_sem);
    }
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) s_target_speed[i] = 0; 
    return ESP_OK;
}

esp_err_t chassis_abort(void) { 
    if (s_distance_tracking) s_distance_cancelled = true;
    s_distance_tracking = false;
    s_distance_motor_mask = 0;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        s_current_speed[i] = 0; s_target_speed[i] = 0; s_running[i] = 0;
        s_distance_final_pulse[i] = false;
        gpio_set_level(s_step_gpio[i], 0);
    }
    if (s_move_done_sem) xSemaphoreGive(s_move_done_sem); 
    return ESP_OK; 
}

esp_err_t chassis_set_speed(uint32_t hz) {
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) { s_target_speed[i] = hz; s_running[i] = 1; }
    return ESP_OK;
}

esp_err_t chassis_set_dir(chassis_motor_t motor, bool dir_positive) {
    if (motor >= CHASSIS_MOTOR_NUM) return ESP_ERR_INVALID_ARG;
    gpio_set_level(s_dir_gpio[motor], dir_positive ? 1 : 0); return ESP_OK;
}

static esp_err_t get_mode_params(chassis_mode_t mode, uint32_t *speed_hz, uint8_t en[CHASSIS_MOTOR_NUM], uint8_t dir[CHASSIS_MOTOR_NUM]) {
    uint32_t hz_move = g_spdMove * SPEED_MOVE_MULT;
    uint32_t hz_rot  = g_spdRot * SPEED_ROT_MULT;
    
    const uint8_t lt_en[4] = ROT_LT_EN, lt_dir[4] = ROT_LT_DIR;
    const uint8_t rt_en[4] = ROT_RT_EN, rt_dir[4] = ROT_RT_DIR;
    const uint8_t lb_en[4] = ROT_LB_EN, lb_dir[4] = ROT_LB_DIR;
    const uint8_t rb_en[4] = ROT_RB_EN, rb_dir[4] = ROT_RB_DIR;

    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) en[i] = 1;

    switch (mode) {
    case CHASSIS_MODE_FORWARD:        *speed_hz = hz_move; dir[0] = DIR_FWD_M1 & 1; dir[1] = DIR_FWD_M2 & 1; dir[2] = DIR_FWD_M3 & 1; dir[3] = DIR_FWD_M4 & 1; break;
    case CHASSIS_MODE_BACKWARD:       *speed_hz = hz_move; dir[0] = (DIR_FWD_M1 ^ INVERT_BACKWARD) & 1; dir[1] = (DIR_FWD_M2 ^ INVERT_BACKWARD) & 1; dir[2] = (DIR_FWD_M3 ^ INVERT_BACKWARD) & 1; dir[3] = (DIR_FWD_M4 ^ INVERT_BACKWARD) & 1; break;
    case CHASSIS_MODE_STRAFE_LEFT:    *speed_hz = hz_move; dir[0] = DIR_STL_M1 & 1; dir[1] = DIR_STL_M2 & 1; dir[2] = DIR_STL_M3 & 1; dir[3] = DIR_STL_M4 & 1; break;
    case CHASSIS_MODE_STRAFE_RIGHT:   *speed_hz = hz_move; dir[0] = (DIR_STL_M1 ^ INVERT_ST_RIGHT) & 1; dir[1] = (DIR_STL_M2 ^ INVERT_ST_RIGHT) & 1; dir[2] = (DIR_STL_M3 ^ INVERT_ST_RIGHT) & 1; dir[3] = (DIR_STL_M4 ^ INVERT_ST_RIGHT) & 1; break;
    case CHASSIS_MODE_ROTATE_LEFT:    *speed_hz = hz_rot;  dir[0] = DIR_RTL_M1 & 1; dir[1] = DIR_RTL_M2 & 1; dir[2] = DIR_RTL_M3 & 1; dir[3] = DIR_RTL_M4 & 1; break;
    case CHASSIS_MODE_ROTATE_RIGHT:   *speed_hz = hz_rot;  dir[0] = (DIR_RTL_M1 ^ INVERT_RT_RIGHT) & 1; dir[1] = (DIR_RTL_M2 ^ INVERT_RT_RIGHT) & 1; dir[2] = (DIR_RTL_M3 ^ INVERT_RT_RIGHT) & 1; dir[3] = (DIR_RTL_M4 ^ INVERT_RT_RIGHT) & 1; break;
    case CHASSIS_MODE_ROTATE_LT:      *speed_hz = hz_rot;  for(int i=0;i<4;i++){en[i]=lt_en[i]; dir[i]=lt_dir[i];} break;
    case CHASSIS_MODE_ROTATE_RT:      *speed_hz = hz_rot;  for(int i=0;i<4;i++){en[i]=rt_en[i]; dir[i]=rt_dir[i];} break;
    case CHASSIS_MODE_ROTATE_LB:      *speed_hz = hz_rot;  for(int i=0;i<4;i++){en[i]=lb_en[i]; dir[i]=lb_dir[i];} break;
    case CHASSIS_MODE_ROTATE_RB:      *speed_hz = hz_rot;  for(int i=0;i<4;i++){en[i]=rb_en[i]; dir[i]=rb_dir[i];} break;
    default: return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t chassis_move_at_speed_reverse(chassis_mode_t mode, uint32_t speed_hz, bool reverse) {
    if (speed_hz < 1) speed_hz = 1;
    if (speed_hz > 5000) speed_hz = 5000;
    
    uint32_t def_hz; uint8_t en[CHASSIS_MOTOR_NUM]; uint8_t dir[CHASSIS_MOTOR_NUM];
    if (get_mode_params(mode, &def_hz, en, dir) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (reverse) {
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) dir[i] ^= 1;
    }

    bool need_restart = 0;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (en[i] && s_current_dir[i] != dir[i]) { need_restart = 1; break; }
    }

    if (need_restart) {
        chassis_abort();
        uint64_t now = esp_timer_get_time();
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            gpio_set_level(s_dir_gpio[i], dir[i] ? 1 : 0);
            s_current_dir[i] = dir[i];
            if (en[i]) {
                s_current_speed[i] = speed_hz; s_target_speed[i] = speed_hz;
                s_next_pulse_us[i] = now; s_running[i] = 1;
            } else {
                s_current_speed[i] = 0; s_target_speed[i] = 0; s_running[i] = 0; // 停脉冲但不切EN
            }
        }
    } else {
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            if (en[i]) { s_target_speed[i] = speed_hz; s_running[i] = 1; }
        }
    }
    return ESP_OK;
}

esp_err_t chassis_move_at_speed(chassis_mode_t mode, uint32_t speed_hz) {
    return chassis_move_at_speed_reverse(mode, speed_hz, false);
}

static esp_err_t apply_and_run(uint32_t speed_hz, const uint8_t en[CHASSIS_MOTOR_NUM], const uint8_t dir[CHASSIS_MOTOR_NUM]) {
    chassis_abort();
    uint64_t now = esp_timer_get_time();
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        gpio_set_level(s_dir_gpio[i], dir[i] ? 1 : 0);
        s_current_dir[i] = dir[i];
        if (en[i]) {
            s_current_speed[i] = speed_hz; s_target_speed[i] = speed_hz;
            s_next_pulse_us[i] = now; s_running[i] = 1;
        } else {
            s_current_speed[i] = 0; s_target_speed[i] = 0; s_running[i] = 0;
        }
    }
    return ESP_OK;
}

static esp_err_t apply_and_run_differential(uint32_t hz_14, uint32_t hz_23, const uint8_t dir[CHASSIS_MOTOR_NUM]) {
    bool direction_changed = false;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (s_current_dir[i] != dir[i]) {
            direction_changed = true;
            break;
        }
    }
    if (direction_changed) chassis_abort();

    uint64_t now = esp_timer_get_time();
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        gpio_set_level(s_dir_gpio[i], dir[i] ? 1 : 0);
        s_current_dir[i] = dir[i];
        uint32_t target_hz = (i == 0 || i == 3) ? hz_14 : hz_23;
        if (target_hz < 1) target_hz = 1;

        if (s_distance_tracking && (s_distance_motor_mask & (1U << i))) {
            s_distance_cruise_hz[i] = target_hz;
            if (s_distance_steps[i] < s_distance_target_steps && !s_running[i]) {
                s_next_pulse_us[i] = now;
                s_running[i] = 1;
            }
        } else {
            s_target_speed[i] = target_hz;
            if (!s_running[i]) {
                s_next_pulse_us[i] = now;
                s_running[i] = 1;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t distance_to_steps(float distance_mm, uint32_t *steps_out) {
    if (steps_out == NULL || !isfinite(distance_mm) || distance_mm <= 0.1f ||
        !isfinite(CHASSIS_DISTANCE_CALIBRATION_FACTOR) ||
        CHASSIS_DISTANCE_CALIBRATION_FACTOR <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float circumference = 2.0f * (float)M_PI * WHEEL_RADIUS_MM;
    float pulses_per_mm = (MOTOR_STEPS_PER_REV * MOTOR_MICROSTEPS) / circumference;
    float calibrated_steps = distance_mm * CHASSIS_DISTANCE_CALIBRATION_FACTOR *
                             pulses_per_mm;
    if (!isfinite(calibrated_steps) || calibrated_steps >= (float)UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t steps = (uint32_t)lroundf(calibrated_steps);
    *steps_out = steps > 0 ? steps : 1;
    return ESP_OK;
}

static esp_err_t start_distance_tracking(chassis_mode_t mode, float distance_mm,
                                         uint32_t *cruise_hz, uint32_t *target_steps) {
    uint8_t en[CHASSIS_MOTOR_NUM];
    uint8_t dir[CHASSIS_MOTOR_NUM];
    uint32_t mode_hz;
    esp_err_t ret = get_mode_params(mode, &mode_hz, en, dir);
    if (ret != ESP_OK) return ret;
    if (!s_move_done_sem) return ESP_ERR_INVALID_STATE;
    uint32_t steps;
    ret = distance_to_steps(distance_mm, &steps);
    if (ret != ESP_OK) return ret;

    chassis_abort();
    xSemaphoreTake(s_move_done_sem, 0);
    s_distance_cancelled = false;

    s_distance_target_steps = steps;
    s_distance_motor_mask = 0;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        s_distance_steps[i] = 0;
        s_distance_final_pulse[i] = false;
        gpio_set_level(s_dir_gpio[i], dir[i] ? 1 : 0);
        s_current_dir[i] = dir[i];

        if (en[i]) {
            uint32_t hz = mode_hz < 1 ? 1 : mode_hz;
            s_distance_motor_mask |= (uint8_t)(1U << i);
            s_distance_cruise_hz[i] = hz;
            s_current_speed[i] = 0;
            s_target_speed[i] = hz;
            s_next_pulse_us[i] = esp_timer_get_time();
            s_running[i] = 0;
        } else {
            s_distance_cruise_hz[i] = 0;
            s_current_speed[i] = 0;
            s_target_speed[i] = 0;
            s_running[i] = 0;
        }
    }

    if (s_distance_motor_mask == 0) {
        s_distance_tracking = false;
        return ESP_ERR_INVALID_STATE;
    }

    *cruise_hz = mode_hz < 1 ? 1 : mode_hz;
    *target_steps = steps;
    s_distance_tracking = true;
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (s_distance_motor_mask & (1U << i)) {
            s_running[i] = 1;
        }
    }
    return ESP_OK;
}

esp_err_t chassis_move(chassis_mode_t mode) {
    uint32_t speed_hz; uint8_t en[CHASSIS_MOTOR_NUM]; uint8_t dir[CHASSIS_MOTOR_NUM];
    if (get_mode_params(mode, &speed_hz, en, dir) != ESP_OK) return ESP_ERR_INVALID_ARG;
    return apply_and_run(speed_hz, en, dir);
}

esp_err_t chassis_move_distance(chassis_mode_t mode, float distance_mm) {
    if (distance_mm <= 0.1f) return ESP_OK;
    uint32_t cruise_hz;
    uint32_t target_steps;
    return start_distance_tracking(mode, distance_mm, &cruise_hz, &target_steps);
}

esp_err_t chassis_move_distance_sync(chassis_mode_t mode, float distance_mm) {
    if (distance_mm <= 0.1f) return ESP_OK;
    uint32_t cruise_hz;
    uint32_t target_steps;
    esp_err_t ret = start_distance_tracking(mode, distance_mm, &cruise_hz, &target_steps);
    if (ret != ESP_OK) return ret;

    uint64_t expected_ms = (target_steps * 1000ULL) / cruise_hz;
    uint64_t timeout_ms = expected_ms * 4ULL + 5000ULL;
    if (xSemaphoreTake(s_move_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        chassis_abort();
        return ESP_ERR_TIMEOUT;
    }
    return s_distance_cancelled ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t chassis_move_angle_sync(chassis_mode_t mode, float angle_deg) {
    if (angle_deg <= 0.1f) return ESP_OK;
    return chassis_move_distance_sync(mode, (angle_deg * (float)M_PI / 180.0f) * CENTER_TO_WHEEL_MM);
}

esp_err_t chassis_move_straight_sync(chassis_mode_t mode, float distance_mm, float target_yaw_deg) {
    if (distance_mm <= 0.1f) return ESP_OK;
    uint32_t base_hz;
    uint32_t target_steps;
    esp_err_t start_ret = start_distance_tracking(mode, distance_mm, &base_hz, &target_steps);
    if (start_ret != ESP_OK) return start_ret;
    uint8_t en[CHASSIS_MOTOR_NUM];
    uint8_t dir[CHASSIS_MOTOR_NUM];
    if (get_mode_params(mode, &base_hz, en, dir) != ESP_OK) {
        chassis_abort();
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t expected_ms = (target_steps * 1000ULL) / base_hz;
    uint64_t timeout_ms = expected_ms * 4ULL + 5000ULL;
    int64_t start_time_us = esp_timer_get_time();
    as201_data_t d;
    chassis_pid_t straight_yaw_pid;
    chassis_straight_pid_init(&straight_yaw_pid);
    bool straight_pid_has_previous_error = false;

    while (1) {
        if (g_stop_requested) { chassis_abort(); return ESP_FAIL; }
        if (!s_distance_tracking) {
            return s_distance_cancelled ? ESP_ERR_INVALID_STATE : ESP_OK;
        }
        uint64_t elapsed_ms = (uint64_t)((esp_timer_get_time() - start_time_us) / 1000);
        if (elapsed_ms >= timeout_ms) {
            chassis_abort();
            return ESP_ERR_TIMEOUT;
        }

        float error = 0.0f;
        if (!as201_get_latest(&d)) {
            chassis_abort();
            return ESP_ERR_TIMEOUT;
        }
        error = target_yaw_deg - d.yaw;
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        float adjust = 0.0f;
        if (fabsf(error) < straight_yaw_pid.dead_zone_deg) {
            chassis_pid_reset(&straight_yaw_pid);
            straight_pid_has_previous_error = true;
            straight_yaw_pid.prev_error = error;
        } else {
            if (!straight_pid_has_previous_error) {
                straight_yaw_pid.prev_error = error;
                straight_pid_has_previous_error = true;
            }
            adjust = chassis_straight_pid_update(&straight_yaw_pid, error,
                STRAIGHT_YAW_CORRECTION_PERIOD_MS / 1000.0f);
        }

        float direction_sign = (mode == CHASSIS_MODE_BACKWARD) ?
                                STRAIGHT_YAW_BACKWARD_CORRECTION_SIGN : 1.0f;
        adjust *= STRAIGHT_YAW_CORRECTION_SIGN * direction_sign;
        float max_adjust = base_hz * STRAIGHT_YAW_MAX_ADJUST_RATIO;
        if (adjust > max_adjust) adjust = max_adjust;
        if (adjust < -max_adjust) adjust = -max_adjust;

        int32_t hz_14 = base_hz - adjust;
        int32_t hz_23 = base_hz + adjust;
        if (hz_14 < 1) hz_14 = 1;
        if (hz_23 < 1) hz_23 = 1;

        apply_and_run_differential((uint32_t)hz_14, (uint32_t)hz_23, dir);
        vTaskDelay(pdMS_TO_TICKS(STRAIGHT_YAW_CORRECTION_PERIOD_MS));
    }
}

esp_err_t chassis_test_motor(int motor_id, bool forward) {
    if (motor_id < 1 || motor_id > 4) return ESP_ERR_INVALID_ARG;
    chassis_abort();
    int idx = motor_id - 1;
    gpio_set_level(s_dir_gpio[idx], forward ? 1 : 0);
    s_target_speed[idx] = 1000; s_current_speed[idx] = 1000;
    s_next_pulse_us[idx] = esp_timer_get_time(); s_running[idx] = 1;
    return ESP_OK;
}

const char *chassis_mode_name(chassis_mode_t mode) {
    switch (mode) {
    case CHASSIS_MODE_FORWARD: return "FORWARD"; case CHASSIS_MODE_BACKWARD: return "BACKWARD"; case CHASSIS_MODE_STRAFE_LEFT: return "STRAFE_LEFT";
    case CHASSIS_MODE_STRAFE_RIGHT: return "STRAFE_RIGHT"; case CHASSIS_MODE_ROTATE_LEFT: return "ROTATE_LEFT"; case CHASSIS_MODE_ROTATE_RIGHT: return "ROTATE_RIGHT";
    case CHASSIS_MODE_ROTATE_LT: return "ROT_LT"; case CHASSIS_MODE_ROTATE_RT: return "ROT_RT";
    case CHASSIS_MODE_ROTATE_LB: return "ROT_LB"; case CHASSIS_MODE_ROTATE_RB: return "ROT_RB";
    default: return "UNKNOWN";
    }
}