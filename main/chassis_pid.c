#include "chassis_pid.h"
#include "config.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

_Static_assert(pdMS_TO_TICKS(PID_CTRL_PERIOD_MS) > 0,
               "PID_CTRL_PERIOD_MS is below the FreeRTOS tick resolution");

static const char *TAG = "PID";

static float normalize_deg180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static esp_err_t handle_rotation_timeout(float target_yaw_deg) {
    chassis_stop();

    as201_data_t latest;
    if (as201_get_latest(&latest)) {
        float error = normalize_deg180(target_yaw_deg - latest.yaw);
        if (fabsf(error) <= CHASSIS_ROTATE_TIMEOUT_ACCEPT_ERROR_DEG) {
            ESP_LOGW(TAG, "旋转达到时间上限，保留 %.2f° 残差并继续后续动作", error);
            return ESP_OK;
        }

        ESP_LOGE(TAG, "❌ 转向超时！当前:%.1f°, 目标:%.1f°, 误差:%.2f° (允许:%.2f°)",
                 latest.yaw, target_yaw_deg, error,
                 CHASSIS_ROTATE_TIMEOUT_ACCEPT_ERROR_DEG);
    } else {
        ESP_LOGE(TAG, "❌ 转向超时且无法读取陀螺仪角度");
    }

    return ESP_ERR_TIMEOUT;
}

/* PID-A：原地旋转 PID，使用 PID_* 配置。 */
void chassis_pid_init(chassis_pid_t *pid) {
    pid->kp = PID_KP_DEFAULT; pid->ki = PID_KI_DEFAULT; pid->kd = PID_KD_DEFAULT;
    pid->integral_max = PID_INTEGRAL_MAX; pid->output_max = PID_OUTPUT_MAX;
    pid->dead_zone_deg = PID_DEAD_ZONE_DEG;
    chassis_pid_reset(pid);
}

/* PID-B：直线差速 PID，使用 STRAIGHT_YAW_PID_* 配置。 */
void chassis_straight_pid_init(chassis_pid_t *pid) {
    if (pid == NULL) return;
    pid->kp = STRAIGHT_YAW_PID_KP;
    pid->ki = STRAIGHT_YAW_PID_KI;
    pid->kd = STRAIGHT_YAW_PID_KD;
    pid->integral_max = STRAIGHT_YAW_PID_INTEGRAL_MAX;
    pid->output_max = STRAIGHT_YAW_PID_OUTPUT_MAX_HZ;
    pid->dead_zone_deg = STRAIGHT_YAW_CORRECTION_DEAD_ZONE_DEG;
    chassis_pid_reset(pid);
}

void chassis_pid_reset(chassis_pid_t *pid) {
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->filtered_derivative = 0.0f;
}

float chassis_pid_update(chassis_pid_t *pid, float error_deg, float dt_sec) {
    if (pid == NULL || dt_sec <= 0.0f) return 0.0f;

    float error_delta = normalize_deg180(error_deg - pid->prev_error);
    float derivative = error_delta / dt_sec;
    const float derivative_filter_alpha = dt_sec / (0.1f + dt_sec);
    pid->filtered_derivative += derivative_filter_alpha *
                                (derivative - pid->filtered_derivative);

    float integral = pid->integral + error_deg * dt_sec;
    if (integral > pid->integral_max) integral = pid->integral_max;
    if (integral < -pid->integral_max) integral = -pid->integral_max;
    pid->integral = integral;
    pid->prev_error = error_deg;

    float output = pid->kp * error_deg +
                   pid->ki * pid->integral +
                   pid->kd * pid->filtered_derivative;
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;
    return output;
}

/* 直线控制使用条件积分，避免输出限幅时积分继续累积造成回摆。 */
float chassis_straight_pid_update(chassis_pid_t *pid, float error_deg, float dt_sec) {
    if (pid == NULL || dt_sec <= 0.0f) return 0.0f;

    float error_delta = normalize_deg180(error_deg - pid->prev_error);
    float derivative = error_delta / dt_sec;
    float filter_tau_sec = STRAIGHT_YAW_PID_DERIVATIVE_FILTER_MS / 1000.0f;
    float filter_alpha = dt_sec / (filter_tau_sec + dt_sec);
    pid->filtered_derivative += filter_alpha *
                                (derivative - pid->filtered_derivative);

    float candidate_integral = pid->integral + error_deg * dt_sec;
    if (candidate_integral > pid->integral_max) candidate_integral = pid->integral_max;
    if (candidate_integral < -pid->integral_max) candidate_integral = -pid->integral_max;

    float proportional_derivative = pid->kp * error_deg +
                                    pid->kd * pid->filtered_derivative;
    float candidate_output = proportional_derivative +
                             pid->ki * candidate_integral;
    bool drives_further_into_saturation =
        (candidate_output > pid->output_max && error_deg > 0.0f) ||
        (candidate_output < -pid->output_max && error_deg < 0.0f);
    if (!drives_further_into_saturation) pid->integral = candidate_integral;

    pid->prev_error = error_deg;
    float output = proportional_derivative + pid->ki * pid->integral;
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;
    return output;
}

static bool is_pivot_mode(chassis_mode_t mode) {
    return mode == CHASSIS_MODE_ROTATE_LT || mode == CHASSIS_MODE_ROTATE_RT ||
           mode == CHASSIS_MODE_ROTATE_LB || mode == CHASSIS_MODE_ROTATE_RB;
}

static float mode_world_rotation_sign(chassis_mode_t mode) {
    switch (mode) {
    case CHASSIS_MODE_ROTATE_LEFT:
    case CHASSIS_MODE_ROTATE_LT:
    case CHASSIS_MODE_ROTATE_RB:
        return 1.0f;
    case CHASSIS_MODE_ROTATE_RIGHT:
    case CHASSIS_MODE_ROTATE_RT:
    case CHASSIS_MODE_ROTATE_LB:
        return -1.0f;
    default:
        return 0.0f;
    }
}

float chassis_yaw_delta_for_rotation(chassis_mode_t rot_mode, float angle_deg) {
    return YAW_SIGN * mode_world_rotation_sign(rot_mode) * angle_deg;
}

esp_err_t chassis_rotate_to_yaw_mode(chassis_mode_t rot_mode, float target_yaw_deg, int timeout_ms) {
    if (timeout_ms <= 0 ||
        (rot_mode != CHASSIS_MODE_ROTATE_LEFT && rot_mode != CHASSIS_MODE_ROTATE_RIGHT &&
         !is_pivot_mode(rot_mode))) {
        return ESP_ERR_INVALID_ARG;
    }

    TickType_t start_tick = xTaskGetTickCount();
    chassis_stop();
    vTaskDelay(pdMS_TO_TICKS(300)); 

    target_yaw_deg = normalize_deg180(target_yaw_deg);

    as201_data_t gyro;
    while (!as201_get_latest(&gyro)) {
        if (g_stop_requested) { chassis_stop(); return ESP_FAIL; }
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(timeout_ms)) {
            return handle_rotation_timeout(target_yaw_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    ESP_LOGI(TAG, "🎯 开始旋转: 目标 %.1f°, 当前 %.1f°", target_yaw_deg, gyro.yaw);
    
    bool is_reached = false;
    int confirm_cnt = 0;
    chassis_pid_t rotation_pid;
    chassis_pid_init(&rotation_pid);
    rotation_pid.prev_error = normalize_deg180(target_yaw_deg - gyro.yaw);
    float initial_error = rotation_pid.prev_error;
    float expected_yaw_sign = YAW_SIGN * mode_world_rotation_sign(rot_mode);
    bool pivot_reverse_active = is_pivot_mode(rot_mode) &&
                                initial_error * expected_yaw_sign < 0.0f;

    while (1) {
        if (g_stop_requested) { chassis_stop(); return ESP_FAIL; }
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(timeout_ms)) {
            return handle_rotation_timeout(target_yaw_deg);
        }
        if (!as201_get_latest(&gyro)) {
            chassis_stop();
            vTaskDelay(pdMS_TO_TICKS(PID_CTRL_PERIOD_MS));
            continue;
        }
        
        float error = normalize_deg180(target_yaw_deg - gyro.yaw);
        float output_hz = chassis_pid_update(
            &rotation_pid, error, PID_CTRL_PERIOD_MS / 1000.0f);

        if (!is_reached) {
            if (fabsf(error) < rotation_pid.dead_zone_deg) { 
                confirm_cnt++;
                if (confirm_cnt >= 5) {
                    chassis_abort();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(timeout_ms)) {
                        return handle_rotation_timeout(target_yaw_deg);
                    }
                    as201_data_t check_d;
                    if (as201_get_latest(&check_d)) {
                        float check_err = normalize_deg180(target_yaw_deg - check_d.yaw);
                        if (fabsf(check_err) < rotation_pid.dead_zone_deg) {
                            is_reached = true;
                            ESP_LOGI(TAG, "✅ 旋转完成且二次确认停稳! 最终: %.1f°", check_d.yaw);
                        } else {
                            confirm_cnt = 0;
                            chassis_pid_reset(&rotation_pid);
                            ESP_LOGW(TAG, "⚠️ 惯性偏移至: %.1f°，继续调整！", check_d.yaw);
                            vTaskDelay(pdMS_TO_TICKS(PID_CTRL_PERIOD_MS));
                            continue;
                        }
                    } else {
                        confirm_cnt = 0;
                        chassis_pid_reset(&rotation_pid);
                        ESP_LOGW(TAG, "⚠️ 无法读取陀螺仪确认数据，继续检查");
                        vTaskDelay(pdMS_TO_TICKS(PID_CTRL_PERIOD_MS));
                        continue;
                    }
                }
            } else {
                confirm_cnt = 0;
            }
        }

        if (is_reached) {
            chassis_stop();
            return ESP_OK; 
        }

        if (confirm_cnt > 0) {
            chassis_abort();
        } else {
            float abs_output_hz = fabsf(output_hz);
            bool pivot_reverse = false;
            if (is_pivot_mode(rot_mode)) {
                pivot_reverse = output_hz * expected_yaw_sign < 0.0f;
                if (pivot_reverse != pivot_reverse_active) {
                    ESP_LOGI(TAG, "绕角PID切换至%s方向：误差 %.1f°，输出 %.1fHz",
                             pivot_reverse ? "反向" : "正向", error, output_hz);
                    pivot_reverse_active = pivot_reverse;
                }
            }

            if (abs_output_hz < PID_ROT_MIN_HZ &&
                !(is_pivot_mode(rot_mode) && error * initial_error < 0.0f)) {
                chassis_abort();
            } else if (is_pivot_mode(rot_mode)) {
                uint32_t speed_hz = (uint32_t)abs_output_hz;
                if (speed_hz < PID_ROT_MIN_HZ) speed_hz = PID_ROT_MIN_HZ;
                if (chassis_move_at_speed_reverse(rot_mode, speed_hz, pivot_reverse) != ESP_OK) {
                    chassis_stop();
                    return ESP_FAIL;
                }
            } else {
                bool want_left = (output_hz > 0.0f);
                if (YAW_SIGN < 0) want_left = !want_left;
                if (INPLACE_ROTATION_DIRECTION_REVERSED) want_left = !want_left;
                chassis_mode_t final_mode = want_left ? CHASSIS_MODE_ROTATE_LEFT : CHASSIS_MODE_ROTATE_RIGHT;
                if (chassis_move_at_speed(final_mode, (uint32_t)abs_output_hz) != ESP_OK) {
                    chassis_stop();
                    return ESP_FAIL;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(PID_CTRL_PERIOD_MS));
    }
}

// 主流程调用时，默认用原地左转模式
esp_err_t chassis_rotate_to_yaw_deg(float target_yaw_deg, int timeout_ms) {
    return chassis_rotate_to_yaw_mode(CHASSIS_MODE_ROTATE_LEFT, target_yaw_deg, timeout_ms);
}

esp_err_t chassis_rotate_by_deg(float delta_deg, int timeout_ms) {
    return chassis_rotate_by_deg_mode(CHASSIS_MODE_ROTATE_LEFT, delta_deg, timeout_ms);
}

esp_err_t chassis_rotate_by_deg_mode(chassis_mode_t rot_mode, float delta_deg, int timeout_ms) {
    if (timeout_ms <= 0) return ESP_ERR_INVALID_ARG;
    as201_data_t gyro;
    TickType_t start_tick = xTaskGetTickCount();
    while (!as201_get_latest(&gyro)) {
        if (g_stop_requested) return ESP_FAIL;
        TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
        if (elapsed_ticks >= pdMS_TO_TICKS(timeout_ms)) {
            chassis_stop();
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    TickType_t elapsed_ticks = xTaskGetTickCount() - start_tick;
    int elapsed_ms = (int)pdTICKS_TO_MS(elapsed_ticks);
    int remaining_ms = timeout_ms - elapsed_ms;
    if (remaining_ms <= 0) {
        chassis_stop();
        return ESP_ERR_TIMEOUT;
    }
    float target_yaw = gyro.yaw + chassis_yaw_delta_for_rotation(rot_mode, delta_deg);
    return chassis_rotate_to_yaw_mode(rot_mode, target_yaw, remaining_ms);
}