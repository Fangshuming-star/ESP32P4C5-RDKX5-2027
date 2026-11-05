#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "as201.h"
#include "motor_chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;       
    float integral;         
    float prev_error;       
    float filtered_derivative;
    float integral_max;     
    float output_max;       
    float dead_zone_deg;    
} chassis_pid_t;

void  chassis_pid_init(chassis_pid_t *pid);
void  chassis_straight_pid_init(chassis_pid_t *pid);
void  chassis_pid_reset(chassis_pid_t *pid);
float chassis_pid_update(chassis_pid_t *pid, float error_deg, float dt_sec);
float chassis_straight_pid_update(chassis_pid_t *pid, float error_deg, float dt_sec);

// 主流程专用：默认原地闭环旋转（传入目标角度和超时时间）
esp_err_t chassis_rotate_to_yaw_deg(float target_yaw_deg, int timeout_ms);

// 调试专用：可指定旋转模式的闭环旋转
esp_err_t chassis_rotate_to_yaw_mode(chassis_mode_t rot_mode, float target_yaw_deg, int timeout_ms);

// 相对角度旋转
esp_err_t chassis_rotate_by_deg(float delta_deg, int timeout_ms);
esp_err_t chassis_rotate_by_deg_mode(chassis_mode_t rot_mode, float delta_deg, int timeout_ms);
float chassis_yaw_delta_for_rotation(chassis_mode_t rot_mode, float angle_deg);

#ifdef __cplusplus
}
#endif