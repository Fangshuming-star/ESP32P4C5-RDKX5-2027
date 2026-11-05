#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_MOTOR_1 = 0, CHASSIS_MOTOR_2, CHASSIS_MOTOR_3, CHASSIS_MOTOR_4, CHASSIS_MOTOR_NUM,
} chassis_motor_t;

typedef enum {
    CHASSIS_MODE_FORWARD = 0,   // 前进
    CHASSIS_MODE_BACKWARD,      // 后退
    CHASSIS_MODE_STRAFE_LEFT,   // 左平移
    CHASSIS_MODE_STRAFE_RIGHT,  // 右平移
    CHASSIS_MODE_ROTATE_LEFT,   // 原地左旋
    CHASSIS_MODE_ROTATE_RIGHT,  // 原地右旋
    CHASSIS_MODE_ROTATE_LT,     // 绕左前轮
    CHASSIS_MODE_ROTATE_RT,     // 绕右前轮
    CHASSIS_MODE_ROTATE_LB,     // 绕左后轮
    CHASSIS_MODE_ROTATE_RB,     // 绕右后轮
    CHASSIS_MODE_NUM,
} chassis_mode_t;

esp_err_t chassis_init(void);
esp_err_t chassis_enable(bool enable);
esp_err_t chassis_stop(void);
esp_err_t chassis_set_speed(uint32_t hz);
esp_err_t chassis_set_dir(chassis_motor_t motor, bool dir_positive);
esp_err_t chassis_move(chassis_mode_t mode);
esp_err_t chassis_move_at_speed(chassis_mode_t mode, uint32_t speed_hz);
esp_err_t chassis_move_at_speed_reverse(chassis_mode_t mode, uint32_t speed_hz, bool reverse);
esp_err_t chassis_move_distance(chassis_mode_t mode, float distance_mm);
esp_err_t chassis_move_distance_sync(chassis_mode_t mode, float distance_mm);

// 核心修复：补充开环角度旋转声明
esp_err_t chassis_move_angle_sync(chassis_mode_t mode, float angle_deg);

esp_err_t chassis_abort(void);
esp_err_t chassis_test_motor(int motor_id, bool forward);
esp_err_t chassis_move_straight_sync(chassis_mode_t mode, float distance_mm, float target_yaw_deg);
const char *chassis_mode_name(chassis_mode_t mode);

#ifdef __cplusplus
}
#endif