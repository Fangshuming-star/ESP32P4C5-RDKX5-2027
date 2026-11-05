#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一、硬件引脚定义区 ==================== */
#define IND_MOTOR_EN_GPIO       6       
#define IND_MOTOR_1_STEP_GPIO   20      
#define IND_MOTOR_1_DIR_GPIO    7       
#define IND_MOTOR_2_STEP_GPIO   22      
#define IND_MOTOR_2_DIR_GPIO    9       
#define IND_MOTOR_3_STEP_GPIO   23      
#define IND_MOTOR_3_DIR_GPIO    10      

/* ==================== 二、接口函数区 ==================== */
void motor_independent_init(void);
void motor_independent_run(int motor_id, bool run, bool dir_positive);
void motor_independent_stop_all(void);
void motor_calibrate_single_async(int motor_id, bool keep_enabled);
void motor_calibrate_all_async(bool keep_enabled);
void motor_move_to_absolute(float x_mm, float y_mm, float z_deg);
void motor_enable_all(void);
void motor_disable_all(void);

#ifdef __cplusplus
}
#endif