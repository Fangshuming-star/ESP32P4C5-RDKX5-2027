#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一、硬件配置区 ==================== */
#define LIMIT_SW1_GPIO      4
#define LIMIT_SW2_GPIO      5
#define LIMIT_SW3_GPIO      31
#define LIMIT_SW_ACTIVE_LEVEL 0  // 低电平触发

/* ==================== 二、接口函数区 ==================== */
void limit_switch_init(void);
bool limit_switch_read(int sw_id);

#ifdef __cplusplus
}
#endif