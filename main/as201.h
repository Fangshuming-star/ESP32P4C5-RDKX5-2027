#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一、数据结构区 ==================== */
typedef struct {
    float ax, ay, az, gx, gy, gz, roll, pitch, yaw, mx, my, mz, temp_c;
    uint8_t mag_level;
} as201_data_t;

/* ==================== 二、接口函数区 ==================== */
esp_err_t as201_init(void);       // 初始化 UART 硬件
esp_err_t as201_read(as201_data_t *out, int timeout_ms); // 阻塞读取一帧
esp_err_t as201_start(void);      // 启动后台接收任务
bool as201_get_latest(as201_data_t *out); // 获取最近一次有效数据

#ifdef __cplusplus
}
#endif