/*
 * wifi_ota.h — 可复用的 ESP32-P4 无线下载（OTA）模块
 *
 * 功能：
 *   - 通过板载 ESP32-C5 协处理器（esp_hosted + esp_wifi_remote）联网
 *   - 启动一个 TCP 服务（默认 3333 端口），接收 "ota <固件URL>" 命令
 *     触发 HTTP OTA 下载并写入 OTA 分区、自动重启
 *   - 同时支持普通消息回显（便于调试 TCP 通讯）
 *
 * 用法（在你的 app_main 里）：
 *   esp_err_t ret = nvs_flash_init();
 *   // ...（可选的 NVS 错误处理）
 *   wifi_ota_start();   // 连上 Wi-Fi 并启动 OTA 服务，返回后即可做你的业务
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接 Wi-Fi（走 C5 协处理器）并启动 TCP OTA 服务。
 *
 * 内部会：
 *   1. 初始化 NVS（若尚未初始化）
 *   2. 初始化 netif / event loop
 *   3. 以 STA 模式连接路由器（阻塞直到连上或超时）
 *   4. 在后台任务里监听 TCP 端口，等待 "ota <url>" 命令
 *
 * @return
 *   - ESP_OK           Wi-Fi 已连上，OTA 服务已在后台运行
 *   - ESP_FAIL         Wi-Fi 连接失败（可查看串口日志）
 */
esp_err_t wifi_ota_start(void);

#ifdef __cplusplus
}
#endif
