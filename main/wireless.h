/*
 * wireless.h —— 无线调试（Wi-Fi + TCP + OTA）项目入口
 *
 * 底层实现放在 components/wifi_ota（esp_hosted + esp_wifi_remote），
 * 这里做三件事：
 *   1. 后台任务里启动 Wi-Fi，不阻塞 app_main 的电机/传感器初始化；
 *   2. 记录并对外提供本机 IP，方便串口日志或屏幕显示；
 *   3. 提供连接状态查询，业务代码可按需使用。
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动无线调试（Wi-Fi 连接 + TCP OTA/回显服务 + 串口日志镜像）。
 *
 * 立即返回，实际连接过程在后台任务中完成，因此不会拖慢 app_main。
 * 重复调用安全。
 *
 * 启动后，本机 USB 串口上打印的 ESP_LOG 日志会同步推送到 TCP 客户端，
 * 用任意 TCP 工具连上 3333 端口即可像串口监视器一样看日志。
 *
 * @return ESP_OK 已启动（或已在运行）；其他值为任务创建失败。
 */
esp_err_t wireless_start(void);

/** @brief Wi-Fi 是否已连接并拿到 IP。 */
bool wireless_is_connected(void);

/**
 * @brief 获取本机 IP 字符串，例如 "192.168.1.23"。
 *
 * @param buf     输出缓冲区
 * @param buf_len 缓冲区长度（建议 >= 16）
 * @return true 已连接且 buf 中写入了 IP；false 尚未连接。
 */
bool wireless_get_ip(char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
