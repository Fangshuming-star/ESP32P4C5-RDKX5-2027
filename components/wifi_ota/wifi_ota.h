/*
 * wifi_ota.h — 可复用的 ESP32-P4 无线下载（OTA）模块
 *
 * 功能：
 *   - 通过板载 ESP32-C5 协处理器（esp_hosted + esp_wifi_remote）联网
 *   - 启动一个 TCP 服务（默认 3333 端口），接收 "ota <固件URL>" 命令
 *     触发 HTTP OTA 下载并写入 OTA 分区、自动重启
 *   - 同时支持普通消息回显（便于调试 TCP 通讯）
 *   - 内置连通性自测：收到 "hello p4" / "ping" / "status" 立刻回包
 *
 * 用法（在你的 app_main 里）：
 *   esp_err_t ret = nvs_flash_init();
 *   // ...（可选的 NVS 错误处理）
 *   wifi_ota_start();   // 连上 Wi-Fi 并启动 OTA 服务，返回后即可做你的业务
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 连接 Wi-Fi（走 C5 协处理器）并启动 TCP OTA 服务。
 *
 * 内部会：
 *   1. 初始化 NVS（若尚未初始化）
 *   2. 读取 NVS 里保存的账号（没有就用 Kconfig 默认）
 *   3. 初始化 netif / event loop，以 STA 模式连接（后台自动重连，不阻塞）
 *   4. 在后台任务里监听 TCP 端口，等待 "ota <url>" 命令
 *
 * @return
 *   - ESP_OK   已开始连接，OTA 服务已在后台运行
 *   - 其他值   初始化失败（可查看串口日志）
 */
esp_err_t wifi_ota_start(void);

/**
 * @brief 无线串口收到普通数据（非 "ota " 命令）时的回调。
 *
 * 注册回调后，收到的数据不再原样回显，而是交给回调处理，
 * 可用于把 PC 发来的指令注入到业务逻辑（例如串口屏事件队列）。
 * 注意 "hello p4" / "ping" / "status" 由内置自测逻辑优先回复，不会进回调。
 */
typedef void (*wifi_ota_rx_cb_t)(const char *data, size_t len);

/** @brief 注册无线串口接收回调（传 NULL 恢复默认回显）。 */
void wifi_ota_set_rx_callback(wifi_ota_rx_cb_t cb);

/**
 * @brief 运行时设置 Wi-Fi 账号：保存到 NVS 并立即重连。
 *
 * 适用于换热点时不想重新编译：串口输入 `wifi <ssid> <password>` 即可。
 *
 * @param ssid    热点名称（1~32 字节，仅 ASCII）
 * @param password 密码（WPA/WPA2 至少 8 位；开放热点传 "" 或 NULL）
 */
esp_err_t wifi_ota_set_credentials(const char *ssid, const char *password);

/** @brief 读取当前使用的 Wi-Fi 账号（密码可传 NULL 不取）。 */
esp_err_t wifi_ota_get_credentials(char *ssid, size_t ssid_len,
                                   char *password, size_t pass_len);

/** @brief 清除 NVS 中保存的账号，恢复 Kconfig 默认值（重启后生效）。 */
esp_err_t wifi_ota_clear_credentials(void);

/** @brief 是否正在接收 PC 推送的固件（推送期间日志镜像暂停）。 */
bool wifi_ota_push_active(void);

/** @brief 是否有 PC 端 TCP 客户端在线。 */
bool wifi_ota_client_connected(void);

/**
 * @brief 向当前 TCP 客户端发送数据（用于把本地显示/日志镜像到无线串口）。
 *
 * 线程安全；无客户端连接时立即返回失败。内部有发送超时，不会长时间阻塞。
 *
 * @return 实际发送字节数；-1 表示无客户端或发送失败。
 */
int wifi_ota_send(const void *data, size_t len);

#ifdef __cplusplus
}
#endif
