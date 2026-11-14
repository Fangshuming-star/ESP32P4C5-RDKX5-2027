#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "as201.h"
#include "motor_chassis.h"
#include "chassis_pid.h"
#include "path_planner.h"
#include "screen.h"
#include "limit_switch.h"
#include "motor_independent.h"
#include "servo.h"
#include "qr_reader.h"
#include "chassis_debug.h"
#include "wireless.h"
#include "mission.h"

static const char *TAG = "MAIN";

/* ============================================================
 * 分区一：主流程全局状态
 *
 * s_start_yaw：
 *   上电后静止采样得到的陀螺仪零位。所有“世界角度 -> 陀螺仪目标”都要减它。
 *   调试：如果车放歪了，重新上电或按屏幕 UAR1 观察 yaw，再重新标定。
 *
 * s_start_yaw_ready：
 *   0/1 标志。1 表示零位已经有效。主要给校准流程判断用。
 *
 * s_cur_x / s_cur_y：
 *   当前世界坐标，单位 mm。初始为启停区中心。
 *   调试：如果寻路起点不对，先检查 g_zoneSel 和 ZONE1/2 坐标。
 *
 * s_mission_task_handle：
 *   任务句柄。非 NULL 表示任务正在跑；屏幕 STOP 后任务自行检查停止标志退出。
 *
 * g_gyro_debug_on：
 *   0/1 开关。1 时 gyro_screen_task 每 200ms 往屏幕发 Y/P/R。
 *   调试：屏幕输入 UAR1 切换。
 * ============================================================ */
static float s_start_yaw = 0.0f;
static bool  s_start_yaw_ready = false;
static float s_cur_x = 0.0f, s_cur_y = 0.0f;
static TaskHandle_t s_mission_task_handle = NULL;
static bool g_gyro_debug_on = false;

#define PATH_COMMAND_SIZE          32
#define PATH_MAX_STRAFE_DIST_MM    300.0f
#define PATH_COMMAND_ACK_TIMEOUT_MS 120000

static QueueHandle_t s_path_command_queue = NULL;
static QueueHandle_t s_path_ack_queue = NULL;
static bool s_path_executor_faulted = false;
static float s_executor_target_yaw = 0.0f;
static bool s_executor_yaw_ready = false;

#define ACTION_EVENT_QR_COMPLETE    (1U << 0)
#define ACTION_EVENT_GRAB_COMPLETE  (1U << 1)

static EventGroupHandle_t s_action_event_group = NULL;

void mission_notify_qr_complete(void)
{
    if (s_action_event_group == NULL) {
        ESP_LOGE(TAG, "二维码完成信号组尚未初始化");
        return;
    }
    xEventGroupSetBits(s_action_event_group, ACTION_EVENT_QR_COMPLETE);
}

void mission_notify_grab_complete(void)
{
    if (s_action_event_group == NULL) {
        ESP_LOGE(TAG, "抓取完成信号组尚未初始化");
        return;
    }
    xEventGroupSetBits(s_action_event_group, ACTION_EVENT_GRAB_COMPLETE);
}

/* ============================================================
 * 分区二：世界角度与陀螺仪角度换算
 *
 * simulator_deg_to_yaw(simulator_deg)：
 *   simulator_deg 使用网页版角度约定；陀螺仪航向需先映射到车体世界角度。
 *   网页与车体世界角度关于 90° 轴镜像，转换关系为 180° - simulator_deg。
 *   HEADING_OFFSET_DEG 是发车方向对应的车体世界角度。
 *   YAW_SIGN=-1 表示陀螺仪 yaw 增大方向与场地角度相反。
 *
 * 调试：
 *   - 车头方向与预期反：改 YAW_SIGN 为 1.0
 *   - 所有角度整体偏：改 HEADING_OFFSET_DEG
 * ============================================================ */
static float simulator_deg_to_yaw(float simulator_deg)
{
    float world_deg = 180.0f - simulator_deg;
    while (world_deg >= 360.0f) world_deg -= 360.0f;
    while (world_deg < 0.0f) world_deg += 360.0f;
    return s_start_yaw + YAW_SIGN * (world_deg - HEADING_OFFSET_DEG);
}

/* current_world_deg()：
 *   把当前陀螺仪 yaw 转回世界角度。
 *   如果陀螺仪没数据，返回 HEADING_OFFSET_DEG，避免除零/NaN。
 */
static float current_world_deg(void)
{
    as201_data_t d;
    if (!as201_get_latest(&d)) return HEADING_OFFSET_DEG;
    return HEADING_OFFSET_DEG + YAW_SIGN * (d.yaw - s_start_yaw);
}

static float yaw_to_simulator_deg(float yaw)
{
    float world_deg = HEADING_OFFSET_DEG + YAW_SIGN * (yaw - s_start_yaw);
    float angle = 180.0f - world_deg;
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float current_simulator_deg(void)
{
    as201_data_t data;
    float angle;
    if (as201_get_latest(&data)) {
        angle = yaw_to_simulator_deg(data.yaw);
    } else if (s_executor_yaw_ready) {
        angle = yaw_to_simulator_deg(s_executor_target_yaw);
    } else {
        angle = 180.0f - current_world_deg();
    }
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static bool sync_executor_yaw_to_actual(void)
{
    as201_data_t data;
    if (!as201_get_latest(&data)) {
        s_executor_yaw_ready = false;
        return false;
    }

    s_executor_target_yaw = data.yaw;
    s_executor_yaw_ready = true;
    return true;
}

static path_point_t get_zone_start_position(void)
{
    float zone_x = g_zoneSel == 1 ? ZONE1_X_MM : ZONE2_X_MM;
    float zone_y = g_zoneSel == 1 ? ZONE1_Y_MM : ZONE2_Y_MM;
    return (path_point_t) {
        .x = zone_x + ZONE_SIZE_MM - g_carW / 2.0f - ZONE_START_MARGIN_MM,
        .y = (g_zoneSel == 1) ?
            zone_y + g_carL / 2.0f + ZONE_START_MARGIN_MM :
            zone_y + ZONE_SIZE_MM - g_carL / 2.0f - ZONE_START_MARGIN_MM
    };
}

static void set_position_to_zone_start(void)
{
    path_point_t start = get_zone_start_position();
    s_cur_x = start.x;
    s_cur_y = start.y;
}

static esp_err_t verify_heading(float target_yaw, chassis_mode_t rotation_mode)
{
    as201_data_t data;
    if (!as201_get_latest(&data)) {
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 航向检查无法读取陀螺仪，停止任务");
        return ESP_FAIL;
    }

    float error = target_yaw - data.yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    ESP_LOGI(TAG, "检查航向: 当前 %.1f°，目标 %.1f°，误差 %.1f°",
             data.yaw, target_yaw, error);

    if (fabsf(error) <= CHASSIS_ROTATE_TIMEOUT_ACCEPT_ERROR_DEG) return ESP_OK;
    if (fabsf(error) > CHASSIS_HEADING_VERIFY_MAX_ERROR_DEG) {
#if CHASSIS_ROTATION_ERROR_ACCUMULATION_ENABLE
        ESP_LOGW(TAG, "航向残差 %.1f° 已保留到后续旋转/差速直行目标，继续任务",
                 error);
        return ESP_OK;
#else
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 航向偏差 %.1f° 超过安全阈值 %.1f°，停止后续任务",
                 error, CHASSIS_HEADING_VERIFY_MAX_ERROR_DEG);
        return ESP_ERR_INVALID_STATE;
#endif
    }

    esp_err_t ret = chassis_rotate_to_yaw_mode(rotation_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
    if (ret != ESP_OK) {
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 角度检查失败/超时，停止后续任务");
    }
    return ret;
}

static esp_err_t rotate_to_heading(float target_yaw)
{
    as201_data_t data;
    if (!as201_get_latest(&data)) {
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 无法读取陀螺仪，不能转到任务点目标航向");
        return ESP_FAIL;
    }

    float error = target_yaw - data.yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    if (fabsf(error) <= CHASSIS_ROTATE_TIMEOUT_ACCEPT_ERROR_DEG) {
        s_executor_target_yaw = data.yaw;
        s_executor_yaw_ready = true;
        ESP_LOGI(TAG, "任务点航向已满足: 当前 %.1f°，目标 %.1f°",
                 data.yaw, target_yaw);
        return ESP_OK;
    }

    s_executor_target_yaw = target_yaw;
    s_executor_yaw_ready = true;
    ESP_LOGI(TAG, "转到任务点目标航向: 当前 %.1f°，目标 %.1f°，误差 %.1f°",
             data.yaw, target_yaw, error);
    esp_err_t ret = chassis_rotate_to_yaw_mode(
        CHASSIS_MODE_ROTATE_LEFT, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
    if (ret == ESP_OK && !sync_executor_yaw_to_actual()) {
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 旋转后无法读取陀螺仪实际航向，停止后续任务");
        return ESP_FAIL;
    }
    if (ret != ESP_OK) {
        chassis_abort();
        g_stop_requested = true;
        ESP_LOGE(TAG, "❌ 无法转到任务点目标航向，停止后续任务");
    }
    return ret;
}

static bool read_current_yaw(float *yaw)
{
    TickType_t start_tick = xTaskGetTickCount();
    as201_data_t data;

    while (!as201_get_latest(&data)) {
        if (g_stop_requested) return false;
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(CHASSIS_ROTATE_TIMEOUT_MS)) {
            chassis_stop();
            g_stop_requested = true;
            ESP_LOGE(TAG, "❌ 等待陀螺仪角度数据超时，停止后续任务");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    *yaw = data.yaw;
    return true;
}

static float normalize_angle_delta(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static bool send_path_command(const char *command)
{
    if (s_path_command_queue == NULL || s_path_ack_queue == NULL) {
        ESP_LOGE(TAG, "路径指令队列尚未初始化");
        return false;
    }
    if (s_path_executor_faulted) {
        ESP_LOGE(TAG, "路径执行器已因确认超时停用");
        return false;
    }
    if (g_stop_requested) return false;

    if (xQueueSend(s_path_command_queue, command, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "路径指令入队失败: %s", command);
        return false;
    }
    ESP_LOGI(TAG, "Planner -> Executor: %s", command);

    esp_err_t result;
    TickType_t ack_timeout = pdMS_TO_TICKS(PATH_COMMAND_ACK_TIMEOUT_MS);
    if (ack_timeout == 0) ack_timeout = 1;
    if (xQueueReceive(s_path_ack_queue, &result, ack_timeout) != pdTRUE) {
        s_path_executor_faulted = true;
        g_stop_requested = true;
        chassis_abort();
        ESP_LOGE(TAG, "等待路径指令确认超时，已停止底盘并停用路径执行器: %s",
                 command);
        return false;
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "路径指令执行失败: %s, err=%s",
                 command, esp_err_to_name(result));
        return false;
    }
    return !g_stop_requested;
}

static bool send_strafe_commands(char direction, float total_distance_mm)
{
    float remaining = total_distance_mm;

    while (remaining > 0.0f) {
        float step_distance = fminf(remaining, PATH_MAX_STRAFE_DIST_MM);
        char command[PATH_COMMAND_SIZE];
        int length = snprintf(command, sizeof(command), "%c:%.3f\n",
                              direction, step_distance);
        if (length < 0 || length >= (int)sizeof(command)) {
            ESP_LOGE(TAG, "平移指令格式化失败，距离 %.1fmm", step_distance);
            return false;
        }
        if (!send_path_command(command)) return false;
        remaining -= step_distance;
    }
    return true;
}

static bool send_move_command(char direction, float distance_mm)
{
    char command[PATH_COMMAND_SIZE];
    int length = snprintf(command, sizeof(command), "%c:%.3f\n",
                          direction, distance_mm);
    if (length < 0 || length >= (int)sizeof(command)) {
        ESP_LOGE(TAG, "移动指令格式化失败，距离 %.1fmm", distance_mm);
        return false;
    }
    return send_path_command(command);
}

static bool send_axis_move(char direction, float distance_mm)
{
    if (distance_mm <= 0.5f) return true;
    if (direction == 'A') direction = 'B';
    else if (direction == 'D') direction = 'A';

    return (direction == 'A' || direction == 'B') ?
        send_strafe_commands(direction, distance_mm) :
        send_move_command(direction, distance_mm);
}

static bool parse_path_command(const char *command, char *opcode, float *value)
{
    if (command == NULL || opcode == NULL || value == NULL ||
        command[0] == '\0' || command[1] != ':') {
        return false;
    }

    char *end = NULL;
    float parsed_value = strtof(command + 2, &end);
    if (end == command + 2 || !isfinite(parsed_value)) return false;
    while (*end != '\0' && isspace((unsigned char)*end)) end++;
    if (*end != '\0' || parsed_value <= 0.0f) return false;

    switch (command[0]) {
    case 'W':
    case 'S':
    case 'A':
    case 'B':
    case 'Q':
    case 'R':
    case 'E':
        break;
    default:
        return false;
    }
    if ((command[0] == 'A' || command[0] == 'B') &&
        parsed_value > PATH_MAX_STRAFE_DIST_MM) {
        return false;
    }
    if ((command[0] == 'Q' || command[0] == 'R' || command[0] == 'E') &&
        parsed_value > 180.0f) {
        return false;
    }

    *opcode = command[0];
    *value = parsed_value;
    return true;
}

static esp_err_t execute_path_command(const char *command)
{
    if (g_stop_requested) return ESP_FAIL;

    char opcode;
    float value;
    if (!parse_path_command(command, &opcode, &value)) {
        ESP_LOGE(TAG, "无效路径指令: %s", command);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_executor_yaw_ready) {
        if (!read_current_yaw(&s_executor_target_yaw)) return ESP_FAIL;
        s_executor_yaw_ready = true;
    }

    if (opcode == 'Q' || opcode == 'R' || opcode == 'E') {
        chassis_mode_t rotation_mode = (opcode == 'Q') ?
            CHASSIS_MODE_ROTATE_LEFT : CHASSIS_MODE_ROTATE_RIGHT;
        if (!read_current_yaw(&s_executor_target_yaw)) return ESP_FAIL;
        s_executor_yaw_ready = true;
        float target_yaw = s_executor_target_yaw +
            chassis_yaw_delta_for_rotation(rotation_mode, value);

        chassis_stop();
        vTaskDelay(pdMS_TO_TICKS(150));
        esp_err_t result = chassis_rotate_to_yaw_mode(
            rotation_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
        chassis_stop();
        if (result == ESP_OK && !sync_executor_yaw_to_actual()) {
            chassis_abort();
            g_stop_requested = true;
            ESP_LOGE(TAG, "❌ 路径旋转后无法读取陀螺仪实际航向，停止后续任务");
            return ESP_FAIL;
        }
        return result;
    }

    chassis_mode_t move_mode;
    switch (opcode) {
    case 'W': move_mode = CHASSIS_MODE_FORWARD; break;
    case 'S': move_mode = CHASSIS_MODE_BACKWARD; break;
    case 'A': move_mode = CHASSIS_MODE_STRAFE_LEFT; break;
    case 'B': move_mode = CHASSIS_MODE_STRAFE_RIGHT; break;
    default: return ESP_ERR_INVALID_ARG;
    }

    chassis_stop();
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_err_t result = chassis_move_straight_sync(
        move_mode, value, s_executor_target_yaw);
    chassis_stop();
    if (result == ESP_OK && !g_stop_requested) {
        result = verify_heading(s_executor_target_yaw, CHASSIS_MODE_ROTATE_LEFT);
    }
    return result;
}

static void path_executor_task(void *arg)
{
    char command[PATH_COMMAND_SIZE];
    while (true) {
        if (xQueueReceive(s_path_command_queue, command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        esp_err_t result = execute_path_command(command);
        if (xQueueSend(s_path_ack_queue, &result, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "发送路径指令确认失败: %s", command);
        }
    }
}

/* ============================================================
 * 分区三：陀螺仪零位标定
 *
 * 作用：
 *   上电后让车静止，采样 1000ms 的 yaw 平均值作为 s_start_yaw。
 *   后续所有闭环旋转都基于这个零位。
 *
 * 参数：
 *   vTaskDelay(500)：等待陀螺仪上电稳定，单位 ms。
 *   t1_us / 1000 < 1000：采样窗口 1000ms。
 *   vTaskDelay(20)：每 20ms 取一帧，约 50 帧求平均。
 *   wait_cnt > 100：最多等 100*50ms=5s，等不到数据就放弃。
 *
 * 调试：
 *   - 标定后 yaw 漂移：检查陀螺仪是否静止、供电是否稳定。
 *   - 标定值异常：打开 AS201_LOG_ENABLE 看原始 yaw。
 * ============================================================ */
static void calibrate_zero_yaw(void)
{
    ESP_LOGI(TAG, "══════ 开始重新标定陀螺仪零位（请保持静止）══════");

    s_start_yaw_ready = false;
    as201_data_t d;
    int wait_cnt = 0;

    /* 等待第一帧有效陀螺仪数据；最多 5 秒 */
    while (!as201_get_latest(&d)) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (g_stop_requested) return;
        if (++wait_cnt > 100) {
            ESP_LOGE(TAG, "❌ 等待陀螺仪数据超时，零位标定失败");
            g_stop_requested = true;
            chassis_abort();
            return;
        }
    }

    /* 上电稳定等待 500ms */
    vTaskDelay(pdMS_TO_TICKS(500));

    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    int n = 0;
    int64_t t1_us = esp_timer_get_time();

    /* 采样窗口 1000ms */
    while ((esp_timer_get_time() - t1_us) / 1000 < ZERO_SAMPLE_WINDOW_MS) {
        if (as201_get_latest(&d)) {
            float yaw_rad = d.yaw * (float)M_PI / 180.0f;
            sin_sum += sinf(yaw_rad);
            cos_sum += cosf(yaw_rad);
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        if (g_stop_requested) return;
    }

    if (n == 0) {
        ESP_LOGE(TAG, "❌ 零位采样期间没有有效陀螺仪数据，标定失败");
        g_stop_requested = true;
        chassis_abort();
        return;
    }
    s_start_yaw = atan2f(sin_sum, cos_sum) * 180.0f / (float)M_PI;
    ESP_LOGI(TAG, "★ 重新标定完成，当前零位: %.2f°", s_start_yaw);
    s_start_yaw_ready = true;
}

/* ============================================================
 * 分区四：陀螺仪屏幕调试任务
 *
 * 作用：
 *   当 g_gyro_debug_on=1 时，每 200ms 向串口屏发送 "Y:.. P:.. R:.."。
 *   用于不接电脑时观察姿态角。
 *
 * 参数：
 *   200ms：刷新周期，太快会刷屏，太慢不好观察。
 *   栈 4096：snprintf + screen_send_str 足够。
 *   优先级 4：低于主任务 5 和陀螺仪 6。
 * ============================================================ */
static void gyro_screen_task(void *arg)
{
    as201_data_t d;
    char msg[64];

    while (1) {
        if (g_gyro_debug_on && as201_get_latest(&d)) {
            snprintf(msg, sizeof(msg), "Y:%.1f P:%.1f R:%.1f", d.yaw, d.pitch, d.roll);
            screen_send_str(msg);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ============================================================
 * 分区五：硬编码动作序列执行
 *
 * 作用：
 *   解析 AUTO_RUN_SEQUENCE_STR，例如 "W:1200 WA:90 W:1200"。
 *   每个 token 格式：操作:数值。
 *
 * 支持操作：
 *   W/S/A/D：差速直行，单位 mm，带陀螺仪纠偏
 *   Q/E    ：原地闭环旋转，单位度
 *   WA/WD/WS/WE：绕角闭环旋转，单位度
 *
 * 调试：
 *   - 某个动作失败会 break，后续不执行；看日志 "❌ 动作 ... 失败"。
 *   - 起步打滑：降低 g_spdMove 或 ACCEL_HZ_PER_STEP。
 *   - 距离不准：核对 WHEEL_RADIUS_MM、MOTOR_STEPS_PER_REV、MOTOR_MICROSTEPS。
 * ============================================================ */
#if AUTO_RUN_SEQUENCE_ENABLE
static void execute_hardcoded_sequence(const char *seq_str)
{
    ESP_LOGI(TAG, "▶ 开始执行硬编码动作序列: %s", seq_str);

    /* strdup 复制一份，因为 strtok 会修改原字符串 */
    char *buf = strdup(seq_str);
    if (!buf) return;

    /* 以空格分割 token */
    char *token = strtok(buf, " ");
    float target_yaw = 0.0f;
    bool target_yaw_ready = false;

    while (token != NULL) {
        if (g_stop_requested) break;

        /* op[4]：最多 3 字符操作符 + '\0'，支持 WA/WD/WS/WE */
        char op[4] = {0};
        float val;

        /* sscanf("%3[^:]:%f")：读到冒号前最多 3 字符，再读浮点 */
        if (sscanf(token, "%3[^:]:%f", op, &val) == 2) {
            for (int i = 0; op[i]; i++) op[i] = toupper(op[i]);
            ESP_LOGI(TAG, "  -> 指令: %s, 数值: %.1f", op, val);

            if (!target_yaw_ready) {
                if (!read_current_yaw(&target_yaw)) break;
                target_yaw_ready = true;
            }

            esp_err_t ret = ESP_OK;
            /* 每个动作前先停稳，避免上一个动作残余速度影响 */
            chassis_stop();
            vTaskDelay(pdMS_TO_TICKS(150));

            chassis_mode_t check_mode = CHASSIS_MODE_ROTATE_LEFT;

            if (strcmp(op, "W") == 0) {
                ret = chassis_move_straight_sync(CHASSIS_MODE_FORWARD, val, target_yaw);
            } else if (strcmp(op, "S") == 0) {
                ret = chassis_move_straight_sync(CHASSIS_MODE_BACKWARD, val, target_yaw);
            } else if (strcmp(op, "A") == 0) {
                ret = chassis_move_straight_sync(CHASSIS_MODE_STRAFE_LEFT, val, target_yaw);
            } else if (strcmp(op, "D") == 0) {
                ret = chassis_move_straight_sync(CHASSIS_MODE_STRAFE_RIGHT, val, target_yaw);
            } else if (strcmp(op, "Q") == 0) {
                target_yaw += chassis_yaw_delta_for_rotation(CHASSIS_MODE_ROTATE_LEFT, val);
                ret = chassis_rotate_to_yaw_deg(target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else if (strcmp(op, "E") == 0) {
                target_yaw += chassis_yaw_delta_for_rotation(CHASSIS_MODE_ROTATE_RIGHT, val);
                ret = chassis_rotate_to_yaw_deg(target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else if (strcmp(op, "WA") == 0) {
                check_mode = CHASSIS_MODE_ROTATE_LT;
                target_yaw += chassis_yaw_delta_for_rotation(check_mode, val);
                ret = chassis_rotate_to_yaw_mode(check_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else if (strcmp(op, "WD") == 0) {
                check_mode = CHASSIS_MODE_ROTATE_RT;
                target_yaw += chassis_yaw_delta_for_rotation(check_mode, val);
                ret = chassis_rotate_to_yaw_mode(check_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else if (strcmp(op, "WS") == 0) {
                check_mode = CHASSIS_MODE_ROTATE_LB;
                target_yaw += chassis_yaw_delta_for_rotation(check_mode, val);
                ret = chassis_rotate_to_yaw_mode(check_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else if (strcmp(op, "WE") == 0) {
                check_mode = CHASSIS_MODE_ROTATE_RB;
                target_yaw += chassis_yaw_delta_for_rotation(check_mode, val);
                ret = chassis_rotate_to_yaw_mode(check_mode, target_yaw, CHASSIS_ROTATE_TIMEOUT_MS);
            } else {
                ESP_LOGW(TAG, "未知指令: %s", op);
                ret = ESP_ERR_INVALID_ARG;
            }

            chassis_stop();
            vTaskDelay(pdMS_TO_TICKS(150));

            if (ret == ESP_OK) {
                ret = verify_heading(target_yaw, check_mode);
            }

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "❌ 动作 %s:%.1f 执行失败/超时！立刻终止后续所有动作！", op, val);
                screen_send_str("C:Action Fail");
                break;
            }
        }
        token = strtok(NULL, " ");
    }

    free(buf);
    chassis_stop();
    ESP_LOGI(TAG, "🏁 硬编码动作执行结束");
}

/* ============================================================
 * 分区六：硬编码任务入口
 *
 * 作用：
 *   先标定零位，再执行 AUTO_RUN_SEQUENCE_STR。
 *   s_mission_task_handle 置 NULL，表示任务结束，屏幕可再次 START。
 * ============================================================ */
static void hardcoded_mission_task(void *arg)
{
    calibrate_zero_yaw();
    if (!g_stop_requested) {
        execute_hardcoded_sequence(AUTO_RUN_SEQUENCE_STR);
    }
    s_mission_task_handle = NULL;
    vTaskDelete(NULL);
}
#endif

/* ============================================================
 * 分区七：执行 A* 路径
 *
 * 输入：
 *   plan->points[0] 是当前点，从 i=1 开始走。
 *
 * 每一步按网页版角度和坐标约定生成 W/S/A/D/Q/E 命令，再映射到 Executor。
 * 单条平移命令不超过 PATH_MAX_STRAFE_DIST_MM。
 *
 * 调试：
 *   - 原地转头超时：调整 CHASSIS_ROTATE_TIMEOUT_MS 或检查 YAW_SIGN。
 *   - 走偏：降低 g_spdMove，或检查陀螺仪零漂。
 *   - 航点太密：增大 GRID_MM 或 simplify_path 已合并直线。
 * ============================================================ */
static bool execute_path(const path_plan_t *plan)
{
#if !CHASSIS_ROTATION_ERROR_ACCUMULATION_ENABLE
    s_executor_yaw_ready = false;
#endif

    for (int i = 1; i < plan->count; i++) {
        if (g_stop_requested) return false;

        float dx = plan->points[i].x - s_cur_x;
        float dy = plan->points[i].y - s_cur_y;
        float dist = fabsf(dx) + fabsf(dy);

        if (dx == 0.0f && dy == 0.0f) continue;

        float current_angle = current_simulator_deg() * (float)M_PI / 180.0f;
        char move_opcode;
        if (dist <= PATH_STRAFE_THRESHOLD_MM) {
            float local_w = -dx * cosf(current_angle) - dy * sinf(current_angle);
            float local_d = -dx * sinf(current_angle) + dy * cosf(current_angle);
            bool moved = true;
            if (fabsf(local_w) > 0.5f) {
                move_opcode = local_w > 0.0f ? 'W' : 'S';
                moved = send_axis_move(move_opcode, fabsf(local_w));
            }
            if (moved && fabsf(local_d) > 0.5f) {
                move_opcode = local_d > 0.0f ? 'D' : 'A';
                moved = send_axis_move(move_opcode, fabsf(local_d));
            }
            if (!moved) {
                ESP_LOGE(TAG, "❌ 航点 %d 的短距离分量移动失败", i);
                g_stop_requested = true;
                chassis_abort();
                return false;
            }
            s_cur_x = plan->points[i].x;
            s_cur_y = plan->points[i].y;
            continue;
        }

        if (fabsf(dx) > 0.5f && fabsf(dy) > 0.5f) {
            ESP_LOGE(TAG, "❌ 航点 %d 非轴对齐，拒绝执行以避免定位误差", i);
            g_stop_requested = true;
            chassis_abort();
            return false;
        }

        float abs_angle = 0.0f;
        if (dx > 0.0f) abs_angle = (float)M_PI;
        else if (dx < 0.0f) abs_angle = 0.0f;
        else if (dy > 0.0f) abs_angle = -(float)M_PI / 2.0f;
        else abs_angle = (float)M_PI / 2.0f;

        float end_angle_s = abs_angle + (float)M_PI;
        if (end_angle_s > (float)M_PI) end_angle_s -= 2.0f * (float)M_PI;
        float diff_w = normalize_angle_delta(
            (abs_angle - current_angle) * 180.0f / (float)M_PI);
        float diff_s = normalize_angle_delta(
            (end_angle_s - current_angle) * 180.0f / (float)M_PI);

        if (fabsf(diff_w) <= fabsf(diff_s)) {
            move_opcode = 'W';
            if (diff_w != 0.0f) {
                char rotation_command[PATH_COMMAND_SIZE];
                char rotation_opcode = diff_w > 0.0f ? 'E' : 'Q';
                int length = snprintf(rotation_command, sizeof(rotation_command),
                                      "%c:%.6g\n", rotation_opcode, fabsf(diff_w));
                if (length < 0 || length >= (int)sizeof(rotation_command) ||
                    !send_path_command(rotation_command)) {
                    ESP_LOGE(TAG, "❌ 转向失败！航点 %d 执行中止！", i);
                    g_stop_requested = true;
                    return false;
                }
            }
        } else {
            move_opcode = 'S';
            if (diff_s != 0.0f) {
                char rotation_command[PATH_COMMAND_SIZE];
                char rotation_opcode = diff_s > 0.0f ? 'E' : 'Q';
                int length = snprintf(rotation_command, sizeof(rotation_command),
                                      "%c:%.6g\n", rotation_opcode, fabsf(diff_s));
                if (length < 0 || length >= (int)sizeof(rotation_command) ||
                    !send_path_command(rotation_command)) {
                    ESP_LOGE(TAG, "❌ 转向失败！航点 %d 执行中止！", i);
                    g_stop_requested = true;
                    return false;
                }
            }
        }

        char planner_opcode = move_opcode;
        if (move_opcode == 'A') move_opcode = 'B';
        else if (move_opcode == 'D') move_opcode = 'A';
#if PATH_LOG_ENABLE
        ESP_LOGI(TAG, " [航点 %d/%d] 目标:(%.0f,%.0f) 距离:%.3fmm 航向:%.2f° 路径指令:%c 底盘指令:%c",
                 i, plan->count - 1,
                 plan->points[i].x, plan->points[i].y,
                 dist, current_angle * 180.0f / (float)M_PI,
                 planner_opcode, move_opcode);
#endif
        bool moved = (move_opcode == 'A' || move_opcode == 'B') ?
            send_strafe_commands(move_opcode, dist) :
            send_move_command(move_opcode, dist);
        if (!moved) {
            ESP_LOGE(TAG, "❌ 航点 %d 移动失败！", i);
            g_stop_requested = true;
            chassis_stop();
            return false;
        }

        s_cur_x = plan->points[i].x;
        s_cur_y = plan->points[i].y;
    }
    return true;
}

/* ============================================================
 * 分区八：任务点动作
 *
 * 扫码/抓取的等待方式由 config.h 中各自的 *_ACTION_WAIT_MODE 开关控制。
 * ============================================================ */
#if SCAN_ACTION_WAIT_MODE == ACTION_WAIT_MODE_COMPLETION || \
    GRAB_ACTION_WAIT_MODE == ACTION_WAIT_MODE_COMPLETION
static bool wait_for_action_signal(EventBits_t event_bit, bool allow_scan_skip)
{
    if (s_action_event_group == NULL) {
        ESP_LOGE(TAG, "动作完成信号组未初始化");
        return false;
    }

    while (!g_stop_requested && !(allow_scan_skip && g_skip_scan)) {
        EventBits_t bits = xEventGroupWaitBits(
            s_action_event_group, event_bit, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
        if ((bits & event_bit) != 0) return true;
    }
    return !g_stop_requested;
}
#endif

#if SCAN_ACTION_WAIT_MODE == ACTION_WAIT_MODE_DELAY || \
    GRAB_ACTION_WAIT_MODE == ACTION_WAIT_MODE_DELAY
static bool wait_action_delay(uint32_t wait_ms, bool allow_scan_skip)
{
    int64_t deadline_us = esp_timer_get_time() + (int64_t)wait_ms * 1000;
    for (;;) {
        if (g_stop_requested || (allow_scan_skip && g_skip_scan)) return !g_stop_requested;
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) return !g_stop_requested;
        uint32_t remaining_ms = (uint32_t)((remaining_us + 999) / 1000);
        uint32_t slice_ms = remaining_ms > 50 ? 50 : remaining_ms;
        TickType_t delay_ticks = pdMS_TO_TICKS(slice_ms);
        if (delay_ticks == 0) delay_ticks = 1;
        vTaskDelay(delay_ticks);
    }
}
#endif

static bool execute_action(action_type_t act)
{
    if (act != ACT_NONE && s_action_event_group == NULL) {
        ESP_LOGE(TAG, "动作完成信号组未初始化");
        return false;
    }

    chassis_stop();

    if (act == ACT_SCAN) {
        ESP_LOGI(TAG, "到达扫码区，等待二维码任务...");
        g_qr_received = false;
        g_skip_scan = false;
        xEventGroupClearBits(s_action_event_group, ACTION_EVENT_QR_COMPLETE);
#if SCAN_ACTION_WAIT_MODE == ACTION_WAIT_MODE_COMPLETION
        ESP_LOGI(TAG, "等待二维码识别完成信号");
        return wait_for_action_signal(ACTION_EVENT_QR_COMPLETE, true);
#else
        ESP_LOGI(TAG, "扫码定时等待 %d ms", SCAN_ACTION_WAIT_MS);
        return wait_action_delay(SCAN_ACTION_WAIT_MS, true);
#endif
    } else if (act == ACT_GRAB) {
        xEventGroupClearBits(s_action_event_group, ACTION_EVENT_GRAB_COMPLETE);
#if GRAB_ACTION_WAIT_MODE == ACTION_WAIT_MODE_COMPLETION
        ESP_LOGI(TAG, "等待抓取控制任务发送完成信号");
        return wait_for_action_signal(ACTION_EVENT_GRAB_COMPLETE, false);
#else
        uint32_t wait_ms = g_grabT > 0 ? (uint32_t)g_grabT * 10U : 0U;
        ESP_LOGI(TAG, "抓取定时等待 %lu ms", (unsigned long)wait_ms);
        bool waited = wait_action_delay(wait_ms, false);
        if (waited) {
            ESP_LOGI(TAG, "抓取定时等待完成，继续下一任务点");
        }
        return waited;
#endif
    }
    return !g_stop_requested;
}

/* ============================================================
 * 分区九：完整任务流程
 *
 * 流程：
 *   1. 标定零位。
 *   2. 从 config 取任务点和序列。
 *   3. 屏幕发 T:... 显示总任务。
 *   4. 对每个任务点：
 *      - 屏幕发 C:当前点、N:下一点。
 *      - A* 寻路。
 *      - execute_path 走路径。
 *      - 到达后转到 heading_deg。
 *      - 执行动作（扫码/抓取）。
 *   5. 可选回启停区。
 *   6. 屏幕发 Mission Done / Aborted。
 *
 * 调试：
 *   - 任务点顺序：改 config.c 的 seq[]。
 *   - 任务点坐标：改 config_get_task_points。
 *   - 不回起点：RETURN_TO_START 改 0。
 * ============================================================ */
static void execute_mission(void)
{
    calibrate_zero_yaw();
    if (g_stop_requested) return;
    s_executor_yaw_ready = false;
    ESP_LOGI(TAG, "从启停区 %d 开始，坐标:(%.0f, %.0f)mm",
             g_zoneSel, s_cur_x, s_cur_y);

    task_point_t tp[TP_COUNT];
    config_get_task_points(tp);

    task_point_id_t seq[12];
    config_get_mission_sequence(seq);

    int valid[12] = { TP_SCAN }, n_valid = 1;
    for (int i = 0; i < 12; i++) {
        if (seq[i] != TP_NONE && seq[i] != TP_SCAN && n_valid < 12) {
            valid[n_valid++] = seq[i];
        }
    }
    if (n_valid == 0) return;

    /* 屏幕显示总任务序列，最多 45 字符 */
    char tbuf[64] = "T:";
    for (int i = 0; i < n_valid; i++) {
        if (strlen(tbuf) + strlen(tp[valid[i]].name) + 2 > 45) break;
        if (i) strncat(tbuf, "+", sizeof(tbuf) - strlen(tbuf) - 1);
        strncat(tbuf, tp[valid[i]].name, sizeof(tbuf) - strlen(tbuf) - 1);
    }
    screen_send_str(tbuf);

    for (int i = 0; i < n_valid; i++) {
        if (g_stop_requested) break;
        task_point_t *target = &tp[valid[i]];

        char cbuf[32];
        snprintf(cbuf, sizeof(cbuf), "C:%s", target->name);
        screen_send_str(cbuf);

        if (i + 1 < n_valid) {
            snprintf(cbuf, sizeof(cbuf), "N:%s", tp[valid[i + 1]].name);
        } else {
            snprintf(cbuf, sizeof(cbuf), "N:%s", RETURN_TO_START ? "RET" : "END");
        }
        screen_send_str(cbuf);

        ESP_LOGI(TAG, "▶ 前往任务点 %s (%.0f, %.0f), 目标航向 %.0f° ...",
                 target->name, target->x_mm, target->y_mm, target->heading_deg);

        float cur_rad = current_simulator_deg() * (float)M_PI / 180.0f;
        path_plan_t plan;
        if (!path_planner_find(s_cur_x, s_cur_y, target->x_mm, target->y_mm, cur_rad, &plan)) {
            screen_send_str("C:No Path");
            g_stop_requested = true;
            chassis_abort();
            break;
        }

        if (!execute_path(&plan)) {
            screen_send_str("C:Path Fail");
            if (!g_stop_requested) {
                g_stop_requested = true;
                chassis_abort();
            }
            break;
        }

        s_cur_x = target->x_mm;
        s_cur_y = target->y_mm;

        chassis_stop();
        vTaskDelay(pdMS_TO_TICKS(150));

        /* 到达后转到任务点要求的世界朝向 */
        float target_yaw = simulator_deg_to_yaw(target->heading_deg);
        if (rotate_to_heading(target_yaw) != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(150));

        if (!execute_action(target->action)) {
            if (!g_stop_requested) {
                ESP_LOGE(TAG, "任务点 %s 的后置动作未完成", target->name);
                g_stop_requested = true;
            }
            break;
        }
        if (!g_stop_requested &&
            verify_heading(target_yaw, CHASSIS_MODE_ROTATE_LEFT) != ESP_OK) {
            break;
        }
    }

    /* 回启停区 */
    if (RETURN_TO_START && !g_stop_requested) {
        screen_send_str("C:Return");
        path_point_t return_point = get_zone_start_position();
        float rx = return_point.x;
        float ry = return_point.y;

        path_plan_t plan;
        if (path_planner_find(s_cur_x, s_cur_y, rx, ry,
                              current_simulator_deg() * (float)M_PI / 180.0f, &plan)) {
            if (!execute_path(&plan)) {
                screen_send_str("C:Path Fail");
                if (!g_stop_requested) {
                    g_stop_requested = true;
                    chassis_abort();
                }
            } else {
                s_cur_x = rx;
                s_cur_y = ry;
            }
        } else {
            screen_send_str("C:No Path");
            g_stop_requested = true;
            chassis_abort();
        }
        if (!g_stop_requested) {
            rotate_to_heading(simulator_deg_to_yaw(RETURN_HEADING_DEG));
        }
    }

    if (g_stop_requested) {
        screen_send_str("C:Aborted");
    } else {
        screen_send_str("C:Mission Done");
        screen_send_str("N:Standby");
    }
}

/* ============================================================
 * 分区十：完整任务任务入口
 * ============================================================ */
static void mission_task(void *arg)
{
    execute_mission();
    s_mission_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================
 * 分区十一：主事件循环
 *
 * 作用：
 *   从屏幕队列收消息，处理 START/STOP/PARAM/POINTS/ENCAL/CAL/DISEN/UAR。
 *
 * 调试：
 *   - 屏幕 START 没反应：检查 screen_get_msg 是否收到 SCREEN_EVT_START。
 *   - CHASSIS_DEBUG_ENABLE=1 时 START 被屏蔽，避免调试任务和任务抢占控制权。
 * ============================================================ */
static void main_task(void *arg)
{
    screen_msg_t msg;

    while (1) {
        /* 100ms 超时，没有消息就继续循环 */
        if (!screen_get_msg(&msg, 100)) continue;

        switch (msg.type) {
        case SCREEN_EVT_POINTS:
            /* 屏幕下发 5 个动态障碍点：vals[p*3+0] 有效标志，+1 X，+2 Y（像素） */
            path_planner_clear_dynamic_obstacles();
            {
                int added_count = 0;
            for (int p = 0; p < 5; p++) {
                int32_t valid = msg.data.points.vals[p * 3];
                int32_t px_x = msg.data.points.vals[p * 3 + 1];
                int32_t px_y = msg.data.points.vals[p * 3 + 2];
                if (valid == 1) {
                    if (px_x > 272 || px_y > 272) {
                        ESP_LOGW(TAG, "屏幕障碍点%d坐标越界，像素坐标=(%" PRId32 ",%" PRId32 ")",
                                 p + 1, px_x, px_y);
                        continue;
                    }
                    float mm_x = (float)px_x * SCREEN_PX_TO_MM;
                    float mm_y = (float)px_y * SCREEN_PX_TO_MM;
                    path_planner_add_dynamic_obstacle(mm_x, mm_y);
                    added_count++;
#if SCREEN_LOG_ENABLE
                    ESP_LOGI(TAG, "屏幕障碍点%d: 像素=(%" PRId32 ",%" PRId32
                             ") 地图=(%.1f,%.1f)mm",
                             p + 1, px_x, px_y, mm_x, mm_y);
#endif
                } else if (valid != 0) {
                    ESP_LOGW(TAG, "屏幕障碍点%d有效标记异常: %" PRId32, p + 1, valid);
                }
            }
#if SCREEN_LOG_ENABLE
            ESP_LOGI(TAG, "障碍物更新完成：收到5个槽位，加入%d个有效障碍点",
                     added_count);
#endif
            }
            break;

        case SCREEN_EVT_PARAM:
            /* 屏幕参数更新：车长/车宽/速度/抓取时间/原料间隙/温区Y/障碍半径/安全半径/启停区 */
            g_carL = msg.data.param.carL;
            g_carW = msg.data.param.carW;
            g_spdMove = msg.data.param.spdMove;
            g_spdRot = msg.data.param.spdRot;
            g_grabT = msg.data.param.grabT;
            g_rawGap = msg.data.param.rawGap;
            g_tempY = msg.data.param.tempY;
            g_obsRad = msg.data.param.obsRad;
            g_safeRad = msg.data.param.safeRad;
            g_zoneSel = msg.data.param.zoneSel;
            path_planner_set_car_size(g_carL, g_carW);
            path_planner_set_obs_radii(g_obsRad, g_safeRad);
            break;

        case SCREEN_EVT_START:
            /* 底盘调试开启时，屏幕 START 不启动任务，避免抢控制 */
            if (CHASSIS_DEBUG_ENABLE) break;
            if (s_path_executor_faulted) {
                ESP_LOGE(TAG, "路径执行器此前确认超时，需重启设备后才能重新运行");
                screen_send_str("C:Executor Error");
                break;
            }

            if (s_mission_task_handle == NULL) {
                g_stop_requested = false;
                set_position_to_zone_start();
#if AUTO_RUN_SEQUENCE_ENABLE
                /* 硬编码动作测试：不寻路，只执行 AUTO_RUN_SEQUENCE_STR */
                xTaskCreate(hardcoded_mission_task, "mission", 8192, NULL, 5, &s_mission_task_handle);
#else
                /* 完整任务：A* 寻路 + 任务点动作 */
                xTaskCreate(mission_task, "mission", 8192, NULL, 5, &s_mission_task_handle);
#endif
            } else {
                /* 任务已存在时再次 START：跳过扫码等待 */
                g_skip_scan = true;
            }
            break;

        case SCREEN_EVT_STOP:
            /* 急停：置全局标志并中止动作；任务退出后自行清空句柄 */
            g_stop_requested = true;
            chassis_abort();
            motor_independent_stop_all();
            chassis_stop();
            break;

        case SCREEN_EVT_ENCAL:
            /* 机械臂带使能校准：保持 EN 低，校准后不释放 */
            if (!ARM_SCREEN_DEBUG_ENABLE) break;
            g_stop_requested = false;
            if (strlen(msg.data.text) == 5) {
                /* "encal" 长度 5：全部轴 */
                motor_calibrate_all_async(true);
            } else {
                if (strchr(msg.data.text, '1')) motor_calibrate_single_async(1, true);
                if (strchr(msg.data.text, '2')) motor_calibrate_single_async(2, true);
                if (strchr(msg.data.text, '3')) motor_calibrate_single_async(3, true);
            }
            break;

        case SCREEN_EVT_CAL:
            /* 机械臂普通校准：校准后按 keep_enabled=false 释放 EN */
            if (!ARM_SCREEN_DEBUG_ENABLE) break;
            g_stop_requested = false;
            if (strlen(msg.data.text) == 3) {
                motor_calibrate_all_async(false);
            } else {
                if (strchr(msg.data.text, '1')) motor_calibrate_single_async(1, false);
                if (strchr(msg.data.text, '2')) motor_calibrate_single_async(2, false);
                if (strchr(msg.data.text, '3')) motor_calibrate_single_async(3, false);
            }
            break;

        case SCREEN_EVT_DISEN:
            /* 机械臂全部失能：EN 拉高，电机不再锁死 */
            if (!ARM_SCREEN_DEBUG_ENABLE) break;
            motor_disable_all();
            break;

        case SCREEN_EVT_UAR:
            /* UAR1：切换陀螺仪屏幕显示 */
            if (strcmp(msg.data.text, "UAR1") == 0) {
                g_gyro_debug_on = !g_gyro_debug_on;
                if (g_gyro_debug_on) screen_send_str("INFO: ON");
                else screen_send_str("INFO: OFF");
            }
            break;

        default:
            break;
        }
    }
}

/* ============================================================
 * 分区十二：app_main 初始化顺序
 *
 * 顺序很重要：
 *   1. 机械臂/限位/舵机：先初始化，避免上电乱动。
 *   2. 底盘：初始化定时器和 GPIO，先使能。
 *   3. 陀螺仪：初始化 UART 并启动接收任务。
 *   4. 路径规划：分配网格内存。
 *   5. 屏幕/二维码：初始化 UART 和接收任务。
 *   6. 标定零位。
 *   7. 设置初始坐标。
 *   8. 创建主任务、陀螺仪屏幕任务、底盘调试任务（可选）。
 *   9. 启动无线调试：后台任务连 Wi-Fi 并监听 TCP 3333（OTA/回显）。
 *
 * 调试：
 *   - 上电电机乱转：先检查 EN 极性和 GPIO 初始化顺序。
 *   - 屏幕没显示：检查 UART2 41/47。
 *   - 陀螺仪无数据：检查 UART1 39/40。
 * ============================================================ */
void app_main(void)
{
    motor_independent_init();   /* 机械臂三轴 + 限位中断 + 脉冲定时器 */
    limit_switch_init();        /* 限位轮询任务，用于日志观察 */
    servo_init();               /* 舵机 LEDC 初始化，A/B 回 90° */

    chassis_init();             /* 底盘 GPIO、脉冲定时器、平滑调速任务 */
    chassis_enable(true);       /* EN 使能；true=锁死 */

    as201_init();               /* 陀螺仪 UART1 */
    as201_start();              /* 陀螺仪后台接收任务 */

    path_planner_init();        /* 分配 A* 网格和节点内存 */
    path_planner_set_car_size(g_carL, g_carW);

    s_path_command_queue = xQueueCreate(1, PATH_COMMAND_SIZE);
    s_path_ack_queue = xQueueCreate(1, sizeof(esp_err_t));
    if (s_path_command_queue == NULL || s_path_ack_queue == NULL) {
        ESP_LOGE(TAG, "路径规划指令队列创建失败");
        abort();
    }
    if (xTaskCreate(path_executor_task, "path_executor", 6144, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "路径执行任务创建失败");
        abort();
    }

    s_action_event_group = xEventGroupCreate();
    if (s_action_event_group == NULL) {
        ESP_LOGE(TAG, "任务动作完成信号组创建失败");
        abort();
    }

    screen_init();              /* 串口屏 UART2 */
    qr_reader_init();           /* 二维码 UART4 */

    calibrate_zero_yaw();       /* 上电静止标定零位 */

    /* 初始坐标：启停区中心 */
    set_position_to_zone_start();

    /* 主事件循环任务：栈 8192，优先级 5 */
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);

    /* 陀螺仪屏幕显示任务：栈 4096，优先级 4 */
    xTaskCreate(gyro_screen_task, "gyro_screen", 4096, NULL, 4, NULL);

    /* 底盘调试控制台：只有 CHASSIS_DEBUG_ENABLE=1 才创建 */
    if (CHASSIS_DEBUG_ENABLE) {
        chassis_debug_init();
    }

    /* 无线调试：后台连 Wi-Fi，不阻塞上面的实时控制任务 */
    wireless_start();

    /* app_main 不能返回，主循环让出 CPU */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}