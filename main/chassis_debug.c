#include "chassis_debug.h"
#include "config.h"
#include "motor_chassis.h"
#include "chassis_pid.h"  
#include "as201.h"        
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CHASSIS_DEBUG";

static void debug_task(void *arg) {
    char line[64];
    ESP_LOGI(TAG, "======== 底盘调试控制台已启动 ========");
    ESP_LOGI(TAG, " W/S/A/D    : 前进/后退/左移/右移 (带差速纠偏)");
    ESP_LOGI(TAG, " WA/WS/WD/WE: 绕 左前/左后/右前/右后 轮旋转");
    ESP_LOGI(TAG, " WA:90      : 绕左前轮精确旋转 90 度 (闭环角度)");
    ESP_LOGI(TAG, " WA         : 绕左前轮持续旋转 (输入 X 停止)");
    ESP_LOGI(TAG, " Q/E        : 原地左/右转 90度 (闭环)");
    ESP_LOGI(TAG, " P1~P4      : 单独测试 1~4 号轮子");
    ESP_LOGI(TAG, " X/stop     : 紧急停止");

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            line[strcspn(line, "\r\n")] = 0; 
            
            if (strcmp(line, "X") == 0 || strcmp(line, "x") == 0 || strcmp(line, "stop") == 0) {
                chassis_abort();
                ESP_LOGW(TAG, "🛑 底盘急停");
                continue;
            }

            if (line[0] == 'P' || line[0] == 'p') {
                if (line[1] >= '1' && line[1] <= '4') {
                    int id = line[1] - '0';
                    chassis_test_motor(id, true); 
                    ESP_LOGI(TAG, "测试轮 %d 正转", id);
                }
                continue;
            }

            for (int i = 0; line[i]; i++) line[i] = toupper((unsigned char)line[i]);

            if (strncmp(line, "WA", 2) == 0 || strncmp(line, "WS", 2) == 0 ||
                strncmp(line, "WD", 2) == 0 || strncmp(line, "WE", 2) == 0) {
                
                char op2 = line[1];
                chassis_mode_t mode = CHASSIS_MODE_NUM;
                if (op2 == 'A') mode = CHASSIS_MODE_ROTATE_LT;
                else if (op2 == 'S') mode = CHASSIS_MODE_ROTATE_LB;
                else if (op2 == 'D') mode = CHASSIS_MODE_ROTATE_RT;
                else if (op2 == 'E') mode = CHASSIS_MODE_ROTATE_RB;

                float val = 0;
                if (strlen(line) > 3 && (line[2] == ':' || line[2] == ' ')) val = atof(line + 3);
                else if (strlen(line) > 2 && (line[2] >= '0' && line[2] <= '9')) val = atof(line + 2); 

                if (val > 0.1f) {
                    ESP_LOGW(TAG, "【绕角旋转】模式: %s, 角度: %.1f°", chassis_mode_name(mode), val);
                    esp_err_t err = chassis_rotate_by_deg_mode(mode, val, CHASSIS_ROTATE_TIMEOUT_MS);
                    if (err == ESP_OK) ESP_LOGI(TAG, "✅ 闭环绕角旋转完成");
                    else ESP_LOGE(TAG, "❌ 闭环绕角旋转失败/超时");
                } else {
                    chassis_move(mode);
                    ESP_LOGW(TAG, "【绕角旋转】持续模式: %s (输入 X 停止)", chassis_mode_name(mode));
                }
                continue;
            }

            char cmd = line[0];
            if ((cmd == 'Q' || cmd == 'E') && strlen(line) == 1) {
                float delta = (cmd == 'Q') ? 90.0f : -90.0f;
                ESP_LOGW(TAG, "【闭环测试】原地旋转 %.1f°", delta);
                esp_err_t err = chassis_rotate_by_deg(delta, CHASSIS_ROTATE_TIMEOUT_MS);
                if (err == ESP_OK) ESP_LOGI(TAG, "闭环旋转完成！");
                else ESP_LOGE(TAG, "闭环旋转失败/超时！");
                continue;
            }

            chassis_mode_t mode = CHASSIS_MODE_NUM;
            bool is_angle = false;

            switch (cmd) {
                case 'W': mode = CHASSIS_MODE_FORWARD; break;
                case 'S': mode = CHASSIS_MODE_BACKWARD; break;
                case 'A': mode = CHASSIS_MODE_STRAFE_LEFT; break;
                case 'D': mode = CHASSIS_MODE_STRAFE_RIGHT; break;
                case 'Q': mode = CHASSIS_MODE_ROTATE_LEFT; is_angle = true; break;
                case 'E': mode = CHASSIS_MODE_ROTATE_RIGHT; is_angle = true; break;
                default: continue;
            }

            float val = 0;
            if (strlen(line) > 2 && sscanf(line + 2, "%f", &val) == 1) {
                if (is_angle) {
                    chassis_move_angle_sync(mode, val);
                    ESP_LOGI(TAG, "开环原地旋转 %.1f 度完毕", val);
                } else {
                    as201_data_t d;
                    float target_yaw = 0.0f;
                    if (as201_get_latest(&d)) target_yaw = d.yaw;
                    chassis_move_straight_sync(mode, val, target_yaw);
                    ESP_LOGI(TAG, "差速直行 %.1f mm 完毕", val);
                }
            } else {
                chassis_move(mode);
                ESP_LOGI(TAG, "持续运动模式: %s (输入 X 停止)", chassis_mode_name(mode));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void chassis_debug_init(void) {
    xTaskCreate(debug_task, "chassis_debug", 4096, NULL, 4, NULL);
}