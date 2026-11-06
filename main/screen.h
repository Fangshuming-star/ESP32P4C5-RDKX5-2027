#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一、硬件与协议配置区 ==================== */
#define SCREEN_UART_PORT_NUM    2       
#define SCREEN_TX_GPIO          41      
#define SCREEN_RX_GPIO          47      
#define SCREEN_BAUD_RATE        115200

typedef enum {
    SCREEN_EVT_NONE = 0, SCREEN_EVT_POINTS, SCREEN_EVT_START, SCREEN_EVT_STOP, SCREEN_EVT_PARAM, 
    SCREEN_EVT_GOTO_MISSION, SCREEN_EVT_ENTER_MISSION, SCREEN_EVT_LEAVE_MISSION, SCREEN_EVT_SWITCH_VIEW1, SCREEN_EVT_SWITCH_VIEW2,
    SCREEN_EVT_CAL, SCREEN_EVT_ENCAL, SCREEN_EVT_DISEN, SCREEN_EVT_UAR
} screen_event_t;

typedef struct {
    screen_event_t type;
    union {
        struct { int32_t vals[15]; } points;
        struct { int32_t carL, carW, spdMove, spdRot, grabT, rawGap, tempY, obsRad, safeRad, zoneSel; } param;
        char text[32]; 
    } data;
} screen_msg_t;

/* ==================== 二、接口函数区 ==================== */
void screen_init(void);
void screen_send_str(const char *msg);
bool screen_get_msg(screen_msg_t *msg, int timeout_ms);

#ifdef __cplusplus
}
#endif