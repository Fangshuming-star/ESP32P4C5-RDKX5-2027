#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 接口函数区 ==================== */
void servo_init(void);
void servo_set_angle(int servo_id, float angle_deg);
bool servo_parse_cmd(char *line);

#ifdef __cplusplus
}
#endif