#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 一、宏与数据结构区 ==================== */
#define PATH_MAX_WAYPOINTS      128
#define PATH_MAX_DYNAMIC_OBS    8

typedef struct { float x, y; } path_point_t;
typedef struct { path_point_t points[PATH_MAX_WAYPOINTS]; int count; } path_plan_t;

/* ==================== 二、接口函数区 ==================== */
void path_planner_init(void);
void path_planner_set_car_size(float length_mm, float width_mm);
void path_planner_set_obs_radii(float obstacle_radius_mm, float safe_radius_mm);
void path_planner_clear_dynamic_obstacles(void);
void path_planner_add_dynamic_obstacle(float x_mm, float y_mm);
int  path_planner_get_dynamic_obstacle_count(void);
bool path_planner_find(float start_x_mm, float start_y_mm, float goal_x_mm,  float goal_y_mm, float start_angle_rad, path_plan_t *out);

#ifdef __cplusplus
}
#endif