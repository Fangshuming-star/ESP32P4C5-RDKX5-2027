#include "path_planner.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "PATH";

/* ==================== 一、场地栅格参数区 ==================== */
#define COLS   ((int)(MAP_SIZE_MM / GRID_MM))
#define ROWS   ((int)(MAP_SIZE_MM / GRID_MM))
#define NODES  (COLS * ROWS)

typedef struct { float x, y, w, h; } rect_t;

// 静态障碍物（加工区等）
static const rect_t STATIC_RECTS[] = {
    {  550,  550, 450, 450 }, { 1400,  550, 450, 450 },
    {  550, 1400, 450, 450 }, { 1400, 1400, 450, 450 },
    {  910, 2250, 580, 150 },
};
#define STATIC_RECT_COUNT (sizeof(STATIC_RECTS) / sizeof(STATIC_RECTS[0]))

/* ==================== 二、全局状态区 ==================== */
static path_point_t s_dyn_obs[PATH_MAX_DYNAMIC_OBS]; 
static int          s_dyn_count = 0;
static uint8_t *s_grid = NULL; 

typedef struct { int x, y; int parent; float g, h, f; uint8_t open; uint8_t closed; } astar_node_t;
static astar_node_t *s_nodes = NULL;

/* ==================== 三、初始化与障碍物管理区 ==================== */
void path_planner_init(void) {
    if (s_grid == NULL) s_grid = heap_caps_malloc(NODES, MALLOC_CAP_8BIT);
    if (s_nodes == NULL) s_nodes = heap_caps_malloc(NODES * sizeof(astar_node_t), MALLOC_CAP_8BIT);
    s_dyn_count = 0;
    ESP_LOGI(TAG, "路径规划器初始化完成");
}
void path_planner_set_car_size(float length_mm, float width_mm) { }
void path_planner_set_obs_radii(float obs_radius_mm, float safe_radius_mm) { }
void path_planner_clear_dynamic_obstacles(void) { s_dyn_count = 0; }
void path_planner_add_dynamic_obstacle(float x_mm, float y_mm) {
    if (s_dyn_count >= PATH_MAX_DYNAMIC_OBS) return;
    s_dyn_obs[s_dyn_count++] = (path_point_t){ x_mm, y_mm };
}
int path_planner_get_dynamic_obstacle_count(void) { return s_dyn_count; }

/* ==================== 四、碰撞检测区 ==================== */
static bool rect_rect_collide(float cx, float cy, const rect_t *r, float angle_rad) {
    float halfX, halfY;
    if (fabsf(cosf(angle_rad)) > 0.5f) { halfX = g_carL / 2.0f; halfY = g_carW / 2.0f; }
    else { halfX = g_carW / 2.0f; halfY = g_carL / 2.0f; }
    return (cx + halfX > r->x && cx - halfX < r->x + r->w && cy + halfY > r->y && cy - halfY < r->y + r->h);
}

static bool rect_circle_collide(float cx, float cy, float ox, float oy, float r, float angle_rad) {
    float halfX, halfY;
    if (fabsf(cosf(angle_rad)) > 0.5f) { halfX = g_carL / 2.0f; halfY = g_carW / 2.0f; }
    else { halfX = g_carW / 2.0f; halfY = g_carL / 2.0f; }
    float closestX = fmaxf(cx - halfX, fminf(ox, cx + halfX));
    float closestY = fmaxf(cy - halfY, fminf(oy, cy + halfY));
    float dx = ox - closestX, dy = oy - closestY;
    return (dx * dx + dy * dy) < (r * r);
}

/* ==================== 五、地图栅格化区 ==================== */
static void build_grid(float angle_rad) {
    if (!s_grid) return;
    float halfX = (fabsf(cosf(angle_rad)) > 0.5f) ? g_carL / 2.0f : g_carW / 2.0f;
    float halfY = (fabsf(cosf(angle_rad)) > 0.5f) ? g_carW / 2.0f : g_carL / 2.0f;
    float raw_y_center = -(RAW_MAT_RADIUS_MM - g_rawGap);
    rect_t temp_rect = { 0.0f, g_tempY - 290.0f, 150.0f, 580.0f };
    for (int gy = 0; gy < ROWS; gy++) {
        for (int gx = 0; gx < COLS; gx++) {
            float cx = gx * GRID_MM + GRID_MM / 2.0f;
            float cy = gy * GRID_MM + GRID_MM / 2.0f;
            uint8_t blocked = 0;
            if (cx - halfX < 0 || cx + halfX > MAP_SIZE_MM || cy - halfY < 0 || cy + halfY > MAP_SIZE_MM) blocked = 1;
            
            if (!blocked) {
                for (int i = 0; i < (int)STATIC_RECT_COUNT; i++) {
                    if (rect_rect_collide(cx, cy, &STATIC_RECTS[i], angle_rad)) { blocked = 1; break; }
                }
            }
            if (!blocked && rect_rect_collide(cx, cy, &temp_rect, angle_rad)) blocked = 1;
            if (!blocked && rect_circle_collide(cx, cy, 1200.0f, raw_y_center, RAW_MAT_RADIUS_MM, angle_rad)) blocked = 1;
            
            if (!blocked) {
                for (int i = 0; i < s_dyn_count; i++) {
                    if (rect_circle_collide(cx, cy, s_dyn_obs[i].x, s_dyn_obs[i].y, (float)g_safeRad, angle_rad)) { blocked = 1; break; }
                }
            }
            s_grid[gy * COLS + gx] = blocked;
        }
    }
}
static int coord_to_idx(int x, int y) { return y * COLS + x; }

/* ==================== 六、A* 寻路算法区 ==================== */
static bool astar_search(int sx, int sy, int gx, int gy, path_plan_t *out) {
    memset(s_nodes, 0, NODES * sizeof(astar_node_t));
    int s_idx = coord_to_idx(sx, sy), g_idx = coord_to_idx(gx, gy);
    s_nodes[s_idx].x = sx; s_nodes[s_idx].y = sy; s_nodes[s_idx].h = (float)(abs(sx - gx) + abs(sy - gy)); s_nodes[s_idx].f = s_nodes[s_idx].h; s_nodes[s_idx].parent = -1; s_nodes[s_idx].open = 1;
    static const int DIRS[4][2] = { {0,1}, {1,0}, {0,-1}, {-1,0} };
    for (;;) {
        int best = -1; float best_f = 1e30f;
        for (int i = 0; i < NODES; i++) {
            if (s_nodes[i].open && s_nodes[i].f < best_f) { best_f = s_nodes[i].f; best = i; }
        }
        if (best < 0) return false;
        if (best == g_idx) {
            int cur = best, count = 0;
            while (cur >= 0 && count < PATH_MAX_WAYPOINTS) {
                out->points[count].x = s_nodes[cur].x * GRID_MM + GRID_MM / 2.0f;
                out->points[count].y = s_nodes[cur].y * GRID_MM + GRID_MM / 2.0f;
                count++; cur = s_nodes[cur].parent;
            }
            for (int i = 0; i < count / 2; i++) { path_point_t tmp = out->points[i]; out->points[i] = out->points[count - 1 - i]; out->points[count - 1 - i] = tmp; }
            out->count = count; return true;
        }
        s_nodes[best].open = 0; s_nodes[best].closed = 1;
        int cx = s_nodes[best].x, cy = s_nodes[best].y;
        for (int d = 0; d < 4; d++) {
            int nx = cx + DIRS[d][0], ny = cy + DIRS[d][1];
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) continue;
            if (s_grid[ny * COLS + nx]) continue;
            int ni = coord_to_idx(nx, ny);
            if (s_nodes[ni].closed) continue;
            float tentative_g = s_nodes[best].g + 1.0f;
            if (!s_nodes[ni].open || tentative_g < s_nodes[ni].g) {
                s_nodes[ni].x = nx; s_nodes[ni].y = ny; s_nodes[ni].parent = best;
                s_nodes[ni].g = tentative_g; s_nodes[ni].h = (float)(abs(nx - gx) + abs(ny - gy)); s_nodes[ni].f = tentative_g + s_nodes[ni].h; s_nodes[ni].open = 1;
            }
        }
    }
}

/* ==================== 七、路径后处理区 ==================== */
static void simplify_path(path_plan_t *path) {
    if (path->count <= 2) return;
    static path_plan_t tmp; tmp.count = 0; tmp.points[tmp.count++] = path->points[0];
    for (int i = 1; i < path->count - 1; i++) {
        path_point_t prev = tmp.points[tmp.count - 1], cur = path->points[i], next = path->points[i + 1];
        bool same_x1 = fabsf(prev.x - cur.x) < GRID_MM / 2;
        bool same_x2 = fabsf(cur.x - next.x) < GRID_MM / 2;
        bool same_y1 = fabsf(prev.y - cur.y) < GRID_MM / 2;
        bool same_y2 = fabsf(cur.y - next.y) < GRID_MM / 2;
        if ((same_x1 && same_x2) || (same_y1 && same_y2)) continue;
        tmp.points[tmp.count++] = cur;
    }
    tmp.points[tmp.count++] = path->points[path->count - 1];
    *path = tmp;
}

static bool append_path_point(path_plan_t *path, path_point_t point) {
    if (path->count > 0) {
        path_point_t last = path->points[path->count - 1];
        if (fabsf(last.x - point.x) < 0.1f &&
            fabsf(last.y - point.y) < 0.1f) {
            return true;
        }
    }
    if (path->count >= PATH_MAX_WAYPOINTS) return false;
    path->points[path->count++] = point;
    return true;
}

static bool axis_segment_is_free(path_point_t start, path_point_t end) {
    int sx = (int)floorf(start.x / GRID_MM);
    int sy = (int)floorf(start.y / GRID_MM);
    int ex = (int)floorf(end.x / GRID_MM);
    int ey = (int)floorf(end.y / GRID_MM);
    if (sx < 0 || sx >= COLS || ex < 0 || ex >= COLS ||
        sy < 0 || sy >= ROWS || ey < 0 || ey >= ROWS) {
        return false;
    }

    if (sy == ey) {
        int step = sx <= ex ? 1 : -1;
        for (int x = sx; ; x += step) {
            if (s_grid[sy * COLS + x]) return false;
            if (x == ex) break;
        }
        return true;
    }
    if (sx == ex) {
        int step = sy <= ey ? 1 : -1;
        for (int y = sy; ; y += step) {
            if (s_grid[y * COLS + sx]) return false;
            if (y == ey) break;
        }
        return true;
    }
    return false;
}

static bool try_direct_orthogonal_path(float start_x, float start_y,
                                       float goal_x, float goal_y,
                                       path_plan_t *path) {
    path_point_t start = { start_x, start_y };
    path_point_t goal = { goal_x, goal_y };
    path_point_t corners[2] = {
        { goal_x, start_y },
        { start_x, goal_y }
    };

    if (fabsf(start_x - goal_x) < 0.1f ||
        fabsf(start_y - goal_y) < 0.1f) {
        if (!axis_segment_is_free(start, goal)) return false;
        path->count = 0;
        return append_path_point(path, start) &&
               append_path_point(path, goal);
    }

    for (int i = 0; i < 2; i++) {
        if (!axis_segment_is_free(start, corners[i]) ||
            !axis_segment_is_free(corners[i], goal)) {
            continue;
        }
        path->count = 0;
        if (append_path_point(path, start) &&
            append_path_point(path, corners[i]) &&
            append_path_point(path, goal)) {
            return true;
        }
        path->count = 0;
    }
    return false;
}

static bool align_path_endpoints(path_plan_t *path, float start_x, float start_y,
                                 float goal_x, float goal_y) {
    if (path->count <= 0) return false;

    path_point_t original[PATH_MAX_WAYPOINTS];
    int original_count = path->count;
    memcpy(original, path->points, original_count * sizeof(path_point_t));
    path->count = 0;

    path_point_t start = { start_x, start_y };
    path_point_t goal = { goal_x, goal_y };
    if (!append_path_point(path, start)) return false;

    if (original_count == 1) {
        path_point_t corner = { goal_x, start_y };
        return append_path_point(path, corner) &&
               append_path_point(path, goal);
    }

    path_point_t grid_start = original[0];
    path_point_t first_next = original[1];
    path_point_t start_corner = {
        fabsf(first_next.x - grid_start.x) > 0.1f ? grid_start.x : start_x,
        fabsf(first_next.x - grid_start.x) > 0.1f ? start_y : grid_start.y
    };
    if (!append_path_point(path, start_corner) ||
        !append_path_point(path, grid_start)) {
        return false;
    }
    for (int i = 1; i < original_count; i++) {
        if (!append_path_point(path, original[i])) return false;
    }

    path_point_t grid_goal = original[original_count - 1];
    path_point_t previous = original[original_count - 2];
    path_point_t goal_corner = {
        fabsf(grid_goal.x - previous.x) > 0.1f ? grid_goal.x : goal_x,
        fabsf(grid_goal.x - previous.x) > 0.1f ? goal_y : grid_goal.y
    };
    return append_path_point(path, goal_corner) &&
           append_path_point(path, goal);
}

static bool find_nearest_free(int *x, int *y) {
    if (!s_grid || !s_grid[(*y) * COLS + (*x)]) return true;
    for (int r = 1; r < 30; r++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                int nx = *x + dx, ny = *y + dy;
                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS && !s_grid[ny * COLS + nx]) { *x = nx; *y = ny; return true; }
            }
        }
    }
    return false;
}

/* ==================== 八、对外总控函数区 ==================== */
bool path_planner_find(float start_x_mm, float start_y_mm, float goal_x_mm,  float goal_y_mm, float start_angle_rad, path_plan_t *out) {
    if (!out || !s_grid) return false;
    out->count = 0;
    build_grid(start_angle_rad);
    int sx = (int)(start_x_mm / GRID_MM), sy = (int)(start_y_mm / GRID_MM);
    int gx = (int)(goal_x_mm  / GRID_MM), gy = (int)(goal_y_mm  / GRID_MM);
    
    if (sx < 0) { sx = 0; }
    if (sx >= COLS) { sx = COLS - 1; }
    if (sy < 0) { sy = 0; }
    if (sy >= ROWS) { sy = ROWS - 1; }
    if (gx < 0) { gx = 0; }
    if (gx >= COLS) { gx = COLS - 1; }
    if (gy < 0) { gy = 0; }
    if (gy >= ROWS) { gy = ROWS - 1; }

    if (try_direct_orthogonal_path(start_x_mm, start_y_mm,
                                   goal_x_mm, goal_y_mm, out)) {
        return true;
    }
    
    if (!find_nearest_free(&sx, &sy)) return false;
    if (!find_nearest_free(&gx, &gy)) return false;
    if (!astar_search(sx, sy, gx, gy, out)) return false;
    simplify_path(out);
    if (!align_path_endpoints(out, start_x_mm, start_y_mm, goal_x_mm, goal_y_mm)) {
        out->count = 0;
        return false;
    }
    return true;
}