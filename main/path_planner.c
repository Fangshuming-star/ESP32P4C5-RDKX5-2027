#include "path_planner.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "PATH";

/* ==================== 一、场地栅格参数区 ==================== */
#define COLS   ((int)((MAP_SIZE_MM + GRID_MM - 1.0f) / GRID_MM))
#define ROWS   ((int)((MAP_SIZE_MM + GRID_MM - 1.0f) / GRID_MM))
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
static float s_car_length_mm = 260.0f;
static float s_car_width_mm = 240.0f;
static float s_dynamic_radius_mm = 0.0f;

typedef struct {
    int32_t parent;
    uint32_t g;
    uint32_t f;
    uint32_t heap_pos;
    uint8_t state;
} astar_node_t;
static astar_node_t *s_nodes = NULL;
static uint32_t *s_open_heap = NULL;

/* ==================== 三、初始化与障碍物管理区 ==================== */
void path_planner_init(void) {
    if (s_grid == NULL) s_grid = heap_caps_malloc(NODES, MALLOC_CAP_8BIT);
    if (s_nodes == NULL) s_nodes = heap_caps_malloc(NODES * sizeof(astar_node_t), MALLOC_CAP_8BIT);
    if (s_open_heap == NULL) s_open_heap = heap_caps_malloc(NODES * sizeof(uint32_t), MALLOC_CAP_8BIT);
    s_dyn_count = 0;
    s_car_length_mm = (float)g_carL;
    s_car_width_mm = (float)g_carW;
    s_dynamic_radius_mm = (float)((g_safeRad > g_obsRad) ? g_safeRad : g_obsRad);
    if (!s_grid || !s_nodes || !s_open_heap) {
        ESP_LOGE(TAG, "路径规划内存分配失败 (grid=%p nodes=%p heap=%p)",
                 (void *)s_grid, (void *)s_nodes, (void *)s_open_heap);
        return;
    }
    ESP_LOGI(TAG, "路径规划器初始化完成");
}
void path_planner_set_car_size(float length_mm, float width_mm) {
    if (!isfinite(length_mm) || !isfinite(width_mm) ||
        length_mm <= 0.0f || width_mm <= 0.0f) {
        ESP_LOGE(TAG, "无效车体尺寸: %.1f x %.1f mm", length_mm, width_mm);
        return;
    }
    s_car_length_mm = length_mm;
    s_car_width_mm = width_mm;
}
void path_planner_set_obs_radii(float obs_radius_mm, float safe_radius_mm) {
    if (!isfinite(obs_radius_mm) || !isfinite(safe_radius_mm) ||
        obs_radius_mm < 0.0f || safe_radius_mm < 0.0f) {
        ESP_LOGE(TAG, "无效动态障碍半径: obstacle=%.1f safe=%.1f mm",
                 obs_radius_mm, safe_radius_mm);
        return;
    }
    s_dynamic_radius_mm = fmaxf(obs_radius_mm, safe_radius_mm);
}
void path_planner_clear_dynamic_obstacles(void) { s_dyn_count = 0; }
void path_planner_add_dynamic_obstacle(float x_mm, float y_mm) {
    if (s_dyn_count >= PATH_MAX_DYNAMIC_OBS) return;
    s_dyn_obs[s_dyn_count++] = (path_point_t){ x_mm, y_mm };
}
int path_planner_get_dynamic_obstacle_count(void) { return s_dyn_count; }

/* ==================== 四、地图栅格化区 ==================== */
static void car_aabb_half_extents(float angle_rad, float *half_x, float *half_y) {
    if (fabsf(cosf(angle_rad)) > 0.5f) {
        *half_x = s_car_length_mm / 2.0f;
        *half_y = s_car_width_mm / 2.0f;
    } else {
        *half_x = s_car_width_mm / 2.0f;
        *half_y = s_car_length_mm / 2.0f;
    }
}

static void mark_rect(const rect_t *rect, float half_x, float half_y) {
    float min_x = rect->x - half_x;
    float min_y = rect->y - half_y;
    float max_x = rect->x + rect->w + half_x;
    float max_y = rect->y + rect->h + half_y;
    if (max_x < 0.0f || max_y < 0.0f ||
        min_x > MAP_SIZE_MM || min_y > MAP_SIZE_MM) {
        return;
    }

    int sx = (int)floorf(min_x / GRID_MM);
    int sy = (int)floorf(min_y / GRID_MM);
    int ex = (int)ceilf(max_x / GRID_MM);
    int ey = (int)ceilf(max_y / GRID_MM);
    sx = (sx < 0) ? 0 : sx;
    sy = (sy < 0) ? 0 : sy;
    ex = (ex >= COLS) ? COLS - 1 : ex;
    ey = (ey >= ROWS) ? ROWS - 1 : ey;

    for (int y = sy; y <= ey; y++) {
        for (int x = sx; x <= ex; x++) {
            s_grid[y * COLS + x] = 1;
        }
    }
}

static void mark_circle(float ox, float oy, float radius,
                        float half_x, float half_y) {
    float expanded_radius = fmaxf(half_x, half_y) + radius;
    int sx = (int)floorf((ox - expanded_radius) / GRID_MM);
    int sy = (int)floorf((oy - expanded_radius) / GRID_MM);
    int ex = (int)ceilf((ox + expanded_radius) / GRID_MM);
    int ey = (int)ceilf((oy + expanded_radius) / GRID_MM);
    sx = (sx < 0) ? 0 : sx;
    sy = (sy < 0) ? 0 : sy;
    ex = (ex >= COLS) ? COLS - 1 : ex;
    ey = (ey >= ROWS) ? ROWS - 1 : ey;

    for (int y = sy; y <= ey; y++) {
        for (int x = sx; x <= ex; x++) {
            float cx = x * GRID_MM + GRID_MM / 2.0f;
            float cy = y * GRID_MM + GRID_MM / 2.0f;
            float dx = cx - ox;
            float dy = cy - oy;
            if (dx * dx + dy * dy <= expanded_radius * expanded_radius) {
                s_grid[y * COLS + x] = 1;
            }
        }
    }
}

static void build_grid(float angle_rad) {
    if (!s_grid) return;
    memset(s_grid, 0, NODES);

    float half_x, half_y;
    car_aabb_half_extents(angle_rad, &half_x, &half_y);
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            float cx = x * GRID_MM + GRID_MM / 2.0f;
            float cy = y * GRID_MM + GRID_MM / 2.0f;
            if (cx - half_x < 0.0f || cx + half_x > MAP_SIZE_MM ||
                cy - half_y < 0.0f || cy + half_y > MAP_SIZE_MM) {
                s_grid[y * COLS + x] = 1;
            }
        }
    }

    for (int i = 0; i < (int)STATIC_RECT_COUNT; i++) {
        mark_rect(&STATIC_RECTS[i], half_x, half_y);
    }
    rect_t temp_rect = { 0.0f, g_tempY - 290.0f, 150.0f, 580.0f };
    mark_rect(&temp_rect, half_x, half_y);

    float raw_y_center = -(RAW_MAT_RADIUS_MM - g_rawGap);
    mark_circle(1200.0f, raw_y_center, RAW_MAT_RADIUS_MM, half_x, half_y);
    for (int i = 0; i < s_dyn_count; i++) {
        mark_circle(s_dyn_obs[i].x, s_dyn_obs[i].y,
                    s_dynamic_radius_mm, half_x, half_y);
    }
}
static int coord_to_idx(int x, int y) { return y * COLS + x; }

/* ==================== 五、A* 寻路算法区 ==================== */
enum { NODE_UNSEEN, NODE_OPEN, NODE_CLOSED };

static void heap_swap(uint32_t a, uint32_t b) {
    uint32_t node_a = s_open_heap[a];
    uint32_t node_b = s_open_heap[b];
    s_open_heap[a] = node_b;
    s_open_heap[b] = node_a;
    s_nodes[node_a].heap_pos = b;
    s_nodes[node_b].heap_pos = a;
}

static void heap_sift_up(uint32_t pos) {
    while (pos > 0) {
        uint32_t parent = (pos - 1) / 2;
        if (s_nodes[s_open_heap[parent]].f <= s_nodes[s_open_heap[pos]].f) break;
        heap_swap(parent, pos);
        pos = parent;
    }
}

static void heap_sift_down(uint32_t pos, uint32_t heap_size) {
    for (;;) {
        uint32_t left = 2 * pos + 1;
        uint32_t right = left + 1;
        uint32_t best = pos;
        if (left < heap_size &&
            s_nodes[s_open_heap[left]].f < s_nodes[s_open_heap[best]].f) {
            best = left;
        }
        if (right < heap_size &&
            s_nodes[s_open_heap[right]].f < s_nodes[s_open_heap[best]].f) {
            best = right;
        }
        if (best == pos) break;
        heap_swap(pos, best);
        pos = best;
    }
}

static uint32_t heap_pop(uint32_t *heap_size) {
    uint32_t result = s_open_heap[0];
    (*heap_size)--;
    if (*heap_size > 0) {
        s_open_heap[0] = s_open_heap[*heap_size];
        s_nodes[s_open_heap[0]].heap_pos = 0;
        heap_sift_down(0, *heap_size);
    }
    return result;
}

static uint32_t heuristic(int x, int y, int gx, int gy) {
    return (uint32_t)(abs(x - gx) + abs(y - gy));
}

static bool astar_search(int sx, int sy, int gx, int gy, path_plan_t *out) {
    memset(s_nodes, 0, NODES * sizeof(astar_node_t));
    uint32_t heap_size = 0;
    int start_idx = coord_to_idx(sx, sy);
    int goal_idx = coord_to_idx(gx, gy);
    astar_node_t *start = &s_nodes[start_idx];
    start->parent = -1;
    start->g = 0;
    start->f = heuristic(sx, sy, gx, gy);
    start->state = NODE_OPEN;
    start->heap_pos = heap_size;
    s_open_heap[heap_size++] = (uint32_t)start_idx;

    static const int DIRS[4][2] = { {0, -1}, {1, 0}, {0, 1}, {-1, 0} };
    while (heap_size > 0) {
        uint32_t current_idx = heap_pop(&heap_size);
        astar_node_t *current = &s_nodes[current_idx];
        current->state = NODE_CLOSED;
        if (current_idx == goal_idx) {
            int32_t node_idx = goal_idx;
            out->count = 0;
            while (node_idx >= 0 && out->count < PATH_MAX_WAYPOINTS) {
                int x = node_idx % COLS;
                int y = node_idx / COLS;
                out->points[out->count++] = (path_point_t) {
                    x * GRID_MM + GRID_MM / 2.0f,
                    y * GRID_MM + GRID_MM / 2.0f
                };
                node_idx = s_nodes[node_idx].parent;
            }
            if (node_idx >= 0) {
                out->count = 0;
                return false;
            }
            for (int i = 0; i < out->count / 2; i++) {
                path_point_t tmp = out->points[i];
                out->points[i] = out->points[out->count - 1 - i];
                out->points[out->count - 1 - i] = tmp;
            }
            return true;
        }

        int cx = current_idx % COLS;
        int cy = current_idx / COLS;
        for (int d = 0; d < 4; d++) {
            int nx = cx + DIRS[d][0];
            int ny = cy + DIRS[d][1];
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) continue;
            int next_idx = coord_to_idx(nx, ny);
            astar_node_t *next = &s_nodes[next_idx];
            if (s_grid[next_idx] || next->state == NODE_CLOSED) continue;

            uint32_t tentative_g = current->g + 1;
            if (next->state == NODE_UNSEEN || tentative_g < next->g) {
                next->parent = current_idx;
                next->g = tentative_g;
                next->f = tentative_g + heuristic(nx, ny, gx, gy);
                if (next->state == NODE_UNSEEN) {
                    next->state = NODE_OPEN;
                    next->heap_pos = heap_size;
                    s_open_heap[heap_size++] = (uint32_t)next_idx;
                }
                heap_sift_up(next->heap_pos);
            }
        }
    }
    return false;
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

static bool append_path_point(path_plan_t *path, path_point_t point);
static bool axis_segment_is_free(path_point_t start, path_point_t end);

static bool optimize_path(path_plan_t *path) {
    if (path->count <= 2) return true;

    static path_plan_t source;
    source = *path;
    path->count = 0;
    if (!append_path_point(path, source.points[0])) return false;

    int current = 0;
    while (current < source.count - 1) {
        int furthest = current + 1;
        for (int candidate = source.count - 1; candidate > current + 1; candidate--) {
            path_point_t from = source.points[current];
            path_point_t to = source.points[candidate];
            path_point_t corners[2] = {
                { to.x, from.y },
                { from.x, to.y }
            };
            for (int corner = 0; corner < 2; corner++) {
                if (axis_segment_is_free(from, corners[corner]) &&
                    axis_segment_is_free(corners[corner], to)) {
                    if (!append_path_point(path, corners[corner])) return false;
                    furthest = candidate;
                    break;
                }
            }
            if (furthest == candidate) break;
        }
        if (!append_path_point(path, source.points[furthest])) return false;
        current = furthest;
    }
    return true;
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
    if (!s_grid || *x < 0 || *x >= COLS || *y < 0 || *y >= ROWS) return false;
    if (!s_grid[(*y) * COLS + (*x)]) return true;
    for (int r = 1; r <= 10; r++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                int nx = *x + dx, ny = *y + dy;
                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS &&
                    !s_grid[ny * COLS + nx]) {
                    *x = nx;
                    *y = ny;
                    return true;
                }
            }
        }
    }
    return false;
}

/* ==================== 八、对外总控函数区 ==================== */
bool path_planner_find(float start_x_mm, float start_y_mm, float goal_x_mm,  float goal_y_mm, float start_angle_rad, path_plan_t *out) {
    if (!out) return false;
    out->count = 0;
    if (!s_grid || !s_nodes || !s_open_heap) {
        ESP_LOGE(TAG, "路径规划器未成功初始化");
        return false;
    }
    if (!isfinite(start_x_mm) || !isfinite(start_y_mm) ||
        !isfinite(goal_x_mm) || !isfinite(goal_y_mm) ||
        !isfinite(start_angle_rad)) {
        ESP_LOGE(TAG, "路径规划输入包含非有限数值");
        return false;
    }
    build_grid(start_angle_rad);
    int sx = (int)floorf(start_x_mm / GRID_MM), sy = (int)floorf(start_y_mm / GRID_MM);
    int gx = (int)floorf(goal_x_mm / GRID_MM), gy = (int)floorf(goal_y_mm / GRID_MM);
    
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
    if (!optimize_path(out)) {
        out->count = 0;
        return false;
    }
    if (!align_path_endpoints(out, start_x_mm, start_y_mm, goal_x_mm, goal_y_mm)) {
        out->count = 0;
        return false;
    }
    return true;
}