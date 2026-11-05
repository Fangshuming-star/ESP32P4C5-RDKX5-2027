#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 * 1. 调试模式与日志
 * 调试模式通常只开一种；串口日志过多、影响实时性时，关闭不需要的日志开关。
 * ============================================================ */

// 1=启用底盘 UART0 调试控制台；启用后屏幕 START 不启动任务。需要从屏幕运行任务时设为 0。
#define CHASSIS_DEBUG_ENABLE    0
// 1=启用机械臂串口调试控制台，可输入 encal/cal 校准；不调试机械臂时设为 0。
#define ARM_SERIAL_DEBUG_ENABLE 0
// 1=允许通过屏幕调试机械臂；需要屏蔽屏幕机械臂控制时设为 0。
#define ARM_SCREEN_DEBUG_ENABLE 1

// 1=START 执行下方硬编码动作；0=执行 A* 寻路任务。
#define AUTO_RUN_SEQUENCE_ENABLE 1
// 硬编码动作：W/S 前进/后退，A/D 左右平移，Q/E 原地转，WA/WD/WS/WE 绕角转。
// 每个动作的格式为“动作:数值”，动作之间用空格分隔；直线单位 mm，旋转单位度。
#define AUTO_RUN_SEQUENCE_STR   "A:100 W:2120 Q:90 W:1070 Q:90 W:1050 Q:90 W:970 E:90 W:1070 A:100"

// 任务等待方式：0=等待固定时间；1=等待对应任务发出完成信号。
#define ACTION_WAIT_MODE_DELAY          1
#define ACTION_WAIT_MODE_COMPLETION     0
// 默认扫码定时等待；抓取按 g_grabT 定时等待。需要等任务完成时改为 COMPLETION。
#define SCAN_ACTION_WAIT_MODE           ACTION_WAIT_MODE_DELAY
#define GRAB_ACTION_WAIT_MODE          ACTION_WAIT_MODE_DELAY
// 定时模式下扫码动作的等待时间（ms）；抓取时间由屏幕参数 g_grabT 设置（单位10ms）。
#define SCAN_ACTION_WAIT_MS             5000
#if (SCAN_ACTION_WAIT_MODE != ACTION_WAIT_MODE_DELAY && \
     SCAN_ACTION_WAIT_MODE != ACTION_WAIT_MODE_COMPLETION) || \
    (GRAB_ACTION_WAIT_MODE != ACTION_WAIT_MODE_DELAY && \
     GRAB_ACTION_WAIT_MODE != ACTION_WAIT_MODE_COMPLETION)
#error "Action wait modes must be ACTION_WAIT_MODE_DELAY or ACTION_WAIT_MODE_COMPLETION"
#endif

// 1=打印 AS201 陀螺仪原始数据；数据量大或串口繁忙时设为 0。
#define AS201_LOG_ENABLE        aADX
// 1=打印 A* 寻路航点；排查路线时开启，正常运行可关闭。
#define PATH_LOG_ENABLE         1
// 1=打印串口屏收发日志；排查屏幕通信时开启，避免刷屏可关闭。
#define SCREEN_LOG_ENABLE       1
// 1=打印限位开关触发日志；排查归零/限位时开启。
#define LIMIT_SW_LOG_ENABLE     0
// 1=打印二维码接收日志；排查二维码通信时开启。
#define QR_LOG_ENABLE           1

/* ============================================================
 * 2. 舵机配置
 * 舵机角度需在机构实际可动范围内；先小幅试动，避免撞击机械限位。
 * ============================================================ */

// 1=启用 1 号舵机（A，夹爪）；未安装或需要禁用时设为 0。
#define SERVO_1_ENABLE          1
// 1=启用 2 号舵机（B，置物盘）；未安装或需要禁用时设为 0。
#define SERVO_2_ENABLE          1
// 1=启用预留 3 号舵机（C）；未连接舵机时保持 0。
#define SERVO_3_ENABLE          0
// 1=启用预留 4 号舵机（D）；未连接舵机时保持 0。
#define SERVO_4_ENABLE          0
// A 舵机夹爪闭合端角度；夹不紧可小幅减小，夹得过紧或顶住可增大。
#define SERVO_A_MIN_ANGLE       70.0f
// A 舵机夹爪张开端角度；开口不足可小幅增大，接近机械限位则减小。
#define SERVO_A_MAX_ANGLE       165.0f
// B 舵机置物盘的最小允许角度；若机构零位不同，按实际安全范围调整。
#define SERVO_B_MIN_ANGLE       0.0f
// B 舵机置物盘的最大允许角度；若到端点撞击，减小到机构安全角度。
#define SERVO_B_MAX_ANGLE       180.0f

/* ============================================================
 * 3. 机械臂步进电机与行程
 * 方向参数只在方向相反时翻转；速度参数以 Hz 为单位，需结合负载逐步调整。
 * ============================================================ */

// 1 号轴归零方向（0/1）；若归零时远离限位开关，将 0 与 1 互换。
#define CAL_DIR_1               0
// 2 号轴归零方向（0/1）；若归零时远离限位开关，将 0 与 1 互换。
#define CAL_DIR_2               0
// 3 号轴归零方向（0/1）；若归零时远离限位开关，将 0 与 1 互换。
#define CAL_DIR_3               1
// 1 号轴使用的限位开关编号（1~3）；改为该轴实际接线对应的开关编号。
#define CAL_SW_1                1
// 2 号轴使用的限位开关编号（1~3）；改为该轴实际接线对应的开关编号。
#define CAL_SW_2                2
// 3 号轴使用的限位开关编号（1~3）；改为该轴实际接线对应的开关编号。
#define CAL_SW_3                3
// 1 号轴归零速度（Hz）；撞限位或惯性过大时减小，归零太慢时可谨慎增大。
#define SPEED_CAL_1             400
// 2 号轴归零速度（Hz）；撞限位或惯性过大时减小，归零太慢时可谨慎增大。
#define SPEED_CAL_2             400
// 3 号轴归零速度（Hz）；撞限位或惯性过大时减小，归零太慢时可谨慎增大。
#define SPEED_CAL_3             400
// 1 号轴正常移动速度（Hz）；运动过慢可增大，丢步、振动或卡滞时减小。
#define SPEED_MOVE_1            1000
// 2 号轴正常移动速度（Hz）；运动过慢可增大，丢步、振动或卡滞时减小。
#define SPEED_MOVE_2            1000
// 3 号轴正常移动速度（Hz）；运动过慢可增大，丢步、振动或卡滞时减小。
#define SPEED_MOVE_3            1000

// 步进电机整步数/转；按电机铭牌设置，常见值为 200，不要用它补偿行程误差。
#define MOTOR_STEPS_PER_REV     200.0f
// 底盘麦克纳姆轮有效半径（mm）；按受载后的有效半径实测填写，距离误差优先用下方标定系数修正。
#define WHEEL_RADIUS_MM         80.0f
// 底盘驱动器细分数；必须与驱动器拨码/配置一致，不能按距离误差随意更改。
#define MOTOR_MICROSTEPS        2
// 底盘距离标定系数；标定路程偏短时增大，偏长时减小，建议依据实测比例修正。
#define CHASSIS_DISTANCE_CALIBRATION_FACTOR 1.9184f
// 车中心到有效旋转轮组的距离（mm）；影响原地旋转对应的轮子行程，按实测几何尺寸调整。
#define CENTER_TO_WHEEL_MM      149.164f

// X 轴齿条驱动齿轮齿数；按实际齿轮规格设置，改大则每转行程增大。
#define X_GEAR_TEETH            22.0f
// X 轴齿轮模数；按齿轮规格设置，改大则齿轮节圆直径及每转行程增大。
#define X_MODULE                1.25f
// X 轴驱动器细分数；按驱动器实际设置，改大提高脉冲分辨率但所需脉冲也增加。
#define X_MICROSTEPS            16.0f
// X 轴软件最大行程（mm）；按机构安全行程设置，减小可留更多限位余量。
#define X_MAX_TRAVEL_MM         178.0f
// Y 轴丝杆导程（mm/转）；按丝杆规格设置，改大则电机每转的直线行程增大。
#define Y_SCREW_LEAD_MM         20.0f
// Y 轴驱动器细分数；按驱动器实际设置，改大提高脉冲分辨率但所需脉冲也增加。
#define Y_MICROSTEPS            1.0f
// Y 轴软件最大行程（mm）；按机构安全行程设置，减小可留更多限位余量。
#define Y_MAX_TRAVEL_MM         164.0f
// Z 轴电机齿数；按实际传动齿轮规格设置，参与电机角度到平台角度的换算。
#define Z_MOTOR_TEETH           25.0f
// Z 轴平台齿数；按实际传动齿轮规格设置，改大则相同电机转角带动平台转角减小。
#define Z_PLATFORM_TEETH        112.0f
// Z 轴驱动器细分数；按驱动器实际设置，改大提高角度分辨率但所需脉冲也增加。
#define Z_MICROSTEPS            16.0f
// Z 轴软件最大转角（度）；按平台安全旋转范围设置，减小可避免碰撞。
#define Z_MAX_TRAVEL_DEG        360.0f

/* ============================================================
 * 4. 底盘运动周期与旋转超时
 * 周期受 FreeRTOS tick 限制；修改周期时需同时观察控制平顺性和 CPU 负载。
 * ============================================================ */

// 闭环旋转/等待陀螺仪的最长等待时间（ms）；旋转超时后仅当残差达标才继续，否则报错停止。
#define CHASSIS_ROTATE_TIMEOUT_MS 15000
// 旋转超时及动作后航向检查允许保留的最大误差（度）；增大更易放行，但后续需继续纠偏。
#define CHASSIS_ROTATE_TIMEOUT_ACCEPT_ERROR_DEG 2.0f
// 底盘速度平滑控制周期（ms）；缩短响应更快但调速更频繁，当前有效范围为 1~10ms。
#define CHASSIS_SPEED_CONTROL_PERIOD_MS 10
// 每个速度控制周期的加速步进（Hz）；起步打滑/冲击时减小，起步过慢时增大。
#define ACCEL_HZ_PER_STEP       2
// 每个速度控制周期的减速步进（Hz）；减速不平顺时增大，停车过冲时减小。
#define DECEL_HZ_PER_STEP       2
#if CHASSIS_SPEED_CONTROL_PERIOD_MS < 1 || CHASSIS_SPEED_CONTROL_PERIOD_MS > 10
#error "CHASSIS_SPEED_CONTROL_PERIOD_MS must be between 1 and 10 ms"
#endif

/* ============================================================
 * 5. 航向坐标、方向与直线差速纠偏
 * 先确认陀螺仪方向与底盘运动方向，再调 PID。方向错误时不要用增益补偿。
 * ============================================================ */

// 陀螺仪 yaw 到底盘世界方向的符号（+1/-1）；方向映射反了时切换符号。
#define YAW_SIGN                (-1.0f)
// 原地旋转 PID 的左右方向反转开关（0/1）；原地转向与目标相反时切换。
#define INPLACE_ROTATION_DIRECTION_REVERSED 1
// 地图世界角度 0 度对应的陀螺仪角度（度）；地图方向整体偏移时按实测零位修正。
#define HEADING_OFFSET_DEG      180.0f

/* 直线差速 PID：作用于 W/S/A/D 行驶时的 1/4 与 2/3 轮组速度差。
 * 调参建议：确认方向系数正确后先调 KP；持续偏差再少量增加 KI；
 * 摆动/变化过快时再增加 KD。一次只改一个参数并记录实测结果。
 */
// 比例增益（Hz/度）；误差越大差速越大，纠偏太弱时增大，左右摆动时减小。
#define STRAIGHT_YAW_PID_KP 50.0f
// 积分增益（Hz/(度·秒)）；用于消除持续小偏差，缓慢偏航可小幅增大，低频摆动时减小。
#define STRAIGHT_YAW_PID_KI 0.1f
// 微分增益（Hz/(度/秒)）；用于抑制误差快速变化，过冲/摆动时小幅增大，噪声放大时减小。
#define STRAIGHT_YAW_PID_KD 10.0f
// 积分状态限幅（度·秒）；增大可积累更多持续纠偏量，但更易产生积分过冲。
#define STRAIGHT_YAW_PID_INTEGRAL_MAX 20.0f
// PID 单侧差速输出上限（Hz）；纠偏输出受限时可适当增大，同时检查轮速上限和稳定性。
#define STRAIGHT_YAW_PID_OUTPUT_MAX_HZ 500.0f
// 微分低通滤波时间常数（ms）；增大可减小噪声但响应变慢，减小响应更快但更敏感。
#define STRAIGHT_YAW_PID_DERIVATIVE_FILTER_MS 10.0f
// 直线纠偏误差死区（度）；小于此值不输出差速，陀螺仪噪声导致抖动时可增大。
#define STRAIGHT_YAW_CORRECTION_DEAD_ZONE_DEG 0.05f
// 单侧差速相对基准轮速的最大比例；纠偏不足可增大，轮组差速过大或偏航恶化时减小。
#define STRAIGHT_YAW_MAX_ADJUST_RATIO 0.5f
// 直线差速总方向系数（+1/-1）；前进时越纠越偏就切换符号，先低速验证。
#define STRAIGHT_YAW_CORRECTION_SIGN 1.0f
// 后退差速方向系数（+1/-1）；仅后退方向纠偏反常时切换，前进不受此参数影响。
#define STRAIGHT_YAW_BACKWARD_CORRECTION_SIGN (-1.0f)
// 直线航向采样/差速更新周期（ms）；当前有效范围为 1~10ms，降低周期会增加更新频率。
#define STRAIGHT_YAW_CORRECTION_PERIOD_MS 10
// 每周期差速输出变化上限（Hz）；差速变化过猛时减小，纠偏响应过慢时谨慎增大。
#define STRAIGHT_YAW_SPEED_STEP_HZ 30
#if STRAIGHT_YAW_CORRECTION_PERIOD_MS < 1 || STRAIGHT_YAW_CORRECTION_PERIOD_MS > 10
#error "STRAIGHT_YAW_CORRECTION_PERIOD_MS must be between 1 and 10 ms"
#endif

/* ============================================================
 * 6. 原地/绕角旋转 PID
 * 先调方向与 KP，再调 KD 减少过冲；KI 通常保持较小，避免积累误差。
 * ============================================================ */

// 旋转比例增益（约 Hz/度）；旋转太慢可增大，临近目标震荡或过冲时减小。
#define PID_KP_DEFAULT          20.0f
// 旋转积分增益；用于补偿持续偏差，通常保持 0，确有稳定偏差再小幅增加。
#define PID_KI_DEFAULT          0.0f
// 旋转微分增益；提前减速/抑制过冲，过冲时可增大，输出抖动或响应迟钝时减小。
#define PID_KD_DEFAULT          5.0f
// 旋转到达死区（度）；增大可减少目标附近反复追转但降低精度，过小可能难以稳定到达。
#define PID_DEAD_ZONE_DEG       0.30f
// 原地旋转 PID 更新周期（ms）；当前有效范围为 1~10ms，需不小于系统 tick 分辨率。
#define PID_CTRL_PERIOD_MS      10
#if PID_CTRL_PERIOD_MS < 1 || PID_CTRL_PERIOD_MS > 10
#error "PID_CTRL_PERIOD_MS must be between 1 and 10 ms"
#endif
// 动作后检查的最大允许航向误差（度）；超过则视为异常停止，增大将放宽安全检查。
#define CHASSIS_HEADING_VERIFY_MAX_ERROR_DEG 10.0f
// 旋转速度参考系数（Hz/度）；当前闭环 PID 路径未使用，修改不会改变闭环旋转速度。
#define PID_ROT_SPEED_HZ_PER_DEG 2.0f
// 绕角旋转速度上限参考值（Hz）；当前闭环 PID 路径未使用，修改不会改变闭环旋转速度。
#define PID_ROT_MAX_HZ          500.0f
// 闭环旋转最低速度（Hz）；与电机驱动支持的最低非零步进频率一致。
#define PID_ROT_MIN_HZ          1.0f
// 原地旋转 PID 积分限幅；增大允许更大的积分补偿，但可能加剧过冲；KI 为 0 时不起作用。
#define PID_INTEGRAL_MAX        500.0f
// 原地旋转 PID 输出限幅（Hz）；增大可提高最大旋转输出，过快/丢步时减小。
#define PID_OUTPUT_MAX          400.0f

/* ============================================================
 * 7. 屏幕速度档位换算
 * 屏幕档位乘以对应系数得到电机脉冲频率；提速前确认电机不丢步。
 * ============================================================ */

// 屏幕平移速度档位到脉冲频率的倍率（Hz/档）；增大所有屏幕平移速度，丢步时减小。
#define SPEED_MOVE_MULT         50
// 屏幕旋转速度档位到脉冲频率的倍率（Hz/档）；增大屏幕旋转速度，过冲/丢步时减小。
#define SPEED_ROT_MULT          150

/* ============================================================
 * 8. 地图、寻路与场地尺寸
 * 地图坐标单位为 mm；地图尺寸、启停区和任务点坐标必须使用同一坐标系。
 * ============================================================ */

// 正方形地图边长（mm）；必须与场地实际尺寸及屏幕坐标映射一致。
#define MAP_SIZE_MM             2400.0f
// A* 网格边长（mm）；减小路线更精细但计算量增加，增大寻路更快但路线更粗。
#define GRID_MM                 40.0f
// 路径段短于等于此距离时优先使用全向平移；更长时选择前进/倒车并仅按直角转向。
#define PATH_STRAFE_THRESHOLD_MM 150.0f
// 地图原料区圆形障碍半径（mm）；按场地实际尺寸设置，增大可留出更多避让空间。
#define RAW_MAT_RADIUS_MM       150.0f
// 启停区边长（mm）；按场地标记尺寸设置，影响启停区中心坐标计算。
#define ZONE_SIZE_MM            300.0f
// 1 号启停区左下角 X 坐标（mm）；用于场地范围定义。
#define ZONE1_X_MM              2100.0f
// 1 号启停区左下角 Y 坐标（mm）；用于场地范围定义。
#define ZONE1_Y_MM              0.0f
// 2 号启停区左下角 X 坐标（mm）；用于场地范围定义。
#define ZONE2_X_MM              2100.0f
// 2 号启停区左下角 Y 坐标（mm）；用于场地范围定义。
#define ZONE2_Y_MM              2100.0f
// 屏幕像素到地图毫米的换算比例；屏幕绘图区尺寸变化时需同步修正分母 272。
#define SCREEN_PX_TO_MM         (MAP_SIZE_MM / 272.0f)

/* ============================================================
 * 9. 陀螺仪零点采样
 * 当前主流程使用 ZERO_SAMPLE_WINDOW_MS 作为采样窗口；标记为预留的参数不影响现有行为。
 * ============================================================ */

// 零点稳定性检测窗口（ms）；当前代码未使用，调整此项不会改变零点采样行为。
#define ZERO_STABLE_WINDOW_MS       5000
// 零点稳定性允许角度波动（度）；当前代码未使用，调整此项不会改变零点采样行为。
#define ZERO_STABLE_THRESHOLD_DEG   0.8f
// 零点采样持续时间（ms）；增大可获得更长采样窗口，减少时采样更快但数据更少。
#define ZERO_SAMPLE_WINDOW_MS       1000
// 零点采样等待超时（ms）；当前代码未使用，调整此项不会改变零点等待时限。
#define ZERO_TIMEOUT_MS             15000

/* ============================================================
 * 10. 运行时屏幕参数与任务配置
 * 下列 g_ 参数由 config.c 提供默认值，部分参数也可由屏幕 PARAM 消息更新。
 * ============================================================ */

// 车长（mm），用于寻路碰撞边界；填写实际外廓长度，设大更保守、设小可能导致贴障碍。
extern int g_carL;
// 车宽（mm），用于寻路碰撞边界；填写实际外廓宽度，设大更保守、设小可能导致贴障碍。
extern int g_carW;
// 屏幕平移速度档位；增大可加快平移，但起步打滑/走偏时应减小。
extern int g_spdMove;
// 屏幕旋转速度档位；增大可加快旋转，但过冲/丢步时应减小。
extern int g_spdRot;
// 抓取后的等待时间，单位为 10ms；动作未完成时增大，确认完成较早时可减小。
extern int g_grabT;
// 原料区间隙（mm），参与原料区位置推算；按场地实测间隙调整。
extern int g_rawGap;
// 温区 Y 坐标（mm）；按地图测量值设置，增大表示温区向地图上方移动。
extern int g_tempY;
// 障碍物半径（mm）；由屏幕参数接收，但当前寻路障碍检测未读取此变量，修改暂不影响避障。
extern int g_obsRad;
// 寻路附加安全半径（mm）；增大可让路线离动态障碍更远，过大可能导致无路可走。
extern int g_safeRad;
// 启停区选择：1=ZONE1，2=ZONE2；需与实际起始区域一致。
extern int g_zoneSel;

// 任务停止请求标志；由运行流程设置，不是用户调参项。
extern volatile bool g_stop_requested;
// 二维码接收状态标志；由二维码通信流程设置，不是用户调参项。
extern volatile bool g_qr_received;
// 跳过扫描请求标志；由屏幕/任务流程设置，不是用户调参项。
extern volatile bool g_skip_scan;

/* ============================================================
 * 11. 任务点、动作类型与任务结束行为
 * 任务点坐标由 config.c 中的 config_get_task_points() 设置。
 * ============================================================ */

typedef enum {
    TP_NONE = 0, TP_SCAN, TP_RAW, TP_PROC1, TP_PROC2, TP_TEMP, TP_COUNT
} task_point_id_t;

typedef enum { ACT_NONE = 0, ACT_SCAN, ACT_GRAB } action_type_t;

typedef struct {
    float x_mm, y_mm;       // 地图坐标（mm）：X 向右、Y 向上。
    float heading_deg;      // 到达任务点时的世界航向（度）。
    action_type_t action;   // 到达后执行的动作类型。
    const char *name;       // 日志/显示使用的任务点名称。
} task_point_t;

// 原料点 Y 坐标推算：原料区间隙 + 半车宽 + 20mm 预留；预留量增大可离边界更远。
#define TP_RAW_Y  (g_rawGap + g_carW / 2.0f + 20.0f)

// 获取各任务点坐标、目标航向与动作定义。
void config_get_task_points(task_point_t tp[TP_COUNT]);
// 获取默认任务点执行顺序；数组容量为 12，未使用项填 TP_NONE。
void config_get_mission_sequence(task_point_id_t seq[12]);

// 1=任务结束后返回所选启停区，0=停在任务终点。
#define RETURN_TO_START         1
// 返回启停区后的目标世界航向（度）；默认使用地图航向零点。
#define RETURN_HEADING_DEG      HEADING_OFFSET_DEG
