#include "config.h"

volatile bool g_stop_requested = false;
volatile bool g_qr_received = false;
volatile bool g_skip_scan = false;

int g_carL = 260;               // 车体外廓长度(mm)；按实测值填写，调大寻路更保守。
int g_carW = 240;               // 车体外廓宽度(mm)；按实测值填写，调大寻路更保守。
int g_spdMove = 20;             // 屏幕平移速度档位；走偏、打滑或丢步时减小。
int g_spdRot = 10;              // 屏幕旋转速度档位；过冲、振动或丢步时减小。
int g_grabT = 100;              // 抓取等待时间，单位10ms；抓取动作未完成时增大。
int g_rawGap = 80;              // 原料区间隙(mm)；默认值与网页版一致，可由屏幕参数调整。
int g_tempY = 1200;             // 温区Y坐标(mm)；增大表示温区沿网页版Y轴向下移动。
int g_obsRad = 50;              // 屏幕传入的障碍物半径(mm)；当前未参与寻路障碍检测。
int g_safeRad = 0;              // 动态障碍安全半径(mm)；增大避让距离，但可能减少可行路径。
int g_zoneSel = 1;              // 起始/返回启停区；1=ZONE1，2=ZONE2。

void config_get_task_points(task_point_t tp[TP_COUNT])
{
    // 坐标和航向约定与网页版一致；按实际场地测量调整各任务点位置。
    tp[TP_NONE]   = (task_point_t){    0,                       0,                       0,   ACT_NONE, ""    };
    tp[TP_SCAN]   = (task_point_t){ 2200, 1200,  90,   ACT_SCAN, "SCAN"  };
    tp[TP_RAW]    = (task_point_t){ 1200, TP_RAW_Y, 0, ACT_GRAB, "RAW"   };
    tp[TP_PROC]   = (task_point_t){ 1200, 2095, 180, ACT_GRAB, "PROC"  };
    tp[TP_TEMP]   = (task_point_t){  305, g_tempY, 270, ACT_GRAB, "TEMP"  };
}

void config_get_mission_sequence(task_point_id_t seq[12])
{
    static const task_point_id_t mission_sequence[MISSION_SEQUENCE_LENGTH] =
        DEFAULT_MISSION_SEQUENCE;

    for (int i = 0; i < 12; i++) seq[i] = TP_NONE;
    for (int i = 0; i < MISSION_SEQUENCE_LENGTH; i++) {
        seq[i] = mission_sequence[i];
    }
}