#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 二维码任务识别成功后调用。
void mission_notify_qr_complete(void);

// 抓取控制任务完成当前抓取后调用。
void mission_notify_grab_complete(void);

#ifdef __cplusplus
}
#endif
