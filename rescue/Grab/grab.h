#ifndef RESCUE_GRAB_H
#define RESCUE_GRAB_H

#include "common.h"
#include "cmsis_os.h"
#include "maixcam.h"
#include "motor.h"
#include "rudder_control.h"

/* ===== 调参区：下面这些图像坐标阈值和超时时间需要在实车上实测后调整 ===== */
#define GRAB_TASK_PERIOD_MS             10U     // 任务周期（毫秒）
#define GRAB_VISION_TIMEOUT_MS          150U    // 视觉数据超过此时间未更新就视为失效
#define GRAB_IMAGE_CENTER_X             120U    // 期望目标位于画面横向中心的像素坐标
#define GRAB_APPROACH_Y                 150U    // 接近目标时期望的纵向坐标（值越大表示越近）
#define GRAB_TARGET_NEAR_Y              178U    // 目标 y 超过此值，认为已经贴得足够近
#define GRAB_ZONE_APPROACH_Y            150U    // 接近投放区时期望的纵向坐标
#define GRAB_ZONE_NEAR_Y                170U    // 投放区 y 超过此值，认为已经到达投放位置
#define GRAB_LOST_TARGET_TICKS          20U     // 目标连续丢失多少个周期后放弃（20×10ms=200ms）
#define GRAB_ZONE_SEARCH_TIMEOUT_MS     15000U  // 长时间找不到投放区时的超时时间
#define GRAB_PUSH_DURATION_MS           700U    // 前推投放动作的持续时间
#define GRAB_VERIFY_FRAMES              5U      // 需要连续确认的视觉帧数
#define GRAB_VERIFY_TIMEOUT_MS          1500U   // 等待投放验证结果的最长时间

/* 队伍默认值：1=蓝队，2=红队；填 0 表示不预设，等 Grab_SetTeam() 指定 */
#define GRAB_TEAM_DEFAULT               0U

#define grab_size                       768U    // 任务堆栈大小（字节）
#define grab_task_priority              5U      // 任务优先级
extern uint16_t rudder_cur_ccr[3];
extern uint16_t ccr_2_max;
extern uint16_t ccr_2_min;
extern TaskHandle_t rescue_grab_handler;
void rescue_grab_task(void *pvParameters);
void rescue_grab_task_create(void);

/* 设置队伍颜色：1=蓝，2=红。需要在主控/用户选定队伍后调用 */
void Grab_SetTeam(uint8_t team);
/* 读取当前已设置的队伍（0 表示尚未设置） */
uint8_t Grab_GetTeam(void);

#endif
