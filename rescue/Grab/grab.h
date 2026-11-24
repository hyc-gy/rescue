#ifndef RESCUE_GRAB_H
#define RESCUE_GRAB_H

#include "maixcam.h"

/* 当前只联调蓝方1号位。序号传给动作执行端，不塞进视觉mode字段。 */
#define GRAB_TEAM_DEFAULT             MAIXCAM_TEAM_DEFAULT
#define GRAB_START_POSITION_DEFAULT   1U
#define GRAB_TASK_PERIOD_MS           10U
#define GRAB_VISION_TIMEOUT_MS        150U
#define GRAB_VERIFY_FRAMES            5U
#define GRAB_EMPTY_CONFIRM_MS         1000U
#define GRAB_BASE_MAX_ANGLE_DEG        30.0f
#define grab_size                     768U  /* FreeRTOS栈深度，单位StackType_t */
#define grab_task_priority            5U

typedef struct {
    uint8_t normal, core, patient, hazard;
} CargoCounts;

typedef enum {
    GRAB_POSE_START = 1,       /* 出发最低头，整车高度<=20cm */
    GRAB_POSE_FIND_OBJECT = 2, /* 抬头找物、接近夹取 */
    GRAB_POSE_CHECK_CARGO = 3, /* 低头，全画面检查夹内物品 */
    GRAB_POSE_FIND_BASE = 4    /* 更抬头寻找己方安全区 */
} GrabCameraPose;

typedef enum {
    GRAB_PREPARING_START,
    GRAB_WAITING_START,
    GRAB_LEAVING_START,
    GRAB_SETTING_OBJECT_POSE,
    GRAB_SEARCHING_OBJECT,
    GRAB_APPROACHING_OBJECT,
    GRAB_SETTING_CONFIRM_POSE,
    GRAB_CONFIRMING_CARGO,
    GRAB_CLEARING_CARGO,
    GRAB_SETTING_BASE_POSE,
    GRAB_RETURNING_TO_BASE,
    GRAB_ADJUSTING_TO_CENTER,
    GRAB_WAITING_DELIVERY
} GrabStage;

typedef enum {
    GRAB_ACTION_STOP = 0,
    GRAB_ACTION_PREPARE_START,   /* 姿态1、闭爪、发车航向初始化，不能发车 */
    GRAB_ACTION_LEAVE_START,    /* 独立启动指令后执行1号位起步、到姿态2 */
    GRAB_ACTION_SET_CAMERA_POSE,
    GRAB_ACTION_SEARCH_OBJECT,  /* 搜索方式/超时由执行端决定 */
    GRAB_ACTION_APPROACH_OBJECT,/* 按当前新目标接近并夹取，到位闭爪后报告完成 */
    GRAB_ACTION_CLEAR_AND_SCATTER,/* 张开->后退释放->闭合->前进撞散 */
    GRAB_ACTION_SEARCH_BASE,
    GRAB_ACTION_FOLLOW_BASE,    /* 跟整区/跟线由navigation.action区分 */
    GRAB_ACTION_HOLD_LAST_SPEED,/* 只维持实际上一速度，不用旧坐标继续算PID */
    GRAB_ACTION_ADJUST_TO_CENTER,/* 根据有符号偏角向中线调整，转回后报告结束 */
    GRAB_ACTION_WAIT_DELIVERY   /* 等待尚未定义的推入/送达适配，不能自动投放 */
} GrabAction;

typedef enum {
    GRAB_FEEDBACK_DONE = 1,
    GRAB_FEEDBACK_BASE_ALIGNED, /* 当前视觉参照已居中，angle_deg为相对己方基地方向 */
    GRAB_FEEDBACK_FAILED        /* 不伪造完成，保留阶段，等待执行端恢复/进一步处理 */
} GrabFeedback;

/* 每次新动作id递增；同id反复提交只更新观测，不应重启动作。
 * suspended为1时执行端停车/暂停；恢复后同id可继续，绝不能沿用旧速度。
 * 反馈必须带id，旧动作的迟到反馈会被丢弃。 */
typedef struct {
    uint32_t id;
    uint8_t action, stage, camera_pose, team, start_position, suspended;
    uint16_t hold_remaining_ms;
    float base_angle_deg;
    CargoCounts cargo;
    MaixcamObservation target;
    MaixcamNavigation navigation;
} GrabActionRequest;

typedef void (*GrabActionExecutor)(const GrabActionRequest *request);

/* 只在任务上下文调用，回调必须非阻塞，不能在内部while等待动作结束。 */
void Grab_Init(void);
void Grab_BindExecutor(GrabActionExecutor executor);
void Grab_Start(void); /* 只在WAITING_START接受独立启动请求，相机运行不会触发它 */
void Grab_SetTeam(uint8_t team); /* 发车前配置；改完需重新初始化当前一轮 */
uint8_t Grab_GetTeam(void);
GrabStage Grab_GetStage(void);
uint8_t Grab_GetVisionMode(void);
uint8_t Grab_IsFirstNormalDelivered(void);
void Grab_GetActionRequest(GrabActionRequest *out);
void Grab_ReportAction(uint32_t id, GrabFeedback event, float base_angle_deg);
/* 推入及送达判据暂未定义：只预留结果接口。
 * 只有未来真实送达判据通过才能传1，动作完成本身不允许作为证据。
 * 失败/未知传0，保留任务及首件限制，等待后续定义处理。 */
void Grab_ReportDelivery(uint32_t request_id, uint8_t delivered);
CargoCounts Grab_GetConfirmedCargo(void);
/* 可独立核对的业务更新入口；不直接发电机/舵机输出。 */
void Grab_Process(uint32_t now_ms, const MaixcamObservation *vision,
                  const MaixcamNavigation *navigation);
void rescue_grab_task(void *pvParameters);
void rescue_grab_task_create(void);
extern TaskHandle_t rescue_grab_handler;

#endif
