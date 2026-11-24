#ifndef MAIXCAM_H
#define MAIXCAM_H

#include "common.h"

/* 主控唯一默认队色：1蓝、2红，Grab任务也沿用此值。 */
#define MAIXCAM_TEAM_DEFAULT 1U

/* ===== 通信协议常量 ===== */
#define MAIXCAM_PROTOCOL_VERSION 3U      // 协议版本号，命令帧与结果帧都必须与它一致
#define MAIXCAM_RESULT_FRAME_SIZE 24U    // 视觉模块回传的结果帧长度（字节）
#define MAIXCAM_COMMAND_FRAME_SIZE 7U    // 主控下发的命令帧长度（字节）

/* 视觉工作模式：主控通过命令帧告诉 Maixcam 当前该识别什么 */
typedef enum {
    MAIXCAM_MODE_OBSERVE = 0,            // 观察模式（不针对特定目标）
    MAIXCAM_MODE_NORMAL_MATERIAL = 1,    // 普通物资
    MAIXCAM_MODE_CORE_MATERIAL = 2,      // 核心物资
    MAIXCAM_MODE_PATIENT = 3,            // 伤员
    MAIXCAM_MODE_MATERIAL_ZONE = 5,      // 物资区
    MAIXCAM_MODE_PATIENT_ZONE = 6,       // 伤员区
    MAIXCAM_MODE_RESCUE_TARGETS = 7,     // 救援目标（原"找球"）
    MAIXCAM_MODE_MIXED_MATERIALS = 8,    // 混合物资
    MAIXCAM_MODE_CONFIRM_CARGO = 9       // 确认夹持物
} MaixcamMode;

/* 目标类型：视觉模块回传的"当前锁定到的目标是什么" */
typedef enum {
    MAIXCAM_TARGET_NONE = 0,                  // 无目标
    MAIXCAM_TARGET_NORMAL_MATERIAL = 1,       // 普通物资
    MAIXCAM_TARGET_CORE_MATERIAL = 2,         // 核心物资
    MAIXCAM_TARGET_PATIENT = 3,               // 伤员
    MAIXCAM_TARGET_HAZARD = 4,                // 危险物
    /* 5—8只保留原V3四半区兼容，新整区/分界线使用下方MaixcamReference。 */
    MAIXCAM_TARGET_BLUE_MATERIAL_ZONE = 5,    // 蓝方物资区
    MAIXCAM_TARGET_BLUE_PATIENT_ZONE = 6,     // 蓝方伤员区
    MAIXCAM_TARGET_RED_MATERIAL_ZONE = 7,     // 红方物资区
    MAIXCAM_TARGET_RED_PATIENT_ZONE = 8       // 红方伤员区
} MaixcamTargetType;

/* 以下是业务数据定义，不规定新的串口帧格式。旧7/24字节协议保持不变。
 * 整区/分界线不再借用旧的四半区类型；新收发适配由双方确认后接入。 */
typedef enum {
    MAIXCAM_NAV_NOT_RETURNING_TO_BASE = 0,
    MAIXCAM_NAV_FOLLOW_ZONE,
    MAIXCAM_NAV_FOLLOW_SPLIT,
    MAIXCAM_NAV_HOLD_LAST_SPEED,
    MAIXCAM_NAV_SEARCH_ZONE
} MaixcamNavigationAction;

typedef enum {
    MAIXCAM_REFERENCE_NONE = 0,
    MAIXCAM_REFERENCE_BLUE_ZONE,
    MAIXCAM_REFERENCE_RED_ZONE,
    MAIXCAM_REFERENCE_RED_SPLIT,
    MAIXCAM_REFERENCE_BLUE_SPLIT
} MaixcamReference;

/* 原结果帧的一次完整快照。frame_id是本机接收代数，不是新增线上字段。 */
typedef struct {
    struct Vision_Manager data;
    uint32_t frame_id;
} MaixcamObservation;

/* 完整导航观察。无当前框时坐标/分数必须清零，不能用旧框冒充新检测。
 * mode保留物资/伤员的运输目的，reference表示实际跟随的整区/分界线。 */
typedef struct {
    uint8_t sequence, mode, team, action, reference, reference_visible;
    uint16_t x, y, w, h, hold_remaining_ms;
    uint8_t confidence, frame_valid;
    uint32_t updated_ms, frame_id;
} MaixcamNavigation;

void Maixcam_GetObservation(MaixcamObservation *out);
void Maixcam_GetNavigation(MaixcamNavigation *out);
/* 队友完成新导航协议解析后，每个有效新画面调用一次本函数。
 * 输入sequence/mode/team/action/reference/框/分数/剩余ms；本函数填接收时刻及frame_id。
 * 返回0表示业务字段不合法或重复帧，未发布；返回1表示已发布。
 * 该接口不是宣称现有24字节帧已经含有上述字段。 */
uint8_t Maixcam_PublishNavigation(const MaixcamNavigation *observation);

/* 兼容旧的调用代码：把原来的宏名映射到新的语义模式，
 * 这样 grab_task 等旧代码不用改就能继续编译运行 */
#define bolls_mode ((uint8_t)MAIXCAM_MODE_RESCUE_TARGETS)   // 找球 → 救援目标
#define confirm_mode ((uint8_t)MAIXCAM_MODE_CONFIRM_CARGO)  // 确认 → 确认夹持物
#define cross_mode ((uint8_t)MAIXCAM_MODE_OBSERVE)          // 新模型没有交叉线标签，退回观察模式
#define base_mode ((uint8_t)MAIXCAM_MODE_MATERIAL_ZONE)     // 基地 → 物资区

void Maixcam_init(void);                                    // 初始化：清状态 + 开 DMA 空闲中断接收

/* 下发一条模式命令。mode=工作模式, team=队伍(1蓝/2红)。
 * 与上次相同且相机已新鲜回显时不重复；否则继续发送。返回1=成功/已回显，0=失败 */
uint8_t Maixcam_SendCommand(uint8_t mode, uint8_t team);

/* 设置队伍颜色。若之前已发过命令，会用新队伍重发一次当前模式 */
uint8_t Maixcam_SetTeam(uint8_t team);

/* 兼容旧接口：用当前队伍下发模式命令（忽略返回值） */
void Set_Vision_Mode(uint8_t mode);

/* 判断视觉数据是否新鲜：收到过有效帧，且距上次更新不超过 timeout_ms 毫秒 */
uint8_t Maixcam_IsFresh(uint32_t timeout_ms);

/* UART5 空闲中断处理（配合 DMA 收包），由 stm32h7xx_it.c 调用 */
void maixcam_uart_irq_handler(void);

#endif
