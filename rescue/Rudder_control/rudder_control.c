#include "rudder_control.h"

TaskHandle_t rudder_handler;

/* ============================================================================
 * 舵机控制（TIM1 四路 PWM，50Hz）
 *
 *   CH1 → 舵机1：夹爪开合（直接写 CCR1，立即动作，不做缓动）
 *   CH2 → 舵机2：视觉云台俯仰（由 rudder_task 每 10ms 缓动一步）
 *   CH3/CH4 → 预留，未使用
 *
 * CCR 值与角度的关系（由机械安装方向决定）：
 *   CCR 越大 → 舵机角度越低
 *   舵机2：135 = 抬到最高（看远处）    190 = 压到最低（看正下方）
 * ==========================================================================*/

#define STEP_SLOW 1    // 步长1，数值越小转得越慢；改2/3会变快

/* 记录舵机的当前 CCR 值。下标 1 对应舵机1（夹爪），下标 2 对应舵机2（云台） */
uint16_t rudder_cur_ccr[3] = {0, 34, 135}; // 下标1、2对应舵机1、2
uint16_t ccr_2_max = 135;//最高姿态
uint16_t ccr_2_min = 190;//最低姿态
uint16_t target_ccr_2 = 135;   // 舵机2的目标 CCR，由 set_vision_angle() 修改

/* 初始化：启动 TIM1 四路 PWM，并把两路舵机摆到初始位置 */
void rudder_init(void){
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    rudder_cur_ccr[1] = 150;      // 夹爪初始：合拢
    rudder_cur_ccr[2] = 135;     // 云台初始：抬到最高
    TIM1->CCR1 = rudder_cur_ccr[1];
    TIM1->CCR2 = rudder_cur_ccr[2];
}

/* 平滑缓动底层函数
 * 每次调用只让当前值朝目标走 STEP_SLOW 步，走过头就精确贴到目标值。
 * 配合 10ms 的任务周期，形成"每 10ms 走 1 个单位"的匀速缓动，
 * 避免舵机被一次性大角度指令带着猛甩。 */
static void rudder_smooth_ctrl(uint8_t rudder_idx, uint16_t target_ccr)
{
    if(rudder_idx < 1 || rudder_idx > 2) return; // 越界保护

    uint16_t *cur = &rudder_cur_ccr[rudder_idx];
    if(*cur < target_ccr)
    {
        *cur += STEP_SLOW;                       // 需要变大：向上加
        if(*cur > target_ccr) *cur = target_ccr; // 别冲过头
    }
    else if(*cur > target_ccr)
    {
        *cur -= STEP_SLOW;                       // 需要变小：向下减
        if(*cur < target_ccr) *cur = target_ccr; // 别冲过头
    }
    // 更新PWM
    switch (rudder_idx) {
        case 1: TIM1->CCR1 = *cur; break;
        case 2: TIM1->CCR2 = *cur; break;
        default: break;
    }
}

/* 设置舵机2的目标角度。
 * 这里只改目标值，不直接写寄存器——由 rudder_task 平滑推过去 */
void set_vision_angle(uint16_t angle){
    target_ccr_2 = angle;
}

/* 对外接口（替换原来rudder_control）：缓动一路舵机到指定 CCR */
void rudder_control(uint8_t rudder_idx, uint16_t target_ccr){
    rudder_smooth_ctrl(rudder_idx, target_ccr);
}

/* 记录当前舵机工作模式（预留接口，逻辑中尚未使用） */
void Set_Rudder_Mode(uint8_t mode){
    my_car.rudder_mode = mode;
}

/* 夹取：夹爪合紧（34 为最紧位置，直接写寄存器、无缓动） */
void grab_posture(void){
    TIM1->CCR1 = 34;
}

/* 张开：夹爪打开到中等开口，用于抓取前准备 */
void open_posture(void){
    TIM1->CCR1 = 53;
}

/* 大张：夹爪开到最大，用于把误夹的目标丢掉 */
void open_big_posture(void){
    TIM1->CCR1 = 60;
}

/* ============================================================================
 * 舵机任务：10ms 周期，只负责把舵机2（视觉云台）平滑推到目标角度
 * 夹爪（舵机1）不经过本任务，由 grab_posture/open_posture 直接写 CCR1 立即动作
 * ==========================================================================*/
void rudder_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t TASK_PERIOD = pdMS_TO_TICKS(10); // 10ms周期
    rudder_init();
    while (1) {
        rudder_control(2, target_ccr_2);   // 舵机2 朝目标缓动一步
        vTaskDelayUntil(&xLastWakeTime, TASK_PERIOD);
    }

}

/* 创建舵机任务 */
void rudder_task_create(void){
    xTaskCreate((TaskFunction_t ) rudder_task,          // 任务入口
                (const char *)"rudder_task",            // 任务名称
                (uint32_t) rudder_size,                 // 堆栈大小
                (void* )NULL,                           // 传给任务的参数
                (UBaseType_t) rudder_task_priority,     // 优先级
                (TaskHandle_t *)&rudder_handler );      // 任务句柄
}
