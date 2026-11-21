#ifndef rudder_control_H__
#define rudder_control_H__
#include "common.h"
#include "my_delay.h"

/* 夹取数量模式宏（历史遗留，当前代码未使用） */
#define one_grab 0x01
#define more_grab 0x02

void rudder_init(void);              // 启动 TIM1 四路 PWM，并把舵机复位到初始位置
void grab_posture(void);             // 夹爪合紧（TIM1->CCR1 = 34）
void open_posture(void);             // 夹爪张开（TIM1->CCR1 = 53）
void open_big_posture(void);         // 夹爪大张（TIM1->CCR1 = 60，用于丢弃误夹的目标）

/* 设置视觉云台舵机（舵机2）的目标角度。
 * 注意：这里只是记录目标值，真正转动由 rudder_task 每 10ms 平滑逼近 */
void set_vision_angle(uint16_t angle);

extern TaskHandle_t rudder_handler;                 // 舵机任务句柄
void rudder_task(void *pvParameters);               // 舵机任务入口函数
#define rudder_size  (256)                          // 任务堆栈大小（字节）
#define rudder_task_priority 5                      // 任务优先级
void rudder_task_create(void);                      // 创建舵机任务

#endif
