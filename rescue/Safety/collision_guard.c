#include "collision_guard.h"
#include "motor.h"
#include <math.h>

/* 内部静态状态 */
static int32_t applied[2];           // 两路电机（左/右）最近一次输出的 PWM 值
static uint32_t stall_since[2];       // 每路"堵转条件"首次成立的时间戳
static uint32_t bumper_since;         // 碰撞开关首次按下的时间戳
static uint8_t stall_active[2];       // 每路当前是否处于堵转判定窗口内
static uint8_t bumper_active;         // 碰撞开关当前是否处于消抖窗口内

/* 弱符号：板级 override 入口，默认无硬件保险杠 */
__weak uint8_t collision_bumper_read(void) { return 0; }

/* 触发一次碰撞事件：首次触发累加计数器，原因位按或叠加 */
static void trip(uint8_t reason) {
    if (!my_car.collision_manager.detected) my_car.collision_manager.event_count++;
    my_car.collision_manager.detected=1;
    my_car.collision_manager.reason |= reason;
}

/* 初始化：清所有状态 */
void collision_guard_init(void) {
    my_car.collision_manager.detected=0;
    my_car.collision_manager.reason=0;
    my_car.collision_manager.stall_ticks=0;
    my_car.collision_manager.event_count=0;
    applied[0]=applied[1]=0;
    stall_active[0]=stall_active[1]=bumper_active=0;
}

/* 上层便捷查询接口 */
uint8_t collision_guard_blocked(void) {
    return my_car.collision_manager.detected;
}

/* motor_task 每次写 PWM 前调用：记录当前值供碰撞守护读取 */
void collision_guard_record_pwm(uint8_t index,int32_t pwm) {
    if (index>=1 && index<=2) applied[index-1]=pwm;
}

/* 周期性检测：碰撞开关 + 电机堵转 */
void collision_guard_update(void) {
    uint32_t now=HAL_GetTick();
    float counts[2]={motor_L0.measure,motor_R0.measure};   // 两路最近 10ms 编码器读数
    taskENTER_CRITICAL();

    /* 碰撞开关检测（带消抖） */
    if (collision_bumper_read()) {
        if (!bumper_active) { bumper_active=1; bumper_since=now; }
        if ((uint32_t)(now-bumper_since)>=COLLISION_BUMPER_DEBOUNCE_MS)
            trip(COLLISION_REASON_BUMPER);
    } else bumper_active=0;   // 开关松开：取消本次判定

    /* 每路电机堵转检测 */
    for (unsigned i=0;i<2;i++) {
        /* 条件：PWM 够大（说明控制器在发力） && 编码器几乎没读数（说明轮子没转） */
        if (fabsf((float)applied[i])>=COLLISION_PWM_THRESHOLD &&
            fabsf(counts[i])<COLLISION_MIN_COUNTS) {
            if (!stall_active[i]) { stall_active[i]=1; stall_since[i]=now; }
            /* 堵转持续超过阈值才算真堵转（瞬时失速不算） */
            if ((uint32_t)(now-stall_since[i])>=COLLISION_STALL_MS)
                trip(COLLISION_REASON_STALL);
        } else stall_active[i]=0;   // 任一条件不满足：判定窗口重置
    }

    taskEXIT_CRITICAL();
}

/* 清除碰撞状态。碰撞开关还按着时拒绝清零，返回 0 让调用方重试 */
uint8_t collision_guard_clear(void) {
    taskENTER_CRITICAL();
    if (collision_bumper_read()) { taskEXIT_CRITICAL(); return 0; }
    my_car.collision_manager.detected=0;
    my_car.collision_manager.reason=0;
    my_car.collision_manager.stall_ticks=0;
    stall_active[0]=stall_active[1]=bumper_active=0;
    applied[0]=applied[1]=0;
    taskEXIT_CRITICAL();
    return 1;
}
