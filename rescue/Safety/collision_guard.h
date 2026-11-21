#ifndef RESCUE_COLLISION_GUARD_H
#define RESCUE_COLLISION_GUARD_H
#include "common.h"

/* 碰撞原因位标志，collision_manager.reason 是按位或累加的 */
#define COLLISION_REASON_BUMPER 1U   // 外部碰撞开关（比如物理保险杠）
#define COLLISION_REASON_STALL 2U    // 电机堵转（有 PWM 但编码器没读数）

/* ===== 堵转检测阈值 =====
 * 这些只是初始值，装到车上后需要实际调参。 */
#define COLLISION_PWM_THRESHOLD 120    // PWM 绝对值低于此值视为"没发力"，不检测堵转
#define COLLISION_MIN_COUNTS 2.0f      // 编码器读数绝对值低于此值视为"没转动"
#define COLLISION_STALL_MS 500U        // 同时满足"有 PWM + 无转动"持续多久才算堵转
#define COLLISION_BUMPER_DEBOUNCE_MS 20U   // 碰撞开关消抖时间（毫秒）

void collision_guard_init(void);
/* 每 10ms 调用一次，检查碰撞开关 + 电机堵转，触发则置位 collision_manager.detected */
void collision_guard_update(void);
/* 返回 1 表示被阻挡；供上层状态机查询 */
uint8_t collision_guard_blocked(void);
/* 复位碰撞状态。只能在任务上下文里调用；如果碰撞开关还按着会拒绝清零（返回 0），
 * 避免"人还没挪开就误以为安全了"。 */
uint8_t collision_guard_clear(void);
/* motor_set_pwm 输出前调用，记录当前每路 PWM 以便堵转检测做对比 */
void collision_guard_record_pwm(uint8_t index, int32_t pwm);
/* 板级适配器：在具体工程里 override 它来接真实碰撞开关 GPIO。
 * 返回 1 = 实际接触，0 = 已释放。默认实现永远返回 0（没装保险杠时可用）。
 * 千万别在不确定硬件连接前就随便绑一个 GPIO！ */
uint8_t collision_bumper_read(void);

#endif
