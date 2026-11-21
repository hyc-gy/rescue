#include "pose_estimator.h"
#include "motor.h"
#include <math.h>

#define DEG_RAD 0.01745329251994329577f   // 度 → 弧度 换算系数

/* 内部静态变量 */
static uint32_t last_ms, last_imu_ms;   // 上次 pose 更新时间戳、上次 IMU 帧时间戳
static float last_imu_yaw;              // 上次 IMU 累计 yaw
static uint8_t imu_anchor;              // IMU 基准点是否已建立（1=已对齐首帧）

/* 把任意角度规范化到 [-180°, +180°] */
static float wrap(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 初始化位姿估计器：原点 (0,0,0) */
void pose_init(void) { pose_set_origin(0.0f, 0.0f); }

/* 设置全局坐标系原点：同时清空所有内部状态，重新开始积分 */
void pose_set_origin(float x, float y) {
    taskENTER_CRITICAL();
    my_car.pose_manager.x=x; 
    my_car.pose_manager.y=y;
    my_car.pose_manager.distance=0; 
    my_car.pose_manager.valid=0;
    my_car.pose_manager.speed_mps=0; 
    my_car.pose_manager.imu_fresh=0;
    last_ms=HAL_GetTick(); 
    imu_anchor=0; 
    last_imu_ms=0;
    taskEXIT_CRITICAL();
}

/* 核心融合函数：用两轮编码器做差速里程计，在 IMU 帧到来时用陀螺仪 yaw 纠正航向
 * 每 10ms 调用一次。*/
void pose_update_from_encoders(void) {
    uint32_t now=HAL_GetTick(); 
    uint32_t dt=now-last_ms;
    float yaw; 
    uint32_t stamp; 
    uint8_t valid;
    yaw=my_car.angle_manager.yaw;
    stamp=my_car.angle_manager.update_ms;
    valid=my_car.angle_manager.valid;
    /* L0/R0 是 motor_task 实际驱动的两个轮子 */
    float dl=motor_L0.measure*POSE_LEFT_SIGN*POSE_LEFT_M_PER_COUNT;   // 左轮本周期走了多少米
    float dr=motor_R0.measure*POSE_RIGHT_SIGN*POSE_RIGHT_M_PER_COUNT; // 右轮本周期走了多少米
    float ds=(dl+dr)*0.5f;                          // 车身中心本周期前进距离
    float dh=(dr-dl)/POSE_TRACK_WIDTH_M/DEG_RAD;    // 纯运动学推导的航向增量（度）

    /* 判断 IMU 数据是否新鲜：有 valid 标记 + 时间戳在超时窗口内 */
    uint8_t fresh=valid && (uint32_t)(now-stamp)<=POSE_IMU_TIMEOUT_MS;

    if (fresh) {
        if (imu_anchor && stamp!=last_imu_ms) {
            /* IMU 新帧来了，用陀螺仪 yaw 增量替代纯运动学 dh（更准） */
            dh=wrap(POSE_GYRO_SIGN*(yaw-last_imu_yaw));
            last_imu_yaw=yaw; 
            last_imu_ms=stamp;
        } else if (!imu_anchor) {
            /* 第一帧 IMU：只建立基准点，不积分 */
            last_imu_yaw=yaw; 
            last_imu_ms=stamp; 
            imu_anchor=1;
        } else {
            /* IMU 还没更新（同一帧内多次 update），本次 dh 不动，
             * 避免每 10ms 重复消费同一个陀螺仪增量导致航向漂移 */
            dh=0;
        }
    } else imu_anchor=0;   /* IMU 超时了：下帧重新对齐基准 */

    /* 中点航向 → 本周期积分方向（梯形法则，比用首尾更准） */
    float mid=(my_car.angle_manager.yaw+dh/2.0f)*DEG_RAD;
    my_car.pose_manager.x+=ds*cosf(mid);
    my_car.pose_manager.y+=ds*sinf(mid);
    float dx = my_car.pose_manager.target_x-my_car.pose_manager.x;
    float dy = my_car.pose_manager.target_y-my_car.pose_manager.y;      
    my_car.pose_manager.distance=sqrtf(dx*dx+dy*dy);       // 累计行驶距离（里程）
    my_car.pose_manager.speed_mps=dt ? ds*1000.0f/dt : 0;   // 瞬时速度（m/s）      
    my_car.pose_manager.valid=POSE_CALIBRATED && dt>0 && dt<=100U;  // 本帧 pose 是否可信
    last_ms=now;
}
