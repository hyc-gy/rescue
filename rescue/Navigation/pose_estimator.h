#ifndef RESCUE_POSE_ESTIMATOR_H
#define RESCUE_POSE_ESTIMATOR_H
#include "common.h"

/* ===== 标定参数 =====
 * 在实车上测量好之后把下面 POSE_CALIBRATED 改成 1。
 * 比例单位：车轮编码器正交计数 → 米（含减速比）。
 * 正编码器计数 + 正航向增量必须对应"前进"和"逆时针"。
 */
#define POSE_CALIBRATED 0          // 标定完成后置 1，否则 pose 输出视为无效
#define POSE_LEFT_M_PER_COUNT 0.0001f   // 左轮每编码器计数对应的米数
#define POSE_RIGHT_M_PER_COUNT 0.0001f  // 右轮每编码器计数对应的米数
#define POSE_LEFT_SIGN 1.0f        // 左轮编码器方向符号（前进为正则 +1）
#define POSE_RIGHT_SIGN 1.0f       // 右轮编码器方向符号
#define POSE_TRACK_WIDTH_M 0.15f   // 左右轮中心距（米）
#define POSE_GYRO_SIGN 1.0f       // 陀螺仪 yaw 方向符号（逆时针为正则 +1）
#define POSE_IMU_TIMEOUT_MS 200U   // IMU 数据新鲜度超时阈值（毫秒）

void pose_init(void);
/* 设置全局坐标系原点；仅在小车静止时调用。
 * x/y 单位米，yaw 单位度（逆时针为正）。 */
void pose_set_origin(float x, float y);
/* 每 10ms 调用一次，用编码器 + IMU 融合更新小车位姿。 */
void pose_update_from_encoders(void);

#endif
