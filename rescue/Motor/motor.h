#ifndef RESCUE_MOTOR_H
#define RESCUE_MOTOR_H

#include "common.h"
#include "pid.h"
#include "my_delay.h"
#include "rudder_control.h"
#include "hc05.h"
#include "maixcam.h"

typedef enum {
    MOTOR_STOP,           // 1. 停止模式    
    MOTOR_SEARCH_TARGET,    // 2. 寻目标模式
    MOTOR_TRACK_TARGET,   // 3. 追踪目标模式
    MOTOR_FORWARD,        // 4. 前进模式
    MOTOR_BACKWARD,       // 5. 后退模式
    MOTOR_BACKWARD_little, // 6. 小幅后退模式   
    MOTOR_POSITION,     // 7. 位置模式
    MOTOR_FORWARD_FAST, // 8. 快速模式
    MOTOR_BLUETOOTH,    // 9. 蓝牙速度模式：直接跟踪 motor_L0/R0.target
} MotorState;

void motor_init(void);
void pid_init(void);
void encoder_init(void);
void motor_pid_clear(void);
void motor_set_pwm(uint8_t motor_idx, int32_t pid_out);
void motor_set_state(MotorState state);
void get_encoder(void);
void set_vision_y_kp(float kp);

extern TaskHandle_t motor_handler; 				// 电机任务句柄
void motor_task(void *pvParameters);  			// 电机任务入口函数
#define motor_size  (1024)			// 任务堆栈大小
#define motor_task_priority 5 			// 任务优先级
void motor_task_create(void);

extern	struct PID_param L0_param;
extern	struct PID_param R0_param;
extern	struct PID_param L1_param;
extern	struct PID_param R1_param;

extern struct I_pid_obj motor_L0;
extern struct I_pid_obj motor_R0;
extern struct I_pid_obj motor_L1;
extern struct I_pid_obj motor_R1;

extern struct P_pid_obj Vision_x_pid_obj;
extern struct PID_param Vision_x_pid_param;

extern struct P_pid_obj Vision_w_pid_obj;
extern struct PID_param Vision_w_pid_param;

extern struct P_pid_obj distance_pid_obj;
extern struct PID_param distance_pid_param;

extern struct P_pid_obj yaw_pid_obj;
extern struct PID_param yaw_pid_param;  

// G：平衡环，T：转向环，均为预留的陀螺仪 PID
extern struct P_pid_obj gyroT_pid;
extern struct P_pid_obj gyroG_pid;
extern struct PID_param gyroT_pid_param, gyroG_pid_param;

#endif //RESCUE_MOTOR_H
