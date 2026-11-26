#include "motor.h"
#include "pid.h"
#include "pose_estimator.h"
#include "collision_guard.h"
#include "stdlib.h"
#include "common.h"
#include "math.h"
#include <stdio.h>

#include "pose_estimator.h"

float MAX_SPEED_rate = 0.8f;//0.8
int32_t MOTOR_PWM_MAX = 900;
float x_speed=0;
static MotorState motor_state = MOTOR_STOP;
TaskHandle_t motor_handler;
//extern uint8_t yaw_full_circle ;
//extern float yaw_total ;
//extern float delta;

struct I_pid_obj motor_L0 = {0,0,0,0,0,0}; // 左前轮（0 表示前排）
struct I_pid_obj motor_R0 = {0,0,0,0,0,0};  // 右前轮
struct I_pid_obj motor_L1 = {0,0,0,0,0,0};  // 左后轮
struct I_pid_obj motor_R1 = {0,0,0,0,0,0};  // 右后轮

struct PID_param L0_param = {0,0,0,0,0,0};
struct PID_param R0_param = {0,0,0,0,0,0};
struct PID_param L1_param = {0,0,0,0,0,0};
struct PID_param R1_param = {0,0,0,0,0,0};

struct P_pid_obj Vision_x_pid_obj = {0,0,0,0,0,0,0};
struct PID_param Vision_x_pid_param = {0,0,0,0,0,0};

struct P_pid_obj Vision_y_pid_obj = {0,0,0,0,0,0,0};
struct PID_param Vision_y_pid_param = {0,0,0,0,0,0};


struct P_pid_obj distance_pid_obj = {0,0,0,0,0,0,0};
struct PID_param distance_pid_param = {0,0,0,0,0,0};

struct P_pid_obj yaw_pid_obj = {0,0,0,0,0,0,0};
struct PID_param yaw_pid_param = {0,0,0,0,0,0};

struct P_pid_obj gyroT_pid = {0,0,0,0,0,0,0};
struct P_pid_obj gyroG_pid = {0,0,0,0,0,0,0};
struct PID_param gyroT_pid_param = {0,0,0,0,0,0};
struct PID_param gyroG_pid_param = {0,0,0,0,0,0};

void encoder_init(void){
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

void motor_pid_clear(void){
    motor_L0 = (struct I_pid_obj){0,0,0,0,0,0};
    motor_R0 = (struct I_pid_obj){0,0,0,0,0,0};
    motor_L1 = (struct I_pid_obj){0,0,0,0,0,0};
    motor_R1 = (struct I_pid_obj){0,0,0,0,0,0};
}

void pid_init(void){

    MOTOR_PWM_MAX = (int32_t)(htim12.Instance->ARR * MAX_SPEED_rate);
    // 增量式 PID（电机内环），带抗积分饱和
    L0_param.kp = 57;//24;//60
    L0_param.ki = 5.0;//0.8f;//10
    L0_param.kd = 0.12f;//5
    L0_param.outputMax = MOTOR_PWM_MAX;

    R0_param.kp = 57;//24;//60
    R0_param.ki = 5.0;//0.8f;//10
    R0_param.kd = 0.12f;//5
    R0_param.outputMax = MOTOR_PWM_MAX;

    L1_param.kp = 60.0f;
    L1_param.ki = 10.0f;
    L1_param.kd = 5.0f;
    L1_param.outputMax = MOTOR_PWM_MAX;

    R1_param.kp = 60.0f;
    R1_param.ki = 10.0f;
    R1_param.kd = 5.0f;
    R1_param.outputMax = MOTOR_PWM_MAX;

    // 左右（横向偏差）
    Vision_x_pid_param.kp=0.1f;//0.1
    Vision_x_pid_param.ki=0;
    Vision_x_pid_param.kd=0.08f;//0.08
    Vision_x_pid_param.outputMax = 15;//15
    Vision_x_pid_param.outputMin = -15;
    Vision_x_pid_param.differential_filterK =0.5f;

    // 前后（纵向偏差）
    Vision_y_pid_param.kp=0.35f;//0.35
    Vision_y_pid_param.ki=0;
    Vision_y_pid_param.kd=0.55f;//0.55
    Vision_y_pid_param.outputMax = 28;//25
    Vision_y_pid_param.outputMin = -28;
    Vision_y_pid_param.differential_filterK =0.5f;

    distance_pid_param.kp=25.0f;
    distance_pid_param.ki=1.0f;
    distance_pid_param.kd=0.0f;
    distance_pid_param.outputMax = 15;
    distance_pid_param.outputMin = -15;
    distance_pid_param.differential_filterK =0.5f;
    
    yaw_pid_param.kp=0.45f;
    yaw_pid_param.ki=0.0f;
    yaw_pid_param.kd=0.2f;
    yaw_pid_param.outputMax = 28;
    yaw_pid_param.outputMin = -28;
    yaw_pid_param.differential_filterK =0.5f;

    gyroT_pid_param.kp=0.45f;
    gyroT_pid_param.ki=0;
    gyroT_pid_param.kd=0.2f;
    gyroT_pid_param.outputMax = 28;
    gyroT_pid_param.outputMin = -28;
    gyroT_pid_param.differential_filterK =0.5f;
    
    motor_pid_clear();
}

void motor_init(void){
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
    pid_init();
}

void get_encoder(void){
    motor_L0.measure = -(float)(int16_t)TIM2->CNT;
    motor_R0.measure = -(float)(int16_t)TIM4->CNT;
    motor_L1.measure = -(float)(int16_t)TIM3->CNT;
    motor_R1.measure = -(float)(int16_t)TIM8->CNT;

	TIM3->CNT = 0;
    TIM2->CNT = 0;
    TIM4->CNT = 0;
    TIM8->CNT = 0;
}

void motor_set_direction(uint8_t motor_idx, uint8_t motor_direction){
    switch(motor_idx){
        case 1:
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,motor_direction);
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,!motor_direction);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,motor_direction);
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,!motor_direction);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,motor_direction);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,!motor_direction);
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,motor_direction);
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,!motor_direction);
            break;
        default:
            break;
    }
}

void motor_set_pwm(uint8_t motor_idx, int32_t pid_out){
    taskENTER_CRITICAL();
    if (collision_guard_blocked()) pid_out = 0;
    if (pid_out > MOTOR_PWM_MAX) pid_out = MOTOR_PWM_MAX;
    if (pid_out < -MOTOR_PWM_MAX) pid_out = -MOTOR_PWM_MAX;
    collision_guard_record_pwm(motor_idx, pid_out);
    int32_t ccr = 0;
    if (pid_out >= 0){
        if (pid_out > MOTOR_PWM_MAX)
            ccr = MOTOR_PWM_MAX;
        else
            ccr = pid_out;
        switch (motor_idx){
            case 1:
                TIM15->CCR2 = ccr;
                motor_set_direction(1,1);// 正转
                break;
            case 2:
                TIM12->CCR2 = ccr;
                motor_set_direction(2,1);// 正转
                break;
            case 3:
                TIM15->CCR1 = ccr;
                motor_set_direction(3,1);// 正转
                break;
            case 4:
                TIM12->CCR1 = ccr;
                motor_set_direction(4,1);// 正转
                break;
            default:
                break;
        }
    }
    else{
        if (pid_out < -MOTOR_PWM_MAX)
            ccr = MOTOR_PWM_MAX;
        else
            ccr = -pid_out;
        switch (motor_idx){
            case 1:
                motor_set_direction(1,0);
                TIM15->CCR2 = ccr;
                break;
            case 2:
                motor_set_direction(2,0);
                TIM12->CCR2 = ccr;
                break;
            case 3:
                motor_set_direction(3,0);
                TIM15->CCR1 = ccr;
                break;
            case 4:
                motor_set_direction(4,0);
                TIM12->CCR1 = ccr;
                break;
            default:
                break;
        }
    }
    taskEXIT_CRITICAL();
}

static void vofa_motor_send(void)
{
    char buf[256];

    /*int len = snprintf(buf, sizeof(buf),

                       "%.0f,%.0f,%.0f,%d\n",
                       my_car.angle_manager.yaw,delta,yaw_total,yaw_full_circle);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20);*/
}

static const float r1_targets[] = { 5, 10, 15, 20, 25};
static uint8_t r1_target_idx = 0;
static uint16_t r1_step_cnt = 0;
static float r1_test_target = 0;
static uint8_t cnt = 0;

/* ===== 碰撞恢复参数 ===== */
#define COLLISION_HOLD_MS       300U    // 碰撞后先停车等待的时间
#define COLLISION_ESCAPE_MS     300U    // 倒退脱困的持续时间
#define COLLISION_ESCAPE_PWM    300     // 脱困时使用的倒退 PWM

/* 碰撞恢复：非阻塞状态机，每 10ms 推进一步
 * 返回 1 表示正在恢复，调用方应跳过正常电机控制
 * 阶段1：停车等待 → 尝试解除锁存
 * 阶段2：倒退脱困
 * 恢复期间 motor_task 独占电机，grab_task 写入的 motor_state 不生效；
 * 恢复结束时强制置 MOTOR_STOP，避免残留的旧运动指令让小车再次撞上去。 */
static uint8_t collision_recover_step(uint32_t now)
{
    static uint32_t since = 0U;
    static uint8_t  phase = 0U;          /* 0=正常 1=停车等待 2=倒退脱困 */

    if (collision_guard_blocked() && phase != 1U) {
        phase = 1U;
        since = now;
        motor_set_state(MOTOR_STOP);     /* 立刻抢占控制权，丢弃旧的运动指令 */
        motor_pid_clear();
    }
    if (phase == 0U) return 0U;

    if (phase == 1U) {
        motor_set_pwm(1, 0);             /* 先停稳，释放机械应力 */
        motor_set_pwm(2, 0);
        if ((uint32_t)(now - since) >= COLLISION_HOLD_MS) {
            if (collision_guard_clear()) { phase = 2U; since = now; }
            else since = now;            /* 保险杠未松开，稍后再试 */
        }
    } else {
        if ((uint32_t)(now - since) < COLLISION_ESCAPE_MS) {
            motor_set_pwm(1, -COLLISION_ESCAPE_PWM);   /* 倒退脱困 */
            motor_set_pwm(2, -COLLISION_ESCAPE_PWM);
        } else {
            motor_set_pwm(1, 0);
            motor_set_pwm(2, 0);
            motor_pid_clear();
            motor_set_state(MOTOR_STOP); /* 交还控制权，且不保留旧指令 */
            phase = 0U;                  /* 恢复完成 */
        }
    }
    return 1U;
}

void motor_task(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t PERIOD = pdMS_TO_TICKS(10);

    while(1){
        //open_posture();
        // my_delay_ms(2000);
        // open_big_posture();
        // my_delay_ms(2000);
        //grab_posture();
    //TIM1->CCR1 = 150;
    // if(TIM1->CCR1 == 150)
    // {
    //   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    // }
    // my_delay_ms(1000);
    // //TIM1->CCR1 = 60;
    // if(TIM1->CCR1 == 60)
    // {
    //   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    // }
    // my_delay_ms(1000);
        get_encoder();
        pose_update_from_encoders();
        collision_guard_update();
        Maixcam_SetTeam(2);
        if(bluetooth_work)
        {
            HC05_PollCommand();                     // 解析蓝牙命令（A/Z、调参、目标位置）   
        }
        /* 碰撞恢复期间由 motor_task 独占电机，跳过正常控制 */
        if(++cnt==10)
        {
            //HC05_Printf("%d,%d\n",(int16_t)(motor_L0.measure),(int16_t)(motor_R0.measure));

            HC05_Printf("%d,%d,%d\n",(int)(my_car.angle_manager.yaw * 100),(int)(100*my_car.pose_manager.x),my_car.pose_manager.finished/*(int)(100*my_car.pose_manager.y)*/);
            cnt =0;
        }
        if (collision_recover_step(HAL_GetTick())) {
            vTaskDelayUntil(&xLastWakeTime, PERIOD);
            continue;
        }
//        float speed =  (motor_L0.measure+motor_R0.measure)*1.0f/2;
        //motor_state=MOTOR_TRACK_TARGET;
        switch(motor_state){
            case MOTOR_STOP:
                motor_set_pwm(1,0);
                motor_set_pwm(2,0);
                //motor_set_pwm(3,0);
                //motor_set_pwm(4,0);
                break;
            case MOTOR_SEARCH_TARGET:   
                motor_set_pwm(1,-140);
                motor_set_pwm(2,140);
                //motor_set_pwm(3,150);
                //motor_set_pwm(4,-150);
                break;
            case MOTOR_TRACK_TARGET:
                Vision_x_pid_obj.measure = my_car.vision_manager.x;
                Vision_y_pid_obj.measure = my_car.vision_manager.y;
                Vision_x_pid_obj.target = my_car.speed_manager.x_target;
                Vision_y_pid_obj.target = my_car.speed_manager.y_target;

                float x_speed = positional_PID(&Vision_x_pid_obj, &Vision_x_pid_param);
                float y_speed = positional_PID(&Vision_y_pid_obj, &Vision_y_pid_param);
//                motor_L0.target =  speed - x_speed + y_speed;
//                motor_R0.target =  speed + x_speed + y_speed;
//                motor_L1.target =  speed - x_speed + y_speed;
//                motor_R1.target =  speed + x_speed + y_speed;
                motor_L0.target =  x_speed+ y_speed; ; //motor_L0.target =  -x_speed + y_speed;
                motor_R0.target =  -x_speed+ y_speed; ;// motor_R0.target =  x_speed + y_speed;*/
               // motor_L1.target =  -x_speed + y_speed;
                //motor_R1.target =  x_speed + y_speed;

//                char buf_1[64];
//                snprintf(buf_1, sizeof(buf_1), "speed:%.2f,%.2f\r\n", x_speed, y_speed);
//                HC05_SendData((uint8_t *)buf_1, strlen(buf_1));
                // 固定目标值的 PID 测试
                /*motor_L0.target =30;
                motor_R0.target = 30;
                motor_L1.target =0;
                motor_R1.target = 0;*/

                incremental_PID(&motor_L0,&L0_param);
                incremental_PID(&motor_R0,&R0_param);
                //incremental_PID(&motor_L1,&L1_param);
                //incremental_PID(&motor_R1,&R1_param);
                /*if ((my_car.vision_manager.x>my_car.speed_manager.x_target&&my_car.vision_manager.x-my_car.speed_manager.x_target<20)||(my_car.speed_manager.x_target>my_car.vision_manager.x&&my_car.speed_manager.x_target-my_car.vision_manager.x<20)) {
                    motor_L0.output=0;
                    motor_R0.output=0;
                }*/


                motor_set_pwm(1,(int32_t)motor_L0.output);
                motor_set_pwm(2,(int32_t)motor_R0.output);
                //motor_set_pwm(3,(int32_t)motor_L1.output);
                //motor_set_pwm(4,(int32_t)motor_R1.output);
                //motor_set_pwm(1,-600);
                //motor_set_pwm(2,-600);

                break;
            case MOTOR_BACKWARD:
                motor_set_pwm(1,-500);
                motor_set_pwm(2,-500);
                //motor_set_pwm(3,-600);
                //motor_set_pwm(4,-600);
                break;
            case MOTOR_BACKWARD_little:
                motor_set_pwm(1,-100);
                motor_set_pwm(2,-100);
                break;
            case MOTOR_FORWARD_FAST:
                motor_set_pwm(1,700);
                motor_set_pwm(2,700);
                //motor_set_pwm(3,200);
                //motor_set_pwm(4,200);
                break;
            case MOTOR_FORWARD:
                motor_set_pwm(1,500);
                motor_set_pwm(2,500);
                //motor_set_pwm(3,500);
                //motor_set_pwm(4,500);
                break;
            case MOTOR_POSITION:
                float turn_speed = 0;
                distance_pid_obj.bias = my_car.pose_manager.distance;
                gyroT_pid.bias = wrap(my_car.angle_manager.target_gyroT - my_car.angle_manager.yaw);
                float forward_speed = 0;
                if(my_car.pose_manager.target_x == -1 && my_car.pose_manager.target_y == -1)
                {
                    /* 纯转向模式：(-1,-1) 是"不前进"标记，只跟踪 target_yaw。
                     * 注意必须先更新 yaw_pid_obj.bias，否则 error_PID 会用上一拍的旧误差。 */
                    yaw_pid_obj.bias = wrap(my_car.angle_manager.target_yaw - my_car.angle_manager.yaw);
                    turn_speed = error_PID(&yaw_pid_obj, &yaw_pid_param);
                    forward_speed = 0;
                    if(fabsf(yaw_pid_obj.bias) <= 4.0f)
                    {
                        my_car.pose_manager.finished = 1;
                    }
                }
                else{
                if (gyroT_pid.bias>45 || gyroT_pid.bias<-45)
                {
                    distance_pid_obj.bias = 0;
                }
                if (distance_pid_obj.bias>=0 && distance_pid_obj.bias<=0.005)
                {
                    gyroT_pid.bias = 0;
                    distance_pid_obj.bias = 0;
                    yaw_pid_obj.bias = wrap(my_car.angle_manager.target_yaw - my_car.angle_manager.yaw);
                    turn_speed = error_PID(&yaw_pid_obj, &yaw_pid_param);
                }
                else{
                    turn_speed = error_PID(&gyroT_pid, &gyroT_pid_param);
                }
                forward_speed = error_PID(&distance_pid_obj, &distance_pid_param);
                if(distance_pid_obj.bias <= 0.025 && yaw_pid_obj.bias <= 4 && yaw_pid_obj.bias >= -4)
                {
                    my_car.pose_manager.finished = 1;
                }
            }

                motor_L0.target = forward_speed-turn_speed/2;
                motor_R0.target = forward_speed+turn_speed/2;

                incremental_PID(&motor_L0,&L0_param);
                incremental_PID(&motor_R0,&R0_param);
                motor_set_pwm(1,(int32_t)motor_L0.output);
                motor_set_pwm(2,(int32_t)motor_R0.output);

                break;
            case MOTOR_BLUETOOTH:
                /* 蓝牙速度模式：target 由蓝牙命令写入，内环增量式 PID 直接跟踪 */
                incremental_PID(&motor_L0,&L0_param);
                incremental_PID(&motor_R0,&R0_param);
                motor_set_pwm(1,(int32_t)motor_L0.output);
                motor_set_pwm(2,(int32_t)motor_R0.output);
                break;
            default:
                motor_state = MOTOR_STOP;
                break;
        }
        // 调试输出
       /* static uint16_t vofa_cnt = 0;
        vofa_cnt++;

        if(vofa_cnt >= 10)   // motor_task 周期 10ms，10 次约 100ms
        {
            vofa_cnt = 0;
            vofa_motor_send();
        }*/

        // 周期延时
        vTaskDelayUntil(&xLastWakeTime, PERIOD);

        /*r1_step_cnt++;
        if(r1_step_cnt >= 200)   // 200 * 10ms = 2s 切换一次
        {
            r1_step_cnt = 0;
            r1_target_idx++;

            if(r1_target_idx >= sizeof(r1_targets) / sizeof(r1_targets[0]))
            {
                r1_target_idx = 0;
            }

            r1_test_target = r1_targets[r1_target_idx];
        }*/
    }
}

void motor_set_state(MotorState state){
    taskENTER_CRITICAL();
    motor_state = state;
    taskEXIT_CRITICAL();
}



void motor_task_create(void){
    xTaskCreate((TaskFunction_t ) motor_task,// 任务入口函数
                (const char *)"motor_task",	  // 任务名称
                (uint32_t) motor_size,    // 任务堆栈大小
                (void* )NULL,                  // 传给任务的参数
                (UBaseType_t) motor_task_priority, // 任务优先级
                (TaskHandle_t *)&motor_handler ); // 任务句柄
}
