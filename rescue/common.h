#ifndef RESCUE_COMMON_H
#define RESCUE_COMMON_H
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
//队伍颜色：1：蓝色，2：红色

#define sin_work 0
#define sin_motor_idx 1
#define motor_work 1
#define grab_bolls_work 1//1
#define rudder_work 1//1
#define bluetooth_work 1
#define no_target 0
#define toward_target 1
#define disrupt_bolls 2
//初始位置，1=左边，2=右边
#define initial_pose 2
#define PI 3.14159265358979323846
struct Vision_Manager{
    uint8_t team;
    uint8_t target_num;
    uint8_t is_valid;
    uint8_t target_type;
    uint8_t target_mode;
    uint8_t sequence;
    uint8_t confidence;
    uint8_t hazard_count;
    uint8_t normal_count;
    uint8_t core_count;
    uint8_t patient_count;
    uint8_t frame_valid;
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint32_t updated_ms;
};

struct Speed_Manager{
    float L0_speed;
    float R0_speed;
    float L1_speed;
    float R1_speed;
    float Car_speed;
    uint16_t  x_target;
    uint16_t  y_target;
};

struct Angle_Manager{
    float yaw;
    float last_yaw;
    float target_yaw;
    float target_gyroT;//移动中的偏航角
    uint32_t update_ms;
    uint8_t valid;
};

struct Pose_Manager{
    float x;
    float y;
    float target_x;
    float target_y;
    float speed_mps;
    uint8_t imu_fresh;
    float distance;
    uint8_t valid;
    uint8_t finished;
};

struct Collision_Manager{
    uint8_t reason;
    uint8_t detected;
    uint16_t stall_ticks;
    uint32_t event_count;
};

struct MY_Car{
    struct Vision_Manager vision_manager;
    struct Speed_Manager speed_manager;
    struct Angle_Manager angle_manager;
    struct Pose_Manager pose_manager;
    struct Collision_Manager collision_manager;
    uint8_t track_mode;
    uint8_t rudder_mode;
};

volatile extern struct MY_Car my_car;

#endif //RESCUE_COMMON_H
