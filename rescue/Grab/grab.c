#include "grab.h"
#include "collision_guard.h"
#include "cmsis_os.h"
#include <stdint.h>

TaskHandle_t rescue_grab_handler;

/* ===== 救援夹取状态机 ===== */
typedef enum {
    RESCUE_GRAB_WAIT_TEAM = 0,      // 等待队伍颜色设置
    RESCUE_GRAB_SEARCH,             // 原地旋转搜索目标
    RESCUE_GRAB_APPROACH_TARGET,    // 视觉闭环接近目标
    RESCUE_GRAB_CONFIRM_CARGO,      // 夹紧后确认夹持类别和数量
    RESCUE_GRAB_TARGET_LOST,        // 目标丢失，开启位置环
    RESCUE_GRAB_APPROACH_ZONE,      // 视觉闭环接近对应投放区
    RESCUE_GRAB_PRECHECK,           // 投放前预检：记录投放区当前数量作为基线
    RESCUE_GRAB_PUSH,               // 前推，把目标推进投放区
    RESCUE_GRAB_VERIFY,             // 验证投放是否成功
    RESCUE_GRAB_STOP                // 停止模式    
} RescueGrabState;

/* ============================================================================
 * 主流程状态流转：
 *
 *   WAIT_TEAM（等队伍）
 *        │ 队伍已设置
 *        ▼
 *   SEARCH（旋转找目标）──看到可抓目标──→ APPROACH_TARGET（追目标）
 *        ▲                                      │ 目标 y 足够大（贴到跟前）
 *        │                                      ▼
 *        │                       CONFIRM_CARGO（夹紧并确认类别/数量）
 *        │                         │正确                 │错误/超时
 *        │                         ▼                      └──→ SEARCH
 *        │                  APPROACH_ZONE（找投放区）
 *        │                                      │ 投放区 y 足够大
 *        │                                      ▼
 *        │                                  PRECHECK（记录投放区已有数量）
 *        │                                      │ 连续确认若干帧
 *        │                                      ▼
 *        │                                   PUSH（前推 700ms）
 *        │                                      ▼
 *        └──────────投放成功──────────────── VERIFY（对比数量是否增加）
 *
 * 任一阶段只要视觉里出现"危险物"，都会强制停车并退回 SEARCH。
 * ==========================================================================*/

static volatile uint8_t configured_team = 0U;          // 已设置的队伍：0=未设置，1=蓝，2=红
static RescueGrabState grab_state = RESCUE_GRAB_WAIT_TEAM;  // 当前状态
static uint8_t selected_type = MAIXCAM_TARGET_NONE;    // 本轮锁定的目标类型
static uint8_t first_normal_delivered = 0U;            // 是否已投放过第一个普通物资
static uint8_t lost_target_ticks = 0U;                 // 目标连续丢失的周期计数
static uint8_t verify_ticks = 0U;                      // 已验证的视觉帧计数
static uint8_t cargo_confirm_ticks = 0U;               // 夹持确认通过的连续帧数
static uint8_t cargo_error_ticks = 0U;                // 夹持确认失败的连续帧数
static uint8_t confirm_baseline = 0U;                  // 投放前投放区内该类型的数量基线
static uint8_t confirm_last_sequence = 0U;             // 上一次处理过的帧序号（用于去重）
static uint8_t confirm_sequence_valid = 0U;            // 帧序号是否有效（1=已有基准可比较）
static uint32_t state_started_ms = 0U;                 // 当前状态的进入时刻（毫秒）
static float yaw_total = 0.0f;                        // 总角度（度）
static uint8_t yaw_full_circle = 0U;                  // 完整圈数
static uint8_t SEARCH_angle_init = 0U;                // 搜索角度是否已初始化
static uint8_t lost_mode = 0U;                      // 目标丢失模式
static uint8_t last_match = 0U;                    // 上一次匹配的目标类型
void calc_circle(float curr_yaw)
{
    static float yaw_last = 0.0f;
    float delta = curr_yaw - yaw_last;

    // 处理0/360跳变
    if(delta > 180.0f)      // 上一帧接近0，现在接近360，逆时针跨零点
        delta -= 360.0f;
    else if(delta < -180.0f)// 上一帧接近360，现在接近0，顺时针跨零点
        delta += 360.0f;

    yaw_total += delta;
    // 顺时针一圈 diff ≤ -360；逆时针一圈 diff ≥ +360
    if( yaw_total >= 360.0f ||  yaw_total<= -360.0f)
    {
        yaw_full_circle++;  // 置位一圈标记
        yaw_total=0;
    }
    yaw_last = curr_yaw;
}
/* 切换状态：统一重置各计数器与计时起点，避免上一状态的数据污染新状态 */
static void enter_state(RescueGrabState state)
{
    grab_state = state;
    state_started_ms = HAL_GetTick();
    lost_target_ticks = 0U;
    verify_ticks = 0U;
    confirm_sequence_valid = 0U;
    cargo_confirm_ticks = 0U;
    cargo_error_ticks = 0U;       // 夹持失败计数不能跨状态残留，否则下次确认会被立刻判失败
    last_match = 0U;              // 上一次匹配结果清零，重新开始"连续多帧一致"的判断
    confirm_baseline = 0U;        // 投放区数量基线只在 PRECHECK 重新采样
    lost_mode = 0U;               // 目标丢失后的去向由新状态重新决定
    SEARCH_angle_init = 0U;       // 再次进入搜索时重新把云台抬到初始角度
    yaw_full_circle = 0U;
    yaw_total = 0.0f;
}

/* 按目标类型取出视觉回传的对应数量（普通/核心/伤员） */
static uint8_t target_count_for_type(uint8_t type)
{
    switch (type) {
    case MAIXCAM_TARGET_NORMAL_MATERIAL:
        return my_car.vision_manager.normal_count;
    case MAIXCAM_TARGET_CORE_MATERIAL:
        return my_car.vision_manager.core_count;
    case MAIXCAM_TARGET_PATIENT:
        return my_car.vision_manager.patient_count;
    default:
        return 0U;
    }
}
static uint8_t cargo_matches_selected_target(void)
{
    if(my_car.vision_manager.hazard_count != 0U)
    {
        return 0U;
    }
    if(my_car.vision_manager.target_num == 0U)
    {
        return 0U;
    }
    if(first_normal_delivered == 0U)
    {
        if(my_car.vision_manager.normal_count == 1U && my_car.vision_manager.core_count == 0U && my_car.vision_manager.patient_count == 0U)
        {
            selected_type = MAIXCAM_TARGET_NORMAL_MATERIAL;
            return 1U;
        }
        else
        {
            return 0U;
        }
    }
    if(my_car.vision_manager.patient_count == 1U && my_car.vision_manager.core_count == 0U && my_car.vision_manager.normal_count == 0U)
    {
        selected_type = MAIXCAM_TARGET_PATIENT;
        return 2U;
    }
    else if(my_car.vision_manager.core_count + my_car.vision_manager.normal_count <= 3U && my_car.vision_manager.patient_count == 0U)
    {
        selected_type = MAIXCAM_TARGET_CORE_MATERIAL;
        return 3U;
    }
    return 0U;
}

/* 目标类型 → 对应的投放区视觉模式（伤员送伤员区，其余送物资区） */
static uint8_t zone_mode_for_type(uint8_t type)
{
    if (type == MAIXCAM_TARGET_PATIENT) {
        return MAIXCAM_MODE_PATIENT_ZONE;
    }
    return MAIXCAM_MODE_MATERIAL_ZONE;
}


/* 投放结果验证：确认目标确实被放进投放区 */
static void verify_delivery(void)
{
    /* 只要画面里还有危险物，就不算投放成功（可能是危险物混入） */
    if (my_car.vision_manager.hazard_count != 0U) {
        return;
    }
    /* 投放区内该类型的数量比投放前多 → 说明投进去了 */
    if (target_count_for_type(selected_type) > confirm_baseline) {
        if (selected_type == MAIXCAM_TARGET_NORMAL_MATERIAL) {
            first_normal_delivered = 1U;      // 记录"第一个普通物资已投放"
        }
        enter_state(RESCUE_GRAB_SEARCH);      // 回去找下一个目标
        Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
    }
}

/* 设置队伍颜色（1=蓝，2=红），同时同步给视觉模块 */
void Grab_SetTeam(uint8_t team)
{
    if (team == 1U || team == 2U) {
        configured_team = team;
        (void)Maixcam_SetTeam(team);
    }
}

/* 读取当前队伍设置 */
uint8_t Grab_GetTeam(void)
{
    return configured_team;
}

/* ============================================================================
 * 救援夹取任务主体：10ms 周期，按状态机推进
 * ==========================================================================*/
void rescue_grab_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)pvParameters;

    /* 若头文件里预设了队伍，这里直接应用，省去外部调用 Grab_SetTeam */
#if (GRAB_TEAM_DEFAULT == 1U) || (GRAB_TEAM_DEFAULT == 2U)
    Grab_SetTeam(GRAB_TEAM_DEFAULT);
#endif
    open_posture();                 // 上电先把夹爪张开
    motor_set_state(MOTOR_STOP);

    for (;;) {
        uint32_t now = HAL_GetTick();
        /* 视觉是否新鲜：超时未更新就不采信任何目标信息 */
        uint8_t vision_fresh = Maixcam_IsFresh(GRAB_VISION_TIMEOUT_MS);
        uint8_t target_valid = vision_fresh && my_car.vision_manager.is_valid;
        /* 蓝牙手动模式优先：此时状态机完全不碰电机，避免把 A 命令设好的状态覆盖掉 */
        if (bluetooth_work && HC05_InManualMode()) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GRAB_TASK_PERIOD_MS));
            continue;
        }
        /* 碰撞保护：检测与恢复都在 motor_task 里完成（motor_task 独占电机），
         * 这里只负责在碰撞期间跳过状态机，避免继续下发运动指令。 */
        if (collision_guard_blocked()) {
            motor_set_state(MOTOR_STOP);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GRAB_TASK_PERIOD_MS));
            continue;
        }

        /* 危险物熔断：除等待/搜索/预检/验证外，任何阶段看到危险物都立即放弃当前动作 */
        // if (vision_fresh && my_car.vision_manager.hazard_count != 0U &&
        //     grab_state != RESCUE_GRAB_WAIT_TEAM &&
        //     grab_state != RESCUE_GRAB_SEARCH &&
        //     grab_state != RESCUE_GRAB_PRECHECK &&
        //     grab_state != RESCUE_GRAB_VERIFY) {
        //     motor_set_state(MOTOR_STOP);
        //     command_search_mode();
        //     enter_state(RESCUE_GRAB_SEARCH);
        // }

        switch (grab_state) {
        /* --- 等待队伍颜色：没设置就原地停着 --- */
        case RESCUE_GRAB_WAIT_TEAM:
            motor_set_state(MOTOR_STOP);
            if (configured_team == 1U || configured_team == 2U) {
                (void)Maixcam_SetTeam(configured_team);
                Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
                enter_state(RESCUE_GRAB_SEARCH);
            }
            break;

        /* --- 搜索：原地旋转，等待视觉给出可抓目标 --- */
        case RESCUE_GRAB_SEARCH:
            if (SEARCH_angle_init==0) {
                set_vision_angle(ccr_2_max);
                SEARCH_angle_init=1;
            }
            if(first_normal_delivered == 0U)
            {
                Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
                my_car.speed_manager.y_target = GRAB_APPROACH_Y;
                /* 视觉失效，或视觉端还没切到搜索模式 → 先停着等 */
                if (!vision_fresh || my_car.vision_manager.target_mode != MAIXCAM_MODE_NORMAL_MATERIAL) {
                    motor_set_state(MOTOR_STOP);
                    break;
                }
            }
            else
            {
                Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
                my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
                my_car.speed_manager.y_target = GRAB_APPROACH_Y;
                /* 视觉失效，或视觉端还没切到搜索模式 → 先停着等 */
                if (!vision_fresh || my_car.vision_manager.target_mode != MAIXCAM_MODE_RESCUE_TARGETS) {
                    motor_set_state(MOTOR_STOP);
                    break;
                }   
            }
            /* 画面里有危险物 → 不追，原地停 */
            // if (my_car.vision_manager.hazard_count != 0U) {
            //     motor_set_state(MOTOR_STOP);
            //     break;
            // }
            if (target_valid) {
                motor_set_state(MOTOR_STOP);
                /* 锁定当前看到的目标类型，进入"接近目标"状态 */
                selected_type = my_car.vision_manager.target_type;
                // if(selected_type == MAIXCAM_TARGET_PATIENT)
                // {
                //     Set_Vision_Mode(MAIXCAM_MODE_PATIENT);
                // }
                // else if(selected_type == MAIXCAM_TARGET_CORE_MATERIAL)
                // {
                //     Set_Vision_Mode(MAIXCAM_MODE_CORE_MATERIAL);
                // }
                // else if(selected_type == MAIXCAM_TARGET_NORMAL_MATERIAL)
                // {
                //     Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                // }
                if(first_normal_delivered == 0U)
                {
                    Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                }
                else
                {
                    Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
                }
                enter_state(RESCUE_GRAB_APPROACH_TARGET);   
            } else {
                motor_set_state(MOTOR_SEARCH_TARGET);   // 没有合适目标 → 继续旋转搜索
                calc_circle(my_car.angle_manager.yaw);
                if(yaw_full_circle >= 2U)
                {
                    yaw_full_circle = 0;
                    uint16_t find_angle_target = rudder_cur_ccr[2] + 2;
                    set_vision_angle(find_angle_target);
                    if (find_angle_target > 180) {
                    // 压到 180° 仍无球，才承认找不到，转去找交叉线
                        find_angle_target = 180;
                        SEARCH_angle_init=0;//
                        set_vision_angle(find_angle_target);
                        enter_state(RESCUE_GRAB_TARGET_LOST);
                        lost_mode = 1U;     // enter_state 会清标志位，必须在它之后再赋值
                    }
                }
            }
            break;

        case RESCUE_GRAB_TARGET_LOST:
            switch (lost_mode) {
                case 0U: my_car.pose_manager.target_x = CENTER_X;
                         my_car.pose_manager.target_y = CENTER_Y;
                    break;
                case 1U: my_car.pose_manager.target_x = CENTER_X;
                         my_car.pose_manager.target_y = CENTER_Y;
                    break;
                case 2U: my_car.pose_manager.target_x = ZONE_X;
                         my_car.pose_manager.target_y = PATIENT_ZONE_Y;       
                    break;
                case 3U: my_car.pose_manager.target_x = ZONE_X;
                         my_car.pose_manager.target_y = MATERIAL_ZONE_Y;
                    break;
                default: break;
            }
            my_car.pose_manager.finished =0;
            motor_set_state(MOTOR_POSITION);
            while(my_car.pose_manager.finished == 0);
            if(my_car.pose_manager.finished)
            {
                enter_state(RESCUE_GRAB_SEARCH);
            }
            break;
        /* --- 接近目标：视觉闭环追踪，贴到足够近为止 --- */
        case RESCUE_GRAB_APPROACH_TARGET:
            if(first_normal_delivered == 0U)
            {
                Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
                my_car.speed_manager.y_target = GRAB_APPROACH_Y;
                /* 视觉失效，或视觉端还没切到搜索模式 → 先停着等 */
                if (!vision_fresh || my_car.vision_manager.target_mode != MAIXCAM_MODE_NORMAL_MATERIAL) {
                    motor_set_state(MOTOR_STOP);
                    break;
                }
            }
            else
            {
                Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
                my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
                my_car.speed_manager.y_target = GRAB_APPROACH_Y;
                /* 视觉失效，或视觉端还没切到搜索模式 → 先停着等 */
                if (!vision_fresh || my_car.vision_manager.target_mode != MAIXCAM_MODE_RESCUE_TARGETS) {
                    motor_set_state(MOTOR_STOP);
                    break;
                }   
            }
            /* 目标丢失，或目标类型变了（比如换成了别的东西） */
            if (!target_valid) {
                 if (++lost_target_ticks >= GRAB_LOST_TARGET_TICKS) {   // 连续丢失超时 → 放弃
                     motor_set_state(MOTOR_STOP);
                     Set_Vision_Mode(MAIXCAM_MODE_RESCUE_TARGETS);
                     enter_state(RESCUE_GRAB_SEARCH);
                 }
                 break;
             }
            lost_target_ticks = 0U;                 // 本帧看到目标，丢失计数清零
            my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
            my_car.speed_manager.y_target = GRAB_APPROACH_Y;
            motor_set_state(MOTOR_TRACK_TARGET);    // 视觉 x/y 双 PID 闭环前进
            /* 到达抓取距离后停车夹紧，再进入独立的夹持确认状态。 */
            if (my_car.vision_manager.y > 155 && rudder_cur_ccr[2] < ccr_2_min)
                {
                    uint16_t new_target = rudder_cur_ccr[2] + 1;
                    if (new_target > ccr_2_min)
                        new_target = ccr_2_min;
                    set_vision_angle(new_target);
                }
                // 开爪预动作：球进入水平中心区、y>85、舵机已压到 145° 以上 → 提前张爪
                if(my_car.vision_manager.x > 105 && my_car.vision_manager.x < 145 && my_car.vision_manager.y > 85 && rudder_cur_ccr[2] >= 145 ){
                    open_posture();
                }
                // 夹取判定（双重条件互印证：球在画面里"够近" 且 舵机确实"够低"）
                // 条件A：y>170 且 舵机>=175
                // 条件B：舵机>=190（到底） 且 y>160
                if(my_car.vision_manager.x > 105 && my_car.vision_manager.x < 145 && ((my_car.vision_manager.y > 170 && rudder_cur_ccr[2] >=175)||(rudder_cur_ccr[2] >=190&&my_car.vision_manager.y>160 ))){  //185 170
                    motor_set_pwm(1,50);      // 慢速再蹭一小段，确保夹爪位置到位
                    motor_set_pwm(2,50);
                    my_delay_ms(150);
                    grab_posture();           // 合爪
                    enter_state(RESCUE_GRAB_CONFIRM_CARGO);
                }
            // if (my_car.vision_manager.y >= GRAB_TARGET_NEAR_Y) {
            //     motor_set_state(MOTOR_STOP);
            //     open_posture();

            //     my_delay_ms(100);
            //     grab_posture();
            //     Set_Vision_Mode(MAIXCAM_MODE_CONFIRM_CARGO);
            //     enter_state(RESCUE_GRAB_CONFIRM_CARGO);
            // }
            break;

        /* --- 夹持确认：只统计确认 ROI 的类别数，连续多帧正确才继续 --- */
        case RESCUE_GRAB_CONFIRM_CARGO:
            motor_set_state(MOTOR_STOP);
            set_vision_angle(ccr_2_max);
            my_delay_ms(500);
            grab_posture();                    // 确认期间保持夹紧，避免物品掉落
            Set_Vision_Mode(MAIXCAM_MODE_CONFIRM_CARGO);
            if (vision_fresh && my_car.vision_manager.target_mode == MAIXCAM_MODE_CONFIRM_CARGO &&
                (!confirm_sequence_valid || my_car.vision_manager.sequence != confirm_last_sequence)) {
                confirm_last_sequence = my_car.vision_manager.sequence;
                confirm_sequence_valid = 1U;
                uint8_t cur_match = cargo_matches_selected_target();
                if (cur_match && last_match == cur_match) 
                {
                    cargo_confirm_ticks++; 
                    cargo_error_ticks = 0U;
                } 
                else {
                    cargo_confirm_ticks = 0U;
                    cargo_error_ticks++;
                }
                last_match = cur_match;
                if(cargo_error_ticks >= GRAB_VERIFY_FRAMES)
                {
                    open_big_posture();                 // 大张口
                    motor_set_state(MOTOR_BACKWARD);
                    my_delay_ms(500);                   // 后退把球丢掉
                    motor_set_state(MOTOR_STOP);
                    grab_posture();       
                    motor_set_state(MOTOR_FORWARD);
                    my_delay_ms(700);
                    motor_set_state(MOTOR_BACKWARD);
                    my_delay_ms(300);
                    enter_state(RESCUE_GRAB_SEARCH);
                    break;
                }
                if (cargo_confirm_ticks >= GRAB_VERIFY_FRAMES)
                {
                    cargo_confirm_ticks = 0U;
                    Set_Vision_Mode(zone_mode_for_type(selected_type));
                    enter_state(RESCUE_GRAB_APPROACH_ZONE);
                }
            }
            break;

        /* --- 接近投放区：先找到投放区，再闭环靠近 --- */
        case RESCUE_GRAB_APPROACH_ZONE:
            my_car.pose_manager.finished=0;
            my_car.angle_manager.target_yaw = 180;
            my_car.pose_manager.target_x = my_car.pose_manager.x;
            my_car.pose_manager.target_y = CENTER_Y;
            motor_set_state(MOTOR_POSITION);
            while(my_car.pose_manager.finished == 0);
            set_vision_angle(ccr_2_max);
            Set_Vision_Mode(zone_mode_for_type(selected_type));
            if (!vision_fresh || my_car.vision_manager.target_mode != zone_mode_for_type(selected_type)) {
                motor_set_state(MOTOR_STOP);
                break;
            }
            if (!target_valid) {                    // 还没看到投放区
                motor_set_state(MOTOR_SEARCH_TARGET);   // 原地旋转继续找
                calc_circle(my_car.angle_manager.yaw);
                if(yaw_full_circle >= 2U)
                {
                    enter_state(RESCUE_GRAB_TARGET_LOST);   // 先切状态（会清标志），再决定去向
                    if(my_car.vision_manager.target_type == MAIXCAM_MODE_PATIENT_ZONE)
                    lost_mode = 2U;
                    else if(my_car.vision_manager.target_type == MAIXCAM_MODE_MATERIAL_ZONE)
                    lost_mode = 3U;
                }
                break;
            }
            if(my_car.vision_manager.x < 105 )
            {
                my_car.pose_manager.target_x = -1;
                my_car.pose_manager.target_y = -1;
                my_car.angle_manager.target_yaw = 90;
                my_car.pose_manager.finished=0;
                motor_set_state(MOTOR_POSITION);
                while(my_car.pose_manager.finished == 0);
                my_car.angle_manager.target_yaw = 180;
                break;
            }
            if(my_car.vision_manager.x > 145)
            {
                my_car.pose_manager.target_x = -1;
                my_car.pose_manager.target_y = -1;
                my_car.angle_manager.target_yaw = -90;
                my_car.pose_manager.finished=0;
                motor_set_state(MOTOR_POSITION);
                while(my_car.pose_manager.finished == 0);
                my_car.angle_manager.target_yaw = 180;
                break;
            }
            my_car.speed_manager.x_target = GRAB_IMAGE_CENTER_X;
            my_car.speed_manager.y_target = GRAB_ZONE_APPROACH_Y;
            motor_set_state(MOTOR_TRACK_TARGET);    // 闭环靠近投放区
            /* 已经到达投放位置 → 准备预检 */
            if (my_car.vision_manager.y >= GRAB_ZONE_NEAR_Y) {
                motor_set_state(MOTOR_STOP);
                //Set_Vision_Mode(MAIXCAM_MODE_CONFIRM_CARGO);
                //enter_state(RESCUE_GRAB_PRECHECK);
                enter_state(RESCUE_GRAB_PUSH);
            }
            break;
        /* --- 前推：一段固定时长的开环前进，把目标推进投放区 --- */
        case RESCUE_GRAB_PUSH:
            if(selected_type == MAIXCAM_MODE_PATIENT_ZONE)
                my_car.angle_manager.target_yaw = 150;
            else
            {
                my_car.angle_manager.target_yaw = -150;
            }
            my_car.pose_manager.finished=0;
            motor_set_state(MOTOR_POSITION);
            while(my_car.pose_manager.finished == 0);
            motor_set_state(MOTOR_FORWARD);
            my_delay_ms(500);
            motor_set_state(MOTOR_STOP);
            open_posture();
            my_delay_ms(100);
            grab_posture();
            motor_set_state(MOTOR_BACKWARD);
            my_delay_ms(500);
            motor_set_state(MOTOR_STOP);
            if(first_normal_delivered == 0U)
                first_normal_delivered = 1U;
            enter_state(RESCUE_GRAB_SEARCH);
            break;
        /* --- 预检：在投放前记录投放区已有的该类型数量，作为验证基线 --- */
        case RESCUE_GRAB_PRECHECK:
            motor_set_state(MOTOR_STOP);
            Set_Vision_Mode(MAIXCAM_MODE_CONFIRM_CARGO);
            /* 只处理"新的一帧"（帧序号变化），避免同一帧被重复计数 */
            if (vision_fresh && my_car.vision_manager.target_mode == MAIXCAM_MODE_CONFIRM_CARGO &&
                (!confirm_sequence_valid ||
                 my_car.vision_manager.sequence != confirm_last_sequence)) {
                confirm_last_sequence = my_car.vision_manager.sequence;
                confirm_sequence_valid = 1U;
                if (++verify_ticks >= GRAB_VERIFY_FRAMES) {
                    confirm_baseline = target_count_for_type(selected_type);   // 记录基线
                    enter_state(RESCUE_GRAB_PUSH);
                }
            }
            /* 预检超时 → 放弃本次投放 */
            if ((uint32_t)(now - state_started_ms) >= GRAB_VERIFY_TIMEOUT_MS) {
                if(first_normal_delivered == 0U)
                    Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                else
                    Set_Vision_Mode(MAIXCAM_MODE_PATIENT);
                enter_state(RESCUE_GRAB_SEARCH);
            }
            break;

        /* --- 验证：观察投放区数量是否增加，判断投放是否成功 --- */
        case RESCUE_GRAB_VERIFY:
            motor_set_state(MOTOR_STOP);
            Set_Vision_Mode(MAIXCAM_MODE_CONFIRM_CARGO);
            if (vision_fresh && my_car.vision_manager.target_mode == MAIXCAM_MODE_CONFIRM_CARGO &&
                (!confirm_sequence_valid ||
                 my_car.vision_manager.sequence != confirm_last_sequence)) {
                confirm_last_sequence = my_car.vision_manager.sequence;
                confirm_sequence_valid = 1U;
                if (verify_ticks < GRAB_VERIFY_FRAMES) {
                    verify_ticks++;                 // 继续累积验证帧数
                }
                if (verify_ticks >= GRAB_VERIFY_FRAMES) {
                    verify_delivery();              // 帧数够了 → 判定是否投放成功
                    /* 仍未成功（verify_delivery 没有切状态）且已超时 → 放弃重来 */
                    if (grab_state == RESCUE_GRAB_VERIFY &&
                        (uint32_t)(now - state_started_ms) >= GRAB_VERIFY_TIMEOUT_MS) {
                        if(first_normal_delivered == 0U)
                        Set_Vision_Mode(MAIXCAM_MODE_NORMAL_MATERIAL);
                        else
                         Set_Vision_Mode(MAIXCAM_MODE_PATIENT);
                        enter_state(RESCUE_GRAB_SEARCH);
                    }
                }
            }
            break;

        /* --- 停止 / 非法状态 --- */
        case RESCUE_GRAB_STOP:
        default:
            motor_set_state(MOTOR_STOP);
            break;
        }

        /* 固定 10ms 周期 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GRAB_TASK_PERIOD_MS));
    }
}

/* 创建救援夹取任务 */
void rescue_grab_task_create(void)
{
    xTaskCreate((TaskFunction_t)rescue_grab_task,     // 任务入口
                (const char *)"rescue_grab",           // 任务名称
                (uint32_t)grab_size,                   // 堆栈大小
                (void *)NULL,                          // 传给任务的参数
                (UBaseType_t)grab_task_priority,       // 优先级
                (TaskHandle_t *)&rescue_grab_handler); // 任务句柄
}
