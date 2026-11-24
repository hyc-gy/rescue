#include "grab.h"
#include "motor.h"
#include "hc05.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

/* 本文件只决定下一业务动作；电机/PID/舵机/起步/搜索/位移由绑定的执行端实现。
 * 每个动作带id，必须收到对应的真实完成反馈才推进，不用固定延时假定完成。
 * 无执行适配时任务持续停车，不能把接口未接通当成动作成功。
 */
TaskHandle_t rescue_grab_handler;
static GrabActionExecutor action_executor;
static GrabActionRequest request, published_request;
static GrabStage stage;
static uint8_t configured_team = GRAB_TEAM_DEFAULT;
static uint8_t vision_mode, start_requested, pose_done, camera_reset_seen;
static uint8_t first_normal_delivered, execution_failed, can_hold_speed;
static uint8_t good_frames, bad_frames, empty_timing, last_confirm_sequence;
static uint32_t camera_reset_baseline, last_confirm_frame, empty_since_ms, navigation_baseline;
static uint32_t pose_ready_since_ms;
static CargoCounts confirmed_cargo, previous_cargo, published_cargo;
static struct {
    uint32_t id, completed_ms;
    uint8_t pending, event;
    float angle;
} feedback;
static uint8_t delivery_confirmed;

static uint32_t lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

static void unlock(uint32_t saved) { __set_PRIMASK(saved); }

static uint8_t vision_fresh(uint32_t now, const MaixcamObservation *v)
{
    return v->data.frame_valid && v->data.team == configured_team &&
           v->data.target_mode == vision_mode &&
           (uint32_t)(now - v->data.updated_ms) <= GRAB_VISION_TIMEOUT_MS;
}

static uint8_t navigation_fresh(uint32_t now, const MaixcamNavigation *n)
{
    return n->frame_valid && n->frame_id != navigation_baseline &&
           n->team == configured_team && n->mode == vision_mode &&
           (uint32_t)(now - n->updated_ms) <= GRAB_VISION_TIMEOUT_MS;
}

static uint8_t cargo_equal(CargoCounts a, CargoCounts b)
{
    return a.normal == b.normal && a.core == b.core &&
           a.patient == b.patient && a.hazard == b.hazard;
}

static uint8_t cargo_allowed(CargoCounts c)
{
    if (c.hazard) return 0U;
    if (!first_normal_delivered)
        return c.normal == 1U && c.core == 0U && c.patient == 0U;
    if (c.patient)
        return c.patient == 1U && c.normal == 0U && c.core == 0U;
    uint16_t count = (uint16_t)c.normal + c.core;
    return count >= 1U && count <= 3U;
}

static void new_action(GrabAction action, GrabCameraPose pose)
{
    uint32_t next = request.id + 1U;
    if (next == 0U) next = 1U;
    memset(&request, 0, sizeof(request));
    request.id = next;
    request.action = action;
    request.camera_pose = pose;
    request.stage = stage;
    request.team = configured_team;
    request.start_position = GRAB_START_POSITION_DEFAULT;
    request.cargo = confirmed_cargo;
    execution_failed = 0U;
    feedback.pending = 0U;
}

static void ensure_action(GrabAction action, GrabCameraPose pose)
{
    if (request.action != action) new_action(action, pose);
}

static void clear_confirmation(void)
{
    good_frames = bad_frames = empty_timing = 0U;
    previous_cargo = (CargoCounts){0};
}

/* mode=0的实际新帧回显是清除相机跟线历史的屏障。
 * 不能只发命令就假定相机已处理；需要回显后才重进确认/回基地模式。 */
static void begin_pose(GrabStage next, GrabCameraPose pose, uint32_t frame_id)
{
    stage = next;
    vision_mode = MAIXCAM_MODE_OBSERVE;
    camera_reset_baseline = frame_id;
    camera_reset_seen = pose_done = 0U;
    clear_confirmation();
    can_hold_speed = 0U;
    new_action(GRAB_ACTION_SET_CAMERA_POSE, pose);
}

static void begin_object_pose(uint32_t frame_id)
{
    confirmed_cargo = (CargoCounts){0};
    begin_pose(GRAB_SETTING_OBJECT_POSE, GRAB_POSE_FIND_OBJECT, frame_id);
}

static uint8_t object_mode(void)
{
    return first_normal_delivered ? MAIXCAM_MODE_RESCUE_TARGETS : MAIXCAM_MODE_NORMAL_MATERIAL;
}

static void begin_search(void)
{
    stage = GRAB_SEARCHING_OBJECT;
    vision_mode = object_mode();
    new_action(GRAB_ACTION_SEARCH_OBJECT, GRAB_POSE_FIND_OBJECT);
    request.suspended = 1U; /* 等待目标模式的新帧，不能沿用观察模式的旧结果。 */
}

static uint8_t object_visible(const MaixcamObservation *v)
{
    uint8_t type = v->data.target_type;
    return v->data.is_valid && v->data.target_num &&
           (type == MAIXCAM_TARGET_NORMAL_MATERIAL ||
            (first_normal_delivered && (type == MAIXCAM_TARGET_CORE_MATERIAL ||
                                        type == MAIXCAM_TARGET_PATIENT)));
}

void Grab_Init(void)
{
    uint32_t saved = lock();
    memset(&request, 0, sizeof(request));
    memset(&feedback, 0, sizeof(feedback));
    confirmed_cargo = (CargoCounts){0};
    first_normal_delivered = start_requested = delivery_confirmed = 0U;
    pose_done = camera_reset_seen = 0U;
    MaixcamObservation baseline;
    Maixcam_GetObservation(&baseline);
    camera_reset_baseline = baseline.frame_id;
    last_confirm_frame = navigation_baseline = 0U;
    can_hold_speed = 0U;
    clear_confirmation();
    vision_mode = MAIXCAM_MODE_OBSERVE;
    stage = GRAB_PREPARING_START;
    new_action(GRAB_ACTION_PREPARE_START, GRAB_POSE_START);
    published_request = request;
    published_cargo = confirmed_cargo;
    unlock(saved);
}

void Grab_BindExecutor(GrabActionExecutor executor) { action_executor = executor; }

void Grab_Start(void)
{
    uint32_t saved = lock();
    if (stage == GRAB_WAITING_START) start_requested = 1U;
    unlock(saved);
}

void Grab_SetTeam(uint8_t team)
{
    if (team == 1U || team == 2U) {
        configured_team = team;
        (void)Maixcam_SetTeam(team);
    }
}

uint8_t Grab_GetTeam(void) { return configured_team; }
GrabStage Grab_GetStage(void) { return (GrabStage)published_request.stage; }
uint8_t Grab_GetVisionMode(void) { return vision_mode; }
uint8_t Grab_IsFirstNormalDelivered(void) { return first_normal_delivered; }
CargoCounts Grab_GetConfirmedCargo(void)
{
    uint32_t saved = lock();
    CargoCounts out = published_cargo;
    unlock(saved);
    return out;
}

void Grab_GetActionRequest(GrabActionRequest *out)
{
    uint32_t saved = lock();
    *out = published_request;
    unlock(saved);
}

void Grab_ReportAction(uint32_t id, GrabFeedback event, float base_angle_deg)
{
    if (event < GRAB_FEEDBACK_DONE || event > GRAB_FEEDBACK_FAILED) return;
    if (event == GRAB_FEEDBACK_BASE_ALIGNED &&
        (!isfinite(base_angle_deg) || base_angle_deg < -180.0f || base_angle_deg > 180.0f)) return;
    uint32_t saved = lock();
    if (id == published_request.id && (event != GRAB_FEEDBACK_BASE_ALIGNED ||
        (published_request.action == GRAB_ACTION_FOLLOW_BASE && !published_request.suspended))) {
        feedback.id = id;
        feedback.event = event;
        feedback.completed_ms = HAL_GetTick();
        feedback.angle = base_angle_deg;
        feedback.pending = 1U;
    }
    unlock(saved);
}

void Grab_ReportDelivery(uint32_t id, uint8_t delivered)
{
    uint32_t saved = lock();
    if (published_request.stage == GRAB_WAITING_DELIVERY && published_request.id == id && delivered == 1U)
        delivery_confirmed = 1U;
    unlock(saved);
}

void Grab_Process(uint32_t now, const MaixcamObservation *v, const MaixcamNavigation *n)
{
    uint8_t event = 0U;
    float angle = 0.0f;
    uint32_t completed_ms = 0U;
    uint32_t saved = lock();
    if (feedback.pending && feedback.id == request.id) {
        event = feedback.event;
        angle = feedback.angle;
        completed_ms = feedback.completed_ms;
        feedback.pending = 0U;
    }
    unlock(saved);
    if (event == GRAB_FEEDBACK_FAILED) execution_failed = 1U;
    else if (event) execution_failed = 0U;
    request.suspended = execution_failed;
    /* 执行端失败不能变成业务完成；由执行端恢复后给同id的真实完成反馈。 */
    if (execution_failed) goto publish;

    uint8_t fresh = vision_fresh(now, v);
    uint8_t setting_pose = stage == GRAB_PREPARING_START || stage == GRAB_SETTING_OBJECT_POSE ||
                           stage == GRAB_SETTING_CONFIRM_POSE || stage == GRAB_SETTING_BASE_POSE;
    if (setting_pose && event == GRAB_FEEDBACK_DONE) {
        pose_done = 1U;
        camera_reset_seen = 0U;
        pose_ready_since_ms = completed_ms;
    }
    /* 移动途中收到的模式0回显不能提前记为完成屏障；必须晚于姿态稳定反馈。 */
    if (setting_pose && pose_done && fresh && v->data.target_mode == MAIXCAM_MODE_OBSERVE &&
        v->frame_id != camera_reset_baseline &&
        (int32_t)(v->data.updated_ms - pose_ready_since_ms) > 0)
        camera_reset_seen = 1U;

    switch (stage) {
    case GRAB_PREPARING_START:
        if (pose_done && camera_reset_seen) {
            stage = GRAB_WAITING_START;
            new_action(GRAB_ACTION_STOP, GRAB_POSE_START);
        }
        break;
    case GRAB_WAITING_START:
        if (start_requested) {
            start_requested = 0U;
            stage = GRAB_LEAVING_START;
            new_action(GRAB_ACTION_LEAVE_START, GRAB_POSE_FIND_OBJECT);
        }
        break;
    case GRAB_LEAVING_START:
        if (event == GRAB_FEEDBACK_DONE) begin_search();
        break;
    case GRAB_SETTING_OBJECT_POSE:
    case GRAB_SETTING_CONFIRM_POSE:
    case GRAB_SETTING_BASE_POSE:
        if (!pose_done || !camera_reset_seen) break;
        if (stage == GRAB_SETTING_OBJECT_POSE) begin_search();
        else if (stage == GRAB_SETTING_CONFIRM_POSE) {
            stage = GRAB_CONFIRMING_CARGO;
            vision_mode = MAIXCAM_MODE_CONFIRM_CARGO;
            last_confirm_frame = v->frame_id;
            last_confirm_sequence = v->data.sequence;
            clear_confirmation();
            new_action(GRAB_ACTION_STOP, GRAB_POSE_CHECK_CARGO);
        } else {
            stage = GRAB_RETURNING_TO_BASE;
            /* 旧的同模式导航即使仍在150ms窗口内，也不能跨转向/姿态屏障复用。 */
            navigation_baseline = n->frame_id;
            vision_mode = confirmed_cargo.patient ? MAIXCAM_MODE_PATIENT_ZONE : MAIXCAM_MODE_MATERIAL_ZONE;
            new_action(GRAB_ACTION_SEARCH_BASE, GRAB_POSE_FIND_BASE);
            request.suspended = 1U; /* 新导航观察实际到达后才执行搜索/跟随。 */
        }
        break;
    case GRAB_SEARCHING_OBJECT:
        request.suspended = !fresh;
        if (fresh && object_visible(v)) {
            stage = GRAB_APPROACHING_OBJECT;
            new_action(GRAB_ACTION_APPROACH_OBJECT, GRAB_POSE_FIND_OBJECT);
            request.target = *v;
        }
        break;
    case GRAB_APPROACHING_OBJECT:
        if (event == GRAB_FEEDBACK_DONE) {
            begin_pose(GRAB_SETTING_CONFIRM_POSE, GRAB_POSE_CHECK_CARGO, v->frame_id);
        } else if (!fresh) {
            request.suspended = 1U;
            memset(&request.target, 0, sizeof(request.target));
        } else if (!object_visible(v)) begin_search();
        else request.target = *v;
        break;
    case GRAB_CONFIRMING_CARGO:
        if (!fresh) {
            clear_confirmation(); /* 断流不是空夹，不能凑足1秒后擅自重选。 */
            break;
        }
        if (v->frame_id != last_confirm_frame) {
            /* 漏掉了中间帧就不能声称“连续”：数量计数和空夹时间都重新开始。 */
            if (v->frame_id != last_confirm_frame + 1U ||
                v->data.sequence != (uint8_t)(last_confirm_sequence + 1U))
                clear_confirmation();
            last_confirm_frame = v->frame_id;
            last_confirm_sequence = v->data.sequence;
            CargoCounts cargo = {v->data.normal_count, v->data.core_count,
                                 v->data.patient_count, v->data.hazard_count};
            if (cargo.normal == 0U && cargo.core == 0U && cargo.patient == 0U && cargo.hazard == 0U) {
                good_frames = bad_frames = 0U;
                if (!empty_timing) { empty_timing = 1U; empty_since_ms = now; }
            } else {
                empty_timing = 0U;
                if (cargo_allowed(cargo)) {
                    good_frames = good_frames && cargo_equal(cargo, previous_cargo) ? good_frames + 1U : 1U;
                    bad_frames = 0U;
                } else { good_frames = 0U; ++bad_frames; }
                previous_cargo = cargo;
                if (bad_frames >= GRAB_VERIFY_FRAMES) {
                    stage = GRAB_CLEARING_CARGO;
                    vision_mode = MAIXCAM_MODE_OBSERVE;
                    new_action(GRAB_ACTION_CLEAR_AND_SCATTER, GRAB_POSE_CHECK_CARGO);
                    request.cargo = cargo;
                } else if (good_frames >= GRAB_VERIFY_FRAMES) {
                    confirmed_cargo = cargo;
                    begin_pose(GRAB_SETTING_BASE_POSE, GRAB_POSE_FIND_BASE, v->frame_id);
                }
            }
        }
        if (stage == GRAB_CONFIRMING_CARGO && empty_timing &&
            (uint32_t)(now - empty_since_ms) >= GRAB_EMPTY_CONFIRM_MS)
            begin_object_pose(v->frame_id);
        break;
    case GRAB_CLEARING_CARGO:
        if (event == GRAB_FEEDBACK_DONE) begin_object_pose(v->frame_id);
        break;
    case GRAB_RETURNING_TO_BASE:
        if (!navigation_fresh(now, n)) {
            can_hold_speed = 0U;
            request.suspended = 1U;
            memset(&request.navigation, 0, sizeof(request.navigation));
            break;
        }
        if (n->action == MAIXCAM_NAV_FOLLOW_ZONE || n->action == MAIXCAM_NAV_FOLLOW_SPLIT) {
            ensure_action(GRAB_ACTION_FOLLOW_BASE, GRAB_POSE_FIND_BASE);
            request.navigation = *n;
            can_hold_speed = n->action == MAIXCAM_NAV_FOLLOW_SPLIT;
            if (event == GRAB_FEEDBACK_BASE_ALIGNED) {
                if (fabsf(angle) > GRAB_BASE_MAX_ANGLE_DEG) {
                    stage = GRAB_ADJUSTING_TO_CENTER;
                    vision_mode = MAIXCAM_MODE_OBSERVE;
                    new_action(GRAB_ACTION_ADJUST_TO_CENTER, GRAB_POSE_FIND_BASE);
                    request.base_angle_deg = angle;
                } else if (n->action == MAIXCAM_NAV_FOLLOW_SPLIT) {
                    stage = GRAB_WAITING_DELIVERY;
                    new_action(GRAB_ACTION_WAIT_DELIVERY, GRAB_POSE_FIND_BASE);
                }
            }
        } else if (n->action == MAIXCAM_NAV_HOLD_LAST_SPEED) {
            uint32_t elapsed = now - n->updated_ms;
            uint16_t remaining = elapsed < n->hold_remaining_ms ? (uint16_t)(n->hold_remaining_ms - elapsed) : 0U;
            ensure_action(GRAB_ACTION_HOLD_LAST_SPEED, GRAB_POSE_FIND_BASE);
            memset(&request.navigation, 0, sizeof(request.navigation));
            request.hold_remaining_ms = remaining;
            request.suspended = remaining == 0U || !can_hold_speed;
            if (!remaining) can_hold_speed = 0U;
        } else if (n->action == MAIXCAM_NAV_SEARCH_ZONE) {
            can_hold_speed = 0U;
            ensure_action(GRAB_ACTION_SEARCH_BASE, GRAB_POSE_FIND_BASE);
            memset(&request.navigation, 0, sizeof(request.navigation));
        } else request.suspended = 1U;
        break;
    case GRAB_ADJUSTING_TO_CENTER:
        /* 完成表示本次调整已结束且已转回，不假定一定走够距离；随后重新识别判断。 */
        if (event == GRAB_FEEDBACK_DONE)
            begin_pose(GRAB_SETTING_BASE_POSE, GRAB_POSE_FIND_BASE, v->frame_id);
        break;
    case GRAB_WAITING_DELIVERY:
        if (delivery_confirmed) {
            delivery_confirmed = 0U;
            /* 唯一解除首件限制的入口：外部真实送达判据，不是动作结束。 */
            if (confirmed_cargo.normal == 1U && confirmed_cargo.core == 0U &&
                confirmed_cargo.patient == 0U && confirmed_cargo.hazard == 0U)
                first_normal_delivered = 1U;
            begin_object_pose(v->frame_id);
        }
        break;
    default:
        ensure_action(GRAB_ACTION_STOP, GRAB_POSE_START);
        break;
    }
publish:
    request.stage = stage;
    request.team = configured_team;
    request.start_position = GRAB_START_POSITION_DEFAULT;
    if (stage != GRAB_CLEARING_CARGO) request.cargo = confirmed_cargo;
    saved = lock();
    published_request = request;
    published_cargo = confirmed_cargo;
    unlock(saved);
}

void rescue_grab_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    Grab_Init();
    Grab_SetTeam(configured_team);
    motor_set_state(MOTOR_STOP);
    for (;;) {
        /* 保留原有手动控制优先级；执行适配也必须遵守底层手动/保护仲裁。 */
        if (bluetooth_work && HC05_InManualMode()) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GRAB_TASK_PERIOD_MS));
            continue;
        }
        MaixcamObservation vision;
        MaixcamNavigation navigation;
        Maixcam_GetObservation(&vision);
        Maixcam_GetNavigation(&navigation);
        Grab_Process(HAL_GetTick(), &vision, &navigation);
        (void)Maixcam_SendCommand(vision_mode, configured_team);
        GrabActionRequest current;
        Grab_GetActionRequest(&current);
        if (action_executor) action_executor(&current);
        else motor_set_state(MOTOR_STOP); /* 未接动作端，不自动启动旧控制流程。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GRAB_TASK_PERIOD_MS));
    }
}

void rescue_grab_task_create(void)
{
    xTaskCreate((TaskFunction_t)rescue_grab_task, "rescue_grab", (uint32_t)grab_size,
                NULL, (UBaseType_t)grab_task_priority, &rescue_grab_handler);
}
