#include "maixcam.h"
#include "usart.h"

/* ============================================================================
 * Maixcam 视觉模块串口通信（UART5，115200bps，DMA + 空闲中断）
 *
 * 一、命令帧：主控 → Maixcam，共 7 字节
 *   偏移   0     1      2        3      4       5      6
 *        0xA5  0x5A   协议版本   mode   team   CRC_H  CRC_L
 *   CRC16 覆盖前 5 字节（0xA5 0x5A 版本 mode team）
 *
 * 二、结果帧：Maixcam → 主控，共 24 字节
 *   偏移   0     1      2        3     4      5      6      7
 *        0xA5  0x5B   协议版本  seq   mode   team   type   num
 *   偏移   8     9      10       11    12     13     14     15
 *        hazard normal  core    patient X_H    X_L    Y_H    Y_L
 *   偏移   16    17     18       19    20     21     22     23
 *         W_H   W_L     H_H     H_L   conf   valid  CRC_H  CRC_L
 *   CRC16 覆盖前 22 字节；目标中心/宽高均为大端序 uint16
 *
 * 三、接收策略：单字节喂进状态机，逐字节对齐包头，CRC 校验通过才采用
 * ==========================================================================*/

#define MAIXCAM_RX_DMA_SIZE 64U                 // DMA 接收缓冲区大小
#define MAIXCAM_COMMAND_HEADER_0 0xA5U          // 命令帧头字节 0
#define MAIXCAM_COMMAND_HEADER_1 0x5AU          // 命令帧头字节 1
#define MAIXCAM_RESULT_HEADER_0 0xA5U           // 结果帧头字节 0
#define MAIXCAM_RESULT_HEADER_1 0x5BU           // 结果帧头字节 1

static uint8_t rx_dma_buffer[MAIXCAM_RX_DMA_SIZE];  // DMA 直接写入的原始字节
static uint8_t rx_frame[MAIXCAM_RESULT_FRAME_SIZE]; // 攒帧用的滑动窗口
static uint8_t rx_frame_length;                     // 当前窗口里已攒的字节数
static uint8_t last_mode = 0xFFU;                   // 上次成功发送的模式（0xFF 表示还没发过）
static uint8_t last_team = 0xFFU;                   // 上次成功发送的队伍
static uint8_t last_command_valid;                  // 命令缓存是否有效（1=可以据此去重）
/* 业务快照，不新增线上字段；新导航由队友的收包适配调用PublishNavigation发布。 */
static uint32_t observation_frame_id;
static MaixcamNavigation latest_navigation;
static uint8_t configured_team = MAIXCAM_TEAM_DEFAULT;

extern DMA_HandleTypeDef hdma_uart5_rx;

/* CRC-16/CCITT-FALSE：多项式 0x1021，初值 0xFFFF，高位在前，不做结果异或 */
static uint16_t maixcam_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 按大端序读取一个 uint16（高字节在前） */
static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/* 判断模式号是否为协议里定义的合法值 */
static uint8_t maixcam_mode_valid(uint8_t mode)
{
    switch (mode) {
        case MAIXCAM_MODE_OBSERVE:
        case MAIXCAM_MODE_NORMAL_MATERIAL:
        case MAIXCAM_MODE_CORE_MATERIAL:
        case MAIXCAM_MODE_PATIENT:
        case MAIXCAM_MODE_ZONE:
        case MAIXCAM_MODE_RESCUE_TARGETS:
        case MAIXCAM_MODE_CONFIRM_CARGO:
            return 1U;
        default:
            return 0U;
    }
}

/* 把一帧校验通过的结果帧解析进全局 my_car.vision_manager
 * 解析前先做合法性检查，任何一项不合法就整帧丢弃，避免脏数据影响决策 */
static void maixcam_publish_result(const uint8_t *frame)
{
    const uint8_t *body = &frame[2];   // body 指向跳过 2 字节帧头后的负载
    uint8_t type = body[4];            // 目标类型
    uint8_t target_valid = body[19];   // 目标有效性标志

    /* 合法性检查：版本号 / 模式号 / 队伍 / 目标类型 / 置信度 / 有效性 全部要合理 */
    if (body[0] != MAIXCAM_PROTOCOL_VERSION ||
        !maixcam_mode_valid(body[2]) || body[3] > 2U || type > MAIXCAM_TARGET_RED_PATIENT_ZONE ||
        body[18] > 100U || target_valid > 1U ||
        (target_valid && type == MAIXCAM_TARGET_NONE)) {
        return;
    }

    /* 重复帧不能刷新新鲜度或被业务重复计数。 */
    if (my_car.vision_manager.frame_valid &&
        my_car.vision_manager.sequence == body[1] &&
        my_car.vision_manager.target_mode == body[2] &&
        my_car.vision_manager.team == body[3]) return;
    ++observation_frame_id;
    my_car.vision_manager.sequence = body[1];               // 帧序号
    my_car.vision_manager.target_mode = body[2];            // 视觉端当前工作模式
    my_car.vision_manager.team = body[3];                   // 队伍颜色
    my_car.vision_manager.target_type = type;                // 目标类型
    my_car.vision_manager.target_num = body[5];              // 目标总数
    my_car.vision_manager.hazard_count = body[6];           // 危险物数量
    my_car.vision_manager.normal_count = body[7];           // 普通物资数量
    my_car.vision_manager.core_count = body[8];             // 核心物资数量
    my_car.vision_manager.patient_count = body[9];          // 伤员数量
    my_car.vision_manager.x = read_u16_be(&body[10]);       // 目标中心 X
    my_car.vision_manager.y = read_u16_be(&body[12]);       // 目标中心 Y
    my_car.vision_manager.w = read_u16_be(&body[14]);       // 目标宽度
    my_car.vision_manager.h = read_u16_be(&body[16]);       // 目标高度
    my_car.vision_manager.confidence = body[18];            // 置信度（0~100）
    my_car.vision_manager.is_valid = target_valid;          // 目标是否有效
    my_car.vision_manager.frame_valid = 1U;                 // 标记"收到过有效帧"
    my_car.vision_manager.updated_ms = HAL_GetTick();       // 记录更新时间，供 IsFresh 判断
}

/* 快照时短暂屏蔽中断，防止UART更新到一半时读出混合帧；恢复原中断状态。 */
void Maixcam_GetObservation(MaixcamObservation *out)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    out->data = my_car.vision_manager;
    out->frame_id = observation_frame_id;
    __set_PRIMASK(saved);
}

void Maixcam_GetNavigation(MaixcamNavigation *out)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    *out = latest_navigation;
    __set_PRIMASK(saved);
}

uint8_t Maixcam_PublishNavigation(const MaixcamNavigation *n)
{
    if ((n->team != 1U && n->team != 2U) || !maixcam_mode_valid(n->mode) ||
        n->action > MAIXCAM_NAV_SEARCH_ZONE || n->reference_visible > 1U ||
        n->confidence > 100U || n->hold_remaining_ms > 500U) return 0U;
    uint8_t returning = n->mode == MAIXCAM_MODE_ZONE ;
    uint8_t visible = n->action == MAIXCAM_NAV_FOLLOW_ZONE || n->action == MAIXCAM_NAV_FOLLOW_SPLIT;
    if (returning) {
        if (n->action == MAIXCAM_NAV_NOT_RETURNING_TO_BASE) return 0U;
    } else if (n->action != MAIXCAM_NAV_NOT_RETURNING_TO_BASE) return 0U;
    if (visible) {
        uint8_t expected = n->action == MAIXCAM_NAV_FOLLOW_ZONE
            ? (n->team == 1U ? MAIXCAM_REFERENCE_BLUE_ZONE : MAIXCAM_REFERENCE_RED_ZONE)
            : (n->team == 1U ? MAIXCAM_REFERENCE_BLUE_SPLIT : MAIXCAM_REFERENCE_RED_SPLIT);
        if (!n->reference_visible || n->reference != expected || !n->w || !n->h ||
            n->confidence < 50U || n->hold_remaining_ms) return 0U;
    } else if (n->reference_visible || n->reference != MAIXCAM_REFERENCE_NONE ||
               n->x || n->y || n->w || n->h || n->confidence) return 0U;
    if (n->action != MAIXCAM_NAV_HOLD_LAST_SPEED && n->hold_remaining_ms) return 0U;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (latest_navigation.frame_valid && latest_navigation.sequence == n->sequence &&
        latest_navigation.mode == n->mode && latest_navigation.team == n->team) {
        __set_PRIMASK(saved);
        return 0U;
    }
    uint32_t next = latest_navigation.frame_id + 1U;
    latest_navigation = *n;
    latest_navigation.updated_ms = HAL_GetTick();
    latest_navigation.frame_id = next;
    latest_navigation.frame_valid = 1U;
    __set_PRIMASK(saved);
    return 1U;
}

/* 丢弃滑动窗口里的第一个字节，用于重新对齐帧头 */
static void maixcam_drop_first_byte(void)
{
    if (rx_frame_length > 0U) {
        --rx_frame_length;
        for (uint8_t i = 0; i < rx_frame_length; ++i) {
            rx_frame[i] = rx_frame[i + 1U];
        }
    }
}

/* 接收状态机：每来一个字节就调用一次，负责攒帧、找包头、校验 CRC */
static void maixcam_parse_byte(uint8_t byte)
{
    /* 先入窗：窗口满了就挤掉最旧的一个字节 */
    if (rx_frame_length < MAIXCAM_RESULT_FRAME_SIZE) {
        rx_frame[rx_frame_length++] = byte;
    } else {
        maixcam_drop_first_byte();
        rx_frame[rx_frame_length++] = byte;
    }

    for (;;) {
        /* 步骤1：把窗口头部挪到第一个 0xA5 上 */
        while (rx_frame_length >= 1U && rx_frame[0] != MAIXCAM_RESULT_HEADER_0) {
            maixcam_drop_first_byte();
        }
        /* 步骤2：第二个字节必须是 0x5B，否则说明是假包头，丢掉首字节重来 */
        if (rx_frame_length >= 2U && rx_frame[1] != MAIXCAM_RESULT_HEADER_1) {
            maixcam_drop_first_byte();
            continue;
        }
        /* 步骤3：字节数不够一整帧，先等后续数据 */
        if (rx_frame_length < MAIXCAM_RESULT_FRAME_SIZE) {
            return;
        }

        /* 步骤4：比较 CRC。末尾 2 字节是发送方算好的 CRC16 */
        uint16_t expected_crc = read_u16_be(&rx_frame[22]);
        if (maixcam_crc16(rx_frame, 22U) == expected_crc) {
            maixcam_publish_result(rx_frame);   // 校验通过 → 发布结果
            rx_frame_length = 0U;               // 清窗，等下一帧
            return;
        }
        /* CRC 不匹配：可能是假包头，丢一个字节继续找 */
        maixcam_drop_first_byte();
    }
}

/* 把一段 DMA 数据逐字节喂给状态机 */
static void maixcam_parse_bytes(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i) {
        maixcam_parse_byte(data[i]);
    }
}

/* 初始化：清空接收状态，打开空闲中断，启动 DMA 接收 */
void Maixcam_init(void)
{
    rx_frame_length = 0U;
    observation_frame_id = 0U;
    latest_navigation = (MaixcamNavigation){0};
    configured_team = MAIXCAM_TEAM_DEFAULT;
    last_command_valid = 0U;
    my_car.vision_manager.frame_valid = 0U;
    my_car.vision_manager.is_valid = 0U;
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart5, rx_dma_buffer, MAIXCAM_RX_DMA_SIZE);
}

/* 下发模式命令帧（7 字节）。
 * 返回1：发送成功或已新鲜回显无需重发；返回0：参数非法/发送失败。 */
uint8_t Maixcam_SendCommand(uint8_t mode, uint8_t team)
{
    if (!maixcam_mode_valid(mode) || team > 2U) {
        return 0U;                          // 参数非法，直接拒绝
    }
    /* 同模式且相机已新鲜回显才去重；未生效或重启后继续发送，避免永久等不到模式。 */
    if (last_command_valid && last_mode == mode && last_team == team &&
        Maixcam_IsFresh(150U) && my_car.vision_manager.target_mode == mode &&
        my_car.vision_manager.team == team) {
        return 1U;
    }

    /* 组帧：包头 + 版本 + 模式 + 队伍，后两字节留给 CRC */
    uint8_t command[MAIXCAM_COMMAND_FRAME_SIZE] = {
        MAIXCAM_COMMAND_HEADER_0, MAIXCAM_COMMAND_HEADER_1,
        MAIXCAM_PROTOCOL_VERSION, mode, team, 0U, 0U
    };
    uint16_t crc = maixcam_crc16(command, 5U);   // CRC 只算前 5 字节
    command[5] = (uint8_t)(crc >> 8);            // 高字节在前
    command[6] = (uint8_t)crc;

    if (HAL_UART_Transmit(&huart5, command, sizeof(command), 100U) != HAL_OK) {
        return 0U;
    }
    last_mode = mode;                    // 记录，供下次去重
    last_team = team;
    last_command_valid = 1U;
    my_car.track_mode = mode;            // 同步到全局，便于调试查看
    configured_team = team;
    return 1U;
}

/* 设置队伍颜色（1=蓝，2=红）。
 * 如果之前已经发过命令，就用新队伍把当前模式再发一遍；否则只记录，等首次发命令时带上 */
uint8_t Maixcam_SetTeam(uint8_t team)
{
    if (team != 1U && team != 2U) {
        return 0U;
    }
    configured_team = team;
    if (!last_command_valid) {
        return 1U;                       // 还没发过命令，无需重发
    }
    last_command_valid = 0U;             // 让下面的 SendCommand 认为需要重发
    return Maixcam_SendCommand(last_mode, team);
}

/* 兼容旧接口：用当前队伍下发模式命令 */
void Set_Vision_Mode(uint8_t mode)
{
    (void)Maixcam_SendCommand(mode, configured_team);
}

/* 判断视觉数据是否新鲜：收到过有效帧，且距上次更新不超过 timeout_ms */
uint8_t Maixcam_IsFresh(uint32_t timeout_ms)
{
    return my_car.vision_manager.frame_valid &&
           (uint32_t)(HAL_GetTick() - my_car.vision_manager.updated_ms) <= timeout_ms;
}

/* UART5 空闲中断处理：由 stm32h7xx_it.c 的 UART5_IRQHandler 调用
 * 利用空闲中断判断"一包数据收完"，然后按实际长度解析 */
void maixcam_uart_irq_handler(void)
{
    //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        HAL_UART_DMAStop(&huart5);                       // 暂停 DMA，保证数据不再变化
        uint16_t received = (uint16_t)(MAIXCAM_RX_DMA_SIZE -
                                      __HAL_DMA_GET_COUNTER(&hdma_uart5_rx));  // 已收字节数
        if (received > MAIXCAM_RX_DMA_SIZE) {            // 防御：异常时夹到缓冲区上限
            received = MAIXCAM_RX_DMA_SIZE;
        }
        maixcam_parse_bytes(rx_dma_buffer, received);    // 交给状态机解析
        HAL_UART_Receive_DMA(&huart5, rx_dma_buffer, MAIXCAM_RX_DMA_SIZE);  // 重新武装 DMA
    }
}
