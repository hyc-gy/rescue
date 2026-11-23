#include "hc05.h"
#include "motor.h"
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

/* ===== 全局资源 ===== */
osMessageQueueId_t hc05_rx_queue;
static volatile uint8_t hc05_manual_mode;       // 1 = 蓝牙手动控制中（grab 状态机让出电机控制权）

static uint8_t hc05_tx_buf[HC05_TX_BUF_SIZE];   // DMA 发送缓冲（发送期间不能被改写）
static volatile uint8_t hc05_tx_busy;           // 1 = DMA 正在发送

static uint8_t hc05_rx_dma[HC05_RX_DMA_SIZE];   // DMA 接收缓冲
static uint8_t hc05_frame[HC05_FRAME_MAX];      // 一帧接收数据（两次空闲中断之间）
static volatile uint16_t hc05_frame_len;        // 本帧长度
static volatile uint8_t hc05_frame_ready;       // 1 = 有新帧可取

/* ============================================================================
 * 初始化：创建队列 + 开启空闲中断 + 启动 DMA 接收
 * 前提：CubeMX 里已给 USART1 配好 TX/RX DMA 通道
 * ==========================================================================*/
void HC05_Init(void)
{
    const osMessageQueueAttr_t queue_attr = {
            .name = "hc05_rx_queue"
    };
    hc05_rx_queue = osMessageQueueNew(HC05_QUEUE_DEPTH, sizeof(uint8_t), &queue_attr);

    hc05_tx_busy = 0;
    hc05_frame_len = 0;
    hc05_frame_ready = 0;

    __HAL_UART_CLEAR_IDLEFLAG(HC05_UART_HANDLE);
    __HAL_UART_ENABLE_IT(HC05_UART_HANDLE, UART_IT_IDLE);
    HAL_UART_Receive_DMA(HC05_UART_HANDLE, hc05_rx_dma, HC05_RX_DMA_SIZE);
}

/* ============================================================================
 * 发送：printf 风格，vsnprintf 组包后 DMA 非阻塞发送
 * 返回后函数立即返回，数据由 DMA 在后台搬运
 * ==========================================================================*/
void HC05_Printf(const char *fmt, ...)
{
    if (fmt == NULL || hc05_tx_busy) return;   // 上一包还没发完，丢弃本包

    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)hc05_tx_buf, HC05_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if (len >= (int)HC05_TX_BUF_SIZE) len = (int)HC05_TX_BUF_SIZE - 1;  // 超长被截断

    hc05_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(HC05_UART_HANDLE, hc05_tx_buf, (uint16_t)len) != HAL_OK) {
        hc05_tx_busy = 0;                      // 启动失败（DMA 未配置/忙），放开标志
    }
}

/* 查询是否正在发送（1=忙） */
uint8_t HC05_TxBusy(void)
{
    return hc05_tx_busy;
}

/* 发送字符串（内部走 DMA） */
void HC05_SendString(const char *str)
{
    if (str == NULL) return;
    HC05_SendData((uint8_t *)str, (uint16_t)strlen(str));
}

/* 发送二进制数据（内部走 DMA） */
void HC05_SendData(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || hc05_tx_busy) return;
    if (len > HC05_TX_BUF_SIZE) len = HC05_TX_BUF_SIZE;

    memcpy(hc05_tx_buf, data, len);            // 拷贝一份，调用方的缓冲可以立刻复用
    hc05_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(HC05_UART_HANDLE, hc05_tx_buf, len) != HAL_OK) {
        hc05_tx_busy = 0;
    }
}

/* DMA 发送完成回调：由 HAL_UART_TxCpltCallback 转发进来，释放忙标志 */
void HC05_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == (*HC05_UART_HANDLE).Instance) {
        hc05_tx_busy = 0;
    }
}

/* 覆盖 HAL 的弱符号回调，把发送完成事件转给本模块 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HC05_TxCpltCallback(huart);
}

/* ============================================================================
 * 接收：DMA + 空闲中断
 * 空闲中断表示"这一串数据发完了"，此时把 DMA 收到的字节取出来，
 * 既逐字节塞进队列（兼容 ReadByte/ReadBuff），也拼成"一帧"供 GetFrame 使用
 * ==========================================================================*/
void HC05_IdleCallback(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(HC05_UART_HANDLE);
    HAL_UART_DMAStop(HC05_UART_HANDLE);        // 先停 DMA，保证数据不再变化

    uint16_t len = (uint16_t)(HC05_RX_DMA_SIZE -
                              __HAL_DMA_GET_COUNTER((*HC05_UART_HANDLE).hdmarx));
    if (len > HC05_RX_DMA_SIZE) len = HC05_RX_DMA_SIZE;   // 防御

    hc05_frame_len = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = hc05_rx_dma[i];

        if (hc05_rx_queue != NULL) {
            osMessageQueuePut(hc05_rx_queue, &byte, 0, 0);  // 非阻塞入队
        }
        if (hc05_frame_len < HC05_FRAME_MAX) {
            hc05_frame[hc05_frame_len++] = byte;
        }
    }
    if (len > 0) hc05_frame_ready = 1;         // 标记有新帧

    HAL_UART_Receive_DMA(HC05_UART_HANDLE, hc05_rx_dma, HC05_RX_DMA_SIZE);  // 重新武装
}

/* 阻塞读取 1 字节（队列为空返回 0xFF） */
uint8_t HC05_ReadByte(uint32_t timeout)
{
    uint8_t val = 0xFF;
    if (hc05_rx_queue == NULL) return 0xFF;
    osStatus_t ret = osMessageQueueGet(hc05_rx_queue, &val, NULL, timeout);
    if (ret != osOK) return 0xFF;
    return val;
}

/* 批量读取字节 */
uint16_t HC05_ReadBuff(uint8_t *buf, uint16_t max_len, uint32_t timeout)
{
    if (buf == NULL || max_len == 0) return 0;

    uint16_t cnt = 0;
    while (cnt < max_len) {
        uint8_t ch = HC05_ReadByte(timeout);
        if (ch == 0xFF) break;                 // 超时
        buf[cnt++] = ch;
    }
    return cnt;
}

/* 取回最后一整帧（按空闲间隔切分）。无新帧返回 0 */
uint16_t HC05_GetFrame(uint8_t *buf, uint16_t max_len)
{
    if (buf == NULL || max_len == 0) return 0;

    uint16_t len = 0;
    taskENTER_CRITICAL();
    if (hc05_frame_ready) {
        len = hc05_frame_len;
        if (len > max_len) len = max_len;
        memcpy(buf, hc05_frame, len);
        hc05_frame_len = 0;
        hc05_frame_ready = 0;
    }
    taskEXIT_CRITICAL();
    return len;
}

/* ============================================================================
 * 蓝牙调试命令解析
 * ==========================================================================*/

/* 跳过分隔符：空格/制表符/冒号/等号，兼容 "skp1.2"、"skp=1.2"、"skp 1.2" */
static const char *hc05_skip_sep(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == ':' || *p == '=') p++;
    return p;
}

/* 从 *pp 处解析一个浮点数，成功后把 *pp 移到数字之后 */
static uint8_t hc05_parse_float(const char **pp, float *out)
{
    const char *p = hc05_skip_sep(*pp);
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) return 0;                 // 没有数字
    *out = v;
    *pp = end;
    return 1;
}

/* 命令是否已经结束（允许尾随空格、\r\n） */
static uint8_t hc05_cmd_end(const char *p)
{
    p = hc05_skip_sep(p);
    return (uint8_t)(*p == '\0' || *p == '\r' || *p == '\n');
}

/* 识别 3 字母 PID 命令：skp/ski/skd、ykp/yki/ykd、dkp/dki/dkd（大小写不敏感） */
static uint8_t hc05_match_pid_key(const char *p, struct PID_param **param, char *term)
{
    if (p[0] == '\0' || p[1] == '\0' || p[2] == '\0') return 0;

    char bank = (char)tolower((unsigned char)p[0]);   // s=L0_param, y=yaw_pid_param, d=distance_pid_param
    char k    = (char)tolower((unsigned char)p[1]);   // 固定为 'k'
    char t    = (char)tolower((unsigned char)p[2]);   // p/i/d

    if (k != 'k' || (t != 'p' && t != 'i' && t != 'd')) return 0;

    switch (bank) {
    case 's': *param = &L0_param;           break;
    case 'y': *param = &yaw_pid_param;      break;
    case 'd': *param = &distance_pid_param; break;
    default: return 0;
    }

    *term = t;
    return 1;
}

/* 找到与位置式 PID 参数配套的对象；L0/R0 是增量式，不需要无扰补偿 */
static struct P_pid_obj *hc05_pos_pid_obj(struct PID_param *param)
{
    if (param == &yaw_pid_param)      return &yaw_pid_obj;
    if (param == &distance_pid_param) return &distance_pid_obj;
    return NULL;
}

/* 位置式 PID 的无扰切换（bumpless transfer）：
 * 参数改完后，用"切换前的输出"反推积分项，使下一拍算出的输出保持不变。
 * 计算依据（见 pid.c 的 positional_PID / error_PID）：
 *   output = kp*bias + ki*integral + kd*differential
 * 下一拍还会先执行 integral += bias，所以这里把这一步也扣掉。
 * ki=0 时积分项无法反推，只能放弃补偿。 */
static void hc05_pid_bumpless(struct P_pid_obj *obj, const struct PID_param *pid)
{
    if (obj == NULL || pid == NULL) return;
    if (pid->ki > -1.0e-6f && pid->ki < 1.0e-6f) return;    // ki≈0，无法通过积分项补偿

    float differential = (obj->bias - obj->last_bias) * pid->differential_filterK +
                         (1.0f - pid->differential_filterK) * obj->last_differential;

    /* 下一拍 PID 内部会给 integral 加多少：error_PID 对 yaw 有 ±5° 的门限 */
    float next_step;
    if (pid == &yaw_pid_param) {
        next_step = (obj->bias > -5.0f && obj->bias < 5.0f) ? obj->bias : 0.0f;
    } else {
        next_step = obj->bias;
    }

    obj->integral = (obj->output - pid->kp * obj->bias - pid->kd * differential) / pid->ki
                    - next_step;
}

uint8_t HC05_ProcessCommand(const char *cmd)
{
    if (cmd == NULL) return 0;

    /* 去掉前导空白 */
    while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') cmd++;
    if (*cmd == '\0') return 0;

    /* --- 1) A / Z：两个电机目标速度 --- */
    if (((*cmd == 'A') || (*cmd == 'a')) && hc05_cmd_end(cmd + 1)) {
        //motor_pid_clear();                  // 清掉旧模式残留的 PID 状态，避免切换瞬间抖动
        motor_L0.target = HC05_RUN_SPEED;
        motor_R0.target = HC05_RUN_SPEED;
        motor_set_state(MOTOR_BLUETOOTH);   // 进入蓝牙速度模式
        hc05_manual_mode = 1U;              // 让 grab 状态机暂停接管电机
        return 1;
    }
    if (((*cmd == 'E') || (*cmd == 'e')) && hc05_cmd_end(cmd + 1)) {
        //motor_pid_clear();                  // 清掉旧模式残留的 PID 状态，避免切换瞬间抖动
        motor_L0.target = -HC05_RUN_SPEED;
        motor_R0.target = -HC05_RUN_SPEED;
        motor_set_state(MOTOR_BLUETOOTH);   // 进入蓝牙速度模式
        hc05_manual_mode = 1U;              // 让 grab 状态机暂停接管电机
        return 1;
    }
    if (((*cmd == 'Z') || (*cmd == 'z')) && hc05_cmd_end(cmd + 1)) {
        //motor_pid_clear();                  // 清掉旧模式残留的 PID 状态，避免切换瞬间抖动
        motor_L0.target = 0.0f;
        motor_R0.target = 0.0f;
        motor_set_state(MOTOR_BLUETOOTH);   // 进入蓝牙速度模式
        hc05_manual_mode = 1U;              // 让 grab 状态机暂停接管电机
        return 1;
    }
    if (((*cmd == 'C') || (*cmd == 'c')) && hc05_cmd_end(cmd + 1)) {
        //motor_pid_clear();                  // 清掉旧模式残留的 PID 状态，避免切换瞬间抖动
        motor_L0.target = HC05_RUN_SPEED/3;
        motor_R0.target = -HC05_RUN_SPEED/3;
        motor_set_state(MOTOR_BLUETOOTH);   // 进入蓝牙速度模式
        hc05_manual_mode = 1U;              // 让 grab 状态机暂停接管电机
        return 1;
    }
    if (((*cmd == 'G') || (*cmd == 'g')) && hc05_cmd_end(cmd + 1)) {
        //motor_pid_clear();                  // 清掉旧模式残留的 PID 状态，避免切换瞬间抖动
        motor_L0.target = -HC05_RUN_SPEED/3;
        motor_R0.target = HC05_RUN_SPEED/3;
        motor_set_state(MOTOR_BLUETOOTH);   // 进入蓝牙速度模式
        hc05_manual_mode = 1U;              // 让 grab 状态机暂停接管电机
        return 1;
    }
    if (((*cmd == 'Y') || (*cmd == 'y')) && hc05_cmd_end(cmd + 1)) {
        open_posture();
        return 1;
    }
    if (((*cmd == 'X') || (*cmd == 'x')) && hc05_cmd_end(cmd + 1)) {
        grab_posture();
        return 1;
    }
    if (((*cmd == 'K') || (*cmd == 'k')) && hc05_cmd_end(cmd + 1)) {
        hc05_manual_mode = 0U;              // 让 grab 状态机接管电机
        motor_L0.target = 0.0f;
        motor_R0.target = 0.0f;
        motor_pid_clear();
        motor_set_state(MOTOR_STOP);   // 进入蓝牙速度模式
        return 1;
    }
    /* --- 2) (x,y,z)：目标位置 + 目标偏航角 --- */
    if (*cmd == '(') {
        const char *p = cmd + 1;
        float x, y, yaw;

        if (!hc05_parse_float(&p, &x))   return 0;
        p = hc05_skip_sep(p);
        if (*p++ != ',')                 return 0;
        if (!hc05_parse_float(&p, &y))   return 0;
        p = hc05_skip_sep(p);
        if (*p++ != ',')                 return 0;
        if (!hc05_parse_float(&p, &yaw)) return 0;
        p = hc05_skip_sep(p);
        if (*p != ')')                   return 0;
        my_car.pose_manager.target_x = x;
        my_car.pose_manager.target_y = y;
        my_car.angle_manager.target_yaw = yaw;
        my_car.pose_manager.finished = 0;
        motor_set_state(MOTOR_POSITION);
        return 1;
    }

    /* --- 3) PID 调参命令 --- */
    struct PID_param *param = NULL;
    char term = 0;
    if (hc05_match_pid_key(cmd, &param, &term)) {
        const char *p = cmd + 3;
        float value;
        if (!hc05_parse_float(&p, &value)) return 0;   // 后面没有数字则视为无效

        struct P_pid_obj *pos_obj = hc05_pos_pid_obj(param);

        /* 参数写入与无扰补偿放在同一临界区，避免 motor_task 在中间插进来
         * 用"新参数 + 旧积分"算出一拍突变输出 */
        taskENTER_CRITICAL();
        if (term == 'p')      param->kp = value;
        else if (term == 'i') param->ki = value;
        else                  param->kd = value;
        hc05_pid_bumpless(pos_obj, param);
        taskEXIT_CRITICAL();
        return 1;
    }

    return 0;
}

uint8_t HC05_PollCommand(void)
{
    char frame[HC05_FRAME_MAX + 1];
    uint16_t len = HC05_GetFrame((uint8_t *)frame, HC05_FRAME_MAX);
    if (len == 0) return 0;

    frame[len] = '\0';

    uint8_t done = 0;
    char *line = strtok(frame, "\r\n");
    while (line != NULL) {
        if (HC05_ProcessCommand(line)) done++;
        line = strtok(NULL, "\r\n");
    }
    return done;
}

/* 是否处于蓝牙手动控制模式（A 进入，Z 退出） */
uint8_t HC05_InManualMode(void)
{
    return hc05_manual_mode;
}
