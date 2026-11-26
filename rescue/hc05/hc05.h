#ifndef HC05_H
#define HC05_H

#include "common.h"

/* ===== 配置参数 ===== */
#define HC05_UART_HANDLE    &huart1     // HC05 挂在 USART1（PA9/PA10）
#define HC05_QUEUE_DEPTH    64          // 接收字节队列长度
#define HC05_TX_BUF_SIZE    256         // 发送缓冲：DMA 发送期间必须保持有效
#define HC05_RX_DMA_SIZE    64          // DMA 接收缓冲大小
#define HC05_FRAME_MAX      128         // 单帧最大长度（两次空闲中断之间的数据）
#define HC05_RUN_SPEED      22.0f       // A 命令写入两个电机的目标速度（>0，按实车调）

/* 外部句柄声明 */
extern osMessageQueueId_t hc05_rx_queue;

/* 初始化：创建接收队列、开启空闲中断、启动 DMA 接收 */
void HC05_Init(void);

/* printf 风格发送（内部 vsnprintf 组包 + DMA 非阻塞发送）
 * 用法：HC05_Printf("yaw:%.1f, x:%d\r\n", yaw, x);
 * 注意：上一包还没发完时本包会被丢弃，避免覆盖正在发送的缓冲区 */
void HC05_Printf(const char *fmt, ...);

/* 是否正处于 DMA 发送中（1=忙，新数据会被丢弃） */
uint8_t HC05_TxBusy(void);

/* 发送字符串（内部走 DMA，非阻塞） */
void HC05_SendString(const char *str);

/* 发送指定长度字节数组（内部走 DMA，非阻塞） */
void HC05_SendData(uint8_t *data, uint16_t len);

/* 阻塞读取 1 字节（从接收队列取，无数据返回 0xFF） */
uint8_t HC05_ReadByte(uint32_t timeout);

/* 批量读取到一个缓冲区，返回实际读到的字节数 */
uint16_t HC05_ReadBuff(uint8_t *buf, uint16_t max_len, uint32_t timeout);

/* 整帧读取：按"空闲中断"切分，一次取回最后一整帧（无新帧返回 0） */
uint16_t HC05_GetFrame(uint8_t *buf, uint16_t max_len);

/* ===== 蓝牙调试命令解析 =====
 * 支持的命令（可带 \r\n）：
 *   A                   进入蓝牙速度模式，两个电机目标速度 = HC05_RUN_SPEED（>0）
 *   Z                   停车，两个电机目标速度 = 0
 *   skp/ski/skd<数值>   修改 L0_param 的 kp/ki/kd，如 "skp1.2"、"skp=1.2"、"skp 1.2"
 *   ykp/yki/ykd<数值>   修改 yaw_pid_param 的 kp/ki/kd
 *   dkp/dki/dkd<数值>   修改 distance_pid_param 的 kp/ki/kd
 *                       位置式 PID（yaw/distance）改参时做无扰切换：
 *                       用旧输出反算积分项，保证切换瞬间输出不跳变（ki=0 时除外）
 *   (x,y,z)             目标位置 target_x/target_y = x/y，目标偏航角 target_yaw = z
 * 返回 1=命令已识别并执行，0=未识别 */
uint8_t HC05_ProcessCommand(const char *cmd);

/* 取一帧接收数据，按行拆分后逐条交给 HC05_ProcessCommand 解析。
 * 需在任务上下文周期调用（内部会进临界区），返回识别成功的命令条数 */
uint8_t HC05_PollCommand(void);

/* 是否处于蓝牙手动控制模式：A 进入（1），Z 退出（0）。
 * grab 任务据此让出电机控制权，避免状态机把蓝牙下发的状态覆盖掉 */
uint8_t HC05_InManualMode(void);

/* 以下两个回调由中断/驱动层调用，应用层无需关心 */
void HC05_IdleCallback(void);                       // 在 USART1_IRQHandler 的空闲中断里调用
void HC05_TxCpltCallback(UART_HandleTypeDef *huart); // 在 HAL_UART_TxCpltCallback 里转发

#endif
