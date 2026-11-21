#ifndef __gyro_h__
#define __gyro_h__

#include "common.h"
#include <string.h>
#include "hc05.h"

/* ===== JY61 陀螺仪（USART3 + DMA + 空闲中断）=====
 * 解析后写入 my_car.angle_manager：
 *   yaw       当前偏航角，范围 0~360°（顺时针递增）
 *   last_yaw  上一帧的 yaw，便于算增量
 *   update_ms 最近一次成功解析的时刻，供上层判断数据是否新鲜
 *   valid     是否收到过有效数据
 * 空闲中断处理函数 gyro_uart_irq_handler() 定义在 gyro.c 内，
 * 由 stm32h7xx_it.c 的 USART3_IRQHandler 调用 */

void gyro_init(void);   // 初始化：清状态、开串口空闲中断、启动 DMA 接收
void gyro_uart_irq_handler(void);   // USART3 空闲中断处理（配合 DMA 收包）

#endif
