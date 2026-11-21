#include "gyro.h"
#define BUFFER_SIZE 100U                       // DMA 接收缓冲区大小
static uint8_t imu_rx_buf[BUFFER_SIZE];        // DMA 直接写入的原始字节
static uint8_t frame[11], used;                // 攒帧窗口 + 已攒字节数
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ============================================================================
 * JY61 陀螺仪数据帧（11 字节，模块固定输出）：
 *
 *   下标  0     1     2     3     4     5     6     7     8     9     10
 *        0x55  0x53  ...   ...   ...   ...  YAW_L YAW_H ...   ...   校验和
 *
 *   - frame[0] = 0x55       帧头
 *   - frame[1] = 0x53       包类型（角度包）；0x51 加速度、0x52 角速度
 *   - frame[6..7]           偏航角原始值，小端序 int16
 *   - frame[10]             前 10 字节的累加和（自然溢出到 uint8）
 *
 * 角度换算：raw × 180 / 32768 → -180~+180°，再 +360 归一到 0~360°
 * ==========================================================================*/

/* 逐字节解析状态机：每收到 1 字节调用一次 */
static void consume(uint8_t byte) {
    /* 还没同步到帧头时，丢弃所有非 0x55 的字节 */
    if (!used && byte!=0x55) return;
    frame[used++]=byte;
    if (used<11) return;                       // 没攒够一帧，继续等

    /* 累加和校验：前 10 字节之和（uint8 自然溢出）与第 11 字节比较 */
    uint8_t sum=0;
    for (unsigned i=0;i<10;i++) sum+=frame[i];
    if (sum==frame[10]) {
        /* 校验通过，只处理角度包 0x53 */
        if (frame[1]==0x53) {
            /* 小端序拼出原始值：低字节 frame[6] + 高字节 frame[7] */
            int16_t raw=(int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7]<<8));
            my_car.angle_manager.last_yaw=my_car.angle_manager.yaw;   // 保存上一帧，便于算增量
            float yaw=(float)raw*180.0f/32768.0f;                     // 归一化到 -180~+180
            // if (yaw<0) yaw+=360.0f;                                   // 再转成 0~360
            my_car.angle_manager.yaw=yaw;
            my_car.angle_manager.update_ms=HAL_GetTick();             // 记录时刻，供上层判新鲜度
            my_car.angle_manager.valid=1;
        }
        used=0;                                // 一帧处理完，清空窗口
    } else {
        /* 校验失败：说明刚才那个 0x55 可能是数据里的巧合
         * 从第 2 个字节开始找下一个 0x55，把它前面的字节全部丢掉，重新对齐 */
        unsigned skip=1;
        while (skip<11 && frame[skip]!=0x55) skip++;
        used=(uint8_t)(11-skip);
        for (unsigned i=0;i<used;i++) frame[i]=frame[i+skip];
    }
}

/* 初始化：清空解析状态，打开空闲中断，启动 DMA 接收 */
void gyro_init(void) {
    used=0; 
    my_car.angle_manager.valid=0;
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);
}

/* USART3 空闲中断处理：由 stm32h7xx_it.c 的 USART3_IRQHandler 调用
 * 注意 huart3 是 USART3 的句柄，对应 PD8/PD9 接 JY61 */
void gyro_uart_irq_handler(void) {
    if (__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        HAL_UART_DMAStop(&huart3);                                   // 先停 DMA，保证数据不再变化
        /* 已接收字节数 = 缓冲区总长 - DMA 剩余未写长度 */
        uint32_t length=BUFFER_SIZE-__HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        for (uint32_t i=0;i<length;i++) consume(imu_rx_buf[i]);      // 逐字节喂给状态机
        HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);         // 重新武装 DMA
    }
}
