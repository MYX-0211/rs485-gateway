/* ============================================================================
 * rs485.c — RS485 半双工总线驱动（USART1 + MAX485，DMA + 空闲中断）
 *   方向控制：PA8(DE) 高=发送 低=接收；发送完成(TC)中断自动切回接收
 *   接收方式：HAL_UARTEx_ReceiveToIdle_DMA —— 总线空闲(>3.5字符)自动判帧，
 *             天然满足 Modbus RTU 帧间隔要求
 *   同步机制：xSemTx=发送完成信号量，xSemRx=接收完成信号量（Modbus 层等待）
 *   双缓冲：rs485_dma_buf 仅供 DMA 写入，事件回调拷贝到 rs485_rx_buf 供解析
 * ==========================================================================*/
#include "rs485.h"
#include "string.h"
#include "FreeRTOS.h"  
#include "semphr.h"        

static SemaphoreHandle_t xSemTx, xSemRx;    // 发送完成 / 接收完成 二值信号量
volatile uint16_t rs485_rx_len  = 0;        // 最近一帧实际接收字节数（Modbus 层校验用）
uint8_t rs485_rx_buf[RS485_RXBUF_SIZE];     // 接收数据缓冲区（解析用）
static uint8_t rs485_dma_buf[RS485_RXBUF_SIZE]; // DMA 专用缓冲：回调中拷贝走，永不参与解析

/* DE 方向切换建立延时（空循环，volatile 防止被优化删除） */
static void de_delay(void)
{
    for (volatile int i = 0; i < 8400; i++);  /* 约 0.2~0.5ms @168MHz */
}

/* 切到发送方向：DE=1，等待收发器完成方向建立 */
static void RS485_TxMode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    de_delay();
}

/* 切到接收方向：DE=0（默认态，避免总线冲突） */
static void RS485_RxMode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  RS485 总线初始化：创建收发信号量 + 默认接收态 + 启动空闲中断 DMA 接收
 * @note   在 MX_USART1_UART_Init 之后、Modbus 事务开始前调用一次
 */
void RS485_Init(void)
{
    xSemTx = xSemaphoreCreateBinary();   /* 发送完成信号量 */
    xSemRx = xSemaphoreCreateBinary();   /* 接收完成信号量 */
    RS485_RxMode();                      /* 默认接收态，避免上电瞬间误发 */
    /* 空闲中断 + DMA 接收：总线空闲(>3.5 字符)自动判帧，天然满足 Modbus RTU 帧间隔 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_dma_buf, RS485_RXBUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);   /* 禁用半传输中断，仅由空闲中断触发一次拷贝 */
}

/**
 * @brief  发送一帧 Modbus 数据（中断方式，非阻塞立即返回）
 * @param  buf 待发送数据
 * @param  len 长度
 * @retval HAL_OK 已启动发送；其他为 HAL 错误
 * @note   发送完成(TC)中断会自动切回接收模式并给 xSemTx 信号量
 */
HAL_StatusTypeDef RS485_SendFrame(uint8_t *buf, uint16_t len)
{
    xSemaphoreTake(xSemTx, 0);   /* 清残留的"完成"信号 */
    RS485_TxMode();              /* 切发送方向 */
    return HAL_UART_Transmit_IT(&huart1, buf, len);  /* 中断发送；TC 中断回调自动切回接收 */
}

/**
 * @brief  等待发送完成（配合 RS485_SendFrame 使用）
 * @retval 0=完成，-1=超时
 */
int RS485_WaitTxDone(uint32_t timeout_ms) 
{
    return (xSemaphoreTake(xSemTx, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? 0 : -1;  
}

/**
 * @brief  等待从机回复帧到达（空闲中断触发后置位）
 * @retval 0=已收到帧（数据在 rs485_rx_buf，长度在 rs485_rx_len），-1=超时
 */
int RS485_WaitRxDone(uint32_t timeout_ms) 
{
    return (xSemaphoreTake(xSemRx, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? 0 : -1;                
}

/* UART 发送完成回调（TC 中断触发，HAL 自动调用） */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        BaseType_t hpw = pdFALSE;
        /* 在 TC（而非 TXE）中断切换方向，保证末位数据完整发出 */
        RS485_RxMode();
        xSemaphoreGiveFromISR(xSemTx, &hpw);   /* 通知 Modbus 层发送完成 */
        portYIELD_FROM_ISR(hpw);
    }
}

/* UART 接收事件回调（总线空闲中断触发） */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        BaseType_t hpw = pdFALSE;
        uint16_t n = (Size <= RS485_RXBUF_SIZE) ? Size : RS485_RXBUF_SIZE;  /* 防越界 */
        memcpy(rs485_rx_buf, rs485_dma_buf, n);   /* DMA 缓冲 → 解析缓冲 */
        rs485_rx_len  = n;
        /* Normal 模式下 DMA 传输完成不会自动重启，需手动重新武装 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_dma_buf, RS485_RXBUF_SIZE);
        xSemaphoreGiveFromISR(xSemRx, &hpw);      /* 通知 Modbus 层已收到一帧 */
        portYIELD_FROM_ISR(hpw);
    }
}


/* Modbus 层发帧前调用：清掉上一帧残留的接收信号 */
void RS485_ClearRxSem(void)
{
    xSemaphoreTake(xSemRx, 0);
}

