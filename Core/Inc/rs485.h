#ifndef __RS485_H
#define __RS485_H

/* ============================================================================
 * rs485.h — RS485 半双工总线驱动接口（USART1 + MAX485）
 * 使用流程：
 *   1. 外设初始化后调用 RS485_Init() 启动空闲中断 + DMA 接收
 *   2. 发帧：RS485_ClearRxSem() → RS485_SendFrame() → RS485_WaitTxDone()
 *   3. 收帧：RS485_WaitRxDone() → 数据在 rs485_rx_buf / rs485_rx_len
 * ==========================================================================*/

#include "main.h"
#include "usart.h"

#define RS485_DE_PORT   RS485_DE_GPIO_Port   /* DE 方向控制引脚（CubeMX 生成） */
#define RS485_DE_PIN    RS485_DE_Pin
#define RS485_RXBUF_SIZE 64                  /* 接收缓冲：0x03 读 N 寄存器回复最长 5+2N 字节 */


extern volatile uint16_t rs485_rx_len;                  // 实际接收到的字节数，由空闲中断回调写入
extern uint8_t rs485_rx_buf[RS485_RXBUF_SIZE];          // 解析用接收缓冲区（DMA 写私有缓冲后由回调拷贝到此）

void RS485_Init(void);                                        // 初始化：创建信号量 + 启动空闲中断+DMA接收
HAL_StatusTypeDef RS485_SendFrame(uint8_t *buf, uint16_t len); // 发送帧（IT 方式，非阻塞立即返回）
int RS485_WaitTxDone(uint32_t timeout_ms);                    // 等待发送完成：0=成功 / -1=超时
int RS485_WaitRxDone(uint32_t timeout_ms);                    // 等待接收完成：0=已收到帧 / -1=超时
void RS485_ClearRxSem(void);                                  // 清掉上一帧残留的接收信号（发帧前调用）

#endif

