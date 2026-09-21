#ifndef __MODBUS_H
#define __MODBUS_H
/* ============================================================================
 * modbus.h — Modbus RTU 主站协议层接口
 * ==========================================================================*/
#include "main.h"

/**
 * @brief  CRC16-Modbus 计算（多项式 0xA001，初值 0xFFFF，低字节在前）
 */
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len);

/**
 * @brief  Modbus 0x03 读保持寄存器（只读/可读写参数区）
 * @param  addr 从机地址（如 0x01 光照 / 0x02 温湿度）
 * @param  reg  起始寄存器地址
 * @param  num  要读的寄存器数量
 * @param  out  解析结果数组指针（大端序合并）
 * @param  timeout_ms 单次收发超时毫秒数
 * @retval 0=成功; -1=发送失败/超时; -2=接收超时(从机无响应)
 *         -3=CRC校验失败; -4=从机异常响应(0x83); -5=帧格式错误/地址不符
 */
int Modbus_ReadHoldingRegs(uint8_t addr, uint16_t reg, uint16_t num,
                           uint16_t *out, uint32_t timeout_ms);

/**
 * @brief  Modbus 0x04 读输入寄存器（只读测量值区）
 * @note   与 0x03 共用同一套底层事务，仅功能码不同。
 *         Modbus 的标准分工：**输入寄存器放只读的实时测量值，保持寄存器放可读写参数**，
 *         所以传感器类从机的测量值通常走 0x04（以各从机手册为准）。
 * @retval 同 0x03，但**异常响应码为 0x84**
 */
int Modbus_ReadInputRegs(uint8_t addr, uint16_t reg, uint16_t num,
                         uint16_t *out, uint32_t timeout_ms);

#endif                   
