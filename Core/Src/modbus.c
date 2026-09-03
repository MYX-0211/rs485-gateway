/* ============================================================================
 * modbus.c — Modbus RTU 主站协议层（RS485 总线）
 *   功能：CRC16-Modbus 校验 + 0x03 读保持寄存器完整事务
 *   事务流程：组帧 → 发送 → 等发送完成 → 等从机回复 → 帧校验 → 大端解析
 *   错误码：-1 发送失败/超时  -2 从机无回复  -3 CRC 错误
 *           -4 从机异常响应(0x83)  -5 帧格式错误/地址不符
 * ==========================================================================*/
#include "modbus.h"
#include "rs485.h"
#include <string.h>

/**
 * @brief  CRC16-Modbus 校验
 * @note   多项式 0xA001（标准 0x8005 的反转形式），初值 0xFFFF，
 *         结果低字节在前、高字节在后
 */
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];                  /* 逐字节混入 */
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001)           /* LSB=1：右移后异或反转多项式 */
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief  读保持寄存器（功能码 0x03）完整事务
 * @param  addr       从机地址
 * @param  reg        起始寄存器地址
 * @param  num        寄存器数量
 * @param  out        解析结果缓冲（大端序合并为 uint16_t）
 * @param  timeout_ms 单次收发超时
 * @retval 0=成功；其余见文件头错误码表
 */
int Modbus_ReadHoldingRegs(uint8_t addr, uint16_t reg, uint16_t num,
                           uint16_t *out, uint32_t timeout_ms)
{
    uint8_t req[8];  /* 请求帧：地址(1)+功能码(1)+起始地址(2)+数量(2)+CRC(2) */
    uint16_t crc;

    /* 1. 组帧（多字节字段大端序；CRC 低字节在前） */
    req[0] = addr;
    req[1] = 0x03;               /* 读保持寄存器 */
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = (uint8_t)(num >> 8);
    req[5] = (uint8_t)(num & 0xFF);
    crc    = Modbus_CRC16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    /* 2. 发送请求帧并等待物理发送完成（TC 中断） */
    RS485_ClearRxSem();               /* 清残留接收信号 */
    if (RS485_SendFrame(req, 8) != HAL_OK) return -1;
    if (RS485_WaitTxDone(timeout_ms) != 0)   return -1;

    /* 3. 等待从机回复（总线空闲中断判帧） */
    if (RS485_WaitRxDone(timeout_ms) != 0)   return -2;

    /* 4. 帧校验 */
    uint16_t expect = 5 + 2 * num;    /* 期望长度：地址+功能码+字节数+数据(2N)+CRC */
    if (rs485_rx_len < 5) return -5;  /* 短于最小有效帧（异常响应） */
    if (rs485_rx_buf[0] != addr) return -5;                  /* 地址不符（总线串扰） */
    if (rs485_rx_buf[1] == (uint8_t)(0x03 | 0x80)) return -4; /* 0x83：从机异常响应 */
    if (rs485_rx_len != expect) return -5;                   /* 长度不符 */
    crc = Modbus_CRC16(rs485_rx_buf, rs485_rx_len - 2);
    uint16_t crc_rx = ((uint16_t)rs485_rx_buf[rs485_rx_len - 1] << 8) |
                      (uint16_t)rs485_rx_buf[rs485_rx_len - 2];
    if (crc != crc_rx) return -3;     /* CRC 不匹配 */

    /* 5. 解析数据区（偏移 3 起，每寄存器 2 字节，大端序） */
    for (uint16_t i = 0; i < num; i++) {
        out[i] = ((uint16_t)rs485_rx_buf[3 + 2 * i] << 8) | rs485_rx_buf[4 + 2 * i];
    }
    return 0;
}
	


