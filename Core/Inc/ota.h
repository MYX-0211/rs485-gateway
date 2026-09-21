#ifndef __OTA_H
#define __OTA_H

/* ============================================================================
 * ota.h — OTA 升级的共享定义（APP 与 Bootloader 两个 Target 都用）
 * ---------------------------------------------------------------------------
 * 这里是唯一权威来源：Flash 分区地址、W25Q64 暂存区布局、暂存区头部结构、
 * CRC 算法都定义在此，避免 APP / Bootloader 各写一份导致"两边约定不一致"。
 *
 * 改动本文件后，两个 Target 都必须重新编译（BOOT 与 rs485_gateway），
 *    并重新烧录 BOOT —— 否则两边对固件格式的理解会错位。
 * ==========================================================================*/

#include <stdint.h>

/* ---------------- 内部 Flash 分区（必须与 Keil Target 的 IROM 一致）---------------
 * rs485_gateway(APP) Target : IROM1 Start=0x08010000 Size=0xB0000
 * BOOT           Target : IROM1 Start=0x08000000 Size=0x10000
 * Keil 的链接地址实际取自 Target 的 <Cpu> 字段里的 IROM(start-end)，
 *    <OnChipMemories>/<IROM> 只是对话框镜像 —— 改地址时两处都要改（详见项目记忆）。 */
#define OTA_APP_ADDR        0x08010000UL           /* APP 起始 */
#define OTA_APP_MAX_SIZE    0xB0000UL              /* 704 KB */
#define OTA_APP_END         (OTA_APP_ADDR + OTA_APP_MAX_SIZE - 1U)

/* ---------------- W25Q64 里的暂存区 / 备份区 ---------------- */
#define OTA_STAGE_ADDR      0x000000UL             /* 新固件暂存区（收包写这里） */
#define OTA_BACKUP_ADDR     0x100000UL             /* 上一版固件备份区（1 MB 偏移） */
#define OTA_REGION_SIZE     0x100000UL             /* 每区 1 MB */

/* ---------------- 暂存区头部 ---------------- */
#define OTA_MAGIC           0x4F544153UL           /* 'OTAS'：magic 相符即表示"有已收齐的固件" */
#define OTA_HDR_SIZE        16U                    /* 头部 16 字节；固件数据从 STAGE_ADDR+16 开始 */

typedef struct {
    uint32_t magic;        /* = OTA_MAGIC */
    uint32_t fw_size;      /* 固件字节数（不含头部） */
    uint32_t fw_crc;       /* 整包 CRC16（低 16 位有效） */
    uint32_t version;      /* 版本号（自增，或编译时间戳，由 APP 侧填写） */
} OtaStageHeader_t;

/* 编译期断言：头部必须正好 OTA_HDR_SIZE 字节，否则固件数据的偏移会算错 */
typedef char ota_hdr_size_assert[(sizeof(OtaStageHeader_t) == OTA_HDR_SIZE) ? 1 : -1];

/* ============================ CRC16 ============================ */

/**
 * @brief  CRC16-Modbus可续算版本—— 供分块累加使用
 * @param  crc 上一段的 CRC 中间值（首次调用传 0xFFFF）
 * @param  buf 本段数据首地址
 * @param  len 本段字节数
 * @retval 累加后的 CRC
 * @note   固件有几十 KB，不可能整段读进 RAM 再算 —— 必须分块累加，
 *         所以校验逻辑要用这个版本，而不是下面的整段版本。
 */
static inline uint16_t Ota_CRC16Update(uint16_t crc, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint8_t  b;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (b = 0U; b < 8U; b++) {
            if (crc & 1U) { crc = (uint16_t)((crc >> 1) ^ 0xA001U); }
            else          { crc = (uint16_t)(crc >> 1); }
        }
    }
    return crc;
}

/**
 * @brief  CRC16-Modbus（整段版本）
 * @note   等价于 Ota_CRC16Update(0xFFFFU, buf, len)。测试/短数据用这个更直观。
 *         固件完整性采用「CRC16 + fw_size」双重判据：长度不符直接拒绝，
 *            因此 CRC16 的 1/65536 漏检率在工程上可忽略。
 */
static inline uint16_t Ota_CRC16(const uint8_t *buf, uint32_t len)
{
    return Ota_CRC16Update(0xFFFFU, buf, len);
}

#endif
