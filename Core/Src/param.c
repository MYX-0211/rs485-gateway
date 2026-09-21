/* ============================================================================
 * param.c — 报警阈值掉电保存（Flash 扇区 11，日志式追加写）
 *   设计要点见 param.h 的注释：只追加、不擦除 → 断电不丢、绝不回默认值。
 *   擦除只在上电阶段且无空槽时发生，避开运行期取指 stall 对 Modbus 的干扰。
 * ==========================================================================*/
#include "param.h"
#include <string.h>
#include <stdio.h>      /* printf（诊断输出；不加会报 #223-D declared implicitly） */

/* 阈值本体在 main.c 定义 */
extern volatile float g_temp_alarm_high;
extern volatile float g_temp_alarm_clear;

/**
 * @brief  CRC16-Modbus（与 modbus.c 同一算法；此处独立实现，避免跨模块耦合）
 */
static uint16_t Param_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8U; b++)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
    return crc;
}

/**
 * @brief  判断一槽是否有效：magic 正确 且 前 16 字节的 CRC 与槽内 crc 低 16 位一致
 * @note   写入过程中断电 → 该槽 CRC 不匹配 → 视为无效，自动退回上一槽 ✓
 */
static int Param_SlotValid(const ParamSlot_t *s)
{
    if (s->magic != PARAM_MAGIC) return 0;
    return (Param_CRC16((const uint8_t *)s, 16U) == (uint16_t)(s->crc & 0xFFFFU)) ? 1 : 0;
}

/**
 * @brief  组装一槽内容（不含写操作）
 */
static void Param_BuildSlot(ParamSlot_t *slot, uint32_t seq, float hi, float lo)
{
    memset(slot, 0, sizeof(ParamSlot_t));
    slot->magic = PARAM_MAGIC;
    slot->seq   = seq;
    slot->hi    = hi;
    slot->lo    = lo;
    slot->crc   = Param_CRC16((const uint8_t *)slot, 16U);
}

/**
 * @brief  把一槽写到指定槽下标（调用前需已 Unlock，且该槽必须已擦除）
 */
static int Param_ProgramSlot(uint32_t idx, const ParamSlot_t *slot)
{
    const uint32_t *src = (const uint32_t *)slot;
    uint32_t base = PARAM_SECTOR_ADDR + idx * PARAM_SLOT_SIZE;
    uint32_t w;

    for (w = 0; w < (PARAM_SLOT_SIZE / 4U); w++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base + w * 4U, src[w]) != HAL_OK) {
            printf("[PARAM] program failed at slot=%lu word=%lu\r\n",
                   (unsigned long)idx, (unsigned long)w);
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  整理：擦整个扇区后把最新值写回槽 0
 * @note   只在上电阶段、且在「扇区已无空槽」时调用 ——
 *         STM32F4 擦写 Flash 会让同一 Bank 的取指 stall（128KB 约 1s），
 *         放在运行期会直接破坏 Modbus 时序，所以推迟到上电时做。
 */
static void Param_Compact(float hi, float lo)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t sector_err = 0U;
    ParamSlot_t slot;

    Param_BuildSlot(&slot, 1U, hi, lo);

    if (HAL_FLASH_Unlock() != HAL_OK) {
        printf("[PARAM] erase skipped: flash unlock failed\r\n");
        return;
    }

    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;     /* 2.7 ~ 3.6 V */
    er.Sector       = FLASH_SECTOR_11;
    er.NbSectors    = 1U;

    if (HAL_FLASHEx_Erase(&er, &sector_err) != HAL_OK) {
        HAL_FLASH_Lock();
        printf("[PARAM] erase failed err=%lu\r\n", (unsigned long)sector_err);
        return;
    }
    (void)Param_ProgramSlot(0U, &slot);
    HAL_FLASH_Lock();

    printf("[PARAM] compacted hi=%.1f lo=%.1f\r\n", (double)hi, (double)lo);
}

int Param_Load(void)
{
    const ParamSlot_t *p = (const ParamSlot_t *)PARAM_SECTOR_ADDR;
    const ParamSlot_t *best = NULL;
    uint32_t free_idx = PARAM_SLOT_MAX;
    uint32_t i;

    for (i = 0U; i < PARAM_SLOT_MAX; i++) {
        if (p[i].magic == PARAM_ERASED_WORD) {
            if (free_idx >= PARAM_SLOT_MAX) free_idx = i;   /* 记住第一个空槽 */
            continue;                                       /* 空槽之后仍是空槽，继续扫完更稳 */
        }
        if (Param_SlotValid(&p[i])) {
            if ((best == NULL) || (p[i].seq > best->seq)) best = &p[i];
        }
    }

    /* 无空槽 → 上电整理（擦除 + 写回最新值）；此刻调度器未启动，stall 无影响 */
    if (free_idx >= PARAM_SLOT_MAX) {
        float hi = (best != NULL) ? best->hi : g_temp_alarm_high;
        float lo = (best != NULL) ? best->lo : g_temp_alarm_clear;
        Param_Compact(hi, lo);
        g_temp_alarm_high  = hi;
        g_temp_alarm_clear = lo;
        return (best != NULL) ? 1 : 0;
    }

    if (best == NULL) return 0;        /* 首次上电：无有效参数，用默认值，且不写 Flash */

    g_temp_alarm_high  = best->hi;
    g_temp_alarm_clear = best->lo;
    printf("[PARAM] loaded hi=%.1f lo=%.1f slot=%lu\r\n",
           (double)best->hi, (double)best->lo, (unsigned long)(best - p));
    return 1;
}

int Param_Save(void)
{
    const ParamSlot_t *p = (const ParamSlot_t *)PARAM_SECTOR_ADDR;
    ParamSlot_t slot;
    uint32_t idx = PARAM_SLOT_MAX;
    uint32_t next_seq = 1U;
    uint32_t i;

    /* 1. 找第一个空槽，并推算下一个 seq */
    for (i = 0U; i < PARAM_SLOT_MAX; i++) {
        if (p[i].magic == PARAM_ERASED_WORD) { idx = i; break; }
        if (Param_SlotValid(&p[i]) && (p[i].seq >= next_seq)) next_seq = p[i].seq + 1U;
    }
    if (idx >= PARAM_SLOT_MAX) {
        /* 不在这里擦除（1s stall 会破坏 Modbus 时序）→ 退化为"本次不保存"，
         * 下次上电由 Param_Load 的整理流程回收。5461 次修改才会遇到一次。 */
        printf("[PARAM] no free slot, save skipped (compact on next boot)\r\n");
        return 1;
    }

    /* 2. 组装快照 */
    Param_BuildSlot(&slot, next_seq, g_temp_alarm_high, g_temp_alarm_clear);

    /* 3. 追加写入（24 字节 = 6 个字，µs 级） */
    if (HAL_FLASH_Unlock() != HAL_OK) {
        printf("[PARAM] save failed: flash unlock failed\r\n");
        return 2;
    }
    if (Param_ProgramSlot(idx, &slot) != 0) {
        HAL_FLASH_Lock();
        return 3;
    }
    HAL_FLASH_Lock();

    printf("[PARAM] saved hi=%.1f lo=%.1f seq=%lu slot=%lu\r\n",
           (double)slot.hi, (double)slot.lo,
           (unsigned long)slot.seq, (unsigned long)idx);
    return 0;
}
