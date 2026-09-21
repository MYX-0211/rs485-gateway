#ifndef __PARAM_H
#define __PARAM_H

/* ============================================================================
 * param.h — 报警阈值掉电保存（Flash 扇区 11，日志式追加写）
 * ---------------------------------------------------------------------------
 * 为什么不用"固定单槽 + 每次擦写"：
 *   STM32F4 只能按扇区擦除，擦 128KB 约 1s，擦除期间断电 → 参数丢失、回默认值。
 * 本模块的做法（单份数据 + 天然多版本，无需 A/B 双备份那套切换逻辑）：
 *   写入：只往扇区里第一个空槽追加（24 字节，µs 级），不擦除
 *   读取：扫全扇区，取「magic 正确 + CRC 有效 + seq 最大」的那一槽
 *   -> 任意时刻断电，之前写入的快照都还完整；最坏只丢"最后一次修改"，
 *      绝不会读出垃圾值、绝不会回默认值。
 * ==========================================================================*/

#include "main.h"

/* ---- 参数区（与 OTA 分区规划对齐，不要随意改地址）----
 * 扇区 11：0x080E0000 ~ 0x080FFFFF（128 KB）
 * 为什么是它：1 MB Flash 的最后一个扇区，与 APP（扇区 4–9）、
 *             固件暂存区（扇区 10）不重叠 → 将来做 OTA 调分区时不用动这里。 */
#define PARAM_SECTOR_ADDR   0x080E0000UL
#define PARAM_SECTOR_SIZE   0x20000UL                                 /* 128 KB */
#define PARAM_SLOT_SIZE     24UL                                      /* 24B = 6 个字（Flash 编程为字粒度） */
#define PARAM_SLOT_MAX      (PARAM_SECTOR_SIZE / PARAM_SLOT_SIZE)     /* 5461 个槽 */

#define PARAM_MAGIC         0x52534757UL        /* 'RSGW'：标记该槽已被写过 */
#define PARAM_ERASED_WORD   0xFFFFFFFFUL        /* 擦除后的 Flash 内容 */

/* 一槽 = 一次参数快照 */
typedef struct {
    uint32_t magic;   /* = PARAM_MAGIC */
    uint32_t seq;     /* 单调递增；读取时取「CRC 有效且 seq 最大」那槽 */
    float    hi;      /* 报警上限 ℃ */
    float    lo;      /* 回差解除线 ℃ */
    uint32_t crc;     /* 对本槽前 16 字节算的 CRC16（低 16 位有效） */
    uint32_t pad;     /* 补足 24 字节 */
} ParamSlot_t;

/**
 * @brief  上电加载参数（在调度器启动前调用）
 * @retval 1=从 Flash 成功加载；0=无有效参数（沿用代码里的默认值）
 * @note   若发现扇区已无空槽，会顺带整理一次（擦除 + 把最新值写回槽 0）。
 *         擦除放在上电阶段，是为了避开运行期 1s 取指 stall 对 Modbus 时序的干扰。
 */
int Param_Load(void);

/**
 * @brief  保存当前参数到 Flash（追加写，不擦除）
 * @retval 0=成功；非 0=失败（无空槽 / 擦写错误）
 * @note   只写 24 字节（µs 级），对中断延迟的影响可忽略。
 *         必须在任务上下文调用（内部会解锁/上锁 Flash，可能阻塞）。
 */
int Param_Save(void);

#endif
