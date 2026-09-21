#ifndef __W25Q64_H
#define __W25Q64_H

/* ============================================================================
 * w25q64.h — W25Q64（8 MB / 64 Mbit SPI NOR Flash）驱动
 * ---------------------------------------------------------------------------
 * 在本工程里的角色：只承担 OTA 固件暂存与版本备份（不存放业务参数，
 * 参数仍在内部 Flash 扇区 11，见 param.c）。
 *
 * 为什么 OTA 用外部 Flash 而不是内部 Flash：
 *   STM32F4 擦写内部Flash 时，同一 Bank 的取指会被 stall（擦 128 KB 扇区约 1 s）。
 *   而 OTA 收包是在 APP 正常运行期进行的（Modbus 轮询 + MQTT 上报同时在跑），
 *   内部 Flash 的 1 s stall 会直接破坏总线时序与心跳。
 *   外部 SPI Flash 通过总线访问，不影响内部取指 -> 收包期间业务不中断。
 *
 * 硬件：SPI1（PA5=SCK / PA6=MISO / PA7=MOSI）+ 软件 CS（PA4）
 *       SPI Mode 0（CPOL=Low, CPHA=1Edge）、8 bit、MSB first、约 10.5 MHz
 * ==========================================================================*/

#include "main.h"

/* ---------------- 容量与粒度 ---------------- */
#define W25Q64_SIZE           0x800000UL     /* 8 MB = 64 Mbit */
#define W25Q64_PAGE_SIZE      256U           /* 页编程粒度：一次最多写 256 B，且不能跨页 */
#define W25Q64_SECTOR_SIZE    4096U          /* 扇区擦除粒度：4 KB（最小可擦单位） */
#define W25Q64_BLOCK_SIZE     65536U         /* 块擦除粒度：64 KB */
#define W25Q64_SECTOR_COUNT   (W25Q64_SIZE / W25Q64_SECTOR_SIZE)   /* 2048 个扇区 */

/* ---------------- JEDEC ID ---------------- */
/* 0x9F 回 3 字节：厂商(Winbond=EF) + 存储类型(SPI NOR=40) + 容量(8MB=17) */
#define W25Q64_JEDEC_ID       0xEF4017UL

/* ---------------- OTA 分区宏已移至 ota.h ----------------
 * 本文件是驱动层，只关心容量/粒度/指令；OTA 的暂存区与备份区布局属于
 * 应用约定，统一放在 Core/Inc/ota.h（APP 与 Bootloader 共用同一份定义）。
 * -> 需要 OTA_STAGE_ADDR / OTA_BACKUP_ADDR / OTA_REGION_SIZE 请 #include "ota.h"。 */

/* ---------------- 返回值 ---------------- */
#define W25Q64_OK             0
#define W25Q64_ERR_ID        -1     /* JEDEC ID 不符（没接好 / 型号不对） */
#define W25Q64_ERR_PARAM     -2     /* 地址或长度越界 */
#define W25Q64_ERR_TIMEOUT   -3     /* 等 BUSY 超时 */
#define W25Q64_ERR_SPI       -4     /* SPI 传输失败 */

/* ============================ 接口 ============================ */

/**
 * @brief  初始化并自检：读 JEDEC ID + 检查/解除块保护
 * @retval W25Q64_OK(0)=通过；负值见上面的错误码
 * @note   只需在上电初始化时调用一次（读 ID 是 µs 级，可放调度器启动前）。
 *         若返回 W25Q64_ERR_ID，先查接线（VCC/GND/CLK/DO/DI/CS）与供电（必须 3.3 V）。
 */
int W25Q64_Init(void);

/**
 * @brief  读 JEDEC ID（0x9F，返回 24 位）
 * @retval 正常应为 0xEF4017；读回 0xFFFFFF / 0x000000 说明通信有问题
 */
uint32_t W25Q64_ReadID(void);

/**
 * @brief  读任意长度数据
 * @param  addr 起始地址（24 位）
 * @param  buf  目标缓冲
 * @param  len  字节数
 * @retval W25Q64_OK / 负值错误码
 */
int W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief  写任意长度数据（自动按 256 B 页边界拆分，调用者无需关心跨页）
 * @param  addr 起始地址
 * @param  buf  源数据
 * @param  len  字节数
 * @retval W25Q64_OK / 负值错误码
 * @note   写之前该区域必须已被擦除（NOR Flash 只能把 1 写成 0，不能把 0 写回 1）。
 *         需要覆盖写时请先调用 W25Q64_EraseSector / W25Q64_EraseRange。
 */
int W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len);

/**
 * @brief  擦除一个 4 KB 扇区（地址会被自动对齐到扇区边界）
 * @note   擦除耗时典型 ~45 ms，期间芯片 BUSY，函数会等它完成
 */
int W25Q64_EraseSector(uint32_t addr);

/**
 * @brief  擦除一个 64 KB 块（地址会被自动对齐到块边界）
 * @note   比逐扇区擦快得多；典型 ~150 ms
 */
int W25Q64_EraseBlock64K(uint32_t addr);

/**
 * @brief  擦除一段区域（按 4 KB 扇区对齐，含头含尾）
 * @param  addr 起始地址
 * @param  len  字节长度（内部向上取整到扇区）
 * @note   OTA 场景在写入前用它一次性清出空间
 */
int W25Q64_EraseRange(uint32_t addr, uint32_t len);

/**
 * @brief  读写一致性自检：擦 1 个扇区 → 写入递增模式 → 读回逐字节比对
 * @param  addr 用于自检的地址（会被擦除！请选暂存区里的地址）
 * @retval W25Q64_OK=一致；负值=失败
 * @note   只用于上电/首次接线验证，不要放进正常业务流程（它会擦掉该扇区）。
 */
int W25Q64_SelfTest(uint32_t addr);

#endif
