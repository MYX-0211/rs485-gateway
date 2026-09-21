/* ============================================================================
 * w25q64.c — W25Q64（8 MB SPI NOR Flash）驱动实现
 * ---------------------------------------------------------------------------
 * 分层：
 *   底层（static）—— 片选控制 / 忙等待 / 写使能 / 读状态寄存器 / 单页编程
 *   中层           —— 读 / 写（自动跨页拆分） / 擦除（扇区、块、区域）
 *   上层           —— Init 自检 / SelfTest 读写一致性验证
 *
 * 由 OTA 流程调用（固件暂存 + 版本备份），不涉及业务参数（参数见 param.c）。
 * ==========================================================================*/
#include "w25q64.h"
#include "spi.h"        /* hspi1：SPI1 句柄（CubeMX 生成，定义在 spi.c） */
#include <string.h>     /* memcmp / memset */
#include <stdio.h>      /* printf：诊断输出；不加会报 #223-D declared implicitly */

/* ======================== W25Q64 指令集 ======================== */
#define CMD_WRITE_ENABLE      0x06U   /* 写使能：每次写/擦前必发 */
#define CMD_READ_STATUS1      0x05U   /* 读状态寄存器 1（bit0=BUSY, bit1=WEL, bit2-6=保护位） */
#define CMD_READ_DATA         0x03U   /* 标准读（最高约 50 MHz） */
#define CMD_PAGE_PROGRAM      0x02U   /* 页编程（≤256 B，不可跨页） */
#define CMD_SECTOR_ERASE      0x20U   /* 擦除 4 KB 扇区 */
#define CMD_BLOCK_ERASE_64K   0xD8U   /* 擦除 64 KB 块 */
#define CMD_JEDEC_ID          0x9FU   /* 读 JEDEC ID（回 3 字节） */
#define CMD_WRITE_STATUS1     0x01U   /* 写状态寄存器 1 */

/* SPI 分块传输上限：HAL_SPI_Transmit/Receive 的 Size 形参是 uint16_t，
 * 超过 65535 会截断 —— 因此长读写必须自己分块。 */
#define SPI_CHUNK_MAX         4096U

/* 状态寄存器 1 的保护位掩码：BP0-BP2(bit2-4) + TB(bit5) + SEC(bit6) */
#define SR1_PROTECT_MASK      0x7CU

/* ---------------- 片选控制（低电平选中）---------------- */
#define CS_LOW()   HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET)

/* ======================== 底层（static） ======================== */

/**
 * @brief  读状态寄存器 1（发完命令后可连续读，高位在前）
 */
static uint8_t W25Q64_ReadSR(void)
{
    uint8_t cmd = CMD_READ_STATUS1, sr = 0xFFU;

    CS_LOW();
    (void)HAL_SPI_Transmit(&hspi1, &cmd, 1U, 100U);
    (void)HAL_SPI_Receive(&hspi1, &sr, 1U, 100U);
    CS_HIGH();
    return sr;
}

/**
 * @brief  等待芯片空闲（轮询 SR1 的 BUSY 位）
 * @param  timeout_ms 超时毫秒数（页编程 ~1 ms、4KB 扇区擦 ~45 ms、64KB 块擦 ~150 ms）
 * @retval W25Q64_OK / W25Q64_ERR_TIMEOUT
 * @note   写/擦命令发出后必须等 BUSY 清零再发下一条，否则命令被忽略（典型隐蔽故障）
 */
static int W25Q64_WaitBusy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (W25Q64_ReadSR() & 0x01U) {
        if ((HAL_GetTick() - t0) > timeout_ms) return W25Q64_ERR_TIMEOUT;
    }
    return W25Q64_OK;
}

/**
 * @brief  发送写使能，并确认 WEL 位已置起
 * @retval W25Q64_OK / W25Q64_ERR_SPI
 * @note   每次页编程/擦除前都要重新发（WEL 在每条写/擦命令后自动清零）
 */
static int W25Q64_WriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    CS_LOW();
    (void)HAL_SPI_Transmit(&hspi1, &cmd, 1U, 100U);
    CS_HIGH();

    /* 确认写使能真的生效（读回 SR1 的 bit1） */
    return ((W25Q64_ReadSR() & 0x02U) != 0U) ? W25Q64_OK : W25Q64_ERR_SPI;
}

/**
 * @brief  单页编程（内部用；调用者保证不跨页）
 * @param  len ≤ W25Q64_PAGE_SIZE
 */
static int W25Q64_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4];

    if ((len == 0U) || (len > W25Q64_PAGE_SIZE)) return W25Q64_ERR_PARAM;
    if (W25Q64_WriteEnable() != W25Q64_OK)       return W25Q64_ERR_SPI;

    cmd[0] = CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);

    CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4U, 500U) != HAL_OK) {
        CS_HIGH();
        return W25Q64_ERR_SPI;
    }
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 1000U) != HAL_OK) {
        CS_HIGH();
        return W25Q64_ERR_SPI;
    }
    CS_HIGH();

    return W25Q64_WaitBusy(10U);      /* 页编程典型 0.7 ms，给 10 ms 余量 */
}

/* ======================== 中层：读 ======================== */

int W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];

    if ((buf == NULL) || (addr + len > W25Q64_SIZE)) return W25Q64_ERR_PARAM;
    if (len == 0U) return W25Q64_OK;
    if (W25Q64_WaitBusy(1000U) != W25Q64_OK) return W25Q64_ERR_TIMEOUT;

    cmd[0] = CMD_READ_DATA;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);

    CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4U, 500U) != HAL_OK) {
        CS_HIGH();
        return W25Q64_ERR_SPI;
    }

    /* 分块收：HAL 的 Size 是 uint16_t，一次不能超过 65535（这里取 4 KB 更稳） */
    while (len > 0U) {
        uint16_t chunk = (len > SPI_CHUNK_MAX) ? (uint16_t)SPI_CHUNK_MAX : (uint16_t)len;
        if (HAL_SPI_Receive(&hspi1, buf, chunk, 2000U) != HAL_OK) {
            CS_HIGH();
            return W25Q64_ERR_SPI;
        }
        buf  += chunk;
        len  -= chunk;
    }
    CS_HIGH();
    return W25Q64_OK;
}

/* ======================== 中层：写（自动跨页拆分） ======================== */

int W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if ((buf == NULL) || (addr + len > W25Q64_SIZE)) return W25Q64_ERR_PARAM;

    while (len > 0U) {
        /* 当前页内还能写多少字节 —— 页编程不能跨页，必须在这里切断 */
        uint16_t page_off = (uint16_t)(addr & (W25Q64_PAGE_SIZE - 1U));
        uint16_t chunk    = (uint16_t)(W25Q64_PAGE_SIZE - page_off);
        int      ret;

        if ((uint32_t)chunk > len) chunk = (uint16_t)len;

        ret = W25Q64_PageProgram(addr, buf, chunk);
        if (ret != W25Q64_OK) return ret;

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return W25Q64_OK;
}

/* ======================== 中层：擦除 ======================== */

int W25Q64_EraseSector(uint32_t addr)
{
    uint8_t cmd[4];

    addr &= ~(W25Q64_SECTOR_SIZE - 1U);          /* 对齐到 4 KB 扇区边界 */
    if (addr >= W25Q64_SIZE) return W25Q64_ERR_PARAM;
    if (W25Q64_WaitBusy(1000U) != W25Q64_OK) return W25Q64_ERR_TIMEOUT;
    if (W25Q64_WriteEnable() != W25Q64_OK)   return W25Q64_ERR_SPI;

    cmd[0] = CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);

    CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4U, 500U) != HAL_OK) {
        CS_HIGH();
        return W25Q64_ERR_SPI;
    }
    CS_HIGH();

    return W25Q64_WaitBusy(500U);     /* 4 KB 扇区擦典型 45 ms，给 500 ms 余量 */
}

int W25Q64_EraseBlock64K(uint32_t addr)
{
    uint8_t cmd[4];

    addr &= ~(W25Q64_BLOCK_SIZE - 1U);           /* 对齐到 64 KB 块边界 */
    if (addr >= W25Q64_SIZE) return W25Q64_ERR_PARAM;
    if (W25Q64_WaitBusy(1000U) != W25Q64_OK) return W25Q64_ERR_TIMEOUT;
    if (W25Q64_WriteEnable() != W25Q64_OK)   return W25Q64_ERR_SPI;

    cmd[0] = CMD_BLOCK_ERASE_64K;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);

    CS_LOW();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4U, 500U) != HAL_OK) {
        CS_HIGH();
        return W25Q64_ERR_SPI;
    }
    CS_HIGH();

    return W25Q64_WaitBusy(1000U);    /* 64 KB 块擦典型 150 ms，给 1 s 余量 */
}

int W25Q64_EraseRange(uint32_t addr, uint32_t len)
{
    uint32_t end;

    if (len == 0U) return W25Q64_OK;

    addr &= ~(W25Q64_SECTOR_SIZE - 1U);                 /* 起始对齐（向下） */
    end   = (addr + len + W25Q64_SECTOR_SIZE - 1U)      /* 结束对齐（向上） */
            & ~(W25Q64_SECTOR_SIZE - 1U);

    if (end > W25Q64_SIZE) return W25Q64_ERR_PARAM;

    /* 整 64 KB 块优先用块擦（快得多），首尾不足块的部分按扇区擦 */
    while ((addr < end) && ((addr % W25Q64_BLOCK_SIZE) == 0U) &&
           ((end - addr) >= W25Q64_BLOCK_SIZE)) {
        int ret = W25Q64_EraseBlock64K(addr);
        if (ret != W25Q64_OK) return ret;
        addr += W25Q64_BLOCK_SIZE;
    }
    while (addr < end) {
        int ret = W25Q64_EraseSector(addr);
        if (ret != W25Q64_OK) return ret;
        addr += W25Q64_SECTOR_SIZE;
    }
    return W25Q64_OK;
}

/* ======================== 上层：识别与自检 ======================== */

uint32_t W25Q64_ReadID(void)
{
    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t id[3] = {0xFFU, 0xFFU, 0xFFU};

    CS_LOW();
    (void)HAL_SPI_Transmit(&hspi1, &cmd, 1U, 100U);
    (void)HAL_SPI_Receive(&hspi1, id, 3U, 100U);
    CS_HIGH();

    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | (uint32_t)id[2];
}

int W25Q64_Init(void)
{
    uint32_t id;
    uint8_t  sr;

    CS_HIGH();          /* 先确保不选中 */
    HAL_Delay(10U);     /* 上电后芯片需要一点时间就绪 */

    id = W25Q64_ReadID();
    printf("[W25Q64] JEDEC ID=0x%06lX (expect 0x%06lX)\r\n",
           (unsigned long)id, (unsigned long)W25Q64_JEDEC_ID);

    if (id != W25Q64_JEDEC_ID) {
        /* 常见原因：VCC 接了 5V、CS/CLK/DO/DI 接错、模块没供电 */
        printf("[W25Q64] ID mismatch! check 3.3V/GND/CLK/DO/DI/CS wiring\r\n");
        return W25Q64_ERR_ID;
    }

    /* 检查块保护位：若被写保护，写/擦会静默失效（不报错但数据没变），是极隐蔽的故障 */
    sr = W25Q64_ReadSR();
    if ((sr & SR1_PROTECT_MASK) != 0U) {
        uint8_t cmd[2];
        printf("[W25Q64] SR1=0x%02X protected, clearing...\r\n", sr);
        if (W25Q64_WriteEnable() == W25Q64_OK) {
            cmd[0] = CMD_WRITE_STATUS1;
            cmd[1] = 0x00U;                     /* 清 BP0-BP2 / TB / SEC */
            CS_LOW();
            (void)HAL_SPI_Transmit(&hspi1, cmd, 2U, 100U);
            CS_HIGH();
            (void)W25Q64_WaitBusy(500U);
        }
        sr = W25Q64_ReadSR();
        if ((sr & SR1_PROTECT_MASK) != 0U) {
            printf("[W25Q64] WARN still protected (SR1=0x%02X) - WP# pin?\r\n", sr);
        }
    }

    printf("[W25Q64] OK, %lu KB ready\r\n", (unsigned long)(W25Q64_SIZE / 1024U));
    return W25Q64_OK;
}

int W25Q64_SelfTest(uint32_t addr)
{
    uint8_t  wr[W25Q64_PAGE_SIZE];
    uint8_t  rd[W25Q64_PAGE_SIZE];
    uint32_t i;
    int      ret;

    addr &= ~(W25Q64_SECTOR_SIZE - 1U);      /* 对齐到扇区（本函数会擦掉该扇区！） */

    printf("[W25Q64] selftest @0x%06lX...\r\n", (unsigned long)addr);

    ret = W25Q64_EraseSector(addr);
    if (ret != W25Q64_OK) { printf("[W25Q64] erase fail (%d)\r\n", ret); return ret; }

    for (i = 0U; i < W25Q64_PAGE_SIZE; i++) wr[i] = (uint8_t)i;   /* 递增模式，能测出位粘连 */

    ret = W25Q64_Write(addr, wr, W25Q64_PAGE_SIZE);
    if (ret != W25Q64_OK) { printf("[W25Q64] write fail (%d)\r\n", ret); return ret; }

    memset(rd, 0, sizeof(rd));
    ret = W25Q64_Read(addr, rd, W25Q64_PAGE_SIZE);
    if (ret != W25Q64_OK) { printf("[W25Q64] read fail (%d)\r\n", ret); return ret; }

    if (memcmp(wr, rd, W25Q64_PAGE_SIZE) != 0) {
        for (i = 0U; i < W25Q64_PAGE_SIZE; i++) {
            if (wr[i] != rd[i]) {
                printf("[W25Q64] MISMATCH at +%lu: wr=0x%02X rd=0x%02X\r\n",
                       (unsigned long)i, (unsigned)wr[i], (unsigned)rd[i]);
                break;
            }
        }
        return W25Q64_ERR_SPI;
    }

    printf("[W25Q64] selftest PASS (%u B verified)\r\n", (unsigned)W25Q64_PAGE_SIZE);
    return W25Q64_OK;
}
