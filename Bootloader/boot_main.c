/* ============================================================================
 * boot_main.c - OTA Bootloader（裸机，无 FreeRTOS）
 * ---------------------------------------------------------------------------
 * 职责：
 *   1. 读 W25Q64 暂存区的固件头，校验整包 CRC；
 *   2. 校验通过则擦除 APP 区并按扇区搬运，搬运后回读校验；
 *   3. 全部通过才清除升级标志，最后跳转到 APP。
 *   校验不通过时直接跳 APP，保持旧固件可运行。
 *
 * 为什么 Bootloader 不复用 Core/Src/main.c：
 *   main.c 里包含 FreeRTOS 的任务创建与调度器启动，以及业务初始化
 *   （RS485/OLED/ESP）。Bootloader 必须保持裸机运行 —— 写 Flash 期间
 *   不能被打断；因此 BOOT Target 排除 main.c，改用本文件。本文件自带
 *   SystemClock_Config()、Error_Handler()、fputc() 与 HAL 时基回调
 *   （这些原本都定义在 main.c 中）。
 *
 * 跳转到 APP 前的清理步骤：
 *   关中断 -> 关 SysTick -> 清 NVIC 使能与挂起位 -> 重设向量表 VTOR ->
 *   __set_MSP(APP 栈顶) -> 取 Reset_Handler 地址并跳转。
 * ==========================================================================*/
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "spi.h"
#include "w25q64.h"
#include "ota.h"          /* OTA 共享定义：分区地址 / 暂存区头部 / CRC16（APP 与 BOOT 同一份） */
#include <stdio.h>

/* 分区地址（OTA_APP_ADDR / OTA_STAGE_ADDR / OTA_MAGIC / OtaStageHeader_t）
 * 全部来自 ota.h，不在此重复定义 —— 避免两边约定不一致。 */
#define APP_SP_MIN      0x20000000UL      /* SRAM 起始 */
#define APP_SP_MAX      0x20020000UL      /* SRAM 结束（F407VG 128KB） */

/* ======================== 自带的最小支撑 ======================== */

/**
 * @brief  HAL 时基回调（TIM6 更新中断里被调用）—— uwTick 就是在这里增长的
 * @note   Bootloader 必须自己实现这个回调
 *         原实现位于 Core/Src/main.c 的 "USER CODE BEGIN 4" 区（CubeMX 生成），
 *         而 BOOT Target 排除了 main.c -> 该强符号缺失 -> 链接器退回到
 *         stm32f4xx_hal_tim.c 里的弱符号空实现-> 中断照进、但 HAL_IncTick()
 *         没人调用 -> uwTick 恒为 0 -> HAL_Delay() 死循环。
 *         实测现象：TIM6 CR1/ARR/PSC/NVIC/DIER 全部正常、无挂起中断，
 *         但 HAL_GetTick() 恒 0；查 map 见 BOOT 链的是
 *         stm32f4xx_hal_tim.o(i.HAL_TIM_PeriodElapsedCallback)（弱符号），
 *         而 APP 链的是 main.o(...)（强符号）。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}

/**
 * @brief  错误处理（main.c 里也有一份；BOOT 不编译 main.c，故此处自带）
 */
void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}

/**
 * @brief  printf 重定向到 USART2（Shell 口）
 * @note   与 main.c 里那份的区别：不依赖 FreeRTOS 互斥量（Bootloader 是裸机，
 *         没有 xMutexUART2，也没有调度器）。
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, 10U);
    return ch;
}

/**
 * @brief  系统时钟：HSE 8 MHz → PLL → SYSCLK 168 MHz
 * @note   从 Core/Src/main.c 复制而来（CubeMX 生成的那份），
 *         BOOT Target 不编译 main.c，故此处自带。
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ======================== 跳转 ======================== */

/**
 * @brief  跳到 APP 运行
 * @param  app_addr APP 起始地址（其第 0 个字是栈顶，第 1 个字是 Reset_Handler）
 * @note   ① 先校验栈顶指针落在 SRAM 范围内 —— 这是判断"APP 区有没有有效固件"的标准手段；
 *         ② 关中断 / 关 SysTick / 清 NVIC 使能与挂起，避免 APP 起来后残留中断；
 *         ③ 重设 SCB->VTOR —— 否则 APP 的中断会跳到 Bootloader 的向量表；
 *         ④ __set_MSP 后取 Reset_Handler 跳转（C 语言里"函数指针调用"即可完成）。
 */
static void JumpToApp(uint32_t app_addr)
{
    uint32_t sp = *(volatile uint32_t *)app_addr;
    uint32_t pc = *(volatile uint32_t *)(app_addr + 4U);
    uint32_t i;

    if ((sp < APP_SP_MIN) || (sp > APP_SP_MAX) || (pc < app_addr)) {
        printf("[BOOT] no valid APP (SP=0x%08lX PC=0x%08lX)\r\n",
               (unsigned long)sp, (unsigned long)pc);
        return;                                  /* 返回后停在 main 的循环里 */
    }

    printf("[BOOT] jumping to APP: SP=0x%08lX PC=0x%08lX\r\n",
           (unsigned long)sp, (unsigned long)pc);

    __disable_irq();
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;

    /* 清掉所有 NVIC 使能与挂起（8 组 × 32 位，覆盖全部 240 个外部中断） */
    for (i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = app_addr;                        /* 关键：把向量表指向 APP */
    __set_MSP(sp);                               /* 切到 APP 的栈 */
    ((void (*)(void))pc)();                      /* 跳进 APP 的 Reset_Handler */
}

/* ======================== OTA 搬运（阶段 3）======================== */

#define OTA_CHUNK   512U      /* 分块大小：用静态缓冲，不占启动栈（启动栈通常仅 1 KB）*/

static uint8_t s_ota_buf[OTA_CHUNK];

/* APP 区各扇区的起始与大小 —— S4 是 64 KB，S5~S9 各 128 KB；
 * STM32F4 的擦除粒度是整扇区，所以这里按扇区为单位列出来。 */
typedef struct { uint32_t start; uint32_t size; uint32_t sector; } OtaAppSector_t;
static const OtaAppSector_t s_appSec[] = {
    { 0x08010000UL, 0x10000UL, FLASH_SECTOR_4 },
    { 0x08020000UL, 0x20000UL, FLASH_SECTOR_5 },
    { 0x08040000UL, 0x20000UL, FLASH_SECTOR_6 },
    { 0x08060000UL, 0x20000UL, FLASH_SECTOR_7 },
    { 0x08080000UL, 0x20000UL, FLASH_SECTOR_8 },
    { 0x080A0000UL, 0x20000UL, FLASH_SECTOR_9 },
};
#define OTA_APP_SEC_N   (sizeof(s_appSec) / sizeof(s_appSec[0]))

/**
 * @brief  计算 W25Q64 上一段区域的 CRC16（分块读 + 累加，不整段进 RAM）
 * @retval 0=成功；1=W25Q64 读失败
 */
static int Ota_CrcExternal(uint32_t ext_addr, uint32_t len, uint16_t *out)
{
    uint16_t crc = 0xFFFFU;

    while (len > 0U) {
        uint32_t n = (len > (uint32_t)OTA_CHUNK) ? (uint32_t)OTA_CHUNK : len;
        if (W25Q64_Read(ext_addr, s_ota_buf, n) != W25Q64_OK) return 1;
        crc = Ota_CRC16Update(crc, s_ota_buf, n);
        ext_addr += n;
        len      -= n;
    }
    *out = crc;
    return 0;
}

/**
 * @brief  计算内部 Flash 上一段区域的 CRC16
 * @note   内部 Flash 是 memory-mapped，可直接按指针访问，无需拷贝到缓冲。
 * @retval 恒为 0（保留返回值是为了与 Ota_CrcExternal 风格统一）
 */
static int Ota_CrcInternal(uint32_t app_addr, uint32_t len, uint16_t *out)
{
    uint16_t       crc = 0xFFFFU;
    const uint8_t *p   = (const uint8_t *)app_addr;

    while (len > 0U) {
        uint32_t n = (len > (uint32_t)OTA_CHUNK) ? (uint32_t)OTA_CHUNK : len;
        crc = Ota_CRC16Update(crc, p, n);
        p   += n;
        len -= n;
    }
    *out = crc;
    return 0;
}

/**
 * @brief  擦除"装得下 fw_size 的前 N 个扇区"
 * @param  fw_size 固件字节数
 * @retval 0=成功；1=参数错；2=解锁失败；3=擦除失败
 * @note   只擦需要的部分：44 KB 固件只需擦 S4 一个扇区（约 50 ms），
 *         而不是把 704 KB（6 个扇区）全擦一遍。
 *         内部 Flash 擦写会让同 Bank 的取指 stall —— Bootloader 自己就在 Flash 里，
 *            所以擦除期间它会被"暂停"，擦完继续。这是 STM32F4 的固有行为，可接受。
 */
static int Ota_EraseAppRegion(uint32_t fw_size)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t err  = 0U;
    uint32_t need = OTA_APP_ADDR + fw_size;
    uint32_t i;

    if (fw_size == 0U) return 1;
    if (HAL_FLASH_Unlock() != HAL_OK) return 2;

    for (i = 0U; i < OTA_APP_SEC_N; i++) {
        if (s_appSec[i].start >= need) break;        /* 后面的扇区用不到，不擦 */
        er.TypeErase    = FLASH_TYPEERASE_SECTORS;
        er.VoltageRange = FLASH_VOLTAGE_RANGE_3;     /* 2.7 ~ 3.6 V */
        er.Sector       = s_appSec[i].sector;
        er.NbSectors    = 1U;
        if (HAL_FLASHEx_Erase(&er, &err) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[BOOT] erase failed at 0x%08lX (err=%lu)\r\n",
                   (unsigned long)s_appSec[i].start, (unsigned long)err);
            return 3;
        }
        printf("[BOOT] erased sector (0x%08lX)\r\n", (unsigned long)s_appSec[i].start);
    }
    HAL_FLASH_Lock();
    return 0;
}

/**
 * @brief  把暂存区的固件搬到 APP 区
 * @retval 0=成功；1=解锁失败；2=W25Q64 读失败；3=Flash 编程失败
 * @note   内部 Flash 只能按字编程，故长度向上取整到 4 字节；
 *         OTA_CHUNK(512) 与 total 都是 4 的倍数 -> 每块长度必为 4 的倍数，不会越界。
 */
static int Ota_CopyStageToApp(const OtaStageHeader_t *h)
{
    uint32_t total = (h->fw_size + 3U) & ~3U;        /* 向上取整到 4 字节 */
    uint32_t off   = 0U;

    if (HAL_FLASH_Unlock() != HAL_OK) return 1;

    while (off < total) {
        uint32_t n = ((total - off) > (uint32_t)OTA_CHUNK) ? (uint32_t)OTA_CHUNK
                                                          : (total - off);
        uint32_t w;

        if (W25Q64_Read(OTA_STAGE_ADDR + OTA_HDR_SIZE + off, s_ota_buf, n) != W25Q64_OK) {
            HAL_FLASH_Lock();
            return 2;
        }
        for (w = 0U; w < n; w += 4U) {
            uint32_t word = (uint32_t)s_ota_buf[w]
                          | ((uint32_t)s_ota_buf[w + 1U] << 8)
                          | ((uint32_t)s_ota_buf[w + 2U] << 16)
                          | ((uint32_t)s_ota_buf[w + 3U] << 24);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  OTA_APP_ADDR + off + w, word) != HAL_OK) {
                HAL_FLASH_Lock();
                printf("[BOOT] program failed @0x%08lX\r\n",
                       (unsigned long)(OTA_APP_ADDR + off + w));
                return 3;
            }
        }
        off += n;
    }
    HAL_FLASH_Lock();
    return 0;
}

/**
 * @brief  清掉暂存区标志（擦首扇区 → magic 变 0xFFFFFFFF）
 * @note   只在"搬运 + 回读校验全部通过"之后才调用。标志还在就意味着
 *         "还有一份待搬运的固件"，下次上电会重试。
 */
static void Ota_ClearStage(void)
{
    (void)W25Q64_EraseSector(OTA_STAGE_ADDR);
}

/**
 * @brief  阶段 3 主流程：校验暂存区 → 擦 APP → 搬运 → 回读校验 → 清标志
 * @retval 0=搬运完成（应跳新固件）；1=无有效暂存固件或失败（保持旧 APP）
 * @note   「刷坏不变砖」的关键在于顺序：
 *         必须先在暂存区把固件完整收齐并 CRC 通过，才动 APP 区。
 *         于是任何时刻断电，至少有一样东西是完整的：
 *           · 校验没过   → APP 区根本没被碰，旧固件完好
 *           · 搬运被中断 → APP 区可能残缺，但暂存区完好-> 下次上电自动重试搬运
 *         -> 只有"暂存区也坏 + APP 区也坏"这种双重故障才会真的变砖，概率极低。
 */
static int Ota_DoUpdate(void)
{
    OtaStageHeader_t hdr;
    uint16_t crc = 0U;

    if (W25Q64_Read(OTA_STAGE_ADDR, (uint8_t *)&hdr, sizeof(hdr)) != W25Q64_OK) {
        printf("[BOOT] stage read failed\r\n");
        return 1;
    }
    if (hdr.magic != OTA_MAGIC) {
        printf("[BOOT] no staged firmware (keep old APP)\r\n");
        return 1;
    }
    if ((hdr.fw_size == 0U) || (hdr.fw_size > OTA_APP_MAX_SIZE)) {
        printf("[BOOT] BAD size=%lu (max %lu), refuse\r\n",
               (unsigned long)hdr.fw_size, (unsigned long)OTA_APP_MAX_SIZE);
        return 1;
    }

    printf("[BOOT] staged fw: size=%lu crc=0x%04X ver=%lu\r\n",
           (unsigned long)hdr.fw_size, (unsigned)(hdr.fw_crc & 0xFFFFU),
           (unsigned long)hdr.version);

    /* ---- 1) 校验暂存区整包（CRC + fw_size 双重判据）---- */
    if ((Ota_CrcExternal(OTA_STAGE_ADDR + OTA_HDR_SIZE, hdr.fw_size, &crc) != 0) ||
        (crc != (uint16_t)(hdr.fw_crc & 0xFFFFU))) {
        printf("[BOOT] CRC FAIL (got 0x%04X, expect 0x%04X) -> keep old APP\r\n",
               (unsigned)crc, (unsigned)(hdr.fw_crc & 0xFFFFU));
        return 1;
    }
    printf("[BOOT] CRC ok\r\n");

    /* ---- 2) 擦 APP 区（只擦需要的扇区）---- */
    if (Ota_EraseAppRegion(hdr.fw_size) != 0) {
        printf("[BOOT] erase FAIL -> keep old APP\r\n");
        return 1;
    }

    /* ---- 3) 搬运 ---- */
    printf("[BOOT] copying %lu bytes to 0x%08lX...\r\n",
           (unsigned long)hdr.fw_size, (unsigned long)OTA_APP_ADDR);
    if (Ota_CopyStageToApp(&hdr) != 0) {
        printf("[BOOT] copy FAIL (stage kept for retry)\r\n");
        return 1;
    }

    /* ---- 4) 回读校验（不能只信"写入成功"）---- */
    (void)Ota_CrcInternal(OTA_APP_ADDR, hdr.fw_size, &crc);
    if (crc != (uint16_t)(hdr.fw_crc & 0xFFFFU)) {
        printf("[BOOT] VERIFY FAIL (got 0x%04X, expect 0x%04X) -> stage kept for retry\r\n",
               (unsigned)crc, (unsigned)(hdr.fw_crc & 0xFFFFU));
        return 1;
    }
    printf("[BOOT] verify ok\r\n");

    /* ---- 5) 全部通过，才清标志 ---- */
    Ota_ClearStage();
    printf("[BOOT] stage cleared, will boot new firmware\r\n");
    return 0;
}

/* ======================== 主流程 ======================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();          /* Shell 口：Bootloader 的调试输出 */
    MX_SPI1_Init();                 /* 读 W25Q64 需要 */

    /* 显式开中断：兜底"启动代码或别处残留 __disable_irq()"的情况。 */
    __enable_irq();

    /* CubeMX 生成的代码没有使能 TIM6（HAL timebase）中断→ uwTick 不增长
     *    → HAL_Delay() 里 while ((HAL_GetTick() - tickstart) < wait) 永远成立 → 死等。
     *    同样的修复在 Core/Src/main.c 的 "USER CODE BEGIN 2" 里也有一份。 */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 这里不能打印 VECT_TAB_OFFSET：它是 system_stm32f4xx.c 里的局部宏，
     *    其他编译单元（含本文件）看不到。要看它请查 map 文件或 SystemInit 反汇编。 */
    printf("\r\n[BOOT] start, APP_ADDR=0x%08lX\r\n", (unsigned long)OTA_APP_ADDR);

    /* ---- 诊断：确认 HAL 时基（TIM6）是否在走 ----
     * 若 tick_b 不比 tick_a 大，说明 uwTick 没增长 → HAL_Delay() 会死循环，
     * 表现为"只打印了第一行就卡住"。
     * 这里把 TIM6 与 NVIC 的寄存器也打出来，一次定位是"定时器没启动"还是"中断没使能"：
     *   TIM6 CR1 bit0 (CEN) : 1 = 定时器已启动
     *   TIM6 CNT            : 计数器实时值，两次读取应不同
     *   NVIC ISER bit22     : 1 = TIM6_DAC_IRQn(54) 已使能（54-32=22） */
    /* 时基自检：tick1 应比 tick0 大 20；不等说明 HAL 时基没起来（HAL_Delay 会死循环） */
    {
        uint32_t t0 = HAL_GetTick();
        HAL_Delay(20U);
        printf("[BOOT] tick %lu -> %lu (expect +20)\r\n",
               (unsigned long)t0, (unsigned long)HAL_GetTick());
    }

    /* ---- 阶段 3：校验暂存区 → 擦 APP → 搬运 → 回读校验 → 清标志 ----
     * Ota_DoUpdate() 在「无有效固件 / 长度非法 / CRC 失败 / 擦写失败」时
     * 都会自行保持旧 APP 并返回，所以这里不需要额外判断。 */
    if (W25Q64_Init() == W25Q64_OK) {
        (void)Ota_DoUpdate();
    } else {
        printf("[BOOT] W25Q64 init failed, skip OTA\r\n");
    }

    /* ---- 跳转到 APP ---- */
    HAL_Delay(50U);                 /* 给串口一点时间把上面的日志发完 */
    JumpToApp(OTA_APP_ADDR);

    printf("[BOOT] jump failed, halting. Use ST-Link to flash APP.\r\n");
    for (;;) { }
}
