#include "oled.h"
#include "oledfont.h"
#include "i2c.h"

/* ------------------------------------------------------------------ *
 * SSD1306 OLED 驱动（I2C1 硬件 I2C，地址 0x78）
 * 显示布局：4 行 × 16 字符（8×16 字体，每字符占 2 页）
 * 写时序：复用 HAL_I2C_Mem_Write 的"寄存器地址"字节充当 SSD1306 控制字节
 *         0x00=命令，0x40=显存数据（列地址自动递增）
 * 可靠性：写失败自动重试一次，抵御偶发 NAK
 * ------------------------------------------------------------------ */
static HAL_StatusTypeDef OLED_Write(uint8_t ctrl, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Mem_Write(OLED_I2C, OLED_ADDR, ctrl,
                           I2C_MEMADD_SIZE_8BIT, buf, len, OLED_I2C_TIMEOUT);
    if (st != HAL_OK) {   /* 失败重试一次，抵御偶发 NAK */
        st = HAL_I2C_Mem_Write(OLED_I2C, OLED_ADDR, ctrl,
                               I2C_MEMADD_SIZE_8BIT, buf, len, OLED_I2C_TIMEOUT);
    }
    return st;
}

static void OLED_WriteCmd(uint8_t cmd) { OLED_Write(0x00, &cmd, 1); }        /* 发单条命令 */
static void OLED_WriteData(uint8_t *buf, uint16_t len) { OLED_Write(0x40, buf, len); } /* 发显存数据 */

/* 设置光标: col 0~127, page 0~7 */
static void OLED_SetPos(uint8_t col, uint8_t page)
{
    OLED_WriteCmd(0xB0 | (page & 0x07));
    OLED_WriteCmd(0x10 | ((col & 0xF0) >> 4));
    OLED_WriteCmd(0x00 | (col & 0x0F));
}

/**
 * @brief  OLED 初始化：上电延时 + 批量下发配置命令 + 清屏 + 开显示
 * @note   在 MX_I2C1_Init 之后调用；若屏幕不亮，优先查 I2C 引脚(SCL=PB6/SDA=PB7)与供电
 */
void OLED_Init(void)
{
    HAL_Delay(100);   /* 上电稳定: SSD1306 要求 VCC 起来后稍等 */

    /* 初始化序列以单条 I2C 事务批量下发（命令间控制字节均为 0x00） */
    static const uint8_t init_seq[] = {
        0xAE,             /* 关显示 */
        0xD5, 0x80,       /* 显示时钟分频 */
        0xA8, 0x3F,       /* 多路复用 1/64（128x64） */
        0xD3, 0x00,       /* 显示偏移 0 */
        0x40,             /* 起始行 = 0 */
        0x8D, 0x14,       /* 电荷泵使能（不开则屏幕全黑） */
        0x20, 0x02,       /* 页寻址模式 */
        0xA1,             /* 段重映射（水平翻转） */
        0xC8,             /* COM 扫描方向（垂直翻转） */
        0xDA, 0x12,       /* COM 硬件引脚配置（128x64） */
        0x81, 0xCF,       /* 对比度 */
        0xD9, 0xF1,       /* 预充电周期 */
        0xDB, 0x40,       /* VCOMH 电平 */
        0xA4,             /* 显示内容跟随 GDDRAM */
        0xA6,             /* 正常显示（非反色） */
    };
    OLED_Write(0x00, (uint8_t *)init_seq, sizeof(init_seq));

    OLED_Clear();
    OLED_WriteCmd(0xAF);   /* 开显示 */
}

/**
 * @brief  全屏清空：8 页（128×64）全部写 0
 */
void OLED_Clear(void)
{
    static const uint8_t zeros[128] = {0};   /* static：全 0 缓冲无需重复清零 */
    for (uint8_t page = 0; page < 8; page++) {
        OLED_SetPos(0, page);
        OLED_WriteData((uint8_t *)zeros, 128);
    }
}

/**
 * @brief  清空指定文本行（y=0~3）
 * @note   直接写 0 覆盖该行对应两页，2 次写事务完成
 */
void OLED_ClearLine(uint8_t y)
{
    if (y >= 4) return;
    static const uint8_t zeros[128] = {0};
    OLED_SetPos(0, y * 2);
    OLED_WriteData((uint8_t *)zeros, 128);
    OLED_SetPos(0, y * 2 + 1);
    OLED_WriteData((uint8_t *)zeros, 128);
}

/**
 * @brief  在指定位置显示单个 ASCII 字符（8×16 字体）
 * @param  x 字符列 0~15，y 文本行 0~3
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch)
{
    uint8_t idx;
    if (x >= 16 || y >= 4) return;            /* 越界保护 */
    if (ch < 32 || ch > 126) ch = ' ';        /* 非打印字符显示为空格 */
    idx = (uint8_t)(ch - 32);
    OLED_SetPos(x * 8, y * 2);
    OLED_WriteData((uint8_t *)&F8X16[idx * 16], 8);
    OLED_SetPos(x * 8, y * 2 + 1);
    OLED_WriteData((uint8_t *)&F8X16[idx * 16 + 8], 8);
}

/**
 * @brief  显示字符串（从 (x,y) 开始）
 * @param  x 起始字符列 0~15，y 文本行 0~3
 * @note   超出 16 列自动截断；行尾剩余列以空格补满，消除长短串切换残影
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    while (*str && x < 16) {
        OLED_ShowChar(x, y, *str++);
        x++;
    }
    while (x < 16) { OLED_ShowChar(x, y, ' '); x++; }
}
