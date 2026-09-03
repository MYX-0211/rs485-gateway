#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* 硬件 I2C 接口（I2C1：SCL=PB6 / SDA=PB7，Fast Mode 400kHz，复用开漏） */
#define OLED_ADDR        0x78    /* SSD1306 写地址 = (0x3C << 1) */
#define OLED_I2C         (&hi2c1)
#define OLED_I2C_TIMEOUT 100     /* I2C 单次事务超时 (ms) */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t y);                  /* 清空一行文本（2 次写事务） */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
/* 显示字符串：x=列 0~15，y=行 0~3；行尾自动补空格，消除长短串覆盖残影 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

#endif
