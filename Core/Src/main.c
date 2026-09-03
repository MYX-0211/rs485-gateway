/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "rs485.h"
#include "modbus.h"
#include "oled.h"
#include <string.h>
#include <stdarg.h>
#include "esp01s.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WIFI_SSID    "YOUR_SSID"       /* 2.4G 热点名 */
#define WIFI_PWD     "YOUR_PASSWORD"   /* 热点密码 */
#define SERVER_IP    "192.168.1.100"   /* TCP Server（PC）的局域网 IP，按实际网络修改 */
#define SERVER_PORT  8000              /* TCP Server 端口，与 PC 端工具保持一致 */
#define REPORT_MS    5000              /* 数据上报周期 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SemaphoreHandle_t xMutexUART2;    /* UART2 打印互斥锁：多任务 printf 串行化，防输出乱行 */  
QueueHandle_t     xQueueSensor;   /* 采集→显示/网络 队列：深 1，Overwrite 写入，永远保留最新帧 */  
SemaphoreHandle_t xMutexRS485;    /* RS485 总线互斥锁：总线为共享资源，禁止多任务同时收发 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f);
void print_hex(const char *tag, uint8_t *buf, uint16_t len);
void UART2_PrintLine(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============================================================================
 * 应用层：FreeRTOS 多任务 + 业务逻辑
 *   任务优先级：sensor(3) > led(2) > net/oled/key(1)
 *   数据流：vTaskSensor 采集 → xQueueSensor(Overwrite 深1) → vTaskOLED / vTaskNet
 *   报警流：Alarm_Check(温湿度越限) → alarm_active → vTaskLED 轮闪 + 蜂鸣器
 * ==========================================================================*/
typedef enum {
    SENSOR_TH = 0,      /* 温湿度传感器（寄存器 0x0000 起） */
    SENSOR_LIGHT        /* 光照传感器（寄存器 0x0002 起） */
} SensorType_t;

typedef struct {
    uint8_t      online;    /* 在线标志：1=最近一次轮询成功 */  
    SensorType_t type;      /* 传感器类型，决定 Sensor_Parse 的解析分支 */  
    uint16_t     reg_start; /* 起始寄存器 */  
    float        temp;      /* 温度 ℃（SENSOR_TH 有效） */  
    float        humi;      /* 湿度 %RH（SENSOR_TH 有效） */  
    float        lux;       /* 光照度 Lux（SENSOR_LIGHT 有效） */  
    uint32_t     err_cnt;   /* 累计通信错误次数（诊断总线健康度） */  
} Sensor_t;


Sensor_t sensor_light = { .type = SENSOR_LIGHT, .reg_start = 0x0002 };  
Sensor_t sensor_th    = { .type = SENSOR_TH,    .reg_start = 0x0000 };

/* 上报载荷：由 vTaskSensor 打包写入，OLED/网络任务消费最新一帧 */  
typedef struct {  
    uint8_t  light_online;  
    uint8_t  th_online;  
    float    lux;  
    float    temp;  
    float    humi;  
    uint8_t  alarm;  
} SensorReport_t;

SensorReport_t g_report;   /* 队列载荷全局缓冲：仅 vTaskSensor 写入，vTaskOLED / vTaskNet 只读 */

/**
 * @brief  解析传感器原始寄存器值 → 物理量（温度/湿度/光照）
 * @param  s    传感器描述：type 决定走哪个解析分支，结果写回 s->temp/humi/lux
 * @param  regs 从机回复的原始寄存器数组（大端序已由 Modbus 层合并）
 * @note   按真实硬件手册编码规则解析，而非通用的 int16 补码假设
 */
void Sensor_Parse(Sensor_t *s, uint16_t *regs)
{
    if (s->type == SENSOR_TH) {
        uint16_t t_raw = regs[0];   /* 寄存器 0：温度原始值 */
        
        /* 厂商特殊温度编码（非标准 int16 补码）：
         * - 正温：直接存储，如 250 = 25.0℃
         * - 负温：10000 + |T×10|，如 10250 = -25.0℃ */
        s->temp = (t_raw < 10000)
                  ? t_raw * 0.1f                    /* 正温：值×0.1 */
                  : -(float)(t_raw - 10000) * 0.1f; /* 负温：减偏移取反 */
        
        s->humi = regs[1] * 0.1f;   /* 湿度：标准正数编码 */
    } else {
        /* 光照为 32 位无符号值，跨两个 16 位寄存器（大端序）传输；
         * 强制转为 uint32_t 再左移，避免 16 位整型移位问题 */
        uint32_t raw = ((uint32_t)regs[0] << 16) | regs[1];

        s->lux = raw / 1000.0f;     /* ÷1000 得标准 Lux */
    }
}

/**
 * @brief  单次读取一个传感器（Modbus 读保持寄存器 2 个字）
 * @param  s    传感器描述（类型/起始寄存器），结果写回 s->temp/humi/lux
 * @param  addr 从机地址（0x01 光照 / 0x02 温湿度）
 * @retval 0=成功，非0=失败（Modbus 层错误码）
 */
static int Sensor_ReadOnce(Sensor_t *s, uint8_t addr)
{
    uint16_t regs[2];
    int ret = Modbus_ReadHoldingRegs(addr, s->reg_start, 2, regs, 300);
    if (ret == 0) { s->online = 1; Sensor_Parse(s, regs); }
    else          { s->online = 0; s->err_cnt++; }
    return ret;
}

/**
 * @brief  RTOS 版传感器轮询：RS485 总线锁 + 3 次重试
 * @param  s    传感器描述（结果写回 online/temp/humi/lux）
 * @param  addr 从机地址（0x01 光照 / 0x02 温湿度）
 * @note   锁外等待 + 锁内单次最短事务；锁超时跳过本轮，最多重试 3 次
 */
static void Sensor_PollRTOS(Sensor_t *s, uint8_t addr)
{
    for (int attempt = 0; attempt < 3; attempt++) {      /* 首轮 + 2 次重试 */
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(50));  /* 重试前让出锁并延时 */

        if (xSemaphoreTake(xMutexRS485, pdMS_TO_TICKS(1000)) != pdTRUE) {
            UART2_PrintLine("[sensor] RS485 bus lock timeout\r\n");
            continue;   /* 锁超时：跳过本轮，等下一采集周期 */
        }
        Sensor_ReadOnce(s, addr);      /* 锁内仅执行单次最短事务 */
        xSemaphoreGive(xMutexRS485);
        if (s->online) break;          /* 读取成功即退出重试 */
    }
    if (!s->online) {                  /* 失败：锁外打印诊断日志 */
        char buf[48];
        snprintf(buf, sizeof(buf), "sensor 0x%02X fail err=%lu\r\n", addr, (unsigned long)s->err_cnt);
        UART2_PrintLine(buf);
    }
}

/**
 * @brief  蜂鸣器开关（有源蜂鸣器，低电平触发）
 * @param  on 1=响，0=停
 */
void Buzzer_Set(uint8_t on)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

volatile float g_temp_alarm_high  = 35.0f;   /* 报警上限 */
volatile float g_temp_alarm_clear = 34.0f;   /* 回差解除线（=上限-1℃，防抖） */

#define ALARM_STEP      0.5f
#define ALARM_HIGH_MAX  60.0f
#define ALARM_HIGH_MIN  -20.0f

typedef enum { UI_NORMAL = 0, UI_SETTING } UIState_t;
volatile UIState_t g_ui_state = UI_NORMAL;

/* 报警标志：1=报警中；volatile——sensor 任务写 / LED 任务读 */
volatile uint8_t alarm_active = 0;

/**
 * @brief  温湿度越限报警状态机（含回差防抖），并驱动蜂鸣器
 * @param  s 温湿度传感器（读其 online/temp，写全局 alarm_active）
 */
void Alarm_Check(Sensor_t *s)
{
    /* 三态判定 + 回差防抖 */
    if (!s->online) {                    /* 离线：强制解除，防止旧值误报 */
        alarm_active = 0;
    }
    else if (!alarm_active && s->temp > g_temp_alarm_high) {  /* 未报警 + 越上限 → 触发 */
        alarm_active = 1;
    }
    else if (alarm_active && s->temp < g_temp_alarm_clear) {  /* 报警中 + 低于回差线 → 解除 */
        alarm_active = 0;
    }
    /* 位于 [clear, high] 回差区间时状态保持不变 */

    Buzzer_Set(alarm_active);            /* 报警输出 → 蜂鸣器 */
}


/**
 * @brief  向 OLED 输出一行文本（固定 16 字符宽）
 * @note   不足 16 字符自动补空格覆盖旧内容，防止长短串切换产生残影
 */
static void UI_PrintLine(uint8_t y, const char *fmt, ...)
{
    char line[20];              /* 略大于 16，容纳格式串展开 */
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    size_t len = strlen(line);
    if (len < 16) memset(line + len, ' ', 16 - len);  /* 补空格 */
    line[16] = '\0';            /* 强制截断，防越界 */

    OLED_ShowString(0, y, line);
}

/**
 * @brief  按传感器最新数据刷新 OLED 正常页 4 行显示
 * @param  r 队列载荷（光照/温湿度/在线状态/报警标志）
 */
void UI_RefreshFrom(SensorReport_t *r)
{
    // 第0行：光照值或离线
    if (r->light_online) UI_PrintLine(0, "S1:%.1f Lux", r->lux);
    else                 UI_PrintLine(0, "S1:OFFLINE");

    // 第1行：温湿度或离线
    if (r->th_online)    UI_PrintLine(1, "S2:%.1fC %.1f%%", r->temp, r->humi);
    else                 UI_PrintLine(1, "S2:OFFLINE");

    // 第2行：两路在线状态
    UI_PrintLine(2, "A:%s B:%s",
                 r->light_online ? "ON " : "OFF",
                 r->th_online    ? "ON " : "OFF");

    // 第3行：报警状态
    UI_PrintLine(3, "ALARM:%s", r->alarm ? "ON " : "OFF");
}

/**
 * @brief  采集任务（优先级 3）：1s 精确周期轮询两路传感器 + 报警判定 + 投递队列
 * @note   vTaskDelayUntil 绝对节拍，长期运行不漂移；结果经 xQueueOverwrite 广播给显示/网络任务
 */
void vTaskSensor(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        Sensor_PollRTOS(&sensor_light, 0x01);   /* 锁外等待 + 锁内单次事务 */
        Sensor_PollRTOS(&sensor_th,    0x02);

        Alarm_Check(&sensor_th);                /* 报警判定（仅温湿度） */

        g_report = (SensorReport_t){
            sensor_light.online, sensor_th.online,
            sensor_light.lux, sensor_th.temp, sensor_th.humi, alarm_active };
        xQueueOverwrite(xQueueSensor, &g_report);   /* Overwrite：深1队列直接覆盖旧值，永远留最新一帧 */

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));   /* 精确 1s，不漂移 */	
    }
}

/**
 * @brief  显示任务（优先级 1）：取队列最新数据刷新 OLED；设置页时显示阈值调节界面
 * @note   Peek 不消费数据，深1+Overwrite 下总是立即拿到最新帧；200ms 节流防闪烁
 */
void vTaskOLED(void *arg)
{
    SensorReport_t r;
    for (;;) {
        if (g_ui_state == UI_SETTING) {          /* ===== 设置页（按键调阈值） ===== */
            UI_PrintLine(0, "==TEMP SET==");
            UI_PrintLine(1, "HI:%4.1fC", g_temp_alarm_high);
            UI_PrintLine(2, "LO:%4.1fC", g_temp_alarm_clear);
            UI_PrintLine(3, "K2:+ K3:-");
            vTaskDelay(pdMS_TO_TICKS(200));      /* 设置页每 200ms 刷新 */
            continue;                            /* 保持任务循环：退出设置页后自动回到正常页 */
        }
        else if (xQueuePeek(xQueueSensor, &r, portMAX_DELAY) == pdTRUE) {
            UI_RefreshFrom(&r);
            vTaskDelay(pdMS_TO_TICKS(200));   /* 节流：深1+Overwrite 下 Peek 总是立即成功 */
        }
    }
}

int esp_ok = 0;   /* ESP TCP 连接状态：1=已连接可直接上报，0=需先重连（仅 vTaskNet 内读写） */

/**
 * @brief  网络上报任务（优先级 1）：每 REPORT_MS 周期向服务器上报最新数据
 * @note   已连接(esp_ok)直接 CIPSEND 发送；断线则后台重连（阻塞期间被高优先级任务抢占）
 */
void vTaskNet(void *arg)  
{  
    SensorReport_t r;  
    char msg[80];  
    TickType_t last = xTaskGetTickCount();  
    for (;;) {  
        vTaskDelayUntil(&last, pdMS_TO_TICKS(REPORT_MS));   /* 5s 精确周期 */  
        if (xQueuePeek(xQueueSensor, &r, 100) == pdTRUE) {  /* 拿最新数据 */  
            snprintf(msg, sizeof(msg), "lux=%.1f,temp=%.1f,humi=%.1f,alarm=%d\r\n",  
                     r.lux, r.temp, r.humi, r.alarm);  
            if (esp_ok) {  
                int ret = ESP_SendTCP(msg);   /* 已连接：直接上报 */  
                printf("[ESP] r=%d %s", ret, msg);  
                if (ret != 0) esp_ok = 0;     /* 发送失败 → 视为断线，下次重连 */  
            } else {  
                /* 断线重连：内部 HAL 忙等可阻塞数十秒，但优先级 2 会被  
                 * sensor(3) 抢占，采集不受影响（OLED 是优先级1，会暂冻） */  
                printf("[ESP] reconnecting...\r\n");  
                esp_ok = (ESP_Init(WIFI_SSID, WIFI_PWD, SERVER_IP, SERVER_PORT) == 0);  
            }  
        }  
    }  
}


/**
 * @brief  LED 指示任务（优先级 2）：报警时三灯 200ms 轮流闪烁，正常时全灭
 */
void vTaskLED(void *arg)
{
    uint8_t cur = 0;                    // 当前点亮序号 0~2
    for (;;) {
        if (alarm_active) {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, LED_OFF);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, LED_OFF);
            switch (cur) {
                case 0: HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_ON); break;
                case 1: HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, LED_ON); break;
                case 2: HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, LED_ON); break;
            }
            cur = (cur + 1) % 3;
            vTaskDelay(pdMS_TO_TICKS(200));   /* 每灯 200ms */
        } else {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, LED_OFF);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

/**
 * @brief  按键任务（优先级 1）：10ms 轮询三键，软件消抖 + 边沿检测
 * @note   KEY1 切换设置页；KEY2/KEY3 在设置页内调节报警阈值
 *         消抖：连续 KEY_DEBOUNCE(2) 次采样一致才判定有效（约 20ms）
 *         边沿检测：按下仅触发一次，松开复位后才允许再次触发 */
#define KEY_DEBOUNCE  2

void vTaskKey(void *arg)
{
    uint8_t last[3]    = {1, 1, 1};   /* 最近一次采样：1=松开 0=按下 */
    uint8_t stable[3]  = {0, 0, 0};   /* 连续一致计数（消抖） */
    uint8_t pressed[3] = {0, 0, 0};   /* 本次按下是否已触发（边沿） */
    for (;;) {
        uint8_t cur[3];
        cur[0] = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
        cur[1] = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
        cur[2] = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);

        for (int i = 0; i < 3; i++) {
            if (cur[i] != last[i]) {        /* 采样变化：更新，清零计数 */
                last[i]   = cur[i];
                stable[i] = 0;
            } else if (++stable[i] >= KEY_DEBOUNCE) {
                stable[i] = 0;              /* 连续一致 → 消抖确认 */
                if (cur[i] == 0) {          /* 稳定在"按下" */
                    if (!pressed[i]) {      /* 边沿：只响应一次 */
                        pressed[i] = 1;
                        if (i == 0) {                       /* KEY1: 切设置页 */
                            g_ui_state = (g_ui_state == UI_NORMAL) ? UI_SETTING : UI_NORMAL;
                            printf("[UI] %s\r\n", g_ui_state ? "SETTING" : "NORMAL");
                        } else if (i == 1) {                /* KEY2: 阈值+ */
                            if (g_ui_state == UI_SETTING) {
                                g_temp_alarm_high += ALARM_STEP;
                                if (g_temp_alarm_high > ALARM_HIGH_MAX) g_temp_alarm_high = ALARM_HIGH_MAX;
                                g_temp_alarm_clear = g_temp_alarm_high - 1.0f;
                                printf("[UI] HI=%.1f\r\n", g_temp_alarm_high);
                            }
                        } else if (i == 2) {                /* KEY3: 阈值- */
                            if (g_ui_state == UI_SETTING) {
                                g_temp_alarm_high -= ALARM_STEP;
                                if (g_temp_alarm_high < ALARM_HIGH_MIN) g_temp_alarm_high = ALARM_HIGH_MIN;
                                g_temp_alarm_clear = g_temp_alarm_high - 1.0f;
                                printf("[UI] HI=%.1f\r\n", g_temp_alarm_high);
                            }
                        }
                    }
                } else {
                    pressed[i] = 0;         /* 松开：复位，允许下次按下 */
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
/* TIM6（HAL timebase）中断使能与优先级校正：
 * CubeMX 生成时该中断未在 NVIC 使能，导致 uwTick 不增长、
 * HAL_Delay 在调度器启动前卡死。此处手动 清 pending →
 * 优先级 0 → 使能，强制恢复 tick 中断。
 * 注：TIM6 中断仅调用 HAL_IncTick，不触碰任何 FreeRTOS API，
 *     优先级 0（最高）不会破坏 RTOS 临界区机制。 */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    
    RS485_Init(); 
    OLED_Init();
	
	/* ESP 配网与 TCP 连接交由 vTaskNet 后台异步完成，避免阻塞调度器启动
	 * （WiFi/CIPSTART 重试可能耗时数十秒，期间 OLED/采集任务照常运行） */

	xQueueSensor = xQueueCreate(1, sizeof(SensorReport_t));  /* 数据队列：深度 1，Overwrite 保留最新帧 */
  xMutexRS485  = xSemaphoreCreateMutex();                  /* RS485 总线互斥锁 */
	xMutexUART2 = xSemaphoreCreateMutex();									/* UART2 打印互斥锁 */
  xTaskCreate(vTaskSensor,   "sensor", 512, NULL, 3, NULL);  /* 采集任务：最高优先级（栈 512，容纳 snprintf 调用链） */
  xTaskCreate(vTaskOLED,     "oled",   512, NULL, 1, NULL);  /* OLED 显示任务（栈 512，容纳 printf(%f) 与 I2C 调用链） */
	xTaskCreate(vTaskNet,      "net",    768, NULL, 1, NULL);  /* 网络上报任务：AT 命令缓冲 + TCP 收发（栈 768） */
	xTaskCreate(vTaskKey, "key", 256, NULL, 1, NULL);   /* 按键扫描任务 */
	xTaskCreate(vTaskLED, "led", 256, NULL, 2, NULL);   /* LED 报警指示任务 */
	vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 栈溢出钩子：打印爆栈任务名并停机，便于定位（优于直接 HardFault） */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("STACK OVERFLOW: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
/* 内存分配失败钩子：heap_4 堆耗尽时触发 */
void vApplicationMallocFailedHook(void)
{
    printf("HEAP EXHAUSTED\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

/* printf 重定向至 UART2（调试口）。
 * 调度器运行期间经 xMutexUART2 加锁，避免多任务 printf 输出交错；
 * 调度器未启动（如时钟配置阶段）则直接发送。 */
int fputc(int ch, FILE *f)
{
    if (xMutexUART2 && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTake(xMutexUART2, portMAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
        xSemaphoreGive(xMutexUART2);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    }
    return ch;
}

/* 整行原子打印：单次加锁发送整行，避免逐字符 printf 的锁开销与行碎片 */
void UART2_PrintLine(const char *s)
{
    uint16_t len = (uint16_t)strlen(s);
    if (xMutexUART2 && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTake(xMutexUART2, portMAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)s, len, 50);
        xSemaphoreGive(xMutexUART2);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)s, len, 50);  /* 调度器未启动时裸用 */
    }
}

void print_hex(const char *tag, uint8_t *buf, uint16_t len)
{
    printf("%s[%d]: ", tag, len);
    for (uint16_t i = 0; i < len; i++) printf("%02X ", buf[i]);
    printf("\r\n");
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
