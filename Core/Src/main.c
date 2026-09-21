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
#include "spi.h"
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
#include "param.h"       /* 报警阈值掉电保存（Flash 扇区 11） */
#include "w25q64.h"      /* 外部 SPI Flash：OTA 固件暂存与备份 */
#include "ota.h"         /* OTA 共享定义：分区地址 / 暂存区头部 / CRC16 */
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
#define WIFI_SSID    "OPPO"       /* 2.4G 热点名 */
#define WIFI_PWD     "123456789"   /* 热点密码 ★请核对位数（9/17 日志里 ESP 收到的是 10 位） */
/* ---- 上报目标：含义随 USE_MQTT（定义在 esp01s.h）变化 ----
 * USE_MQTT=1 → MQTT broker：填PC 的 IP，由 PC 上的 portproxy 转发到 EMQX 虚拟机
 *              （EMQX 在 VMware NAT 网 192.168.137.128，ESP 进不去该网段，故必须经 PC 转发）
 * USE_MQTT=0 -> 裸 TCP：填 PC 网络调试助手的地址，端口与助手的 TCPServer 一致 */
#define SERVER_IP    "10.216.109.38"   /* PC 在 OPPO 热点上的 IP（换网络必须同步改） */
#if USE_MQTT
#define SERVER_PORT  1883              /* MQTT broker 端口 */
#else
#define SERVER_PORT  8000              /* PC 网络调试助手 TCPServer 端口 */
#endif
#define REPORT_MS    5000              /* 数据上报周期 */

/* W25Q64 上电自检开关：1 = 每次上电跑一次读写一致性测试（读 JEDEC ID、
 * 擦一个扇区、写 256 B 递增模式、读回比对）。
 * 接线验证阶段可临时改为 1；正常运行时保持 0 —— 该自检会擦掉暂存区首个扇区，
 * 而暂存区在 OTA 流程中用于存放待搬运的固件。 */
#define W25Q64_SELFTEST_ON_BOOT   0
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
 *   报警流：Alarm_Check(温度越上限) → alarm_active → vTaskLED 轮闪 + 蜂鸣器
 * ==========================================================================*/
typedef enum {
    SENSOR_TH = 0,      /* 温湿度传感器（寄存器 0x0000 起） */
    SENSOR_LIGHT        /* 光照传感器（寄存器 0x0002 起） */
} SensorType_t;

typedef struct {
    uint8_t      online;    /* 在线标志：1=最近一次轮询成功 */  
    SensorType_t type;      /* 传感器类型，决定 Sensor_Parse 的解析分支 */  
    uint8_t      func;      /* Modbus 功能码：0x03 保持寄存器 / 0x04 输入寄存器
                             *    按各从机手册选定 —— 测量值通常在输入寄存器，参数在保持寄存器 */
    uint16_t     reg_start; /* 起始寄存器 */  
    float        temp;      /* 温度 ℃（SENSOR_TH 有效） */  
    float        humi;      /* 湿度 %RH（SENSOR_TH 有效） */  
    float        lux;       /* 光照度 Lux（SENSOR_LIGHT 有效） */  
    uint32_t     err_cnt;   /* 累计通信错误次数（诊断总线健康度） */  
} Sensor_t;


/* 功能码按各从机手册选定（这是"代码 / 手册 / 简历三方一致"的关键）：
 *    光照 B-RS-L30      ：测量值在 0x0002，手册给的是 0x03 读保持寄存器
 *    温湿度 HKDZ-SHT30-RS：测量值在 0x0000/0x0001，手册给的是 0x04 读输入寄存器 */
Sensor_t sensor_light = { .type = SENSOR_LIGHT, .func = 0x03, .reg_start = 0x0002 };  
Sensor_t sensor_th    = { .type = SENSOR_TH,    .func = 0x04, .reg_start = 0x0000 };

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

/* 上报帧序号：vTaskNet 每发一帧自增一次，随 payload 一起上报。
 * 作用：为 ⑦ 断网缓存补传 提供「按序去重 / 检洞」的依据 —— 服务端凭 seq 就能判断
 *       是否缺帧、是否重复；也可以用来验证"同一份数据被连发多遍"这类问题。 */
uint32_t g_report_seq = 0;

/* 下行命令回包缓冲：App_MqttOnCommand() 用它拼 ack 并交给 ESP_MqttPub()。
 * 放全局而不是局部栈：ESP_MqttPub → ESP_SendCmd 调用链较深，而 vTaskNet 栈只有 768 字，
 * 把 160 字节挪到 .bss 更安全。 */
static char g_cmd_ack[160];

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
 * @brief  单次读取一个传感器（按 s->func 选 Modbus 功能码，读 2 个寄存器）
 * @param  s    传感器描述（类型/功能码/起始寄存器），结果写回 s->temp/humi/lux
 * @param  addr 从机地址（0x01 光照 / 0x02 温湿度）
 * @retval 0=成功，非0=失败（Modbus 层错误码）
 * @note   功能码由传感器自己带（0x04 → 输入寄存器；其余按 0x03 保持寄存器处理），
 *         这样新增从机时只需在实例里填手册给的功能码，不必改事务层。
 */
static int Sensor_ReadOnce(Sensor_t *s, uint8_t addr)
{
    uint16_t regs[2];
    int ret = (s->func == 0x04)
            ? Modbus_ReadInputRegs  (addr, s->reg_start, 2, regs, 300)
            : Modbus_ReadHoldingRegs(addr, s->reg_start, 2, regs, 300);

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

/* 参数脏标志：阈值被改（按键 或 MQTT 远程）时置 1，由 vTaskOLED 做防抖后写 Flash。
 * 为什么不改完就立刻写：Flash 擦写会阻塞且属于低频重操作，
 * 不该挤在按键扫描/网络回调的关键路径里；交给低优先级任务统一落盘更稳。 */
volatile uint8_t g_param_dirty = 0;

#define ALARM_STEP      0.5f
#define ALARM_HIGH_MAX  60.0f
#define ALARM_HIGH_MIN  -20.0f

typedef enum { UI_NORMAL = 0, UI_SETTING } UIState_t;
volatile UIState_t g_ui_state = UI_NORMAL;

/* 报警标志：1=报警中；volatile——sensor 任务写 / LED 任务读 */
volatile uint8_t alarm_active = 0;

/**
 * @brief  温度越上限报警状态机（含回差防抖），并驱动蜂鸣器
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

        Alarm_Check(&sensor_th);                /* 报警判定（仅温度越上限） */

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
    static uint8_t save_cnt = 0;      /* 防抖计数：10 × 200ms = 2s */

    for (;;) {
        /* 阈值防抖保存（④ 掉电保存）：
         *    改阈值（按键或 MQTT）只置 g_param_dirty，真正的 Flash 写入放到这里 ——
         *    ① 低优先级任务上下文，不挤占按键/网络路径；
         *    ② 2 秒防抖，避免连按/连发导致反复擦写；
         *    ③ 每轮循环都会执行（无论正常页还是设置页），不会漏掉。 */
        if (g_param_dirty) {
            if (++save_cnt >= 10U) {
                g_param_dirty = 0;
                save_cnt = 0;
                (void)Param_Save();
            }
        } else {
            save_cnt = 0;
        }

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
    char msg[128];  /* 原 96：改装 JSON 后最坏约 89 字节（seq/up 各 10 位 + 三个浮点最坏 9 位），留余量 */
    TickType_t last = xTaskGetTickCount();  
    for (;;) {  
        /* 每轮先处理 MQTT 下行 URC（非阻塞：没有 URC 就立刻返回）。
         * 放在这里的原因：vTaskNet 是本工程唯一使用 USART3 的任务，
         * 而 5s 一轮的延迟对"远程改报警阈值"完全够用。 */
        ESP_PollURC();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(REPORT_MS));   /* 5s 精确周期 */  
        if (xQueuePeek(xQueueSensor, &r, 100) == pdTRUE) {  /* 拿最新数据 */  
            /* JSON 载荷。字段含义：
             *   seq —— 帧序号（服务端据此判缺帧/重复；⑦断网补传的前置）
             *   up  —— 自开机毫秒数（判断数据新鲜度；重启归零，与 seq 互相印证）
             *   lux/temp/humi/alarm —— 传感器数据
             * ★ 末尾不加 \r\n：MQTT 走 MQTTPUBRAW 按长度精确发送，
             *   多一个字节就会让 broker 收到非法 JSON。 */
            g_report_seq++;
            snprintf(msg, sizeof(msg),
                     "{\"seq\":%lu,\"up\":%lu,\"lux\":%.1f,\"temp\":%.1f,\"humi\":%.1f,\"alarm\":%d}",
                     (unsigned long)g_report_seq, (unsigned long)HAL_GetTick(),
                     r.lux, r.temp, r.humi, r.alarm);  
            if (esp_ok) {  
#if USE_MQTT
                int ret = ESP_MqttPub(MQTT_TOPIC_PUB, msg);   /* MQTT 发布（EMQX） */
#else
                int ret = ESP_SendTCP(msg);                   /* 裸 TCP 上报（回退路径） */
#endif
                printf("[ESP] r=%d %s\r\n", ret, msg);  
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

/* ======================== MQTT 下行命令处理（⑥ Day 6） ======================== */

/* ---- 轻量 JSON 取值：从 json 里取 "key":"<字符串>" ----
 * @note 这不是通用 JSON 解析器：载荷格式由本工程约定、完全可控，
 *       用 strstr 定位即可，比引入 JSON 库省代码也省栈。 */
static int JsonGetStr(const char *json, const char *key, char *out, size_t outsz)
{
    char        pat[24];
    const char *p, *q;
    size_t      n;

    snprintf(pat, sizeof(pat), "\"%s\"", key);
    p = strstr(json, pat);
    if (p == NULL) return 0;
    p = strchr(p + strlen(pat), ':');        /* 跳到键后的冒号 */
    if (p == NULL) return 0;
    q = strchr(p, '"');                      /* 值的起始引号 */
    if (q == NULL) return 0;
    q++;
    p = strchr(q, '"');                      /* 值的结束引号 */
    if (p == NULL) return 0;
    n = (size_t)(p - q);
    if (n == 0U || n >= outsz) return 0;
    memcpy(out, q, n);
    out[n] = '\0';
    return 1;
}

/* ---- 轻量十进制解析（手写 atof）----
 * @note 不用 sscanf/atof：MicroLIB 的浮点输入支持不可靠，且会吃栈；
 *       这里只需解析 "12.3" / "-5" / "30.0" 这类简单十进制，手写十几行更稳。 */
static int ParseFloat(const char *s, float *out)
{
    int   neg = 0;
    float v = 0.0f, frac = 0.1f;

    while (*s == ' ') s++;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }
    if (*s < '0' || *s > '9') return 0;
    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { v += (float)(*s - '0') * frac; frac *= 0.1f; s++; }
    }
    *out = neg ? -v : v;
    return 1;
}

/* ---- 取数值字段："key":<数字> ---- */
static int JsonGetNum(const char *json, const char *key, float *out)
{
    char        pat[24];
    const char *p;

    snprintf(pat, sizeof(pat), "\"%s\"", key);
    p = strstr(json, pat);
    if (p == NULL) return 0;
    p = strchr(p + strlen(pat), ':');
    if (p == NULL) return 0;
    return ParseFloat(p + 1, out);
}

/**
 * @brief  下发命令回调（由 esp01s.c 的 ESP_PollURC 调用）
 * @param  payload JSON 载荷（非 '\0' 结尾，需按 len 使用）
 * @param  len     载荷长度
 * @note   支持的命令：
 *           {"cmd":"set_hi","v":30.0}   改报警上限（越界自动钳位）
 *           {"cmd":"get"}               回一条当前状态（闭环验证：改完立刻能查回来）
 *         并发说明：g_temp_alarm_high 同时会被按键任务 vTaskKey写。
 *            本工程按"最后写入者胜"处理：两侧都只做「简单赋值 + 钳位」，不做读-改-写，
 *            而 float 的单次写入在 Cortex-M4 上是原子的，因此不会出现撕裂值。
 *            若将来要求"以服务端为准"，应改为加临界区或引入配置版本号。
 */
/* ======================== OTA 暂存写入（自测用）======================== */

/* ARMCC 为 scatter region 生成边界符号。取"固件有多大"要注意两点：
 * ① Load$$LR_IROM1$$Limit 在 ARMCC 5 下不生成（实测报 L6218E Undefined），
 *    所以只能用执行域符号 Image$$ER_IROM1$$Limit。
 * ② 但 ER_IROM1 只是 RO 段（代码+常量）；.sct 里 RW_IRAM1 挂在 LR_IROM1 之下，
 *    它的初始值也存在于 Flash 中、紧跟 RO 之后 —— 只搬到 RO 末尾会漏掉 RW 初值，
 *    导致搬运后 APP 的全局变量初值出错。
 * -> 折中做法：取 RO 末尾后向上取整到 16 KB，用取整带来的余量覆盖紧随其后的 RW 初值。
 *    本工程 RW-data 仅约 300 B，16 KB 余量绰绰有余；即便将来 RO 增长、偏移变动也不易踩边界。
 * 不要用 4 KB 取整 —— 实测会因余量不足（296 B < RW-data 304 B）而漏掉 RW 初值末尾。
 * 实测：ER_IROM1$$Limit = 0x0801AED8 -> RO 长 44760 B ✓ */
extern uint8_t Image$$ER_IROM1$$Limit;

/**
 * @brief  自测：把当前正在运行的 APP 自己当成一份"新固件"写进 W25Q64 暂存区
 * @param  corrupt_crc 0=写入正确的整包 CRC；非 0=故意写错的 CRC
 * @retval 0=成功；非 0=失败（1 长度非法 / 2 擦除失败 / 3 写数据失败 / 4 写头部失败）
 * @note   用途：在不接 PC 的前提下验证整条 OTA 搬运链路 ——
 *         APP 写暂存区 → 复位 → Bootloader 校验并搬运 → 仍能跑起来。
 *         corrupt_crc != 0 用于验证"刷坏不变砖"：Bootloader 应拒绝搬运、
 *         保持旧 APP 照常运行（这是 OTA 的硬指标）。
 *         这是临时测试功能；现已由 PC 通过 TCP 下发真实固件。
 *         缓冲用 static：本函数在 vTaskNet 上下文执行，其栈只有 768 字
 *            （FreeRTOS 的字 = 4 字节，约 3 KB），512 B 缓冲占比可接受，
 *            但放 .bss 更稳妥，也为后续更大分块留余地。
 *         头部最后写：若写入过程中断电，magic 不是 OTA_MAGIC ->
 *            Bootloader 视为"无有效固件"直接跳旧 APP —— 天然安全。
 */
static int Ota_PrepareSelfTest(int corrupt_crc)
{
    OtaStageHeader_t hdr;
    /* 固件大小上界（见上方 extern 处的说明）：
     *   RO 末尾 → 向上取整到 16 KB。
     * 取整粒度必须大于 RW 初值的大小，否则会漏掉紧跟在 RO 之后的 RW 初值。
     *    实测说明：先用 4 KB 取整，RO 长 44760 B → 取整后 45056 B，
     *    尾部余量只剩 296 B，而 RW-data 是 304 B -> 差 8 字节覆盖不全。
     *    改用 16 KB 取整后余量约 4 KB，远超 RW 初值，之后 RO 再增长也不易踩到边界。 */
    uint32_t ro_end  = (uint32_t)&Image$$ER_IROM1$$Limit;
    uint32_t fw_size = ((ro_end - OTA_APP_ADDR) + 0x3FFFU) & ~0x3FFFU;
    uint32_t off = 0U;
    uint16_t crc = 0xFFFFU;
    static uint8_t buf[512];

    if ((fw_size == 0U) || (fw_size > OTA_APP_MAX_SIZE)) {
        printf("[OTA] bad fw_size=%lu\r\n", (unsigned long)fw_size);
        return 1;
    }
    printf("[OTA] selftest: fw_size=%lu (0x%08lX..0x%08lX)\r\n",
           (unsigned long)fw_size, (unsigned long)OTA_APP_ADDR,
           (unsigned long)(OTA_APP_ADDR + fw_size));

    /* 1) 先擦掉暂存区需要的空间（含头部与固件数据） */
    if (W25Q64_EraseRange(OTA_STAGE_ADDR, OTA_HDR_SIZE + fw_size) != W25Q64_OK) {
        printf("[OTA] erase stage failed\r\n");
        return 2;
    }

    /* 2) 分块把 APP 自己读出来 → 累加 CRC → 写进暂存区（固件数据从 +16 开始） */
    while (off < fw_size) {
        uint32_t n = ((fw_size - off) > (uint32_t)sizeof(buf)) ? (uint32_t)sizeof(buf)
                                                              : (fw_size - off);
        const uint8_t *p = (const uint8_t *)(OTA_APP_ADDR + off);
        uint32_t i;

        for (i = 0U; i < n; i++) buf[i] = p[i];       /* 内部 Flash 只读，直接拷贝 */
        crc = Ota_CRC16Update(crc, buf, n);

        if (W25Q64_Write(OTA_STAGE_ADDR + OTA_HDR_SIZE + off, buf, n) != W25Q64_OK) {
            printf("[OTA] write stage failed @+%lu\r\n", (unsigned long)off);
            return 3;
        }
        off += n;
    }

    /* 3) 头部最后写（见函数注释：保证"半途断电 = 无有效固件"） */
    hdr.magic   = OTA_MAGIC;
    hdr.fw_size = fw_size;
    /* corrupt_crc != 0 时故意写一个错值 —— 用于验证 Bootloader 会拒绝搬运 */
    hdr.fw_crc  = corrupt_crc ? (uint32_t)(crc ^ 0xFFFFU) : (uint32_t)crc;
    hdr.version = 1U;                                  /* 自测固定填 1 */
    if (W25Q64_Write(OTA_STAGE_ADDR, (uint8_t *)&hdr, sizeof(hdr)) != W25Q64_OK) {
        printf("[OTA] write header failed\r\n");
        return 4;
    }

    /* 打印实际写进头部的 crc（hdr.fw_crc），而不是本地算出的正确值 ——
     * 这样 ota_bad 时日志与 Bootloader 侧看到的 expect 值一致，便于对照排查。 */
    printf("[OTA] staged%s: size=%lu crc=0x%04X ver=%lu -> reboot to apply\r\n",
           corrupt_crc ? " (BAD crc!)" : "",
           (unsigned long)fw_size, (unsigned)hdr.fw_crc, (unsigned long)hdr.version);
    return 0;
}

/* ======================== OTA 固件接收======================== */

#define OTA_SRV_PORT        9000U          /* PC 上 ota_sender.py 的监听端口（与脚本一致） */
#define OTA_LINE_MAX        192U           /* 单行数据最大长度（128 hex 字符 + 前后余量） */
#define OTA_RECV_TIMEOUT_MS 60000U         /* 整体接收超时（60 s，够传 98 KB） */
#define OTA_IDLE_TIMEOUT_MS 8000U          /* 连续无数据超时（8 s） */

/* 接收状态机（必须跨 ESP_StreamTake 调用保持）*/
typedef enum { OTA_S_WAIT_IPD = 0, OTA_S_IN_DATA } OtaRxState_t;

static OtaRxState_t s_ota_st     = OTA_S_WAIT_IPD;
static uint32_t     s_ota_need   = 0U;         /* 本次 +IPD 还剩多少字节未收 */
static char         s_ota_hdr[24];             /* 收集 "+IPD,<len>:" 直到冒号 */
static uint8_t      s_ota_hdrlen = 0U;
static char         s_ota_line[OTA_LINE_MAX];  /* 正在拼装的一行数据 */
static uint16_t     s_ota_linelen = 0U;

/* 本次接收的统计与结果 */
static uint32_t s_ota_off    = 0U;             /* 已写入 W25Q64 的字节数 */
static uint16_t s_ota_crc    = 0xFFFFU;        /* 累加 CRC16 */
static uint32_t s_ota_expsz  = 0U;             /* END 帧声明的总长度 */
static uint16_t s_ota_expcrc = 0U;             /* END 帧声明的 CRC */
static uint8_t  s_ota_gotend = 0U;             /* 已收到 END 帧 */
static uint8_t  s_ota_bad    = 0U;             /* 解析/写入过程中出错 */

/**
 * @brief  十进制解析（手写；MicroLIB 下 sscanf 的 %lu 不可靠）
 * @param  endp 输出解析结束位置（可为 NULL）
 */
static uint32_t Ota_ParseU32(const char *s, const char **endp)
{
    uint32_t v = 0U;
    while (*s == ' ') s++;
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10U + (uint32_t)(*s - '0');
        s++;
    }
    if (endp != NULL) *endp = s;
    return v;
}

/**
 * @brief  十六进制字符 → 数值；非法返回 -1
 */
static int Ota_HexNibble(char c)
{
    if ((c >= '0') && (c <= '9')) return (int)(c - '0');
    if ((c >= 'A') && (c <= 'F')) return (int)(c - 'A') + 10;
    if ((c >= 'a') && (c <= 'f')) return (int)(c - 'a') + 10;
    return -1;
}

/**
 * @brief  处理一行数据（已剥掉尾部 CR/LF）
 * @note   分三类：
 *           ① "END,<size>,<crc>" → 结束帧，记下期望值
 *           ② 偶数长度的纯 hex 串 → 解码 64 B，累加 CRC 并写 W25Q64
 *           ③ 其它（OK/ERROR/空行等）→ 直接忽略
 */
static void Ota_HandleLine(const char *line)
{
    static uint8_t bin[OTA_LINE_MAX / 2];      /* 静态：不占任务栈 */
    size_t   len = strlen(line);
    uint32_t binlen;
    uint32_t i;

    if (len == 0U) return;

    /* ---- ① 结束帧 ---- */
    if (strncmp(line, "END,", 4U) == 0) {
        const char *p  = line + 4U;
        const char *pe = NULL;
        s_ota_expsz = Ota_ParseU32(p, &pe);
        if ((pe != NULL) && (*pe == ',')) p = pe + 1;
        s_ota_expcrc = 0U;
        while (*p != '\0') {
            int h = Ota_HexNibble(*p);
            if (h < 0) break;
            s_ota_expcrc = (uint16_t)((s_ota_expcrc << 4) | (uint16_t)h);
            p++;
        }
        s_ota_gotend = 1U;
        printf("[OTA] END frame: size=%lu crc=0x%04X\r\n",
               (unsigned long)s_ota_expsz, (unsigned)s_ota_expcrc);
        return;
    }

    /* ---- ② hex 数据行：必须偶数长度 ---- */
    if ((len & 1U) != 0U) return;

    binlen = (uint32_t)(len / 2U);
    if (binlen > sizeof(bin)) { s_ota_bad = 1U; return; }

    for (i = 0U; i < binlen; i++) {
        int hi = Ota_HexNibble(line[i * 2U]);
        int lo = Ota_HexNibble(line[i * 2U + 1U]);
        if ((hi < 0) || (lo < 0)) return;      /* 含非 hex 字符 → 是状态行，忽略 */
        bin[i] = (uint8_t)((hi << 4) | lo);
    }

    s_ota_crc = Ota_CRC16Update(s_ota_crc, bin, binlen);   /* 累加整包 CRC */

    if ((s_ota_off + binlen) > OTA_APP_MAX_SIZE) {
        printf("[OTA] too much data (%lu), abort\r\n", (unsigned long)s_ota_off);
        s_ota_bad = 1U;
        return;
    }

    /* 这里不能做擦除 —— 擦除必须提前到接收开始之前（见 Ota_RecvFromTCP）。
     *   原因：STM32F4 擦写 Flash 时同 Bank 取指会 stall -> 中断进不来，
     *   而 UART 的 DMA 是 NORMAL 模式（收满一帧就停，要等中断回调里重新武装），
     *   中断被 stall 的几十毫秒里到达的数据会整帧丢失。
     *   实测现象：头部（最后写、无需擦）正确，而内容 CRC 不符 ——
     *   got 0x05AB, expect 0x6F75，即"收到的数据对、写进去的不对"。
     *   -> 擦除已改为在 CIPSTART 之前一次性完成。 */
    if (W25Q64_Write(OTA_STAGE_ADDR + OTA_HDR_SIZE + s_ota_off, bin, binlen) != W25Q64_OK) {
        printf("[OTA] stage write failed at +%lu\r\n", (unsigned long)s_ota_off);
        s_ota_bad = 1U;
        return;
    }
    s_ota_off += binlen;

    /* 每 8 KB 报一次进度，便于观察 */
    if ((s_ota_off & 0x1FFFU) < binlen) {
        printf("[OTA] recv %lu B\r\n", (unsigned long)s_ota_off);
    }
}

/**
 * @brief  把 ESP 收到的一段字节喂给 OTA 状态机
 * @param  buf 数据；n 字节数
 * @note   不能"按行"收：ESP-AT 的输出是 +IPD,<len>:<data>，而 <data>
 *         可能被 TCP 任意拆分（一次 +IPD 里只有半个数据行）。所以必须：
 *           状态 1：把 +IPD,<len>: 逐字节收集到冒号，解析出 <len>
 *           状态 2：精确读取 <len> 个字节，期间再按 \n 切分成"行"交给 Ota_HandleLine
 *         <len> 读完就回到状态 1 —— 这样无论 TCP 怎么拆包都能正确重组。
 */
static void Ota_Feed(const char *buf, uint16_t n)
{
    uint16_t i;

    for (i = 0U; i < n; i++) {
        char c = buf[i];

        if (s_ota_st == OTA_S_WAIT_IPD) {
            if (s_ota_hdrlen < (uint8_t)(sizeof(s_ota_hdr) - 1U)) {
                s_ota_hdr[s_ota_hdrlen++] = c;
            }
            if (c == ':') {                          /* 前缀收齐 */
                s_ota_hdr[s_ota_hdrlen] = '\0';
                if (strncmp(s_ota_hdr, "+IPD,", 5U) == 0) {
                    s_ota_need = Ota_ParseU32(s_ota_hdr + 5U, NULL);
                    s_ota_st   = OTA_S_IN_DATA;
                }
                /* 非 IPD 前缀（OK / ERROR / 空行）→ 直接丢 */
                s_ota_hdrlen = 0U;
            } else if (c == '\n') {
                s_ota_hdrlen = 0U;                   /* 整行都没有 ':' → 丢弃，防无限累积 */
            }
        } else {
            /* ---- 数据模式 ---- */
            if (c == '\n') {
                s_ota_line[s_ota_linelen] = '\0';
                Ota_HandleLine(s_ota_line);
                s_ota_linelen = 0U;
            } else if (c != '\r') {
                if (s_ota_linelen < (uint16_t)(OTA_LINE_MAX - 1U)) {
                    s_ota_line[s_ota_linelen++] = c;
                } else {
                    s_ota_linelen = 0U;              /* 超长行 → 丢弃 */
                }
            }
            if (s_ota_need > 0U) s_ota_need--;
            if (s_ota_need == 0U) s_ota_st = OTA_S_WAIT_IPD;
        }
    }
}

/**
 * @brief  把 ESP 累积流里的残留数据全部消费掉（丢弃）
 * @note   用于在建立 TCP 连接前清理回显，避免被误当成固件数据。
 */
static void Ota_DrainStream(void)
{
    char tmp[128];
    uint32_t guard = 0U;

    while ((ESP_StreamTake(tmp, sizeof(tmp)) > 0U) && (guard++ < 64U)) { }
}

/**
 * @brief  主流程：从 PC 通过 TCP 收固件 → 写 W25Q64 暂存区 → 置标志
 * @retval 0=成功（已置标志，可复位）；非 0=失败
 * @note   与 MQTT 分时复用：接收期间 MQTT 是断开的，收完（或失败）后由调用者
 *         的重连逻辑恢复（esp_ok=0 → 下一轮 ESP_Init）。
 *         与自测写入（Ota_PrepareSelfTest）的区别：本函数收的是真正的 PC 固件。
 *         本函数会阻塞（最长 60 s），只在收到 ota_recv 命令时调用 ——
 *            此时 MQTT 已断开、上报本就停着，阻塞是可接受的。
 */
static int Ota_RecvFromTCP(void)
{
    char     cmd[128];
    char     buf[256];
    uint32_t t0, tlast;
    int      ret = 1;

    /* 清状态 */
    s_ota_st = OTA_S_WAIT_IPD; s_ota_need = 0U; s_ota_hdrlen = 0U;
    s_ota_linelen = 0U; s_ota_off = 0U; s_ota_crc = 0xFFFFU;
    s_ota_expsz = 0U; s_ota_expcrc = 0U; s_ota_gotend = 0U; s_ota_bad = 0U;

    /* ---- 1) 与 MQTT 分时复用：先断开 ---- */
    printf("[OTA] disconnecting MQTT before TCP transfer...\r\n");
    (void)ESP_SendCmd("AT+MQTTDISCONNECT=0\r\n", "OK|ERROR", NULL, 2000);
    HAL_Delay(500);
    Ota_DrainStream();

    /* ---- 2) 建立到 PC 的 TCP 连接之前，先把暂存区擦干净 ----
     * 擦除必须在开始接收之前做，不能边收边擦：
     *   STM32F4 擦写 Flash 时同 Bank 取指会 stall -> 中断进不来；
     *   而 UART 的 DMA 是 NORMAL 模式（收满一帧就停，要等空闲中断回调里重新武装），
     *   中断被 stall 的几十毫秒里到达的数据会整帧丢失。
     * EraseRange 内部对整 64 KB 块会优先用块擦（0xD8，约 150 ms/块）；
     *   704 KB ≈ 11 个块 ≈ 1.7 s —— 远快于逐扇区擦（176 扇区 × 45 ms ≈ 8 s）。
     * 此段执行时 MQTT 已断开、TCP 尚未建立 ->没有任何数据流-> 擦除安全。 */
    printf("[OTA] erasing stage area (%lu KB)...\r\n",
           (unsigned long)((OTA_HDR_SIZE + OTA_APP_MAX_SIZE) / 1024U));
    if (W25Q64_EraseRange(OTA_STAGE_ADDR, OTA_HDR_SIZE + OTA_APP_MAX_SIZE) != W25Q64_OK) {
        printf("[OTA] stage erase failed\r\n");
        return 1;
    }
    printf("[OTA] stage erased\r\n");

    /* ---- 3) 建立到 PC 的 TCP 连接 ---- */
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
             SERVER_IP, OTA_SRV_PORT);
    if (ESP_SendCmd(cmd, "CONNECT|OK", "ERROR", 8000) != 0) {
        printf("[OTA] CIPSTART failed (is ota_sender.py running on the PC?)\r\n");
        return 1;
    }
    printf("[OTA] TCP connected to %s:%u, receiving...\r\n",
           SERVER_IP, (unsigned)OTA_SRV_PORT);

    /* ---- 3) 接收循环 ---- */
    t0    = HAL_GetTick();
    tlast = t0;
    while ((s_ota_gotend == 0U) && (s_ota_bad == 0U)) {
        uint16_t n = ESP_StreamTake(buf, (uint16_t)sizeof(buf));

        if (n > 0U) {
            Ota_Feed(buf, n);
            tlast = HAL_GetTick();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));      /* 没数据就让出 CPU */
        }

        if ((HAL_GetTick() - t0) > OTA_RECV_TIMEOUT_MS) {
            printf("[OTA] recv timeout (%lu B received)\r\n", (unsigned long)s_ota_off);
            break;
        }
        if ((HAL_GetTick() - tlast) > OTA_IDLE_TIMEOUT_MS) {
            printf("[OTA] idle timeout (%lu B received)\r\n", (unsigned long)s_ota_off);
            break;
        }
    }

    /* ---- 4) 回读校验 + 落盘头部 ----
     * 先回读暂存区自己算一遍 CRC，用来一锤定音地区分两种失败：
     *    · 回读 CRC 就错  → 写入侧有问题（SPI 时序 / 信号完整性 / 供电）
     *    · 回读 CRC 对、但 Bootloader 读出来错 → 读取侧有问题
     * 之前连续几轮都是"收到的数据 CRC 对、复位后 Bootloader 读出来错"，
     * 但无法区分是写坏了还是读错了 -> 加上这一步。
     */
    if ((s_ota_gotend != 0U) && (s_ota_bad == 0U)) {
        {
            char     rbuf[256];
            uint16_t rcrc = 0xFFFFU;
            uint32_t roff = 0U;
            int      rerr = 0;

            while (roff < s_ota_off) {
                uint16_t rn = ((s_ota_off - roff) > (uint32_t)sizeof(rbuf))
                              ? (uint16_t)sizeof(rbuf) : (uint16_t)(s_ota_off - roff);
                if (W25Q64_Read(OTA_STAGE_ADDR + OTA_HDR_SIZE + roff,
                                (uint8_t *)rbuf, rn) != W25Q64_OK) {
                    rerr = 1;
                    break;
                }
                rcrc = Ota_CRC16Update(rcrc, (const uint8_t *)rbuf, rn);
                roff += rn;
            }
            if (rerr != 0) {
                printf("[OTA] readback FAILED (W25Q64 read error)\r\n");
            } else {
                printf("[OTA] readback crc=0x%04X recv crc=0x%04X\r\n",
                       (unsigned)rcrc, (unsigned)s_ota_crc);
                if (rcrc != s_ota_crc) {
                    printf("[OTA] * WRITE PATH BROKEN (data not landed correctly) *\r\n");
                }
            }
        }
    }

    if ((s_ota_gotend != 0U) && (s_ota_bad == 0U)) {
        printf("[OTA] recv done: got %lu B, exp %lu B; crc got 0x%04X exp 0x%04X\r\n",
               (unsigned long)s_ota_off, (unsigned long)s_ota_expsz,
               (unsigned)s_ota_crc, (unsigned)s_ota_expcrc);

        if ((s_ota_off == s_ota_expsz) && (s_ota_crc == s_ota_expcrc)) {
            OtaStageHeader_t h;
            h.magic   = OTA_MAGIC;
            h.fw_size = s_ota_off;
            h.fw_crc  = (uint32_t)s_ota_crc;
            h.version = 2U;                      /* PC 下发的固件记 ver=2 */
            /* 头部最后写：中途断电则 magic 不完整 -> Bootloader 视为无有效固件，天然安全 */
            if (W25Q64_Write(OTA_STAGE_ADDR, (uint8_t *)&h, sizeof(h)) == W25Q64_OK) {
                printf("[OTA] STAGED OK: size=%lu crc=0x%04X ver=%lu -> reboot to apply\r\n",
                       (unsigned long)h.fw_size, (unsigned)s_ota_crc,
                       (unsigned long)h.version);
                ret = 0;
            } else {
                printf("[OTA] header write failed\r\n");
            }
        } else {
            printf("[OTA] MISMATCH -> refuse (size/crc not match)\r\n");
        }
    } else {
        printf("[OTA] incomplete transfer (no END frame)\r\n");
    }

    /* ---- 5) 断开 TCP ---- */
    (void)ESP_SendCmd("AT+CIPCLOSE\r\n", "OK|ERROR", NULL, 2000);
    Ota_DrainStream();
    printf("[OTA] TCP closed\r\n");
    return ret;
}

void App_MqttOnCommand(const char *payload, size_t len)
{
    char  json[192];
    char  cmd[16];
    float v = 0.0f;

    if (len >= sizeof(json)) len = sizeof(json) - 1U;
    memcpy(json, payload, len);
    json[len] = '\0';

    if (!JsonGetStr(json, "cmd", cmd, sizeof(cmd))) {
        printf("[MQTT-CMD] no cmd field, ignored\r\n");
        return;
    }

    /* ---- ① 改报警上限 ---- */
    if (strcmp(cmd, "set_hi") == 0) {
        if (!JsonGetNum(json, "v", &v)) {
            printf("[MQTT-CMD] set_hi missing v, ignored\r\n");
            return;
        }
        if (v < ALARM_HIGH_MIN) v = ALARM_HIGH_MIN;       /* 越界钳位 */
        if (v > ALARM_HIGH_MAX) v = ALARM_HIGH_MAX;
        g_temp_alarm_high  = v;
        g_temp_alarm_clear = v - 1.0f;                    /* 与按键逻辑保持一致 */
        g_param_dirty = 1;   /* 标脏：远程改的阈值一样要掉电保存，走同一条持久化路径 */
        printf("[MQTT-CMD] set_hi=%.1f (range %.0f~%.0f)\r\n",
               (double)v, (double)ALARM_HIGH_MIN, (double)ALARM_HIGH_MAX);
        snprintf(g_cmd_ack, sizeof(g_cmd_ack), "{\"ack\":\"set_hi\",\"hi\":%.1f}", (double)v);
        if (esp_ok) (void)ESP_MqttPub(MQTT_TOPIC_PUB, g_cmd_ack);
        return;
    }

    /* ---- ② 查询当前状态 ---- */
    if (strcmp(cmd, "get") == 0) {
        SensorReport_t r;
        if (xQueuePeek(xQueueSensor, &r, 0) != pdTRUE) {  /* 非阻塞：拿不到就用 0 */
            r.lux = 0.0f; r.temp = 0.0f; r.humi = 0.0f; r.alarm = 0;
        }
        snprintf(g_cmd_ack, sizeof(g_cmd_ack),
                 "{\"ack\":\"get\",\"hi\":%.1f,\"lux\":%.1f,\"temp\":%.1f,\"humi\":%.1f,\"alarm\":%d}",
                 (double)g_temp_alarm_high, (double)r.lux, (double)r.temp, (double)r.humi, (int)r.alarm);
        printf("[MQTT-CMD] get\r\n");
        if (esp_ok) (void)ESP_MqttPub(MQTT_TOPIC_PUB, g_cmd_ack);
        return;
    }

    /* ---- OTA 自测：把当前 APP 自己写进 W25Q64 暂存区（验证用）----
     * 用法（MQTTX 向 rs485gw/cmd 发）：
     *   {"cmd":"ota"}      → 把当前 APP 写入暂存区（CRC 正确）
     *   {"cmd":"ota_bad"}  → 同上但故意写错 CRC—— 用于验证
     *                        "校验不过时 Bootloader 拒绝搬运、旧 APP 照常运行"
     *                        （这是 OTA 的硬指标：刷坏不能变砖）
     * 之后复位 → Bootloader 会校验并搬运 → 再次上电应能正常跑。
     * 命令名刻意用短的："ota_selftest_bad" 有 16 字符，会撑爆上面的 char cmd[16]。 */
    if ((strcmp(cmd, "ota") == 0) || (strcmp(cmd, "ota_bad") == 0)) {
        int r = Ota_PrepareSelfTest((strcmp(cmd, "ota_bad") == 0) ? 1 : 0);
        snprintf(g_cmd_ack, sizeof(g_cmd_ack), "{\"ack\":\"%s\",\"r\":%d}", cmd, r);
        if (esp_ok) (void)ESP_MqttPub(MQTT_TOPIC_PUB, g_cmd_ack);
        return;
    }

    /* ---- OTA 接收：从 PC 通过 TCP 收真实固件----
     * 用法（顺序很重要）：
     *   ① MQTTX 向 rs485gw/cmd 发 {"cmd":"ota_recv"}
     *   ② 立刻在 PC 上跑：python tools/ota_sender.py Doc/app_v1.bin
     *   ③ 设备收完并校验通过后会打印 STAGED OK → 复位 → Bootloader 搬运
     * 本命令会让设备断开 MQTT并阻塞数十秒接收固件（与 MQTT 分时复用）；
     *    返回时把 esp_ok 置 0，让 vTaskNet 的重连逻辑在下一轮恢复 MQTT。 */
    if (strcmp(cmd, "ota_recv") == 0) {
        int r = Ota_RecvFromTCP();
        esp_ok = 0;                       /* MQTT 已在接收前断开，交回重连逻辑 */
        printf("[OTA] recv result=%d, MQTT will reconnect\r\n", r);
        return;
    }

    /* ---- 未知命令：忽略并记日志（绝不让非法输入影响设备运行）---- */
    printf("[MQTT-CMD] unknown cmd=[%s], ignored\r\n", cmd);
}

/**
 * @brief  MQTT 断线回调：清掉连接标志，下一轮 vTaskNet 会走现有重连流程
 */
void App_MqttOnDisconnect(void)
{
    esp_ok = 0;
    printf("[MQTT] link lost, will reconnect\r\n");
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
                                g_param_dirty = 1;   /* 标脏：交给 vTaskOLED 防抖落盘 */
                                printf("[UI] HI=%.1f\r\n", g_temp_alarm_high);
                            }
                        } else if (i == 2) {                /* KEY3: 阈值- */
                            if (g_ui_state == UI_SETTING) {
                                g_temp_alarm_high -= ALARM_STEP;
                                if (g_temp_alarm_high < ALARM_HIGH_MIN) g_temp_alarm_high = ALARM_HIGH_MIN;
                                g_temp_alarm_clear = g_temp_alarm_high - 1.0f;
                                g_param_dirty = 1;   /* 标脏：交给 vTaskOLED 防抖落盘 */
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* 必须显式开全局中断，否则经 Bootloader 跳转启动时整个 APP 会被"哑"住：
   *   Bootloader 的 JumpToApp() 跳转前会 __disable_irq()（PRIMASK=1）并清掉
   *   所有 NVIC 使能位；而 Cortex-M 的启动代码（Reset_Handler / SystemInit）
   *   不会自动开中断 -> 若这里不显式开，PRIMASK 会一直是 1
   *   -> 所有中断都进不来 -> TIM6 时基不涨 -> HAL_Delay() 死循环
   *   -> 表现为"卡在第一个用到 HAL_Delay 的地方"（本项目是 W25Q64_Init 里的
   *     HAL_Delay(10)），后面 OLED/OTA 状态打印全都不执行。
   *   实测现象：直接复位启动一切正常，经 Bootloader 跳转就卡住 ——
   *      差异正是 PRIMASK。Bootloader 侧同样也补了这一句。
   *   注：NVIC 使能位不用管 —— 各 MspInit 与 HAL_Init() 会重新使能自己需要的中断。 */
  __enable_irq();

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
    ESP_RxInit();   /* ESP-01S 接收通道：USART3 的 DMA + 空闲中断（须在调度器启动前、USART3 初始化之后） */
    Param_Load();   /* 上电加载报警阈值：有有效参数则覆盖默认值；必要时做一次扇区整理 */
    /* 外部 SPI Flash 自检（须在 MX_SPI1_Init 之后）：读 JEDEC ID，期望 0xEF4017。
     * 读 ID 是 µs 级，放调度器前不影响启动时间。 */
    (void)W25Q64_Init();
#if W25Q64_SELFTEST_ON_BOOT
    /* 读写一致性验证：擦 1 个扇区 → 写 256 B 递增模式 → 读回比对（约 50 ms） */
    (void)W25Q64_SelfTest(OTA_STAGE_ADDR);
#endif

    /* OTA 暂存区状态：STAGED = 有一份待搬运的固件（下次复位会被 Bootloader 搬走）。
     * 这是搬运验收的关键判据：
     *     触发自测后应打印 STAGED；Bootloader 搬运成功后会变成 empty。 */
    {
        OtaStageHeader_t oh;
        if (W25Q64_Read(OTA_STAGE_ADDR, (uint8_t *)&oh, sizeof(oh)) == W25Q64_OK) {
            if (oh.magic == OTA_MAGIC) {
                printf("[OTA] stage=STAGED size=%lu crc=0x%04X ver=%lu\r\n",
                       (unsigned long)oh.fw_size, (unsigned)(oh.fw_crc & 0xFFFFU),
                       (unsigned long)oh.version);
            } else {
                printf("[OTA] stage=empty\r\n");
            }
        }
    }
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
