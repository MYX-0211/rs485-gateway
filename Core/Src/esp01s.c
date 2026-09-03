/*============================================================================
 * esp01s.c — ESP-01S (ESP8266) Wi-Fi 驱动（USART3，115200 8N1）
 *----------------------------------------------------------------------------
 * 主要流程：
 *   1. 初始化：退出透传 → 复位 → AT 探测 → STA 配网 → TCP 连接
 *   2. 上报：CIPSEND 普通模式（等 '>' → 发数据 → 等 SEND OK）
 *
 * 针对 UART 丢字节环境的关键设计：
 *   1. 全缓冲 strstr 搜索，响应前有残留字节也能命中
 *   2. 多关键字匹配（expect/reject 支持 "OK|CONNECT" 任一命中即成功）
 *   3. 分阶段安全重试：仅在数据未发出时重试，避免服务器收到重复帧
 * ==========================================================================*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "esp01s.h"
#include "FreeRTOS.h"
#include "task.h"

/* ======================== 配置宏 ======================== */
#define ESP_RX_BUF_SIZE   128U   /* 接收缓冲区：AT 回复通常 <100 字节 */
#define ESP_CHAR_TIMEOUT  10U    /* 单字符接收超时(ms)：短超时降低阻塞，空闲让出 CPU */

/* ======================== 错误码 ======================== */
typedef enum {
    ESP_OK = 0,
    ESP_ERR_AT,      /* AT 探测失败 */
    ESP_ERR_MODE,    /* CWMODE 模式配置失败 */
    ESP_ERR_WIFI,    /* CWJAP 连不上 Wi-Fi */
    ESP_ERR_TCP,     /* CIPSTART TCP 连接失败 */
    ESP_ERR_TX,      /* CIPSEND 未收到 '>' 提示符 */
    ESP_ERR_ACK      /* 数据未获 'SEND OK' 确认 */
} ESP_Ret;

extern UART_HandleTypeDef huart3; /* ESP-01S 串口句柄（usart.c 中定义） */

/* ======================== 内部工具函数 ======================== */

/**
 * @brief  清空 USART3 接收缓冲区
 * @note   发新 AT 命令前必须调用，避免上次命令的残留回显
 *         污染本次的关键字匹配；读到空闲即停，50ms 总时长兜底防死循环
 */
static void ESP_FlushRX(void)
{
    uint8_t ch;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 50U) {
        if (HAL_UART_Receive(&huart3, &ch, 1, 5) != HAL_OK) break;  /* 读到空闲即退出 */
    }
}

/**
 * @brief  裸发字符串（不加 \r\n，不做等待）
 * @note   仅用于 "+++" 这类特殊指令；正常 AT 命令一律走 ESP_SendCmd
 */
static void ESP_SendRaw(const char *s){
    HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), 200);
}

/**
 * @brief  多关键字匹配：pattern 用 '|' 分隔，任一命中即返回 1
 * @param  rx      接收缓冲区内容（已 '\0' 结尾）
 * @param  pattern 匹配模式，如 "OK|WIFI GOT IP" / "ERROR|ALREADY"
 * @retval 1=命中任一关键字, 0=全部未命中
 * @note   丢字节环境下 OK/CONNECT/WIFI GOT IP 谁完整到谁算成功，
 *         避免"OK 到了但 CONNECT 被拆碎"这类白等超时
 */
static int ESP_MatchAny(const char *rx, const char *pattern)
{
    char tmp[64];                       /* 拷贝模式串，供 strchr 就地分割 */
    size_t n = strlen(pattern);
    if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
    memcpy(tmp, pattern, n);
    tmp[n] = '\0';

    char *p = tmp;
    while (p != NULL) {
        char *tok = p;                  /* 当前关键字起点 */
        p = strchr(tok, '|');
        if (p) *p++ = '\0';             /* 逐段切出各关键字 */
        if (strstr(rx, tok) != NULL) return 1;
    }
    return 0;
}

/* ======================== 核心命令函数 ======================== */

/**
 * @brief  发送 AT 命令并等待期望回复（单命令完整事务）
 * @param  cmd        AT 命令字符串（需包含 \r\n）
 * @param  expect     期望关键字，支持 '|' 多关键字（如 "OK|WIFI GOT IP"）；
 *                    传 NULL 则只发不收（fire-and-forget，如 AT+RST）
 * @param  reject     失败特征关键字，同样支持 '|'（如 "ERROR|FAIL"）；
 *                    传 NULL 表示不检测
 * @param  timeout_ms 总超时时间(ms)
 * @retval 0=成功（命中 expect 任一关键字）；-1=超时；-2=命中 reject（模块明确报错）
 * @note   局部缓冲区 128B，调用任务栈需 ≥256 字；10ms 短超时轮询收字节，
 *         空闲时 vTaskDelay(1) 让出 CPU
 */
int ESP_SendCmd(const char *cmd, const char *expect, const char *reject, uint32_t timeout_ms)
{
    char rx[ESP_RX_BUF_SIZE];     /* 接收缓冲区（栈上） */
    uint16_t idx = 0;             /* 缓冲写入位置 */
    uint32_t t0 = HAL_GetTick();  /* 起始 tick，用于总超时计算 */
    size_t reject_len = reject ? strlen(reject) : 0;  /* 0=不检测失败特征 */
    int    reject_hit = 0;        /* 是否命中失败特征 */

    /* ---- Step 1：清场 ---- */
    ESP_FlushRX();                /* 丢弃上次命令残留字节 */
    memset(rx, 0, sizeof(rx));    /* 清零缓冲，保证 '\0' 结尾 */

    /* ---- Step 2：发送命令（200ms 发送超时） ---- */
    if (HAL_UART_Transmit(&huart3, (uint8_t *)cmd,
                          strlen(cmd), 200) != HAL_OK)
    {
        printf("[ESP-ERR] Transmit failed: %s\r\n", cmd);
        return -1;
    }

    /* ---- fire-and-forget：不需要回复（如 AT+RST），直接返回成功 ---- */
    if (expect == NULL) return 0;

    /* ---- Step 3：在总超时时间内逐字节接收并匹配 ---- */
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        uint8_t ch;
        /* 10ms 短超时轮询：收到立即处理，未收到快速回到循环判断总超时 */
        if (HAL_UART_Receive(&huart3, &ch, 1, ESP_CHAR_TIMEOUT) == HAL_OK)
        {
            /* 3.1 存入缓冲区，末尾补 '\0' 保持字符串合法 */
            if (idx < ESP_RX_BUF_SIZE - 1)
            {
                rx[idx++] = (char)ch;
                rx[idx] = '\0';
            }

            /* 3.2 成功匹配：expect 任一关键字出现 → 本次命令成功 */
            if (ESP_MatchAny(rx, expect) != 0)
            {
                return 0;
            }

            /* 3.3 失败检测：reject 任一关键字出现 → 模块已明确报错，立即退出 */
            if (reject_len && ESP_MatchAny(rx, reject) != 0)
            {
                reject_hit = 1;
                break;
            }
        } else {
            /* 3.4 本轮无数据：让出 CPU，避免饿死低优先级任务。
             * 注意：必须在 else 分支让出——收到数据期间若切走会打断
             * ESP 连续回复，造成 UART 丢字节(ORE) */
            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                vTaskDelay(1);
        }
    }

    /* ---- 超时/失败收尾：打印完整回显便于定位 ---- */
    if (reject_hit) {
        printf("[ESP] REJECT cmd=[%s] reject=[%s] rx=[%s]\r\n",
               cmd, reject, rx);
        return -2;                /* 模块明确报错（如 FAIL/ERROR/ALREADY） */
    }
    printf("[ESP] TIMEOUT cmd=[%s] expect=[%s] rx=[%s]\r\n",
           cmd, expect, rx);
    return -1;                    /* 超时未匹配（多为丢字节或链路异常） */
}

/* ======================== 初始化流程 ======================== */

/**
 * @brief  初始化 ESP-01S：退出透传 → 复位 → 探测 → STA 配网 → 连 TCP
 * @param  ssid    Wi-Fi 热点名（仅支持 2.4G）
 * @param  pwd     Wi-Fi 密码
 * @param  pc_ip   服务器 IP（PC 当前 IP，换网络需同步改 main.c 宏）
 * @param  pc_port 服务器端口（PC 网络助手必须 TCPServer 模式）
 * @retval ESP_OK 成功；其他 ESP_ERR_xxx 对应失败阶段
 * @note   全流程阻塞，最坏约 50s（AT 探测/CWJAP/CIPSTART 各最多重试 3 次）
 */
int ESP_Init(const char *ssid, const char *pwd, const char *pc_ip, uint16_t pc_port)
{
    char cmd[96];                 /* AT 命令拼装缓冲（CWJAP/CIPSTART 用） */

    /* ---- 0) 退出透传态 ----
     * 若模块被存过 AT+SAVETRANSLINK（掉电保存的透传连接），上电会直接透传，
     * 表现为"串口收到 AT/AT+RST 等乱串"。+++ 可退出透传；普通模式此命令被忽略。 */
    ESP_SendRaw("+++");
    HAL_Delay(1000);              /* +++ 之后 1s 内不能发送任何数据 */
    ESP_FlushRX();                /* 丢弃透传残留字节 */

    HAL_Delay(200);

    /* ---- 1) RST 复位：无论上电处于什么状态，先回到干净环境 ---- */
    ESP_SendCmd("AT+RST\r\n", NULL, NULL, 1000);   /* fire-and-forget，不等回显 */
    HAL_Delay(1500);              /* 等待模块重启完成 */
    ESP_FlushRX();

    /* ---- 2) AT 探测：确认模块响应；失败则复位重试（模块可能处于异常态） ---- */
    int at_ok = -1;
    for (int i = 0; i < 3; i++) {
        at_ok = ESP_SendCmd("AT\r\n", "OK", NULL, 2000);
        if (at_ok == 0) break;
        ESP_SendCmd("AT+RST\r\n", NULL, NULL, 1000);   /* 复位后重新探测 */
        HAL_Delay(2000);
        ESP_FlushRX();
    }
    if (at_ok) return ESP_ERR_AT;

    /* ---- 3) 强制普通模式 + 清掉掉电保存的透传连接（防止下次上电又透传） ---- */
    ESP_SendCmd("AT+CIPMODE=0\r\n", "OK", NULL, 2000);        /* 普通模式（CIPSEND 流程） */
    ESP_SendCmd("AT+SAVETRANSLINK=0\r\n", "OK", NULL, 2000);  /* 清除掉电保存的透传配置 */
    ESP_FlushRX();

    /* ---- 4) 网络模式：STA（连接外部路由/热点） + 单连接 ---- */
    if (ESP_SendCmd("AT+CWMODE=1\r\n", "OK", NULL, 2000)) return ESP_ERR_MODE;  /* STA */
    ESP_SendCmd("AT+CIPMUX=0\r\n", "OK", NULL, 2000);   /* 单连接：CIPSEND 不带连接号 */
    ESP_FlushRX();
    HAL_Delay(200);

    /* ---- 5) 连接 Wi-Fi ----
     * expect 多关键字：OK（命令完成）或 WIFI GOT IP（中间事件）任一命中即成功；
     * reject=ERROR|FAIL：密码错/AP 丢失等明确失败立即退出重试，不等满超时 */
    int cwjap_ret = -1;
    for (int i = 0; i < 3; i++) {
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
        cwjap_ret = ESP_SendCmd(cmd, "OK|WIFI GOT IP", "ERROR|FAIL", 10000);
        if (cwjap_ret == 0) break;      /* 连接成功 */
        if (cwjap_ret == -2) break;     /* 明确失败（如密码错误），无需继续等待 */
        HAL_Delay(1000);                /* 超时后延时 1s 再重试 */
    }
    if (cwjap_ret) return ESP_ERR_WIFI;

    ESP_FlushRX();
    HAL_Delay(500);               /* 等网络栈稳定（DHCP 完成） */

    /* ---- 6) 连接 TCP 服务器：CONNECT 或 OK 任一完整到达即成功 ---- */
    int cip_ret = -1;
    for (int i = 0; i < 3; i++) {
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", pc_ip, pc_port);
        cip_ret = ESP_SendCmd(cmd, "CONNECT|OK", "ERROR|ALREADY", 5000);
        if (cip_ret == 0) break;
        if (cip_ret == -2) break;       /* ALREADY CONNECTED 等明确错误 */
        HAL_Delay(1000);
    }
    if (cip_ret) return ESP_ERR_TCP;

    printf("[ESP] Init OK, connected to %s:%u\r\n", pc_ip, pc_port);
    return ESP_OK;
}

/* ======================== 数据上报 ======================== */

/**
 * @brief  通过 TCP 发送一帧数据（CIPSEND 普通模式）
 * @param  data 要发送的数据（长度由 strlen 决定）
 * @retval ESP_OK 成功；ESP_ERR_TX=没拿到 '>'；ESP_ERR_ACK=数据未获确认
 * @note   重试策略：
 *         ① 第一段（等 '>'）失败可安全重试 1 次——此时数据还没发出去；
 *         ② 第二段（等 SEND OK）失败【绝不重发】——数据可能已到服务器，
 *            重发会导致服务器收到两遍（TCP 重复投递）。
 */
int ESP_SendTCP(const char *data)
{
    char cmd[32];                 /* CIPSEND 命令缓冲 */
    size_t len = strlen(data);    /* 数据长度，告知 ESP 需接收的字节数 */

    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u\r\n", (unsigned int)len);

    /* ---- 第一段：等 '>' 提示符。拿不到说明链路已断/模块忙，重试 1 次 ---- */
    int ret = ESP_SendCmd(cmd, ">", NULL, 3000);
    if (ret != 0) {
        ESP_FlushRX();            /* 清掉 busy/ERROR 残留 */
        HAL_Delay(200);
        ret = ESP_SendCmd(cmd, ">", NULL, 3000);
        if (ret != 0) {
            printf("[ESP-ERR] CIPSEND did not get '>' (link may be down)\r\n");
            return ESP_ERR_TX;
        }
    }

    /* ---- 第二段：发数据，等 SEND OK 确认（绝不重发，见函数头注释） ---- */
    if (ESP_SendCmd(data, "SEND OK", NULL, 3000)) {
        printf("[ESP-ERR] Data not acked with 'SEND OK'\r\n");
        return ESP_ERR_ACK;
    }
    return ESP_OK;
}
