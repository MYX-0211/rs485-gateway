/*============================================================================
 * esp01s.c — ESP-01S (ESP8266) Wi-Fi 驱动（USART3，115200 8N1）
 *----------------------------------------------------------------------------
 * 主要流程：
 *   0. ESP_RxInit：启动 USART3 的 DMA + 空闲中断接收通道（调度器启动前调用）
 *   1. 初始化：退出透传 → 复位 → AT 探测 → STA 配网 → TCP 连接
 *   2. 上报：CIPSEND 普通模式（等 '>' → 发数据 → 等 SEND OK）
 *
 * 接收为什么用 DMA + 空闲中断，而不是逐字节轮询：
 *   逐字节阻塞轮询时，只要 CPU 离开轮询点（任务被抢占、或代码里的 vTaskDelay 让出），
 *   期间到达的字节就没人读；USART 只有 1 字节接收寄存器、无 FIFO，后续字节直接触发
 *   ORE 被丢弃。实测该丢字节会把完整回显拆成碎片，甚至出现"rx 里明明有 SEND OK 却匹配失败"。
 *   DMA 让硬件自动搬运字节，空闲中断给出帧边界，任务只在信号量上等待 —— 零丢字节。
 *
 * 关键字匹配：全缓冲 strstr，支持 expect/reject 多关键字（"OK|WIFI GOT IP"）
 * ==========================================================================*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "esp01s.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ======================== 配置宏 ======================== */
#define ESP_DMA_BUF_SIZE   128U   /* DMA 直写缓冲：单次空闲帧上限（AT 回复通常 <100 字节） */
#define ESP_STREAM_SIZE    768U   /* 单条命令周期内累积的字节流上限（容纳启动横幅 + 配网 URC） */

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

/* ======================== 接收通道（DMA + 空闲中断） ======================== */
static uint8_t  esp_dma_buf[ESP_DMA_BUF_SIZE];  /* 仅 DMA 写入，回调中立刻拷贝走 */
static char     esp_stream[ESP_STREAM_SIZE];    /* 累积流：供关键字匹配，始终以 '\0' 结尾 */
static volatile uint16_t esp_stream_len = 0;    /* 累积流当前长度（恒 ≤ ESP_STREAM_SIZE-1） */
static volatile uint8_t  esp_stream_ovf = 0;    /* 累积流溢出标志（1=本条命令内有字节被丢弃） */
static SemaphoreHandle_t xSemEspRx = NULL;      /* 空闲帧到达信号量 */
static volatile uint32_t esp_rx_frames  = 0;    /* 累计空闲帧数（诊断） */
static volatile uint32_t esp_rx_bytes   = 0;    /* 累计接收字节数（诊断） */
static volatile uint32_t esp_rearm_fail = 0;    /* DMA 重新武装失败次数（诊断，>0 说明会丢帧） */
static volatile uint32_t esp_err_cnt    = 0;    /* USART3 接收错误次数（诊断） */
static volatile uint32_t esp_err_code   = 0;    /* 最近一次错误码（HAL_UART_ERROR_xxx 位掩码） */
static volatile uint8_t  esp_rx_dead    = 0;    /* 1=错误路径把接收通道关掉了，需在任务上下文重新武装 */

/* ======================== URC 专用缓冲（⑥ Day 6 下行） ========================
 * 为什么不复用 esp_stream：ESP_SendCmd 开头会 ESP_FlushRX() 清累积流，
 * 若 URC 与命令响应共用一个缓冲，空闲期间到达的 URC 会被下一条命令清掉、永久丢失。
 * 这里让 DMA 回调同时喂两个缓冲：esp_stream 供命令匹配，esp_urc_buf 供 URC 消费。 */
#define ESP_URCBUF_SIZE   256U
static char              esp_urc_buf[ESP_URCBUF_SIZE];
static volatile uint16_t esp_urc_len = 0;   /* 有效长度；ISR 写、任务读，访问需临界区 */

/* 把新到的字节追加到 URC 缓冲；满了丢最旧的（保新，保证最新命令不被截断）
 * @note 在中断上下文调用，不做临界区（唯一写者就是 ISR 自己） */
static void ESP_AppendUrc(const uint8_t *src, uint16_t n)
{
    if (n == 0U) return;

    if (n > ESP_URCBUF_SIZE) {                 /* 单帧就超缓冲：只保留末尾 */
        src += (n - ESP_URCBUF_SIZE);
        n    = ESP_URCBUF_SIZE;
        esp_urc_len = 0;
    }
    if ((uint32_t)esp_urc_len + n > ESP_URCBUF_SIZE) {
        uint16_t drop = (uint16_t)((uint32_t)esp_urc_len + n - ESP_URCBUF_SIZE);
        memmove(esp_urc_buf, esp_urc_buf + drop, esp_urc_len - drop);
        esp_urc_len = (uint16_t)(esp_urc_len - drop);
    }
    memcpy(&esp_urc_buf[esp_urc_len], src, n);
    esp_urc_len = (uint16_t)(esp_urc_len + n);
}

/* 从 URC 缓冲头部丢弃 n 字节
 * @note 在任务上下文调用；自带临界区（防止与 ISR 的追加冲突），调用处不必再包 */
static void ESP_UrcDrop(uint16_t n)
{
    taskENTER_CRITICAL();
    if (n >= esp_urc_len) {
        esp_urc_len = 0;
    } else {
        memmove(esp_urc_buf, esp_urc_buf + n, esp_urc_len - n);
        esp_urc_len = (uint16_t)(esp_urc_len - n);
    }
    taskEXIT_CRITICAL();
}

/* 丢弃 URC 缓冲里的全部内容
 * @note 用途：ESP_MqttConn 自己发过 AT+MQTTDISCONNECT=0，模块会异步吐一条
 *       +MQTTDISCONNECT URC；若不丢弃，它会落进 ESP_PollURC 并触发
 *       App_MqttOnDisconnect() 把 esp_ok 清 0 → 立刻又重连 → 自己踢自己，形成自激循环。 */
static void ESP_ClearUrc(void)
{
    taskENTER_CRITICAL();
    esp_urc_len = 0;
    taskEXIT_CRITICAL();
}

/* 重新武装接收通道（不触碰累积流，因此可在命令中途安全调用）
 * @retval HAL_OK=武装成功；其余=失败（已计入 esp_rearm_fail） */
static HAL_StatusTypeDef ESP_RxArm(void)
{
    HAL_StatusTypeDef st;
    if (huart3.hdmarx == NULL) return HAL_ERROR;
    st = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, esp_dma_buf, ESP_DMA_BUF_SIZE);
    if (st != HAL_OK) esp_rearm_fail++;
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);   /* 只要空闲事件，不要半传输事件 */
    return st;
}

/**
 * @brief  USART 接收错误回调（HAL 弱函数，本工程唯一实现）
 * @param  huart 出错串口句柄
 * @note   为什么必须有这个函数
 *         接收出错（ORE 溢出 / FE 帧错 / NE 噪声）时，HAL 会
 *         ① 清 USART CR3.DMAR（关掉 DMA 收请求）、② 中止 DMA 流、③ 把 RxState 复位为 READY，
 *         然后调用本回调 —— 但不会自动重新武装接收。
 *         本工程原先无人实现该回调（弱函数=空操作）→ 通道就此永久失效，
 *         之后不管模块回什么都收不到。实测现象正是：
 *             re=1 但 dmar=0 en=0 st=0x20，且 dma=0B/0F 永远不动。
 *
 *         本函数运行在中断上下文：只置标志、给信号量，绝不 printf、不调用阻塞式 HAL。
 *         真正的重新武装交给任务上下文（ESP_SendCmd 里检测 esp_rx_dead 后调用 ESP_RxArm），
 *         因为错误路径里的 HAL_DMA_Abort_IT 是异步的，在 ISR 里立刻重新武装可能拿到 BUSY。
 *         仅处理 USART3；其余串口保持原状（原工程无人实现该回调，等价于空操作）。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        esp_err_code = huart->ErrorCode;
        esp_err_cnt++;
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        esp_rx_dead = 1;                 /* 交给任务上下文重新武装 */

        if (xSemEspRx != NULL) {         /* 唤醒正在等待的任务，让它尽快重新武装 */
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(xSemEspRx, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* ======================== 接收通道实现 ======================== */

/**
 * @brief  初始化 ESP 接收通道：创建信号量 + 启动 USART3 的 DMA+空闲中断接收
 * @note   必须在调度器启动前调用一次（main.c 中紧跟 RS485_Init 之后）。
 *         顺序要点：先建信号量再开 DMA，避免空闲事件在信号量尚未创建时触发。
 *         前置依赖：MX_DMA_Init（DMA1 时钟 + NVIC）与 MX_USART3_UART_Init（DMA 句柄已链接）
 */
void ESP_RxInit(void)
{
    HAL_StatusTypeDef arm;

    esp_stream_len = 0;
    esp_stream_ovf = 0;
    esp_stream[0]  = '\0';

    if (xSemEspRx == NULL) {
        xSemEspRx = xSemaphoreCreateBinary();
    }

    if (huart3.hdmarx == NULL) {
        printf("[ESP] RX init FAIL: hdmarx is NULL (DMA not linked)\r\n");
        return;
    }

    arm = ESP_RxArm();
    printf("[ESP] RX init arm=%d RxState=0x%02X\r\n",
           (int)arm, (unsigned)huart3.RxState);
}

/**
 * @brief  USART3 接收事件处理：DMA 缓冲 → 累积流 → 重新武装 DMA → 释放接收信号量
 * @param  huart USART3 句柄
 * @param  Size  本次空闲帧实际收到的字节数
 * @note   在中断上下文执行，因此只做拷贝、重新武装和计数，绝不 printf
 *         （printf 会走 fputc → xSemaphoreTake，在 ISR 里调用是非法的）。
 *         由 rs485.c 的 HAL_UARTEx_RxEventCallback 按 Instance 分流后转发
 *         （HAL 弱回调全工程只能有一处定义）。
 *
 *         必须重新武装：HAL 在空闲事件里已中止 DMA 并把 RxState 复位为 READY，
 *            不重新武装的话每收一帧通道就停一次（实测表现为持久化后只收到第一帧）。
 */
void ESP_RxEventHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t n = (Size <= ESP_DMA_BUF_SIZE) ? Size : ESP_DMA_BUF_SIZE;

    esp_rx_frames++;
    esp_rx_bytes += n;

    /* 追加到累积流：放不下则本次整帧丢弃并置溢出标志
     * （宁可报 ovf=1，也不覆盖已收到的关键内容） */
    if ((uint32_t)esp_stream_len + n <= ESP_STREAM_SIZE - 1U) {
        memcpy(&esp_stream[esp_stream_len], esp_dma_buf, n);
        esp_stream_len = (uint16_t)(esp_stream_len + n);
        esp_stream[esp_stream_len] = '\0';    /* 维持字符串合法：任务侧可直接 strstr */
    } else {
        esp_stream_ovf = 1;
    }

    /* ---- URC 另行累积（不复用 esp_stream，原因见 esp_urc_buf 的说明） ---- */
    ESP_AppendUrc(esp_dma_buf, n);

    (void)ESP_RxArm();            /* 必须重新武装，见函数头注释 */

    if (xSemEspRx != NULL) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(xSemEspRx, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

/* ======================== 内部工具函数 ======================== */

/**
 * @brief  清空接收通道（丢弃上次命令残留的回显）
 * @note   DMA 已在后台持续接收，这里只清累积流与信号量，不做逐字节读取，
 *         因此不会像旧的阻塞轮询那样额外占用 50ms
 */
static void ESP_FlushRX(void)
{
    esp_stream_len = 0;
    esp_stream_ovf = 0;
    esp_stream[0]  = '\0';
    if (xSemEspRx != NULL) {
        (void)xSemaphoreTake(xSemEspRx, 0);   /* 清残留的"收到帧"信号 */
    }
}

/**
 * @brief  取走累积流中已收到的数据（供 OTA 等"数据流"场景）
 * @param  out 目标缓冲
 * @param  max 缓冲容量（字节）
 * @retval 实际取走的字节数；0 = 暂时没有数据
 * @note   与 ESP_SendCmd 的"开头清流"完全不同：前者是丢弃，本函数是消费——
 *         把已收数据搬走、并从流首删除，调用者可在循环里反复调用，实现"边收边处理"。
 *         只应由 OTA 这类独占接收的场景调用；正常 AT 流程里请勿使用，
 *            否则会把命令回显/URC 从流里吃掉，导致 ESP_SendCmd 匹配不到关键字而误超时。
 *         临界区：esp_stream / esp_stream_len 由 DMA 完成回调（中断上下文）写入，
 *            "拷贝 + 前移"必须整体原子，否则会与中断里的追加操作交错、数据错乱。
 *            临界区极短（最多 768 B 的 memmove ≈ 几 µs），远小于一帧的传输时间
 *            （一帧约 12 ms @115200），不会造成丢帧。
 *         溢出标志在取走数据后清零 —— 它是对"本条命令周期"有意义的统计，
 *            数据被消费掉之后就该重置。
 */
uint16_t ESP_StreamTake(char *out, uint16_t max)
{
    uint16_t n = 0U;

    if ((out == NULL) || (max == 0U)) return 0U;

    /* 兼容两种上下文：任务里（调度器已启动）用 FreeRTOS 临界区；
     * 调度器启动前用 __disable_irq（避免在没有 RTOS 时调用 taskENTER_CRITICAL） */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        taskENTER_CRITICAL();
    } else {
        __disable_irq();
    }

    if (esp_stream_len > 0U) {
        n = (esp_stream_len > max) ? max : esp_stream_len;   /* 只取一部分，剩下的下次再取 */

        memcpy(out, esp_stream, n);

        if (n < esp_stream_len) {
            memmove(esp_stream, &esp_stream[n], (size_t)(esp_stream_len - n));
            esp_stream_len = (uint16_t)(esp_stream_len - n);
        } else {
            esp_stream_len = 0U;
        }
        esp_stream[esp_stream_len] = '\0';     /* 维持字符串合法 */
        esp_stream_ovf = 0U;                   /* 数据已被消费，溢出统计同步重置 */
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        taskEXIT_CRITICAL();
    } else {
        __enable_irq();
    }

    return n;
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
 * @note   接收由 DMA + 空闲中断在后台完成：任务在信号量上等待，每到达一个空闲帧
 *         就整帧匹配一次。等待期间任务真正阻塞（不占 CPU、不丢字节）。
 *         关键字在整个累积流上搜索，因此跨帧到达的 "WIFI CONNECTED" + "WIFI GOT IP"
 *         也能一并命中。
 */
int ESP_SendCmd(const char *cmd, const char *expect, const char *reject, uint32_t timeout_ms)
{
    uint32_t t0         = HAL_GetTick();   /* 起始 tick，用于总超时计算 */
    size_t   reject_len = reject ? strlen(reject) : 0;  /* 0=不检测失败特征 */
    int      reject_hit = 0;               /* 是否命中失败特征 */

    if (xSemEspRx == NULL) {
        printf("[ESP-ERR] RX channel not initialized (call ESP_RxInit first)\r\n");
        return -1;
    }

    /* ---- 清场并复位诊断计数 ----
     * 计数按每条命令清零，这样 dma=?B/?F 直接反映"本条命令期间"的接收情况：
     * 若冻结不动，说明期间根本没有字节到达（模块没应答 / 接收通道已停），
     * 而不是被启动阶段的历史数据干扰。 */
    esp_rx_bytes   = 0;
    esp_rx_frames  = 0;
    esp_rearm_fail = 0;
    /* 若上一轮被 RX 错误路径关掉了接收通道，这里在任务上下文重新武装 */
    if (esp_rx_dead) {
        esp_rx_dead = 0;
        (void)ESP_RxArm();
    }
    ESP_FlushRX();                /* 丢弃上次命令残留字节 */

    /* ---- 发送命令（发送超时 200 ms） ---- */
    if (HAL_UART_Transmit(&huart3, (uint8_t *)cmd,
                          strlen(cmd), 200) != HAL_OK)
    {
        printf("[ESP-ERR] Transmit failed: %s\r\n", cmd);
        return -1;
    }

    /* ---- fire-and-forget：不需要回复（如 AT+RST），直接返回成功 ---- */
    if (expect == NULL) return 0;

    /* ---- 每收到一个空闲帧匹配一次，直到总超时 ---- */
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        uint32_t remain = timeout_ms - (HAL_GetTick() - t0);

        /* 在信号量上等待：期间任务阻塞让出 CPU，字节由 DMA 收着，零丢失 */
        if (xSemaphoreTake(xSemEspRx, pdMS_TO_TICKS(remain)) != pdTRUE) {
            break;                /* 总超时且无新帧到达 */
        }

        /* 错误路径可能刚把接收通道关掉（dmar/en 清零、RxState 复位）→ 立刻在任务上下文补武装 */
        if (esp_rx_dead) {
            esp_rx_dead = 0;
            (void)ESP_RxArm();
        }

        /* 3.1 失败检测：reject 任一关键字出现 → 模块已明确报错，立即退出
         * 必须放在 expect 之前判断：
         *    模块的 "ALREADY CONNECTED" 里同时含有 "CONNECT"，若先查 expect，
         *    它会被 expect="CONNECT|OK" 误判为连接成功，reject="ALREADY" 永远轮不到。
         *    后果：esp_ok 被置 1，而随后的 CIPSEND 必然报 "link is not valid" → 死循环重连。
         *    先查 reject 的语义是：响应里出现明确失败特征，就一律算失败。 */
        if (reject_len && ESP_MatchAny(esp_stream, reject) != 0) {
            reject_hit = 1;
            break;
        }

        /* 3.2 成功匹配：expect 任一关键字出现 → 本次命令成功 */
        if (ESP_MatchAny(esp_stream, expect) != 0) {
            return 0;
        }
    }

    /* ---- 超时/失败收尾：打印完整回显与接收统计便于定位 ---- */
    if (reject_hit) {
        printf("[ESP] REJECT cmd=[%s] reject=[%s] rx=[%s]\r\n",
               cmd, reject, esp_stream);
        return -2;                /* 模块明确报错（如 FAIL/ERROR/ALREADY） */
    }
    /* 诊断字段说明（用来区分「模块不说话」和「接收通道已死」）：
     *   dma=?B/?F : 本条命令期间 DMA 实收的字节数 / 空闲帧数（按命令清零）
     *   rf        : 重新武装失败次数（>0 会丢帧）
     *   ovf       : 累积流溢出（1=768B 不够）
     *   re        : USART3 CR1.RE（接收使能）      期望 1
     *   dmar      : USART3 CR3.DMAR（DMA 收请求）  期望 1
     *   en        : DMA1_Stream1 CR.EN（流使能）   期望 1
     *   st        : huart3.RxState（0x22=BUSY_RX 已武装 / 0x20=READY 已停）期望 0x22
     *   → 若 dma=0B/0F 但 re/dmar/en 全为 1、st=0x22，则接收通道是活的，
     *     问题在模块侧（没应答）；若其中任一为 0，则是接收通道已停。 */
    printf("[ESP] TIMEOUT cmd=[%s] expect=[%s] dma=%luB/%luF rf=%lu err=%lu/c%lu ovf=%u re=%u dmar=%u en=%u st=0x%02X rx=[%s]\r\n",
           cmd, expect,
           (unsigned long)esp_rx_bytes, (unsigned long)esp_rx_frames,
           (unsigned long)esp_rearm_fail,
           (unsigned long)esp_err_cnt, (unsigned long)esp_err_code,
           (unsigned)esp_stream_ovf,
           (unsigned)((huart3.Instance->CR1 & USART_CR1_RE)   ? 1U : 0U),
           (unsigned)((huart3.Instance->CR3 & USART_CR3_DMAR) ? 1U : 0U),
           (unsigned)((huart3.hdmarx != NULL &&
                       (huart3.hdmarx->Instance->CR & DMA_SxCR_EN)) ? 1U : 0U),
           (unsigned)huart3.RxState,
           esp_stream);
    return -1;                    /* 超时未匹配 */
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
 *         前置条件：ESP_RxInit() 已调用，否则 AT 回复收不到
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

    /* ---- 1) RST 复位：无论上电处于什么状态，先回到干净环境 ----
     * 必须等它回 OK，不能写成 fire-and-forget
     *    若模块处于忙碌态（正在自动重连 Wi-Fi、或上一条命令还没处理完），
     *    发出去的 AT+RST 会被直接丢弃→ 模块根本不重启 →
     *    它带着上电前遗留的 MQTT 客户端状态继续运行 →
     *    后续 AT+MQTTUSERCFG 必然返回 ERROR（实测现象）。
     *    -> "模块有没有真的重启"是整条链路成立的前提：因为MCU 复位不影响模块，
     *      只有模块自己重启，才能把 MQTT 客户端状态清干净。 */
    if (ESP_SendCmd("AT+RST\r\n", "OK", NULL, 2000) != 0) {
        ESP_FlushRX();
        HAL_Delay(500);
        if (ESP_SendCmd("AT+RST\r\n", "OK", NULL, 2000) != 0) {
            printf("[ESP-WARN] AT+RST not acked x2 (module busy?)\r\n");
        }
    }
    /* 模块重启需 3~5s，之后若存过 AP 还会自动重连并打印 WIFI CONNECTED / WIFI GOT IP，
     * 这段窗口内它不响应 AT，因此必须等够 */
    HAL_Delay(3000);
    ESP_FlushRX();

    /* ---- 2) AT 探测：确认模块响应 ----
     * 失败只等待后重试，绝不重复 AT+RST：
     *    AT+RST 会让模块重启，而重启后它要 3~5s 启动完、还要自动重连 Wi-Fi 并持续打印
     *    WIFI CONNECTED / WIFI GOT IP（此期间不响应 AT）。若每次探测失败都再复位一次，
     *    模块就被永远锁在「启动 → 连上 Wi-Fi → 被复位 → 再启动」的循环里，
     *    表现为"连上 WiFi 后立马退出、一直重复"，而 AT 永远拿不到 OK。 */
    int at_ok = -1;
    for (int i = 0; i < 5; i++) {
        at_ok = ESP_SendCmd("AT\r\n", "OK", NULL, 3000);
        if (at_ok == 0) break;
        HAL_Delay(1000);              /* 等模块把启动/自动重连的 URC 吐完再试 */
    }
    if (at_ok) return ESP_ERR_AT;

    /* ---- 3) 强制普通模式 + 清掉掉电保存的透传连接（防止下次上电又透传） ----
     * 这两条的返回值必须看：若模块仍在透传态（CIPMODE=1），AT 命令会被当数据发走，
     *    后续所有 AT 都会"看似正常、实则无效"，是极难查的一类故障。 */
    if (ESP_SendCmd("AT+CIPMODE=0\r\n", "OK", NULL, 2000) != 0)
        printf("[ESP-WARN] CIPMODE=0 not OK (module in passthrough?)\r\n");
    if (ESP_SendCmd("AT+SAVETRANSLINK=0\r\n", "OK", NULL, 2000) != 0)
        printf("[ESP-WARN] SAVETRANSLINK=0 not OK\r\n");
    ESP_FlushRX();

    /* ---- 4) 网络模式：STA（连接外部路由/热点） + 单连接 ---- */
    if (ESP_SendCmd("AT+CWMODE=1\r\n", "OK", NULL, 2000)) return ESP_ERR_MODE;  /* STA */
    if (ESP_SendCmd("AT+CIPMUX=0\r\n", "OK", NULL, 2000) != 0)
        printf("[ESP-WARN] CIPMUX=0 not OK\r\n");   /* 单连接：CIPSEND 不带连接号 */
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

    /* ---- 6) 建立上行通道：MQTT（默认）或裸 TCP（回退） ----
     * host/port 的含义随 USE_MQTT 变化：MQTT → broker(1883)；TCP → PC 网络助手(8000) */
#if USE_MQTT
    /* MQTT 路径MQTTUSERCFG → MQTTCONN（末参 1 = 固件内置自动重连） */
    if (ESP_MqttConn(pc_ip, pc_port, MQTT_CID_PREFIX) != ESP_OK) return ESP_ERR_TCP;
    printf("[ESP] MQTT connected to %s:%u\r\n", pc_ip, pc_port);

#else
    /* 裸 TCP 回退路径CIPSTART。两条踩过的坑记在这里：
     * ① 建连前必须先 CIPCLOSE：模块重启/断网/服务器先关之后，内部常留一个
     *    失效但未清除的 socket，此时 CIPSTART 回 "ALREADY CONNECTED"（根本不重建），
     *    后续 CIPSEND 必然报 "link is not valid" → "连上了却发不出去"的死循环。
     * ② CIPCLOSE 必须等它自己的响应回来：若写成 fire-and-forget(expect=NULL)，
     *    它稍后返回的 "OK" 会落进下一条 CIPSTART 的等待窗口，被 expect="CONNECT|OK"
     *    捡到 → CIPSTART 被误判成功，而连接其实没建起来。 */
    ESP_SendCmd("AT+CIPCLOSE\r\n", "OK|ERROR", NULL, 1000);
    ESP_FlushRX();

    int cip_ret = -1;
    for (int i = 0; i < 3; i++) {
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", pc_ip, pc_port);
        cip_ret = ESP_SendCmd(cmd, "CONNECT|OK", "ERROR|ALREADY", 5000);
        if (cip_ret == 0) break;
        if (cip_ret == -2) {            /* ALREADY / ERROR：先把残留连接关掉再重试 */
            ESP_FlushRX();
            ESP_SendCmd("AT+CIPCLOSE\r\n", "OK|ERROR", NULL, 1000);
            ESP_FlushRX();
        }
        HAL_Delay(1000);
    }
    if (cip_ret) return ESP_ERR_TCP;

    printf("[ESP] Init OK, connected to %s:%u\r\n", pc_ip, pc_port);
#endif /* USE_MQTT */

    return ESP_OK;
}

/* ======================== 数据上报 ======================== */

/**
 * @brief  通过 TCP 发送一帧数据（CIPSEND 普通模式）
 * @param  data 要发送的数据（长度由 strlen 决定）
 * @retval ESP_OK 成功；ESP_ERR_TX=没拿到 '>'；ESP_ERR_ACK=数据未获确认
 * @note   重试策略：
 *         ① 第一段（等 '>'）失败可安全重试 1 次——此时数据还没发出去；
 *         ② 第二段（等 SEND OK）失败绝不重发——数据可能已到服务器，
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

/* ======================== MQTT 接入 ======================== */

#define ESP_UID_BASE   0x1FFF7A10UL   /* STM32F4 的 96 位唯一 ID 基址（出厂固化，全球唯一） */

/**
 * @brief  接入 MQTT broker（AT+MQTTUSERCFG → AT+MQTTCONN）
 * @param  broker_ip   broker 地址（本工程填 PC 的 IP，经 PC 的 portproxy 转发到 EMQX VM）
 * @param  port        broker 端口（1883）
 * @param  cid_prefix  client_id 前缀；后缀自动用芯片唯一 ID
 * @retval ESP_OK 成功；ESP_ERR_TCP 失败
 * @note   client_id 必须唯一MQTT 规范规定「相同 client_id 的新连接会把旧连接踢下线」。
 *            多台设备共用同一个 id 会互相顶掉，现象是反复上下线、极难排查；
 *            而且本工程 ESP 经 PC 的 portproxy 上云，EMQX 侧看到的源 IP 与 MQTTX 相同
 *            （都是 192.168.137.1），只能靠 client_id 区分设备与调试工具。
 *            这里用 STM32 的 96 位唯一 ID 低 32 位做后缀，天然不冲突。
 *         MQTTCONN 末参 reconnect 用 0（关闭固件自动重连），由应用层做重连：
 *            固件自动重连会在我们主动 DISCONNECT 后立刻连回来，干扰 USERCFG 重配置
 *            （实测会造成"USERCFG 永远 ERROR"的死循环）。应用层已有完整重连链路，
 *            见 App_MqttOnDisconnect() → esp_ok=0 → 下一轮 ESP_Init。
 *         本函数是幂等的：开头先探一次订阅，已连接就直接复用、不重配置。
 */
int ESP_MqttConn(const char *broker_ip, uint16_t port, const char *cid_prefix)
{
    char cmd[128];
    char cid[64];
    uint32_t uid = *(volatile uint32_t *)ESP_UID_BASE;

    snprintf(cid, sizeof(cid), "%s%08lX", cid_prefix, (unsigned long)uid);

    /* ---- 0a) 快路径预检：若已经连着，直接复用，绝不重配置 ----
     * 为什么必须有这一步（实测遇到的死循环）：
     *      AT+MQTTUSERCFG 在「已连接」状态下必然返回 ERROR。
     *      而 MQTTCONN 若配了 reconnect=1，我们发 AT+MQTTDISCONNECT 后模块会
     *      立刻自己连回来，于是 USERCFG 永远失败 → 代码以为没连上 → 又重连
     *      → 又先断开 → 自己踢自己，形成死循环。
     *      实测现象：USERCFG 反复报 ERROR，且 rx 里夹杂
     *      +MQTTCONNECTED:0,1,"<ip>","<port>","",1（模块明说它已经连上了）。
     *      -> 先用「订阅」探一下：订阅成功即证明连接可用，直接返回、不碰配置。 */
    if (ESP_MqttSub(MQTT_TOPIC_SUB) == ESP_OK) {
        ESP_ClearUrc();
        printf("[ESP] MQTT already connected, reuse (cid=%s)\r\n", cid);
        return ESP_OK;
    }

    /* ---- 0b) 断开残留 link，并等它真正拆除 ----
     * 用 expect="OK|ERROR" 等它回完：无连接时 DISCONNECT 本来就回 ERROR，
     *    那是正常结果；若写成 fire-and-forget(expect=NULL)，它稍后返回的 "OK"
     *    会落进下一条 USERCFG 的匹配窗口 → 把配置失败误判成成功。
     * 断开是异步的：回 OK 只代表"已受理"，链路真正拆掉还要几十 ms。
     *    不等就立刻 USERCFG/CONN，会拿到 ERROR。
     * AT+MQTTCLEAN=0 在 1471 固件上无响应（只回显、不回 OK/ERROR），故不调用。 */
    ESP_SendCmd("AT+MQTTDISCONNECT=0\r\n", "OK|ERROR", NULL, 1000);
    HAL_Delay(500);               /* 等链路真正拆掉，再重新配置 */
    ESP_FlushRX();

    /* ---- 1) 配置 MQTT 客户端：scheme=1 即 MQTT over TCP；用户名/密码留空 = 匿名 ----
     * 失败时重试一次：模块有时会残留更深的 link 状态，单次 DISCONNECT 清不掉。
     *    实测触发场景：做完"停 broker / 重启 broker"的断线重连测试后，
     *    模块会进入这种状态，此时 USERCFG 连续报 ERROR。
     * 若两次都失败 → 说明模块状态已无法用 AT 指令恢复，必须给模块彻底断电
     *    （拔 ESP 的 VCC 等 5 秒再上电）。仅复位 MCU 是不够的 ——
     *    因为 MCU 复位不影响模块，模块的 MQTT 客户端状态会一直保留。 */
    snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"\r\n", cid);
    if (ESP_SendCmd(cmd, "OK", "ERROR", 3000)) {
        /* 重试前：再断一次并多等 1s，尽量把残留 link 清干净 */
        ESP_SendCmd("AT+MQTTDISCONNECT=0\r\n", "OK|ERROR", NULL, 1000);
        HAL_Delay(1000);
        ESP_FlushRX();

        if (ESP_SendCmd(cmd, "OK", "ERROR", 3000)) {
            printf("[ESP-ERR] MQTTUSERCFG failed x2 (cid=%s)\r\n", cid);
            /* 打印串必须用英文：ARMCC(AC5) 按 GBK 解析源文件，
             *    字符串字面量里出现中文会触发 #870-D invalid multibyte character sequence */
            printf("[ESP-ERR] ===> POWER-CYCLE the ESP module (unplug VCC for 5s). "
                   "Resetting only the MCU is NOT enough.\r\n");
            return ESP_ERR_TCP;
        }
    }
    printf("[ESP] MQTT client_id=%s\r\n", cid);

    /* ---- 2) 连接 broker ----
     * 末参 reconnect 用 0（关闭固件内置自动重连）：
     *    若开成 1，模块会在我们主动 DISCONNECT 后立刻自己连回来，
     *    干扰紧接着的 USERCFG 重配置（实测导致 USERCFG 永远报 ERROR 的死循环）。
     *    本工程已有完整的应用层重连（收到 +MQTTDISCONNECT → esp_ok=0 → 下一轮 ESP_Init 重连），
     *    固件自动重连是冗余的，且会制造上述干扰。 */
    for (int i = 0; i < 3; i++) {
        snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%u,0\r\n", broker_ip, port);
        if (ESP_SendCmd(cmd, "CONNECT|OK", "ERROR", 8000) == 0) {
            /* 连上后订阅下行主题；订阅失败不算致命—— 上行仍可用，只是失去下行能力 */
            if (ESP_MqttSub(MQTT_TOPIC_SUB) != ESP_OK) {
                printf("[ESP-WARN] MQTTSUB failed, downlink disabled\r\n");
            } else {
                printf("[ESP] MQTT subscribed: %s\r\n", MQTT_TOPIC_SUB);
            }
            /* 丢掉本次建连过程中建连过程中自身产生的 URC（主要是连接前 DISCONNECT 的
             *    +MQTTDISCONNECT）。不清掉的话它会被 ESP_PollURC 当成"链路掉了"，
             *    把 esp_ok 置 0 → 下一轮又重连 → 自己踢自己，表现为"连上就断"的死循环。 */
            ESP_ClearUrc();
            return ESP_OK;
        }
        ESP_FlushRX();
        HAL_Delay(1000);
    }
    printf("[ESP-ERR] MQTTCONN failed %s:%u\r\n", broker_ip, port);
    return ESP_ERR_TCP;
}

/**
 * @brief  通过 MQTT 发布一条消息（AT+MQTTPUBRAW 两段式）
 * @param  topic   主题
 * @param  payload 载荷字节流（JSON）
 * @retval ESP_OK 成功；ESP_ERR_TX 没等到 '>'；ESP_ERR_ACK 未获确认
 * @note   为什么用 MQTTPUBRAW 而不是 MQTTPUB：
 *            MQTTPUB 的载荷是 AT 的字符串参数，JSON 里的双引号会破坏参数格式，
 *            需要转义成 \"，而 1471 固件不保证支持转义；
 *            MQTTPUBRAW 显式给出长度、发送原始字节，完全不碰转义问题。
 *            （本工程已实测 MQTTPUBRAW 可用，返回 > 后再回 +MQTTPUB:OK。）
 *         两个判据必须写对，否则会一直超时（而数据其实已发出）：
 *              第 1 段 expect = ">"          （第 1 段也可能先回一条 OK，那是残留，
 *                                             只有 '>' 才表示"在等原始数据"）
 *              第 2 段 expect = "+MQTTPUB:OK"（不是 "OK"，也不是 CIPSEND 的 "SEND OK"）
 *         载荷绝不加 \r\n：长度用 strlen 算出并与命令里一致，多一字节即非法 JSON。
 *         第 2 段失败绝不重发：数据可能已到 broker，重发会造成重复消息。
 */
int ESP_MqttPub(const char *topic, const char *payload)
{
    char   cmd[64];
    size_t len = strlen(payload);

    snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,0\r\n", topic, (unsigned int)len);

    /* ---- 第 1 段：等 '>'（拿不到说明链路断/模块忙，重试 1 次） ---- */
    if (ESP_SendCmd(cmd, ">", NULL, 3000) != 0) {
        ESP_FlushRX();
        HAL_Delay(200);
        if (ESP_SendCmd(cmd, ">", NULL, 3000) != 0) {
            printf("[ESP-ERR] MQTTPUBRAW did not get '>'\r\n");
            return ESP_ERR_TX;
        }
    }

    /* ---- 第 2 段：发载荷，等 "+MQTTPUB:OK" ---- */
    if (ESP_SendCmd(payload, "+MQTTPUB:OK", NULL, 3000)) {
        printf("[ESP-ERR] MQTTPUB not acked (topic=%s)\r\n", topic);
        return ESP_ERR_ACK;
    }
    return ESP_OK;
}

/**
 * @brief  订阅下行主题（服务端 → 设备）
 * @retval ESP_OK=成功；-1=失败
 * @note   订阅失败不算致命：上行仍可正常工作，只是收不到下行指令，
 *         所以调用处只打警告、不中断流程。
 */
int ESP_MqttSub(const char *topic)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",0\r\n", topic);
    return (ESP_SendCmd(cmd, "OK", "ERROR", 3000) == 0) ? ESP_OK : -1;
}

/**
 * @brief  非阻塞轮询并处理 MQTT 下行 URC
 * @note   任务上下文调用（vTaskNet 每轮）。设计要点：
 *         ① 先取快照再解析，避免边解析边被 ISR 改动；
 *         ② 只处理已收全的行（找到 '\n'），没收全的留在缓冲里等下一轮，不会丢；
 *         ③ 解析是针对固定格式的轻量解析，不是通用 JSON 解析器 ——
 *            载荷由服务端按约定下发、格式可控，这样写够用且省代码/省栈。
 * 已知 URC 格式（实测）：
 *   +MQTTSUBRECV:<LinkID>,"<topic>",<len>,<payload>
 *   +MQTTDISCONNECT
 */
void ESP_PollURC(void)
{
    char     line[ESP_URCBUF_SIZE + 1];
    uint16_t n;
    char    *p, *nl;

    /* ---- ① 取快照（临界区：ISR 可能并发追加） ---- */
    taskENTER_CRITICAL();
    n = esp_urc_len;
    if (n > ESP_URCBUF_SIZE) n = ESP_URCBUF_SIZE;
    if (n > 0U) memcpy(line, esp_urc_buf, n);
    taskEXIT_CRITICAL();
    line[n] = '\0';
    if (n == 0U) return;

    /* ---- ② 断线 URC ---- */
    p = strstr(line, "+MQTTDISCONNECT");
    if (p != NULL) {
        nl = strchr(p, '\n');
        printf("[ESP] URC +MQTTDISCONNECT\r\n");
        App_MqttOnDisconnect();                       /* 应用层置 esp_ok=0 → 复用现有重连流程 */
        ESP_UrcDrop((uint16_t)((nl != NULL) ? (size_t)(nl - line) + 1U : strlen(line)));
        return;
    }

    /* ---- ③ 下行数据 URC ---- */
    p = strstr(line, "+MQTTSUBRECV:");
    if (p == NULL) return;

    nl = strchr(p, '\n');
    if (nl == NULL) return;                           /* 行还没收全，等下一轮 */

    {
        char *q1 = strchr(p, '"');                                   /* topic 起始引号 */
        char *q2 = (q1 != NULL) ? strchr(q1 + 1, '"') : NULL;         /* topic 结束引号 */
        char *c1 = (q2 != NULL) ? strchr(q2 + 1, ',') : NULL;         /* <len> 前的逗号 */
        char *c2 = (c1 != NULL) ? strchr(c1 + 1, ',') : NULL;         /* <payload> 前的逗号 */

        if (c2 != NULL) {
            const char *pl = c2 + 1;
            size_t plen = (size_t)(nl - pl);
            while (plen > 0U && (pl[plen - 1U] == '\r' || pl[plen - 1U] == '\n')) plen--;
            if (plen > 0U) App_MqttOnCommand(pl, plen);   /* 交给应用层执行 */
        }
    }

    ESP_UrcDrop((uint16_t)((size_t)(nl - line) + 1U));   /* 消费掉这一整行 */
}
