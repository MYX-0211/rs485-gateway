#ifndef ESP01S_H
#define ESP01S_H
#include <stdint.h>
#include <stddef.h>          /* size_t：App_MqttOnCommand 的形参类型 */
#include "stm32f4xx_hal.h"   /* UART_HandleTypeDef：ESP_RxEventHandler 形参类型 */

/* ============================================================================
 * esp01s.h — ESP-01S (ESP8266) Wi-Fi 驱动接口（USART3，115200 8N1）
 * 错误码约定：所有函数 0=成功，非0=失败（详见 esp01s.c 的 ESP_Ret 枚举）
 * 使用流程：
 *   0. ESP_RxInit()                  启动 DMA + 空闲中断接收通道（必须在配网前）
 *   1. ESP_Init(ssid, pwd, ip, port) 完成 配网 + 连 TCP
 *   2. 成功后循环调用 ESP_SendTCP(data) 上报数据
 * ==========================================================================*/

/* ======================== 上报通道配置 ========================
 * USE_MQTT = 1 : 走 MQTT（EMQX）上报，ESP_Init 的 broker 接入阶段改为接入 broker
 * USE_MQTT = 0 : 走裸 TCP（CIPSEND）上报 —— 回退路径，用于验证换固件后原链路是否仍可用
 * 注意：两种模式下 ESP_Init(ssid, pwd, host, port) 的 host/port 含义随宏变化：
 *       MQTT → broker 地址/端口(1883)；TCP → PC 网络助手地址/端口(8000)
 * ============================================================== */
#define USE_MQTT          1                  /* 1=MQTT(EMQX)  0=裸 TCP(回退) */
#define MQTT_TOPIC_PUB    "rs485gw/data"     /* 上行主题：设备 → 服务端 */
#define MQTT_TOPIC_SUB    "rs485gw/cmd"      /* 下行主题：服务端 → 设备 */
#define MQTT_CID_PREFIX   "rs485gw_"         /* client_id 前缀，后缀自动用芯片唯一 ID */

/**
 * @brief  初始化 ESP 接收通道：创建接收信号量 + 启动 USART3 的 DMA+空闲中断接收
 * @note   必须在调度器启动前调用一次（main.c 中紧跟 RS485_Init 之后）。
 *         未调用时 ESP_SendCmd 收不到任何回复，会直接返回 -1 并打印提示。
 *         前置依赖：MX_DMA_Init（DMA1 时钟 + NVIC）与 MX_USART3_UART_Init（DMA 句柄已链接）
 */
void ESP_RxInit(void);

/**
 * @brief  USART3 接收事件处理：DMA 缓冲 → 累积流 → 重新武装 → 释放接收信号量
 * @param  huart USART3 句柄
 * @param  Size  本次空闲帧实际收到的字节数
 * @note   在中断上下文执行，只做拷贝/重新武装/计数，不打印。
 *         由 rs485.c 的 HAL_UARTEx_RxEventCallback 分流调用
 *         （HAL 弱回调全工程只能有一处定义，故统一在 rs485.c 定义后转发）
 */
void ESP_RxEventHandler(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  接入 MQTT broker（AT+MQTTUSERCFG + AT+MQTTCONN）
 * @param  broker_ip   broker 地址（本工程为 PC 的 IP，经 PC 的 portproxy 转发到 EMQX）
 * @param  port        broker 端口（1883）
 * @param  cid_prefix  client_id 前缀；后缀自动用 STM32 唯一 ID（保证多设备不冲突）
 * @retval 0=成功；非0=失败
 */
int ESP_MqttConn(const char *broker_ip, uint16_t port, const char *cid_prefix);

/**
 * @brief  通过 MQTT 发布一条消息（AT+MQTTPUBRAW，载荷走原始字节，支持 JSON）
 * @param  topic   主题
 * @param  payload 载荷字节流（★ 不加 \r\n，长度必须与内部计算一致）
 * @retval 0=成功；非0=失败
 */
int ESP_MqttPub(const char *topic, const char *payload);

/**
 * @brief  订阅下行主题（服务端 → 设备）
 * @retval 0=成功；非0=失败（失败不致命，仅失去下行能力）
 */
int ESP_MqttSub(const char *topic);

/**
 * @brief  非阻塞轮询并处理 MQTT 下行 URC
 * @note   由 vTaskNet 每轮调用（任务上下文，不是中断）。
 *         处理两类 URC：
 *           +MQTTSUBRECV:<LinkID>,"<topic>",<len>,<payload>  → 交 App_MqttOnCommand()
 *           +MQTTDISCONNECT                                  → 交 App_MqttOnDisconnect()
 *         未收全的 URC 行会留在独立缓冲里等下一轮，不会丢。
 *         URC 之所以要独立缓冲：ESP_SendCmd 开头会清累积流，
 *         若与命令响应共用缓冲，空闲期间到达的 URC 会被下一条命令清掉。
 */
void ESP_PollURC(void);

/* ---- 以下两个回调由应用层 main.c实现；esp01s.c 只把 URC 内容交出去 ---- */

/**
 * @brief  下行命令回调（应用层实现）
 * @param  payload 载荷（JSON 字节流，不是 '\0' 结尾，需按 len 使用）
 * @param  len     载荷长度
 */
void App_MqttOnCommand(const char *payload, size_t len);

/**
 * @brief  MQTT 断线回调（应用层实现）：收到 +MQTTDISCONNECT 时调用
 */
void App_MqttOnDisconnect(void);

/**
 * @brief  发送 AT 命令并等待期望回复
 * @param  cmd        AT 命令字符串（需包含 \r\n）
 * @param  expect     期望关键字，支持 '|' 多关键字；NULL=只发不收
 * @param  reject     失败特征关键字，支持 '|'；NULL=不检测
 * @param  timeout_ms 总超时(ms)
 * @retval 0=成功, -1=超时未匹配, -2=命中 reject 失败特征
 */
int ESP_SendCmd(const char *cmd, const char *expect, const char *reject, uint32_t timeout_ms);

/**
 * @brief  初始化 ESP-01S：退出透传→复位→探测→STA 配网→连 TCP
 * @param  ssid    Wi-Fi 热点名（2.4G）
 * @param  pwd     Wi-Fi 密码
 * @param  pc_ip   服务器 IP
 * @param  pc_port 服务器端口（PC 网络助手需 TCPServer 模式）
 * @retval 0=成功；非0=失败（ESP_ERR_AT/MODE/WIFI/TCP）
 */
int  ESP_Init(const char *ssid, const char *pwd, const char *pc_ip, uint16_t pc_port);

/**
 * @brief  通过 TCP 发送一帧数据（CIPSEND 普通模式）
 * @param  data 待发送数据
 * @retval 0=成功；ESP_ERR_TX=未收到 '>'；ESP_ERR_ACK=数据未获确认（勿重发）
 */
int  ESP_SendTCP(const char *data);

/**
 * @brief  取走累积流中已收到的数据（供 OTA 等"数据流"场景使用）
 * @param  out 目标缓冲
 * @param  max 缓冲容量（字节）
 * @retval 实际取走的字节数；0 = 暂时没有数据
 * @note   与 ESP_SendCmd 的"开头清流"完全不同：前者是丢弃，本函数是消费——
 *         把已收数据搬走并从流首删除，可在循环里反复调用实现"边收边处理"。
 *         只应由 OTA 这类独占接收的场景调用；正常 AT 流程里请勿使用，
 *            否则会吃掉命令回显/URC，导致 ESP_SendCmd 匹配不到关键字而误超时。
 * @see    esp01s.c 里的实现说明（临界区与溢出标志的处理）
 */
uint16_t ESP_StreamTake(char *out, uint16_t max);

#endif

