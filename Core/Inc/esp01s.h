#ifndef ESP01S_H
#define ESP01S_H
#include <stdint.h>

/* ============================================================================
 * esp01s.h — ESP-01S (ESP8266) Wi-Fi 驱动接口（USART3，115200 8N1）
 * 错误码约定：所有函数 0=成功，非0=失败（详见 esp01s.c 的 ESP_Ret 枚举）
 * 使用流程：
 *   1. ESP_Init(ssid, pwd, ip, port) 完成 配网 + 连 TCP
 *   2. 成功后循环调用 ESP_SendTCP(data) 上报数据
 * ==========================================================================*/

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

#endif

