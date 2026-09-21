#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ota_sender.py — OTA 固件下发器（PC 侧，TCP Server）

配合 STM32 侧的 OTA 流程使用：
    MQTTX 向 rs485gw/cmd 发 {"cmd":"ota_recv"}
        → 设备断开 MQTT、用 AT+CIPSTART 连到本脚本
        → 本脚本把 .bin 逐行下发
        → 设备写入 W25Q64 暂存区并置标志
        → 复位，Bootloader 搬运（这一步已在阶段 3 验证通过）

数据格式（刻意全用 ASCII，避开二进制解析的坑）
------------------------------------------------
每行：<hex 字符串>\r\n        （128 个 hex 字符 = 64 字节原文）
结束：END,<总字节数>,<CRC16 四位数>\r\n

为什么用 hex 编码而不是直接发二进制：
  ESP-AT 收数据是 `+IPD,<len>:<data>` 形式，二进制 data 里可能含 "\r\n"、
  甚至含 "+IPD" 字样，会让基于文本的解析彻底错乱。转成 hex 后数据只含
  [0-9A-F]，解析端只需跳过 `+IPD,<len>:` 前缀即可，稳妥得多。
  代价是传输量翻倍（49152 B → 98304 字符 ≈ 9 秒 @115200），可接受。

为什么每行只用 64 字节：
  STM32 侧复用 esp01s.c 的 768 字节累积流缓冲。一行 128 字符 + `+IPD,130:`
  前缀远小于 768，不会溢出；也不必为此改动驱动层的缓冲大小。

⚠️ 补齐到 16 KB 对齐：
  设备端 OTA 的 fw_size 是「RO 段末尾向上取整到 16 KB」（见 main.c 的说明），
  所以这里也把 bin 补 0xFF 到同样的边界，两边 size/CRC 才能对上。

用法
----
    python tools/ota_sender.py Doc/app_v1.bin            # 默认端口 9000
    python tools/ota_sender.py Doc/app_v1.bin 9000
"""

import os
import socket
import sys
import time


def crc16(data: bytes) -> int:
    """CRC16-Modbus（与固件侧 ota.h 的 Ota_CRC16 同一算法：多项式 0xA001、初值 0xFFFF）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def main() -> int:
    bin_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join('Doc', 'app_v1.bin')
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9000

    if not os.path.exists(bin_path):
        print('[OTA] 找不到固件：%s' % bin_path)
        print('      先用 fromelf 生成：')
        print('      fromelf --bin --output=Doc/app_v1.bin '
              'MDK-ARM/rs485_gateway/rs485_gateway.axf')
        return 1

    data = bytearray(open(bin_path, 'rb').read())
    raw_len = len(data)

    # 补齐到 16 KB 对齐（与设备端 fw_size 的取整粒度一致，补 0xFF = Flash 擦除态）
    pad = (-len(data)) % 16384
    data += b'\xFF' * pad
    size = len(data)
    crc = crc16(bytes(data))

    print('[OTA] 固件 %s' % bin_path)
    print('      原始 %d B，补齐 %d B（0xFF）→ 共 %d B (%.1f KB)' % (
        raw_len, pad, size, size / 1024.0))
    print('      CRC16 = 0x%04X' % crc)
    print('      ⚠️ 设备端日志应显示 size=%d crc=0x%04X（两者必须一致）' % (size, crc))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', port))
    srv.listen(1)
    print('[OTA] 监听 0.0.0.0:%d，等待设备连接……' % port)
    print('      （此时在 MQTTX 发 {"cmd":"ota_recv"}）')

    srv.settimeout(120)          # 等设备最多 2 分钟
    try:
        conn, addr = srv.accept()
    except socket.timeout:
        print('[OTA] 等待超时，未收到设备连接')
        srv.close()
        return 2

    print('[OTA] 设备已连接：%s:%d' % (addr[0], addr[1]))

    CHUNK = 64                   # 每行 64 字节原文 → 128 个 hex 字符
    sent = 0
    lines = 0
    t0 = time.time()

    try:
        for off in range(0, size, CHUNK):
            line = data[off:off + CHUNK].hex().upper() + '\r\n'
            conn.sendall(line.encode('ascii'))
            sent += min(CHUNK, size - off)
            lines += 1
            if lines % 32 == 0:
                pct = sent * 100 // size
                print('      已发 %6d/%d B (%3d%%)' % (sent, size, pct))

        # 结束帧：设备据此校验 CRC 并写头部
        conn.sendall(('END,%d,%04X\r\n' % (size, crc)).encode('ascii'))
        dt = time.time() - t0
        print('[OTA] 发送完成：%d 行 / %d B，用时 %.2f s (%.1f KB/s)' % (
            lines, sent, dt, sent / 1024.0 / dt if dt > 0 else 0))
        print('[OTA] 已发结束帧 END,%d,%04X —— 等设备写入并置标志' % (size, crc))
        time.sleep(3)            # 给设备时间把剩下的数据写进 W25Q64
    except (BrokenPipeError, ConnectionResetError) as e:
        print('[OTA] 连接被设备断开：%s' % e)
        print('      （若设备在收到 END 后主动断开，这是正常的）')
    finally:
        conn.close()
        srv.close()

    print('[OTA] 完成。请复位设备，Bootloader 会搬运并跳转。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
