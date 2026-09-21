#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_printf_ascii.py — 检查源码里 printf/snprintf 的【字符串字面量】是否含非 ASCII 字符

为什么需要它
------------
ARMCC (AC5) 按 **GBK** 解析 C 源文件，而我们的源文件保存为 **UTF-8**。
一旦 printf 的字符串字面量里出现中文，编译就会报：

    warning: #870-D: invalid multibyte character sequence

而且更糟的是**运行时输出的中文会是乱码**（实测：`PC 上 ...` 打印成 `PC 涓? ...`）。
⇒ 纪律：**C 源文件里的字符串字面量一律用 ASCII/英文；中文只能出现在注释里。**

用法
----
    python tools/check_printf_ascii.py                # 扫描默认文件列表
    python tools/check_printf_ascii.py Core/Src/*.c   # 扫描指定文件

退出码：0 = 全部干净；1 = 发现含中文的字面量（便于接进自动化流程）
"""

import glob
import os
import re
import sys

DEFAULT_FILES = [
    'Core/Src/main.c',
    'Core/Src/param.c',
    'Core/Src/w25q64.c',
    'Core/Src/esp01s.c',
    'Core/Src/modbus.c',
    'Core/Src/rs485.c',
    'Core/Src/oled.c',
    'Core/Src/stm32f4xx_it.c',
    'Bootloader/boot_main.c',
]

# 匹配 printf / snprintf / sprintf / fputs / HAL_UART_Transmit(...字符串...)
CALL_RE = re.compile(r'\b(printf|snprintf|sprintf|puts|fputs)\s*\(')
# 提取字符串字面量（正确处理 \" 转义）
STR_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')


def scan(path: str):
    """返回 [(行号, 行内容摘要), ...]"""
    hits = []
    with open(path, encoding='utf-8') as f:
        lines = f.read().split('\n')

    for idx, line in enumerate(lines):
        if not CALL_RE.search(line):
            continue
        # 可能跨行（printf 的参数写在下一行），把下一行也拼进来一起查
        blob = line + ' ' + (lines[idx + 1] if idx + 1 < len(lines) else '')
        for s in STR_RE.findall(blob):
            if any(ord(c) > 127 for c in s):
                hits.append((idx + 1, line.strip()[:78]))
                break
    return hits


def main() -> int:
    args = sys.argv[1:]
    if args:
        files = []
        for a in args:
            files.extend(sorted(glob.glob(a)) if any(ch in a for ch in '*?') else [a])
    else:
        files = DEFAULT_FILES

    total = 0
    checked = 0
    for f in files:
        if not os.path.exists(f):
            continue
        checked += 1
        hits = scan(f)
        if hits:
            total += len(hits)
            print('  [!!] %-28s %d 处含非 ASCII 字面量' % (f, len(hits)))
            for ln, txt in hits:
                print('       第 %-4d 行: %s' % (ln, txt))
        else:
            print('  [OK] %-28s 全 ASCII' % f)

    print()
    if total:
        print('  ⚠️ 共 %d 处 —— ARMCC 会报 #870-D，且运行时中文会乱码。' % total)
        print('     修法：把 printf 里的中文改成英文（中文只保留在注释里）。')
    else:
        print('  全部 %d 个文件通过 ✓' % checked)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
