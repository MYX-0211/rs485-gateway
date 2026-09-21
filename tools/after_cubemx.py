#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
after_cubemx.py — CubeMX 重新生成代码后的【自动修复 + 审计】

为什么需要它
------------
本工程的 FreeRTOS 是【手动移植】的（没有启用 CubeMX 的 FreeRTOS 中间件），
所以 CubeMX 不知道它的存在，每次重新生成都会往 stm32f4xx_it.c 里塞
SVC_Handler / PendSV_Handler / SysTick_Handler 的默认实现；
而 FreeRTOS 的 port.c 已经通过 FreeRTOSConfig.h 的名字映射提供了这三个，
于是链接期报：L6200E: Symbol xxx multiply defined (by port.o and stm32f4xx_it.o)

本脚本做三件事：
  ① 【修复】删掉 it.c / it.h 里 CubeMX 生成的这三个 handler 与声明
  ② 【审计】检查 USART3 DMA 接收链路的 6 处关键配置是否齐全
  ③ 【审计】检查 .ioc 里 USART3 DMA / NVIC 配置是否还在
           —— 这是【根本】：.ioc 丢了的话，上面 6 处下次生成还会再丢

用法
----
    python tools/after_cubemx.py            # 修复 + 审计（推荐每次生成后跑一次）
    python tools/after_cubemx.py --build    # 修复 + 审计后，再调用 Keil 编译
"""

import os
import re
import sys
import shutil
import subprocess

# ---------------------------------------------------------------- 路径
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IT_C = os.path.join(ROOT, 'Core', 'Src', 'stm32f4xx_it.c')
IT_H = os.path.join(ROOT, 'Core', 'Inc', 'stm32f4xx_it.h')
IOC = os.path.join(ROOT, 'rs485_gateway.ioc')
UVPROJX = os.path.join(ROOT, 'MDK-ARM', 'rs485_gateway.uvprojx')
UV4 = r'U:\Keil_v5\UV4\UV4.exe'

# 要删掉的三个 handler（实现由 FreeRTOS 的 port.c 提供）
HANDLERS = [b'SVC_Handler', b'PendSV_Handler', b'SysTick_Handler']

# 需要审计的 6 处关键配置：(相对路径, 关键字, 说明)
CHECKS = [
    ('Core/Src/usart.c',        b'DMA_HandleTypeDef hdma_usart3_rx;',     'USART3 DMA 句柄定义'),
    ('Core/Src/usart.c',        b'USART3 DMA Init',                       'USART3 DMA 初始化块'),
    ('Core/Src/usart.c',        b'__HAL_LINKDMA(uartHandle,hdmarx',       '__HAL_LINKDMA 关联'),
    ('Core/Src/dma.c',          b'__HAL_RCC_DMA1_CLK_ENABLE',             'DMA1 时钟使能'),
    ('Core/Src/dma.c',          b'HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn)', 'DMA1_Stream1 NVIC 使能'),
    ('Core/Src/stm32f4xx_it.c', b'void DMA1_Stream1_IRQHandler',          'DMA1_Stream1 中断入口'),
    ('Core/Inc/stm32f4xx_it.h', b'void DMA1_Stream1_IRQHandler',          '  └ 其函数声明'),
]

# .ioc 里的根本配置
IOC_CHECKS = [
    (b'Dma.USART3_RX',          'USART3 DMA 请求配置'),
    (b'DMA1_Stream1',           'DMA1 Stream1 通道映射'),
    (b'NVIC.DMA1_Stream1_IRQn', 'DMA1_Stream1 NVIC 使能'),
]


def ok(msg):
    print('  [OK]   ' + msg)


def warn(msg):
    print('  [!!]   ' + msg)


def info(msg):
    print('         ' + msg)


def backup(path):
    """首次修改前备份一次（保留 .bak_cubemx 后缀，不覆盖已有备份）"""
    bak = path + '.bak_cubemx'
    if not os.path.exists(bak) and os.path.exists(path):
        shutil.copy2(path, bak)


def strip_in_c(text):
    """从 .c 里删掉三个 handler 的函数定义（保留其上方注释，无害）"""
    removed = []
    for name in HANDLERS:
        pat = re.compile(rb'void ' + name + rb'\(void\)\s*\{.*?\r?\n\}\r?\n',
                         re.DOTALL)
        new, n = pat.subn(b'', text)
        if n:
            removed.append(name.decode())
            text = new
    return text, removed


def strip_in_h(text):
    """从 .h 里删掉三个 handler 的声明"""
    removed = []
    for name in HANDLERS:
        pat = re.compile(rb'^void ' + name + rb'\(void\);\r?\n', re.MULTILINE)
        new, n = pat.subn(b'', text)
        if n:
            removed.append(name.decode())
            text = new
    return text, removed


def main():
    do_build = '--build' in sys.argv
    problems = 0

    print('=' * 68)
    print(' CubeMX 生成后修复与审计')
    print('=' * 68)

    # ---------------- ① 删除三个重复 handler ----------------
    print('\n① 清理 CubeMX 重复生成的三个 FreeRTOS handler')
    for path, fn in ((IT_C, strip_in_c), (IT_H, strip_in_h)):
        if not os.path.exists(path):
            warn('文件不存在：%s' % path)
            continue
        raw = open(path, 'rb').read()
        new, removed = fn(raw)
        if removed:
            backup(path)
            # 统一为 CRLF（防止写回时变成 LF）
            new = new.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            open(path, 'wb').write(new)
            ok('%-22s 已删除：%s' % (os.path.basename(path), ', '.join(removed)))
        else:
            ok('%-22s 无重复 handler（已清理过）' % os.path.basename(path))

    # ---------------- ② 审计 6 处关键配置 ----------------
    print('\n② 审计 USART3 DMA 接收链路（6 处关键配置）')
    for rel, key, desc in CHECKS:
        path = os.path.join(ROOT, rel.replace('/', os.sep))
        if not os.path.exists(path):
            warn('%-46s 文件缺失！' % desc)
            problems += 1
            continue
        if key in open(path, 'rb').read():
            ok('%-46s' % desc)
        else:
            warn('%-46s 缺失！' % desc)
            problems += 1

    # ---------------- ③ 审计 .ioc（根本） ----------------
    print('\n③ 审计 .ioc（根本：丢了的话下次生成 6 处还会再丢）')
    if not os.path.exists(IOC):
        warn('.ioc 文件不存在！')
        problems += 1
    else:
        raw = open(IOC, 'rb').read()
        for key, desc in IOC_CHECKS:
            if key in raw:
                ok('%-46s' % desc)
            else:
                warn('%-46s 缺失！' % desc)
                problems += 1
        if b'Dma.USART3_RX' not in raw:
            info('')
            info('→ 请在 CubeMX 里补：USART3 → DMA Settings → Add → USART3_RX')
            info('  Instance=DMA1 Stream1 / Channel 4，Mode=Normal，Priority=Medium')
            info('  Data Width: Periph=Byte / Mem=Byte；MemInc=Enable')
            info('  再到 NVIC Settings 勾 DMA1 Stream1 global interrupt（优先级 5）')

    # ---------------- 小结 ----------------
    print('\n' + '=' * 68)
    if problems == 0:
        print(' 全部检查通过 ✓')
    else:
        print(' 发现 %d 处问题 —— 见上面的 [!!] 行' % problems)
    print('=' * 68)

    # ---------------- 可选：编译 ----------------
    if do_build:
        print('\n调用 Keil 编译 ……')
        if not os.path.exists(UV4):
            warn('未找到 UV4.exe：%s' % UV4)
        else:
            log = os.path.join(ROOT, 'MDK-ARM', 'after_cubemx_build.log')
            r = subprocess.run([UV4, '-b', UVPROJX, '-j0', '-o', log],
                               capture_output=True)
            if os.path.exists(log):
                for line in open(log, 'rb').read().decode('utf-8', 'replace').splitlines():
                    if 'Error' in line or 'Warning' in line or 'Program Size' in line:
                        print('  ' + line.strip())
            print('  退出码 = %d' % r.returncode)

    return problems


if __name__ == '__main__':
    sys.exit(0 if main() == 0 else 1)
