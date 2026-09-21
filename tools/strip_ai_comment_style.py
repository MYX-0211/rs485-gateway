#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
strip_ai_comment_style.py - 去掉 C 源码注释里的 Markdown / emoji 残留

开源仓库里的注释应该是普通 C 注释，不应出现 Markdown 粗体、反引号、
emoji 符号、全角方括号等。本脚本做机械替换。

替换规则
--------
1. **粗体**          -> 粗体          （避开文档注释开头的 /**）
2. `code`            -> code
3. emoji（⚠️ ⭐ ✅ ❌ …）-> 删除（连同其后一个空格）
4. 【】               -> 删除（保留其中文字）
5. ⇒                 -> ->
6. （2026-09-21）等日期 -> 删除

用法
----
    python tools/strip_ai_comment_style.py --dry-run     # 只统计与预览，不写盘
    python tools/strip_ai_comment_style.py               # 实际写入
"""
import os
import re
import sys

FILES = [
    'Core/Src/main.c', 'Core/Src/esp01s.c', 'Core/Src/param.c', 'Core/Src/w25q64.c',
    'Core/Src/modbus.c', 'Core/Src/rs485.c', 'Core/Src/oled.c', 'Core/Src/stm32f4xx_it.c',
    'Core/Inc/ota.h', 'Core/Inc/w25q64.h', 'Core/Inc/param.h', 'Core/Inc/esp01s.h',
    'Bootloader/boot_main.c',
]

EMOJI = ['⚠️', '⭐', '✅', '❌', '🔧', '📌', '📋', '🔴', '🟡', '🟢', '⚪', '❗', '❓', '💡']


def transform(src: str):
    n = {}
    # 1) Markdown 粗体：保护 /**，再去掉成对星号
    src, n['bold'] = _count_sub(r'\*\*([^\n*]+?)\*\*', r'\1',
                                src.replace('/**', '\x01DOC\x01'))
    src = src.replace('\x01DOC\x01', '/**')
    # 2) 反引号
    src, n['tick'] = _count_sub(r'`([^`\n]+)`', r'\1', src)
    # 3) emoji
    c = 0
    for e in EMOJI:
        c += src.count(e)
        src = src.replace(e + ' ', '').replace(e, '')
    n['emoji'] = c
    # 4) 全角方括号
    c = src.count('【') + src.count('】')
    src = src.replace('【', '').replace('】', '')
    n['bracket'] = c
    # 5) ⇒
    n['arrow'] = src.count('⇒')
    src = src.replace('⇒', '->')
    # 6) 日期
    src, a = _count_sub(r'（2026-\d\d-\d\d）', '', src)
    src, b = _count_sub(r'\(2026-\d\d-\d\d\)', '', src)
    src, d = _count_sub(r'2026-\d\d-\d\d[：:]\s*', '', src)
    n['date'] = a + b + d
    return src, n


def _count_sub(pat, repl, src):
    new, k = re.subn(pat, repl, src)
    return new, k


def main() -> int:
    dry = '--dry-run' in sys.argv
    total = {}
    changed_files = 0
    for f in FILES:
        if not os.path.exists(f):
            continue
        raw = open(f, 'rb').read()
        # 保持 CRLF
        src = raw.decode('utf-8')
        new, n = transform(src)
        if new != src:
            changed_files += 1
            for k, v in n.items():
                total[k] = total.get(k, 0) + v
            print('  %-28s bold=%-3d tick=%-2d emoji=%-3d bracket=%-3d arrow=%-3d date=%d'
                  % (f, n['bold'], n['tick'], n['emoji'], n['bracket'], n['arrow'], n['date']))
            if not dry:
                open(f, 'wb').write(new.encode('utf-8'))
        else:
            print('  %-28s —（无变化）' % f)

    print()
    print('  统计：%s' % total)
    print('  %s修改 %d 个文件' % ('[dry-run] 将' if dry else '已', changed_files))
    return 0


if __name__ == '__main__':
    sys.exit(main())
