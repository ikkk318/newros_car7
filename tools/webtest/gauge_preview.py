#!/usr/bin/env python3
"""
gauge_preview —— 把「实时状态」仪表按 app.css 的真实尺寸画成 PNG，用于肉眼验收。

**为什么需要它**
沙箱里没有浏览器驱动（firefox 是 snap 版、也没有 puppeteer），
但"7 行有没有挤、值会不会换行、颜色对不对"必须看一眼才算验证过。
仪表是纯矩形 + 文本，用 PIL 按 CSS 里的真实像素值复刻即可，
比"读一遍 CSS 想象效果"可靠得多。

**它验证什么**
  · 8 行总高度是否有溢出（手机上一屏能不能放下）
  · 最长的值（速度 "0.30 · -0.28"、模式 "建图 · 手动"）会不会换行
  · tone 色条/字色是否和 CSS 一致

**它不验证什么**
交互（点击/滚动）和真实字体回退 —— 那些只能真机看。

用法：
    python3 tools/webtest/gauge_preview.py
输出：
    tools/webtest/gauge_preview.png
"""
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'gauge_preview.png')

# ── 与 app.css 对齐的尺寸（改 CSS 时这里要跟着改）──
SIDE_W = 360          # .side width
SIDE_PAD = 10         # .side padding
CARD_PAD = 12         # .card padding
CARD_BORDER = 1
ROW_H = 30            # .brow padding 6+6 + line-height 1.4*12 ≈ 29
ROW_GAP = 4           # .board gap
LABEL_W = 44          # .brow .bl min-width
ROW_PAD_X = 9         # .brow padding 左右
FONT_PX = 12
CARD_HEAD_H = 30      # .card-h 高度

# ── 与 app.css 对齐的颜色（--panel2 / --txt / --dim / ok / warn / err）──
BG = (14, 17, 23)
PANEL = (26, 31, 42)
PANEL2 = (34, 40, 54)
LINE = (42, 49, 64)
TXT = (226, 233, 243)
DIM = (138, 150, 170)
OK = (58, 190, 140)
WARN = (233, 174, 62)
ERR = (232, 92, 92)

FONT_PATHS = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
]


def font(size):
    for p in FONT_PATHS:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


# ── 真实一屏数据（取自 board_e2e / board_demo 的实测输出）──
# ★ 顺序必须与 status_board.GAUGES 完全一致（8 槽位）：
#   '动作' 紧跟 '任务'（载货状态是安全信息），'连接' 在最后。
ROWS = [
    ('任务', '搬运中 2/5', 'ok'),
    ('动作', '载货中', 'warn'),
    ('速度', '0.30 · -0.28', 'ok'),
    ('避障', '0.52m 减速', 'warn'),
    ('定位', '正常', 'ok'),
    ('地图', '594×762', 'ok'),
    ('模式', '导航 · 自动', 'ok'),
    ('连接', '已连接', 'ok'),
]


def main():
    inner_w = SIDE_W - SIDE_PAD * 2 - CARD_PAD * 2 - CARD_BORDER * 2
    card_h = CARD_HEAD_H + len(ROWS) * (ROW_H + ROW_GAP) - ROW_GAP + CARD_PAD * 2
    w = SIDE_W
    h = card_h + SIDE_PAD * 2

    img = Image.new('RGB', (w, h), BG)
    d = ImageDraw.Draw(img)
    f_lbl, f_val, f_head = font(FONT_PX), font(FONT_PX), font(13)

    # 卡片
    x0, y0 = SIDE_PAD, SIDE_PAD
    x1, y1 = w - SIDE_PAD, h - SIDE_PAD
    d.rounded_rectangle([x0, y0, x1, y1], radius=14, fill=PANEL, outline=LINE)

    # 卡片头："实时状态" + 徽标
    d.text((x0 + CARD_PAD, y0 + 8), '实时状态', font=f_head, fill=TXT)
    bw = 46
    bx = x1 - CARD_PAD - bw
    d.rounded_rectangle([bx, y0 + 7, bx + bw, y0 + 7 + 17], radius=9,
                        fill=(30, 60, 50), outline=(58, 190, 140))
    d.text((bx + 9, y0 + 9), '正常', font=font(11), fill=OK)

    # 7 行
    ry = y0 + CARD_HEAD_H + CARD_PAD
    tone_color = {'ok': OK, 'warn': WARN, 'error': ERR, 'off': (91, 102, 122)}
    for label, text, tone in ROWS:
        rx0, rx1 = x0 + CARD_PAD, x1 - CARD_PAD
        d.rounded_rectangle([rx0, ry, rx1, ry + ROW_H], radius=8, fill=PANEL2)
        # 左侧 3px 色条
        d.rectangle([rx0, ry, rx0 + 3, ry + ROW_H], fill=tone_color[tone])
        cy = ry + ROW_H // 2
        d.text((rx0 + ROW_PAD_X + 3, cy), label, font=f_lbl, fill=DIM, anchor='lm')
        vx = rx0 + ROW_PAD_X + 3 + LABEL_W + 8
        vcolor = TXT if tone == 'ok' else tone_color[tone]
        d.text((vx, cy), text, font=f_val, fill=vcolor, anchor='lm')
        ry += ROW_H + ROW_GAP

    img.save(OUT)

    # ── 自检：把这些数字算出来，比"看着差不多"可靠 ──
    val_space = inner_w - ROW_PAD_X * 2 - 3 - LABEL_W - 8
    print(f'已生成 {OUT}（{w}×{h}px）')
    print()
    print(f'  一格文本区宽 {val_space}px ≈ {val_space // FONT_PX} 个中文字')
    print(f'  8 行总高 {len(ROWS) * (ROW_H + ROW_GAP) - ROW_GAP}px，'
          f'手机侧栏一屏通常 ≥ 500px → 放得下')
    print()
    worst = 0
    for label, text, _ in ROWS:
        # 中文按全宽、数字英文按 0.55 宽估
        wpx = sum(FONT_PX if ord(c) > 127 else FONT_PX * 0.55 for c in text)
        worst = max(worst, wpx)
        flag = '换行' if wpx > val_space else '放得下'
        print(f'  {label:4s} {text:14s} 约 {wpx:5.0f}px  {flag}')
    print()
    print(f'  最宽一行 {worst:.0f}px / 可用 {val_space}px')


if __name__ == '__main__':
    main()
