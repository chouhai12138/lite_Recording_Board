#!/usr/bin/env python3
"""从 UI 源码里收集用到的字符，生成 8x16(ASCII)/16x16(CJK) 点阵字库 C 头文件。

用法: python3 Scripts/gen_zh_font.py App/ui/ui_core.c App/ui/ui_lcd.c
产出: App/ui/zh_font.h
"""
import sys
from PIL import Image, ImageDraw, ImageFont

ASCII_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
CJK_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
W_ASCII, H = 8, 16
W_CJK = 16


def collect(paths):
    chars = set()
    for p in paths:
        for ch in open(p, encoding="utf-8").read():
            if ch == "\n":
                continue
            if 0x20 <= ord(ch) < 0x7F:
                chars.add(ch)
            elif ord(ch) > 0x7F:
                chars.add(ch)
    return chars


def render(ch, cjk):
    size = 14 if cjk else 12
    font = ImageFont.truetype(CJK_FONT if cjk else ASCII_FONT, size)
    w = W_CJK if cjk else W_ASCII
    img = Image.new("L", (w, H), 0)
    d = ImageDraw.Draw(img)
    # anchor="mm"：按字形包围盒中心对齐，保证上下都不裁切（含括号、下伸部）
    d.text((w / 2.0, H / 2.0), ch, font=font, fill=255, anchor="mm")
    px = img.load()
    rows = []
    for r in range(H):
        bits = 0
        for c in range(w):
            if px[c, r] > 110:
                bits |= 1 << (w - 1 - c)
        rows.append(bits)
    # 最底行留空：防止字贴在单元格底边时被面板/相邻行刷掉 1 像素
    rows = rows[1:] + [0]     # 整体上移 1 像素，最底行留空
    rows = rows[:H]
    return w, rows


def main():
    chars = sorted(collect(sys.argv[1:]), key=ord)
    out = ["/* 由 Scripts/gen_zh_font.py 自动生成，勿手改 */",
           "#ifndef __ZH_FONT_H__", "#define __ZH_FONT_H__", "",
           "#include <stdint.h>", "",
           "typedef struct {",
           "    uint32_t code;      /* Unicode 码点 */",
           "    uint8_t  w;         /* 字宽（8 或 16） */",
           "    const uint8_t *rows;/* 16 行，每行 w/8 字节（MSB 在左） */",
           "} zh_glyph_t;",
           ""]
    for ch in chars:
        cjk = ord(ch) > 0x7F
        w, rows = render(ch, cjk)
        nbytes = w // 8
        arr = []
        for v in rows:
            b = v.to_bytes(2, "big") if nbytes == 2 else v.to_bytes(1, "big")
            arr.extend(b)
        out.append("static const uint8_t g_%04X[%d] = {" % (ord(ch), len(arr)))
        out.append("    " + ", ".join("0x%02X" % b for b in arr))
        out.append("};")
    out.append("")
    out.append("static const zh_glyph_t kZhGlyphs[] = {")
    for ch in chars:
        cjk = ord(ch) > 0x7F
        out.append("    {0x%04X, %d, g_%04X}," % (ord(ch), W_CJK if cjk else W_ASCII, ord(ch)))
    out.append("};")
    out.append("#define ZH_GLYPH_COUNT (sizeof(kZhGlyphs)/sizeof(kZhGlyphs[0]))")
    out.append("")
    out.append("#endif")
    open("App/ui/zh_font.h", "w").write("\n".join(out) + "\n")
    print("glyphs:", len(chars))


if __name__ == "__main__":
    main()
