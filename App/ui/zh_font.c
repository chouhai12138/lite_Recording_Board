/**
 * @file    zh_font.c
 * @brief   中英混排点阵字库渲染（ASCII 8x16，汉字 16x16）
 */
#include <stdint.h>

#include "zh_font.h"
#include "bsp_lcd.h"

static const zh_glyph_t *find_glyph(uint32_t code)
{
    for (uint16_t i = 0; i < ZH_GLYPH_COUNT; i++) {
        if (kZhGlyphs[i].code == code) {
            return &kZhGlyphs[i];
        }
        if (kZhGlyphs[i].code > code) {
            break;
        }
    }
    return 0;
}

static int utf8_next(const char **p, uint32_t *code)
{
    const uint8_t *s = (const uint8_t *)*p;

    if (s[0] < 0x80U) {
        *code = s[0];
        *p += 1;
        return 1;
    }
    if ((s[0] & 0xE0U) == 0xC0U && s[1] != 0U) {
        *code = ((uint32_t)(s[0] & 0x1FU) << 6) | (uint32_t)(s[1] & 0x3FU);
        *p += 2;
        return 2;
    }
    if ((s[0] & 0xF0U) == 0xE0U && s[1] != 0U && s[2] != 0U) {
        *code = ((uint32_t)(s[0] & 0x0FU) << 12) |
                ((uint32_t)(s[1] & 0x3FU) << 6) | (uint32_t)(s[2] & 0x3FU);
        *p += 3;
        return 3;
    }
    *code = '?';
    *p += 1;
    return 1;
}

uint16_t Font_Width(const char *s)
{
    uint16_t w = 0;
    uint32_t code;

    while (*s != '\0') {
        int n = utf8_next(&s, &code);
        const zh_glyph_t *g = find_glyph(code);
        (void)n;
        w += (g != 0) ? g->w : 8;
    }
    return w;
}

void Font_DrawString(uint16_t x, uint16_t y, const char *s, uint16_t color)
{
    uint32_t code;

    while (*s != '\0') {
        const zh_glyph_t *g;
        uint16_t cx = x;

        (void)utf8_next(&s, &code);
        g = find_glyph(code);
        if (g == 0) {
            x += 8;
            continue;
        }

        for (uint8_t r = 0; r < 16; r++) {
            uint16_t bits = 0;
            if (g->w == 16U) {
                bits = (uint16_t)(((uint16_t)g->rows[r * 2] << 8) | g->rows[r * 2 + 1]);
            } else {
                bits = (uint16_t)g->rows[r];
            }

            /* 把一行里连续的亮点合并成若干次 FillRect，减少 SPI 事务 */
            uint8_t c = 0;
            while (c < g->w) {
                if (bits & (1U << (g->w - 1U - c))) {
                    uint8_t run = 1;

                    while ((c + run) < g->w &&
                           (bits & (1U << (g->w - 1U - (c + run))))) {
                        run++;
                    }
                    BSP_LCD_FillRect((uint16_t)(cx + c), (uint16_t)(y + r),
                                     run, 1, color);
                    c = (uint8_t)(c + run);
                } else {
                    c++;
                }
            }
        }
        x = (uint16_t)(x + g->w);
    }
}
