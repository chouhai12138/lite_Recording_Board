/**
 * @file    ui_lcd.c
 * @brief   把 UI 画到 1.54" 屏幕（原厂 ST7789 驱动 + 字库）
 *
 * 只实现 ui_backend_t，导航逻辑仍在 ui_core.c。文案是 ASCII（原厂字库是 GBK 子集，
 * 换自定义字库后可以再改回中文）。
 */
#include <stdio.h>
#include <string.h>

#include "ui.h"
#include "ui_backend.h"
#include "bsp_lcd.h"
#include "lcd_spi_154.h"
#include "zh_font.h"

#define LCD_W           240
#define LCD_H           240

#define C_BG            0x000000
#define C_TITLE_BG      0x1C3B6E
#define C_TITLE_FG      0xFFFFFF
#define C_ITEM_FG       0xE8E8E8
#define C_SEL_BG        0x2E5DA8
#define C_HINT_FG       0x909090
#define C_MSG_BG        0xFFD000
#define C_MSG_FG        0x000000

static void draw_text(uint16_t x, uint16_t y, const char *s, uint32_t color);

#define TITLE_H         30
#define ITEM_Y0         40
#define ITEM_H          22      /* 7 行到 188，给消息条(196)留出空间，不再被盖住底部 */
#define MAX_ROWS        7


/* ---- 增量刷新缓存：只重画有变化的行/提示 ---- */
typedef struct {
    uint8_t valid;
    char    title[32];
    uint8_t count;
    uint8_t cursor;
    uint8_t first;
    int8_t  editing;
    char    items[MAX_ROWS][28];
    int32_t vals[MAX_ROWS];
    char    hint[64];
} lcd_cache_t;

static lcd_cache_t s_cache;

static void draw_row(uint8_t i, uint8_t idx, const char *text, int32_t val,
                     uint8_t sel, int8_t editing)
{
    char    buf[24];
    uint16_t y = (uint16_t)(ITEM_Y0 + i * ITEM_H);

    LCD_SetColor(C_BG);
    LCD_FillRect(0, (uint16_t)(y - 3), LCD_W, (ITEM_H - 2));
    if (sel) {
        LCD_SetColor(C_SEL_BG);
        LCD_FillRect(0, (uint16_t)(y - 3), LCD_W, (ITEM_H - 2));
    }

    draw_text(10, y, text, sel ? 0xFFFFFF : C_ITEM_FG);

    if (val != INT32_MIN) {
        if (editing == (int8_t)idx) {
            snprintf(buf, sizeof(buf), "[%ld]", (long)val);
        } else {
            snprintf(buf, sizeof(buf), "%ld", (long)val);
        }
        draw_text(190, y, buf, sel ? 0xFFFFFF : 0xBFD4FF);
    }
}

static void draw_hint(const char *hint)
{
    LCD_SetColor(C_BG);
    LCD_FillRect(0, 220, LCD_W, 20);
    draw_text(6, 222, hint, C_HINT_FG);
}

static void draw_text(uint16_t x, uint16_t y, const char *s, uint32_t color)
{
    Font_DrawString(x, y, s, (uint16_t)color);
}

static void draw_title(const char *title)
{
    LCD_SetColor(C_TITLE_BG);
    LCD_FillRect(0, 0, LCD_W, TITLE_H);

    Font_DrawString(8, 6, title, C_TITLE_FG);
}


static void lcd_draw_screen(const char *title,
                            const char *const *items, uint8_t count,
                            uint8_t cursor, const int32_t *values,
                            int8_t editing, const char *hint)
{
    uint8_t first = 0;
    uint8_t rows  = (count > MAX_ROWS) ? MAX_ROWS : count;
    int     full;

    if (cursor >= rows) {
        first = (uint8_t)(cursor - rows + 1U);
    }

    full = (!s_cache.valid) ||
           (strcmp(title, s_cache.title) != 0) ||
           (count != s_cache.count) ||
           (first != s_cache.first);

    if (full) {
        LCD_SetBackColor(C_BG);
        LCD_Clear();
        draw_title(title);
        for (uint8_t i = 0; i < rows; i++) {
            uint8_t idx = (uint8_t)(first + i);
            int32_t v   = (values != NULL) ? values[idx] : INT32_MIN;

            draw_row(i, idx, items[idx], v, (idx == cursor) ? 1U : 0U, editing);
            snprintf(s_cache.items[i], sizeof(s_cache.items[i]), "%s", items[idx]);
            s_cache.vals[i] = v;
        }
        draw_hint(hint);
        snprintf(s_cache.hint, sizeof(s_cache.hint), "%s", hint);
        snprintf(s_cache.title, sizeof(s_cache.title), "%s", title);
        s_cache.count   = count;
        s_cache.cursor  = cursor;
        s_cache.first   = first;
        s_cache.editing = editing;
        s_cache.valid   = 1;
        return;
    }

    /* 只重画变化的行 */
    for (uint8_t i = 0; i < rows; i++) {
        uint8_t idx = (uint8_t)(first + i);
        int32_t v   = (values != NULL) ? values[idx] : INT32_MIN;
        int     sel = (idx == cursor) ? 1 : 0;
        int     was = (idx == s_cache.cursor) ? 1 : 0;
        int     changed = (strcmp(items[idx], s_cache.items[i]) != 0) ||
                          (v != s_cache.vals[i]) || (sel != was);

        if (s_cache.editing != editing && (s_cache.editing == (int8_t)idx ||
                                           editing == (int8_t)idx)) {
            changed = 1;
        }
        if (changed) {
            draw_row(i, idx, items[idx], v, (uint8_t)sel, editing);
            snprintf(s_cache.items[i], sizeof(s_cache.items[i]), "%s", items[idx]);
            s_cache.vals[i] = v;
        }
    }
    s_cache.cursor  = cursor;
    s_cache.editing = editing;

    if (strcmp(hint, s_cache.hint) != 0) {
        draw_hint(hint);
        snprintf(s_cache.hint, sizeof(s_cache.hint), "%s", hint);
    }
}

static void lcd_draw_info(const char *title, const char *const *lines,
                          uint8_t count, const char *hint)
{
    s_cache.valid = 0;
    LCD_SetBackColor(C_BG);
    LCD_Clear();
    draw_title(title);

    for (uint8_t i = 0; i < count; i++) {
        draw_text(8, (uint16_t)(44 + i * 26), lines[i], C_ITEM_FG);
    }

    draw_text(6, 222, hint, C_HINT_FG);
}

static void lcd_message(const char *text)
{
    LCD_SetColor(C_MSG_BG);
    LCD_FillRect(0, 196, LCD_W, 24);

    draw_text(8, 198, text, C_MSG_FG);
    s_cache.valid = 0;      /* 覆盖了列表区，消息消失后需要整屏重画一次 */
}

static const ui_backend_t kLcdBackend = {
    .draw_screen = lcd_draw_screen,
    .draw_info   = lcd_draw_info,
    .message     = lcd_message,
};

void UI_LCD_Register(void)
{
    UI_SetBackend(&kLcdBackend);
}
