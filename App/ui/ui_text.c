/**
 * @file    ui_text.c
 * @brief   串口文本后端：把 UI 画成 ASCII/中文「屏幕」
 *
 * 每次状态变化整屏重绘（几百字节，115200 下 < 40 ms）。
 * 以后接 ST7789 时实现一份 ui_backend_t 交给 UI_SetBackend() 即可。
 */
#include <stdint.h>
#include <stdio.h>

#include "ui.h"
#include "ui_backend.h"

/* WCH-Link 的串口桥在满速连续输出时会丢字节（USB CDC 缓冲小），
 * 每行之间让出 1 ms 就能稳定。接真正的 USB-TTL 后可把这里改成 0。 */
#define UI_TX_GAP_MS    3U   /* 115200 下用；改 57600 可设 1 */

static void ui_pace(void)
{
#if UI_TX_GAP_MS > 0
    HAL_Delay(UI_TX_GAP_MS);
#endif
}

#define TEXT_WIDTH  26

static void text_draw_screen(const char *title,
                             const char *const *items, uint8_t count,
                             uint8_t cursor, const int32_t *values,
                             int8_t editing, const char *hint)
{
    printf("\r\n");
    printf("+----------------------------+\r\n");
    printf("| %-26s |\r\n", title);
    printf("+----------------------------+\r\n");

    for (uint8_t i = 0; i < count; i++) {
        int width = 0;
        printf("%c %u. %s", (i == cursor) ? '>' : ' ', (unsigned)(i + 1U), items[i]);
        /* 估算显示宽度：ASCII 1，其他（中文）2 */
        for (const char *p = items[i]; *p != '\0'; p++) {
            width += ((unsigned char)*p < 0x80U) ? 1 : 1;   /* 按字节粗算 */
        }
        (void)width;

        if (values != NULL && values[i] != INT32_MIN) {
            if (editing == (int8_t)i) {
                printf("  [%ld]", (long)values[i]);
            } else {
                printf("  %ld", (long)values[i]);
            }
        }
        printf("\r\n");
        ui_pace();
    }

    printf("+----------------------------+\r\n");
    printf(" %s\r\n", hint);
}

static void text_draw_info(const char *title, const char *const *lines,
                           uint8_t count, const char *hint)
{
    printf("\r\n");
    printf("+----------------------------+\r\n");
    printf("| %-26s |\r\n", title);
    printf("+----------------------------+\r\n");
    for (uint8_t i = 0; i < count; i++) {
        printf(" %s\r\n", lines[i]);
        ui_pace();
    }
    printf("+----------------------------+\r\n");
    printf(" %s\r\n", hint);
}

static void text_message(const char *text)
{
    printf("\r\n * %s\r\n", text);
}

static const ui_backend_t kTextBackend = {
    .draw_screen = text_draw_screen,
    .draw_info   = text_draw_info,
    .message     = text_message,
};

void UI_Text_Register(void)
{
    UI_SetBackend(&kTextBackend);
}
