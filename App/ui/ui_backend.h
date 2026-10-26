/**
 * @file    ui_backend.h
 * @brief   UI 绘制后端接口（串口文本 / 以后换 LVGL-ST7789）
 *
 * UI 核心只产出"标题 + 条目 + 光标 + 数值"，由后端决定怎么画。
 * 换屏幕时只需实现一份新的 ui_backend_t，不动导航逻辑。
 */
#ifndef __UI_BACKEND_H__
#define __UI_BACKEND_H__

#include <stdint.h>

typedef struct {
    /** 列表页：items/count 为条目；cursor 为选中项；editing>=0 表示该项正在编辑；
     *  values 可为 NULL，否则显示对应数值（editing 项用 [ ] 标出）。 */
    void (*draw_screen)(const char *title,
                        const char *const *items, uint8_t count,
                        uint8_t cursor, const int32_t *values, int8_t editing,
                        const char *hint);

    /** 信息页：自定义若干行文本 */
    void (*draw_info)(const char *title, const char *const *lines, uint8_t count,
                      const char *hint);

    /** 底部提示（会停留 ~1.5 s） */
    void (*message)(const char *text);
} ui_backend_t;

void UI_SetBackend(const ui_backend_t *backend);

/* 串口文本后端的注册入口（ui_text.c） */
void UI_Text_Register(void);
/* 1.54" 屏幕后端的注册入口（ui_lcd.c） */
void UI_LCD_Register(void);

#endif /* __UI_BACKEND_H__ */
