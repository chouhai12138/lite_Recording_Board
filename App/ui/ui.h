/**
 * @file    ui.h
 * @brief   4 键菜单 UI（LEFT=返回 / RIGHT=确认 / UP/DOWN=选择）
 *
 * 页面树与冻结的设计一致：
 *   主界面 → MIT / CSP+回放 / 录制 / 归零 / 设置 / 信息 / 安全
 * 目前 MIT、CSP、录制、归零 的叶子项是占位（等 A3 虚拟总线 / A4+ 控制逻辑接上）。
 */
#ifndef __UI_H__
#define __UI_H__

#include "main.h"
#include "bsp_keys.h"

#define UI_MAX_ITEMS    8

typedef enum {
    UI_SCREEN_MAIN = 0,
    UI_SCREEN_MIT,
    UI_SCREEN_CSP,
    UI_SCREEN_REC,
    UI_SCREEN_ZERO,
    UI_SCREEN_SET,
    UI_SCREEN_INFO,
    UI_SCREEN_SAFE,
    UI_SCREEN_COUNT
} ui_screen_t;

/* 可编辑参数（后续会落到 config_store / 24C02） */
typedef struct {
    int32_t kp_drag;        /* 0..500 */
    int32_t kd_drag;        /* 0..100 */
    int32_t gravity_pct;    /* 0..150 */
    int32_t brightness;     /* 10..100 */
    int32_t buzzer;         /* 0/1  */
    int32_t protocol;       /* 0=MIT / 1=CSP */
} ui_cfg_t;

extern ui_cfg_t g_ui_cfg;

void        UI_Init(void);
void        UI_HandleKey(bsp_key_t key, int is_long);
void        UI_Tick(uint32_t now_ms);
void        UI_Render(void);
void        UI_ShowMessage(const char *text);
void        UI_SetScreen(ui_screen_t screen);   /* 调试用：直接跳页 */
ui_screen_t UI_GetScreen(void);

#endif /* __UI_H__ */
