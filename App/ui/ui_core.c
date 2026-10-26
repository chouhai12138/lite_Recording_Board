/**
 * @file    ui_core.c
 * @brief   4 键 UI：屏幕 + 底板按键完成录制/回放
 *
 * 按键（底板，低有效）：
 *   PE14 = UP / PE10 = DOWN / PE12 = RIGHT(确认) / PE8 = LEFT(返回)
 *
 * 页面：
 *   MAIN  → Record / Play / Mode / Zero / Info
 *   REC   → Slot(1-5) / Start-Stop / Back
 *   PLAY  → Slot / Mode(CSP-MIT) / Loops / Gap / Start / Stop / Back
 *   MODE  → ZERO / DRAG / HOLD / Back
 *   ZERO  → Zero all (type 6) / Back
 *   INFO  → 状态信息（LEFT 返回）
 */
#include <stdio.h>
#include <string.h>

#include "ui.h"
#include "ui_backend.h"
#include "mit_ctrl.h"
#include "traj.h"
#include "motor_mgr.h"

ui_cfg_t g_ui_cfg = {
    .kp_drag     = 30,
    .kd_drag     = 1,
    .gravity_pct = 100,
    .brightness  = 80,
    .buzzer      = 1,
    .protocol    = 0,
};

/* ------------------------------ 可编辑参数 ------------------------------ */
static int32_t s_slot  = 1;     /* 1..TRAJ_SLOTS */
static int32_t s_loops = 1;     /* 0 = 无限 */
static int32_t s_gap   = 0;     /* ms */
static int32_t s_pmode = 0;     /* 0 = CSP, 1 = MIT */

static const char *const kMainItems[] = {"录制", "回放", "模式", "归零", "信息"};
static const char *const kModeItems[] = {"零力矩(阻尼)", "重力补偿拖动", "位置保持", "返回"};
static const char *const kZeroItems[] = {"全部归零", "返回"};

/* REC/PLAY 的条目动态生成（Start/Stop 文字随状态变） */
static char        sRecStart[16];
static char        sPlayStart[16];
static const char *sRecItems[3];
static const char *sPlayItems[7];

typedef struct {
    const char        *title;
    const char *const *items;
    uint8_t            count;
} screen_def_t;

static void build_items(ui_screen_t s)
{
    if (s == UI_SCREEN_REC) {
        snprintf(sRecStart, sizeof(sRecStart), "%s",
                 Traj_Recording() ? "停止录制" : "开始录制");
        sRecItems[0] = "槽位";
        sRecItems[1] = sRecStart;
        sRecItems[2] = "返回";
    } else if (s == UI_SCREEN_CSP) {
        snprintf(sPlayStart, sizeof(sPlayStart), "%s",
                 Traj_Playing() ? "回放中..." : "开始回放");
        sPlayItems[0] = "槽位";
        sPlayItems[1] = "方式";
        sPlayItems[2] = "次数";
        sPlayItems[3] = "间隔(ms)";
        sPlayItems[4] = sPlayStart;
        sPlayItems[5] = "停止";
        sPlayItems[6] = "返回";
    }
}

static screen_def_t screen_def(ui_screen_t s)
{
    switch (s) {
        case UI_SCREEN_MAIN: return (screen_def_t){"主菜单", kMainItems, 5};
        case UI_SCREEN_REC:  return (screen_def_t){"录制", sRecItems, 3};
        case UI_SCREEN_CSP:  return (screen_def_t){"回放", sPlayItems, 7};
        case UI_SCREEN_MIT:  return (screen_def_t){"模式", kModeItems, 4};
        case UI_SCREEN_ZERO: return (screen_def_t){"归零", kZeroItems, 2};
        default:             return (screen_def_t){"主菜单", kMainItems, 5};
    }
}

/* ------------------------------ 状态 ------------------------------ */
static const ui_backend_t *s_backend;
static ui_screen_t s_screen = UI_SCREEN_MAIN;
static uint8_t     s_cursor[UI_SCREEN_COUNT];
static int8_t      s_edit = -1;
static int32_t     s_edit_backup;
static char        s_msg[64];
static uint32_t    s_msg_until;
static uint32_t    s_refresh_next;
static uint32_t    s_boot_ms;

void UI_SetBackend(const ui_backend_t *backend)
{
    s_backend = backend;
}

static void go_main(void)
{
    s_screen = UI_SCREEN_MAIN;
    s_edit   = -1;
}

/* ------------------------------ 数值编辑 ------------------------------ */
static int32_t *edit_field(ui_screen_t s, uint8_t idx)
{
    if (s == UI_SCREEN_REC && idx == 0) { return &s_slot; }
    if (s == UI_SCREEN_CSP) {
        if (idx == 0) { return &s_slot; }
        if (idx == 1) { return &s_pmode; }
        if (idx == 2) { return &s_loops; }
        if (idx == 3) { return &s_gap; }
    }
    return NULL;
}

static void clamp_field(ui_screen_t s, uint8_t idx, int32_t *v)
{
    if (s == UI_SCREEN_REC || (s == UI_SCREEN_CSP && idx == 0)) {
        if (*v < 1) { *v = 1; }
        if (*v > TRAJ_SLOTS) { *v = TRAJ_SLOTS; }
    } else if (s == UI_SCREEN_CSP && idx == 1) {
        if (*v < 0) { *v = 0; }
        if (*v > 1) { *v = 1; }
    } else if (s == UI_SCREEN_CSP && idx == 2) {
        if (*v < 0) { *v = 0; }
        if (*v > 20) { *v = 20; }
    } else if (s == UI_SCREEN_CSP && idx == 3) {
        if (*v < 0) { *v = 0; }
        if (*v > 10000) { *v = 10000; }
    }
}

static void adjust_value(int8_t dir)
{
    int32_t *f = edit_field(s_screen, (uint8_t)s_edit);

    if (f == NULL) {
        return;
    }
    *f += dir * ((s_screen == UI_SCREEN_CSP && s_edit == 3) ? 250 : 1);
    clamp_field(s_screen, (uint8_t)s_edit, f);
}

/* ------------------------------ 执行动作 ------------------------------ */
static void do_enter(void)
{
    uint8_t c = s_cursor[s_screen];

    switch (s_screen) {
        case UI_SCREEN_MAIN:
            if (c == 0) { s_screen = UI_SCREEN_REC; }
            else if (c == 1) { s_screen = UI_SCREEN_CSP; }
            else if (c == 2) { s_screen = UI_SCREEN_MIT; }
            else if (c == 3) { s_screen = UI_SCREEN_ZERO; }
            else { s_screen = UI_SCREEN_INFO; }
            break;

        case UI_SCREEN_REC:
            if (c == 0) {
                s_edit = 0;
                s_edit_backup = s_slot;
            } else if (c == 1) {
                if (Traj_Recording()) {
                    Traj_StopRecord();
                    UI_ShowMessage("已保存到Flash");
                    s_refresh_next = 0;
                } else {
                    /* 擦除 256KB 要 2 秒，先把"初始化中"画到屏上 */
                    UI_ShowMessage("初始化中...");
                    UI_Render();
                    if (Traj_StartRecord((uint8_t)(s_slot - 1)) == 0) {
                        UI_ShowMessage("录制中...");
                        s_refresh_next = 0;
                    } else {
                        UI_ShowMessage("启动失败");
                    }
                }
            } else {
                go_main();
            }
            break;

        case UI_SCREEN_CSP:
            if (c == 0) {
                s_edit = 0;
                s_edit_backup = s_slot;
            } else if (c == 1) {
                s_edit = 1;
                s_edit_backup = s_pmode;
            } else if (c == 2) {
                s_edit = 2;
                s_edit_backup = s_loops;
            } else if (c == 3) {
                s_edit = 3;
                s_edit_backup = s_gap;
            } else if (c == 4) {
                Traj_SetPlayMode((uint8_t)s_pmode);
                Traj_SetPlayGap((uint16_t)s_gap);
                if (Traj_StartPlay((uint8_t)(s_slot - 1),
                                   (uint8_t)((s_loops < 0) ? 0 : s_loops)) == 0) {
                    UI_ShowMessage("回放中...");
                } else {
                    UI_ShowMessage("回放失败");
                }
            } else if (c == 5) {
                Traj_StopPlay();
                UI_ShowMessage("已停止");
            } else {
                go_main();
            }
            break;

        case UI_SCREEN_MIT:
            if (c == 0) { MitCtrl_SetMode(MIT_MODE_ZERO); UI_ShowMessage("零力矩"); }
            else if (c == 1) { MitCtrl_SetMode(MIT_MODE_DRAG); UI_ShowMessage("重力补偿拖动"); }
            else if (c == 2) { MitCtrl_SetMode(MIT_MODE_HOLD); UI_ShowMessage("位置保持"); }
            else { go_main(); }
            break;

        case UI_SCREEN_ZERO:
            if (c == 0) {
                int bad = MotorMgr_ZeroAllVerify();

                UI_ShowMessage((bad == 0) ? "归零成功" : "归零失败");
            } else {
                go_main();
            }
            break;

        default:
            break;
    }
}

/* ------------------------------ 公共接口 ------------------------------ */
void UI_Init(void)
{
    memset(s_cursor, 0, sizeof(s_cursor));
    s_screen       = UI_SCREEN_MAIN;
    s_edit         = -1;
    s_msg[0]       = '\0';
    s_msg_until    = 0;
    s_refresh_next = 0;
    s_boot_ms      = HAL_GetTick();
    build_items(UI_SCREEN_REC);
    build_items(UI_SCREEN_CSP);
    UI_Render();
}

void UI_ShowMessage(const char *text)
{
    snprintf(s_msg, sizeof(s_msg), "%s", text);
    s_msg_until = HAL_GetTick() + 1500U;
}

void UI_HandleKey(bsp_key_t key, int is_long)
{
    (void)is_long;

    if (s_screen == UI_SCREEN_INFO) {
        if (key == BSP_KEY_LEFT) {
            go_main();
        }
        UI_Render();
        return;
    }

    if (s_edit >= 0) {
        int32_t *f = edit_field(s_screen, (uint8_t)s_edit);

        if (key == BSP_KEY_UP) {
            adjust_value(+1);
        } else if (key == BSP_KEY_DOWN) {
            adjust_value(-1);
        } else if (key == BSP_KEY_RIGHT) {
            s_edit = -1;
        } else if (key == BSP_KEY_LEFT) {
            if (f != NULL) {
                *f = s_edit_backup;
            }
            s_edit = -1;
        }
        UI_Render();
        return;
    }

    {
        screen_def_t def = screen_def(s_screen);

        switch (key) {
            case BSP_KEY_UP:
                s_cursor[s_screen] = (uint8_t)((s_cursor[s_screen] == 0U)
                                               ? (def.count - 1U)
                                               : (s_cursor[s_screen] - 1U));
                break;
            case BSP_KEY_DOWN:
                s_cursor[s_screen]++;
                if (s_cursor[s_screen] >= def.count) {
                    s_cursor[s_screen] = 0;
                }
                break;
            case BSP_KEY_RIGHT:
                do_enter();
                break;
            case BSP_KEY_LEFT:
                if (Traj_Recording()) {
                    Traj_StopRecord();
                    UI_ShowMessage("已保存到Flash");
                }
                if (s_screen != UI_SCREEN_MAIN) {
                    go_main();
                }
                break;
            default:
                break;
        }
    }
    UI_Render();
}

void UI_Tick(uint32_t now_ms)
{
    if (s_msg[0] != '\0' && (int32_t)(now_ms - s_msg_until) >= 0) {
        s_msg[0] = '\0';
        UI_Render();
    }

    /* 录制/回放中定期刷新状态提示（500ms，避免刷屏太占 SPI） */
    if (Traj_Recording() || Traj_Playing() || s_screen == UI_SCREEN_INFO) {
        if ((int32_t)(now_ms - s_refresh_next) >= 0) {
            s_refresh_next = now_ms + 500U;
            UI_Render();
        }
    }
}

static void fill_values(ui_screen_t s, int32_t *vals, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        const int32_t *f = edit_field(s, i);
        vals[i] = (f != NULL) ? *f : INT32_MIN;   /* 哨兵：该项不显示数值 */
    }
}

void UI_Render(void)
{
    static int32_t vals[UI_MAX_ITEMS];
    uint32_t   now = HAL_GetTick();
    char       hint[64];

    if (s_backend == NULL) {
        return;
    }

    if (s_screen == UI_SCREEN_INFO) {
        char        l0[48], l1[48], l2[48], l3[48], l4[48];
        const char *lines[5];
        const traj_meta_t *mt = Traj_GetMeta((uint8_t)(s_slot - 1));
        uint32_t    up_s = (now - s_boot_ms) / 1000U;

        snprintf(l0, sizeof(l0), "模式: %s", MitCtrl_ModeName(MitCtrl_Mode()));
        snprintf(l1, sizeof(l1), "电机: %u/%u 使能%u",
                 (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_Count(),
                 (unsigned)MotorMgr_EnabledCount());
        snprintf(l2, sizeof(l2), "槽%ld: %s %u帧", (long)s_slot,
                 (mt != NULL && mt->valid) ? "OK" : "--",
                 (unsigned)((mt != NULL) ? mt->frames : 0U));
        snprintf(l3, sizeof(l3), "运行: %lu 秒", (unsigned long)up_s);
        snprintf(l4, sizeof(l4), "按键: 上%d 下%d 左%d 右%d",
                 BSP_Key_IsDown(BSP_KEY_UP), BSP_Key_IsDown(BSP_KEY_DOWN),
                 BSP_Key_IsDown(BSP_KEY_LEFT), BSP_Key_IsDown(BSP_KEY_RIGHT));

        lines[0] = l0; lines[1] = l1; lines[2] = l2;
        lines[3] = l3; lines[4] = l4;
        s_backend->draw_info("信息", lines, 5, "左键返回");
        return;
    }

    build_items(s_screen);
    {
        screen_def_t def = screen_def(s_screen);

        fill_values(s_screen, vals, def.count);

        if (s_edit >= 0) {
            snprintf(hint, sizeof(hint), "上下:调整 右:确认 左:取消");
        } else if (s_screen == UI_SCREEN_REC && Traj_Recording()) {
            const traj_meta_t *mt = Traj_GetMeta((uint8_t)(s_slot - 1));
            uint32_t fr  = (mt != NULL) ? mt->frames : 0U;
            uint32_t sec = (mt != NULL) ? mt->duration_ms / 1000U : 0U;

            snprintf(hint, sizeof(hint), "录制中 %lu.%lus (%lu 帧) 右:停止",
                     (unsigned long)sec,
                     (unsigned long)(((mt != NULL) ? mt->duration_ms : 0U) % 1000U) / 100U,
                     (unsigned long)fr);
        } else if (s_screen == UI_SCREEN_CSP && Traj_Playing()) {
            snprintf(hint, sizeof(hint), "回放中 槽%ld %s",
                     (long)s_slot, (s_pmode == 0) ? "CSP" : "MIT");
        } else {
            snprintf(hint, sizeof(hint), "上下:选择 右:确认 左:返回");
        }
        s_backend->draw_screen(def.title, def.items, def.count,
                               s_cursor[s_screen], vals, s_edit, hint);
    }

    if (s_msg[0] != '\0' && (int32_t)(now - s_msg_until) < 0) {
        s_backend->message(s_msg);
    }
}

void UI_SetScreen(ui_screen_t screen)
{
    if (screen >= UI_SCREEN_COUNT) {
        return;
    }
    s_screen       = screen;
    s_edit         = -1;
    s_refresh_next = 0;
    UI_Render();
}

ui_screen_t UI_GetScreen(void)
{
    return s_screen;
}
