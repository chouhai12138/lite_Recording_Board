/**
 * @file    bsp_keys.h
 * @brief   4 键输入（UP=PD11 / DOWN=PB15 / LEFT=PB13 / RIGHT=PB14）
 *
 * 20 ms 消抖，长按 1 s（BOARD_KEY_LONG_MS）。事件进队列，主循环取用。
 * 支持 BSP_Keys_Inject() 注入虚拟按键，方便没装按键/用串口调试 UI。
 */
#ifndef __BSP_KEYS_H__
#define __BSP_KEYS_H__

#include "main.h"

typedef enum {
    BSP_KEY_UP = 0,
    BSP_KEY_DOWN,
    BSP_KEY_LEFT,
    BSP_KEY_RIGHT,
    BSP_KEY_COUNT
} bsp_key_t;

typedef enum {
    BSP_KEY_EV_PRESS = 0,   /* 消抖后按下 */
    BSP_KEY_EV_RELEASE,     /* 松开（hold_ms = 按住时长） */
    BSP_KEY_EV_LONG,        /* 按住达到长按阈值（只发一次） */
} bsp_key_ev_t;

typedef struct {
    bsp_key_t    key;
    bsp_key_ev_t ev;
    uint32_t     hold_ms;
} bsp_key_event_t;

void BSP_Keys_Init(void);
/** 每 5 ms 调用一次（传 HAL_GetTick()） */
void BSP_Keys_Scan(uint32_t now_ms);
/** 取一个事件；返回 1 表示取到 */
int  BSP_Keys_GetEvent(bsp_key_event_t *ev);
/** 当前是否按下（消抖后） */
int  BSP_Key_IsDown(bsp_key_t key);
/** 该键是否接了（未接的键只接受 Inject） */
int  BSP_Key_IsEnabled(bsp_key_t key);
/** 注入一次虚拟按键（is_long=1 时额外产生 LONG 事件） */
void BSP_Keys_Inject(bsp_key_t key, int is_long);
const char *BSP_Key_Name(bsp_key_t key);

#endif /* __BSP_KEYS_H__ */
