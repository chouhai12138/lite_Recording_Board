/**
 * @file    can_bus.c
 * @brief   总线后端：虚拟总线（Sim/）或真实双 CAN（BSP/bsp_can）
 */
#include "main.h"
#include "can_bus.h"
#include "sim_can.h"
#include "bsp_can.h"

static can_rx_cb_t  s_cb;
static void        *s_user;
static uint8_t      s_hw;
static uint32_t     s_last_tick;

/* ---- 真实 CAN 收帧 → 统一帧结构 ---- */
static void hw_rx_cb(bsp_can_id_t bus, uint32_t id, uint8_t dlc, const uint8_t *data)
{
    rs_can_frame_t f;

    f.id  = id;
    f.dlc = dlc;
    f.bus = (uint8_t)bus;
    for (uint8_t i = 0; i < 8U; i++) {
        f.data[i] = (i < dlc) ? data[i] : 0U;
    }
    if (s_cb != NULL) {
        s_cb(&f, s_user);
    }
}

void CAN_Bus_Init(can_rx_cb_t cb, void *user)
{
    s_cb        = cb;
    s_user      = user;
    s_last_tick = HAL_GetTick();
    s_hw        = 0;
    Sim_Init();
}

int CAN_Bus_SetHardware(int on)
{
    if (on) {
        if (!BSP_CAN_Init(hw_rx_cb)) {
            return 0;
        }
        s_hw = 1;
        return 1;
    }
    s_hw = 0;
    Sim_Init();
    return 1;
}

int CAN_Bus_IsHardware(void)
{
    return s_hw;
}

int CAN_Bus_SetLoopback(int on)
{
    if (!s_hw) {
        BSP_CAN_Init(hw_rx_cb);
        s_hw = 1;
    }
    return BSP_CAN_SetMode((uint8_t)(on ? 1 : 0));
}

int CAN_Bus_Send(uint8_t bus, const rs_can_frame_t *frame)
{
    if (!s_hw) {
        return Sim_HandleTx(frame);
    }
    return BSP_CAN_Send((bsp_can_id_t)bus, frame->id, frame->data, frame->dlc);
}

void CAN_Bus_Poll(void)
{
    rs_can_frame_t f;
    uint32_t now;

    if (s_hw) {
        return;                     /* 真实 CAN 走中断，不需要轮询 */
    }

    now = HAL_GetTick();
    if ((now - s_last_tick) >= 10U) {
        s_last_tick = now;
        Sim_Tick();
    }

    while (Sim_RxPop(&f)) {
        if (s_cb != NULL) {
            s_cb(&f, s_user);
        }
    }
}
