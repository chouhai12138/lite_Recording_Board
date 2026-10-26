/**
 * @file    can_bus.h
 * @brief   总线抽象：可在"虚拟总线（Sim/）"和"真实双 CAN（BSP/bsp_can）"之间切换
 *
 * 上层（motor_mgr）只调这几个函数，切换后端不用改控制逻辑。
 */
#ifndef __CAN_BUS_H__
#define __CAN_BUS_H__

#include "rs_protocol.h"

typedef void (*can_rx_cb_t)(const rs_can_frame_t *frame, void *user);

void CAN_Bus_Init(can_rx_cb_t cb, void *user);
/** 1 = 用真实 CAN1/CAN2；0 = 用虚拟总线 */
int  CAN_Bus_SetHardware(int on);
int  CAN_Bus_IsHardware(void);
/** 真实 CAN 的内部回环模式（自测用，不需要总线上有第二个节点） */
int  CAN_Bus_SetLoopback(int on);
/** bus: 0 = CAN1，1 = CAN2 */
int  CAN_Bus_Send(uint8_t bus, const rs_can_frame_t *frame);
/** 主循环每圈调用：推进虚拟总线、投递收到的帧 */
void CAN_Bus_Poll(void);

#endif /* __CAN_BUS_H__ */
