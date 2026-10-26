/**
 * @file    sim_can.h
 * @brief   虚拟 RobStride 总线：16 台假电机（A3 阶段用，不接真电机也能跑完整逻辑）
 *
 * 行为对齐真实电机（可参考 lite_driver/protocol.hpp）：
 *   · 类型 1（MIT）：更新目标位置，回一帧类型 2 状态
 *   · 类型 3/4：使能/失能，回状态
 *   · 类型 6：写零（raw 归零），回状态
 *   · 类型 17：读参数（MECH_POS / RUN_MODE）
 *   · 类型 22/24：回状态
 *
 * 可注入：掉线（不回帧）、故障位（带在状态帧的 extra 高字节）。
 */
#ifndef __SIM_CAN_H__
#define __SIM_CAN_H__

#include <stdint.h>
#include "rs_protocol.h"

void Sim_Init(void);
/** 推进虚拟电机物理（10 ms 一拍） */
void Sim_Tick(void);
/** 处理一帧下发；返回 1 表示已被虚拟总线消费 */
int  Sim_HandleTx(const rs_can_frame_t *frame);
/** 取一帧虚拟应答；没有返回 0 */
int  Sim_RxPop(rs_can_frame_t *frame);

/* --- 测试注入 --- */
void Sim_SetOffline(uint8_t id, int offline);
void Sim_SetFault(uint8_t id, uint8_t fault_bits);
void Sim_MoveTarget(uint8_t id, double pos_rad);
int  Sim_GetPos(uint8_t id, double *pos);
int  Sim_GetEnabled(uint8_t id);

#endif /* __SIM_CAN_H__ */
