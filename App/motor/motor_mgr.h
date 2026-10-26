/**
 * @file    motor_mgr.h
 * @brief   16 轴电机管理：使能/失能/写零/HOLD、反馈快照、超时降级
 *
 * 现在跑在虚拟总线上（Sim/），接口与将来接真实 CAN 完全一致。
 */
#ifndef __MOTOR_MGR_H__
#define __MOTOR_MGR_H__

#include <stdint.h>
#include "rs_protocol.h"

#define MOTOR_COUNT         16U
#define MOTOR_TIMEOUT_MS    200U

typedef struct {
    uint8_t  id;            /* CAN ID：1..16 */
    uint8_t  bus;           /* 0 = CAN1（前 8 轴），1 = CAN2（后 8 轴） */
    uint8_t  online;
    uint8_t  enabled;
    uint8_t  fault_bits;
    double   pos;           /* rad（raw） */
    double   vel;           /* rad/s */
    double   torque;        /* N·m */
    double   temp;          /* °C */
    double   cmd_pos;       /* HOLD 目标 */
    uint32_t last_rx_ms;
    const rs_limits_t *lim; /* 该轴型号量程（rs-02/rs-00/rs-05）；编码/解码都用它 */
} motor_t;

void MotorMgr_Init(void);

/** 主循环调用：10ms 控制节拍（HOLD 发帧 + 超时检测） */
void MotorMgr_Tick(uint32_t now_ms);

/** 收到一帧总线数据 */
void MotorMgr_OnRx(const rs_can_frame_t *frame, void *user);

/** 向所有电机发一次"读位置"（类型 17） */
void MotorMgr_Scan(void);
/** 只发读位置请求、不等应答（录制的 100Hz 采样用） */
void MotorMgr_RequestPos(void);
/** 读所有电机的某个参数（比如 0x7005 run_mode），结果直接打印出来 */
void MotorMgr_RequestParamAll(uint16_t pid);
/** 向所有电机发一次零力矩 MIT 帧（类型 1），用回包刷状态。
 *  ⚠️ 已使能的电机会因此变成零力矩（松劲），扫描时务必先失能。 */
void MotorMgr_PollZeroMit(void);

void MotorMgr_EnableAll(uint8_t on);
void MotorMgr_Enable(uint8_t idx, uint8_t on);
void MotorMgr_ZeroAll(void);
/** 完整归零：失能 → 类型 6 写零 → 等待 → 回读校验；返回超差的轴数 */
int  MotorMgr_ZeroAllVerify(void);
void MotorMgr_Hold(uint8_t idx);
/** 直接发一帧 MIT（控制层用） */
void MotorMgr_SendMit(uint8_t idx, double pos, double vel,
                      double kp, double kd, double trq);
void MotorMgr_SendParamU8(uint8_t idx, uint16_t pid, uint8_t val);
void MotorMgr_SendParamF32(uint8_t idx, uint16_t pid, float val);
void MotorMgr_SendLocRef(uint8_t idx, float rad);
/** 全机切换运行模式：先失能 → 写 run_mode → 等生效 → 重新使能。
 *  （使能状态下写 run_mode 会被电机忽略，这是之前 CSP/MIT 切不动的根因） */
void MotorMgr_SetRunModeAll(uint8_t run_mode, uint8_t reenable);
/** 指定目标位置（rad，raw）——录制/回放/调试用 */
void MotorMgr_MoveTo(uint8_t idx, double pos_rad);

const motor_t *MotorMgr_Get(uint8_t idx);

/* --- 探测：向 id 区间发 GetDeviceId（类型 0），收集应答里的 UID --- */
typedef struct {
    uint8_t id;
    uint8_t bus;
    uint8_t uid[8];
} motor_disc_t;

void    MotorMgr_Discover(uint8_t from, uint8_t to);
/** 用最近一次探测结果重建电机表（id / 总线 / 数量） */
uint8_t MotorMgr_Adopt(void);
/** 当前电机表里在用几台（默认 16，adopt 后变成实际数量） */
uint8_t MotorMgr_Count(void);
uint32_t MotorMgr_UnknownCount(void);
uint8_t MotorMgr_FirstId(void);
uint8_t MotorMgr_LastId(void);
/** 打开后把收到的每一帧原始打印出来（协议调试） */
void MotorMgr_SetSniff(uint8_t on);
/** 临时关闭 200ms 超时失能（CSP 回放期间用，避免没有状态帧被误判掉线） */
void MotorMgr_SetTimeoutEnabled(uint8_t on);
void    MotorMgr_DiscClear(void);
uint8_t MotorMgr_DiscCount(void);
const motor_disc_t *MotorMgr_DiscAt(uint8_t i);
uint8_t MotorMgr_OnlineCount(void);
uint8_t MotorMgr_EnabledCount(void);

#endif /* __MOTOR_MGR_H__ */
