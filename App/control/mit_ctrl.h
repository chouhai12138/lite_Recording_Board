/**
 * @file    mit_ctrl.h
 * @brief   MIT 控制层：零力矩 / HOLD（带 0.5s Kp 斜坡）/ 重力拖动（先占位）
 *          100 Hz 周期由 main 调用 MitCtrl_Tick 驱动，并统计周期抖动。
 */
#ifndef __MIT_CTRL_H__
#define __MIT_CTRL_H__

#include <stdint.h>

#define MIT_PERIOD_MS       10U      /* 100 Hz */
#define MIT_RAMP_MS         500U     /* Kp 斜坡时长 */

typedef enum {
    MIT_MODE_ZERO = 0,      /* 零力矩：kp=kd=0，电机松劲 */
    MIT_MODE_HOLD,          /* 位置保持：kp/kd，目标=进入时的位置 */
    MIT_MODE_DRAG,          /* 零力拖动：kp=0，逐轴 kd（arms_grippers 表）+ 重力前馈 */
    MIT_MODE_COUNT
} mit_mode_t;

void        MitCtrl_Init(void);
void        MitCtrl_SetMode(mit_mode_t mode);
mit_mode_t  MitCtrl_Mode(void);
const char *MitCtrl_ModeName(mit_mode_t mode);
void        MitCtrl_DebugGravity(void);
void        MitCtrl_SetGains(double kp, double kd);
void        MitCtrl_SetDragKd(double kd);
void        MitCtrl_SetDragKdAt(uint8_t idx, double kd);
double      MitCtrl_DragKdAt(uint8_t idx);
double      MitCtrl_DragKd(void);
double      MitCtrl_Kp(void);
double      MitCtrl_Kd(void);

/** 主循环调用：到 10ms 就发一轮 MIT 帧 */
void MitCtrl_Tick(uint32_t now_ms);

/* 抖动统计（最近 256 拍） */
void     MitCtrl_GetJitter(uint32_t *min_ms, uint32_t *max_ms,
                           uint32_t *avg_x10, uint32_t *overruns);
void     MitCtrl_ResetJitter(void);
uint32_t MitCtrl_Ticks(void);

#endif /* __MIT_CTRL_H__ */
