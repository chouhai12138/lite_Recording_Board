/**
 * @file    mit_ctrl.c
 * @brief   MIT 控制律（100 Hz）：零力矩 / HOLD / 拖动 + 安全钳位 + 抖动统计
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "mit_ctrl.h"
#include "motor_mgr.h"
#include "gravity.h"
#include "rs_protocol.h"

typedef struct {
    double   pos;          /* HOLD 目标 */
    uint32_t ramp_t0;
} ctrl_state_t;

static mit_mode_t   s_mode = MIT_MODE_ZERO;
static double       s_kp = 30.0;
static double       s_kd = 1.0;

/* arms_grippers 默认阻尼（与 ROS hold_controller 的 arms_grippers 表一致）：
 *   0..6   左臂 shoulder_pitch..wrist_pitch
 *   7      左夹爪
 *   8..14  右臂
 *   15     右夹爪 */
static const double kDragKdDefault[MOTOR_COUNT] = {
    1.5, 0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.0,
    1.5, 0.6, 0.6, 0.5, 0.4, 0.4, 0.4, 0.0,
};
static double       s_drag_kd[MOTOR_COUNT];   /* 逐轴拖动阻尼，初值见上表 */
static ctrl_state_t s_state[MOTOR_COUNT];

static uint32_t s_next_ms;
static uint32_t s_last_ms;
static uint32_t s_ticks;

/* 重力前馈：精确解析模型（与 pinocchio rnea 同源） */
#ifndef GRAVITY_FF_ENABLE
#define GRAVITY_FF_ENABLE 1      /* 精确解析式：max_err ≈7e-5 N·m（200 组参考） */
#endif

/* 电机表顺序（见 MotorMgr_Init）：
 *   0..6   左臂 ID 11..17
 *   7      左夹爪 ID 18
 *   8..14  右臂 ID 21..27
 *   15     右夹爪 ID 28
 * 重力补偿只覆盖两侧臂的 7 个转动关节。⚠️ 右臂起点是 8，不是 7。 */
#define ARM_LEFT_IDX0    0U
#define ARM_RIGHT_IDX0   8U
#define ARM_JOINTS_PER   7U

#define JIT_WIN 256U
static uint8_t  s_jit_ring[JIT_WIN];
static uint16_t s_jit_idx;
static uint16_t s_jit_filled;

void MitCtrl_Init(void)
{
    memset(s_state, 0, sizeof(s_state));
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        s_drag_kd[i] = kDragKdDefault[i];
    }
    s_mode    = MIT_MODE_ZERO;
    s_next_ms = HAL_GetTick();
    s_last_ms = 0;
    s_ticks   = 0;
    MitCtrl_ResetJitter();
}

void MitCtrl_ResetJitter(void)
{
    memset(s_jit_ring, 0, sizeof(s_jit_ring));
    s_jit_idx    = 0;
    s_jit_filled = 0;
}

void MitCtrl_GetJitter(uint32_t *min_ms, uint32_t *max_ms,
                       uint32_t *avg_x10, uint32_t *overruns)
{
    uint32_t mn = 0xFFFFFFFFU, mx = 0, sum = 0, over = 0;

    for (uint16_t i = 0; i < s_jit_filled; i++) {
        uint32_t dt = s_jit_ring[i];
        if (dt < mn) { mn = dt; }
        if (dt > mx) { mx = dt; }
        if (dt > MIT_PERIOD_MS + 2U) { over++; }
        sum += dt;
    }
    if (min_ms != NULL)   { *min_ms = (s_jit_filled > 0U) ? mn : 0U; }
    if (max_ms != NULL)   { *max_ms = mx; }
    if (avg_x10 != NULL)  { *avg_x10 = (s_jit_filled > 0U) ? (sum * 10U / s_jit_filled) : 0U; }
    if (overruns != NULL) { *overruns = over; }
}

static void capture_targets(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        const motor_t *m = MotorMgr_Get(i);
        s_state[i].pos     = (m != NULL) ? m->pos : 0.0;
        s_state[i].ramp_t0 = now;
    }
}

/** 回放用的是 CSP，切回 MIT 前必须把 run_mode 改回来，否则 MIT 帧被忽略 */
static void ensure_mit_mode(void)
{
    /* 使能状态下写 run_mode 会被忽略，必须先失能 */
    MotorMgr_SetRunModeAll(RS_RUN_MODE_MIT, 0);
}

void MitCtrl_SetMode(mit_mode_t mode)
{
    if (mode >= MIT_MODE_COUNT) {
        return;
    }
    ensure_mit_mode();

    if (mode == MIT_MODE_ZERO) {
        /* 零力矩也要"使能"才收 MIT 帧，否则 kd 阻尼根本发不出去 */
        MotorMgr_Scan();
        MotorMgr_EnableAll(1);
        printf("ZERO 使能 %u 台（在线 %u，Kd=%.2f）\r\n",
               (unsigned)MotorMgr_EnabledCount(), (unsigned)MotorMgr_OnlineCount(),
               MitCtrl_DragKd());
    } else if (mode == MIT_MODE_HOLD) {
        /* HOLD 必须能读到位置，否则拒绝 */
        if (MotorMgr_OnlineCount() == 0U) {
            printf("HOLD 拒绝：没有在线电机\\r\\n");
            return;
        }
        MotorMgr_EnableAll(1);
        capture_targets();
    } else if (mode == MIT_MODE_DRAG) {
        MotorMgr_Scan();
        MotorMgr_EnableAll(1);
        capture_targets();
        printf("DRAG 使能 %u 台（在线 %u）\r\n",
               (unsigned)MotorMgr_EnabledCount(), (unsigned)MotorMgr_OnlineCount());
    }
    s_mode = mode;
}

mit_mode_t MitCtrl_Mode(void)
{
    return s_mode;
}

const char *MitCtrl_ModeName(mit_mode_t mode)
{
    switch (mode) {
        case MIT_MODE_ZERO: return "ZERO";
        case MIT_MODE_HOLD: return "HOLD";
        case MIT_MODE_DRAG: return "DRAG";
        default:            return "?";
    }
}

/* 诊断：按当前姿态打印重力前馈力矩（raw 空间，可直接对照 MIT 帧） */
void MitCtrl_DebugGravity(void)
{
    float raw7[GRAV_DOF], tau7[GRAV_DOF];

    for (int side = 0; side < 2; side++) {
        const char *tag = (side == 0) ? "left " : "right";
        const uint8_t base = (side == 0) ? ARM_LEFT_IDX0 : ARM_RIGHT_IDX0;

        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            const motor_t *m = MotorMgr_Get((uint8_t)(base + k));
            raw7[k] = (m != NULL) ? (float)m->pos : 0.0f;
        }
        Gravity_TorqueRawF(side, raw7, tau7);

        printf("  raw_%s:", tag);
        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            printf(" %+.3f", raw7[k]);
        }
        printf("\r\n");

        printf("  tau_%s:", tag);
        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            printf(" %+.2f", tau7[k]);
        }
        printf(" N·m\r\n");

        /* 电机反馈力矩（raw）。HOLD 静态下≈真实重力负载，可与 tau 对照 */
        printf("  fb_%s :", tag);
        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            const motor_t *m = MotorMgr_Get((uint8_t)(base + k));
            printf(" %+.2f", (m != NULL) ? m->torque : 0.0);
        }
        printf(" N·m\r\n");
    }
}

void MitCtrl_SetDragKd(double kd)
{
    if (kd < 0.0) { kd = 0.0; }
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        s_drag_kd[i] = kd;
    }
}

void MitCtrl_SetDragKdAt(uint8_t idx, double kd)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }
    if (kd < 0.0) { kd = 0.0; }
    s_drag_kd[idx] = kd;
}

double MitCtrl_DragKdAt(uint8_t idx)
{
    return (idx < MOTOR_COUNT) ? s_drag_kd[idx] : 0.0;
}

double MitCtrl_DragKd(void)
{
    return s_drag_kd[0];
}

void MitCtrl_SetGains(double kp, double kd)
{
    if (kp < 0.0) { kp = 0.0; }
    if (kd < 0.0) { kd = 0.0; }
    s_kp = kp;
    s_kd = kd;
}

double MitCtrl_Kp(void)
{
    return s_kp;
}

double MitCtrl_Kd(void)
{
    return s_kd;
}

uint32_t MitCtrl_Ticks(void)
{
    return s_ticks;
}

void MitCtrl_Tick(uint32_t now_ms)
{
    double kp_scale = 1.0;

    if ((int32_t)(now_ms - s_next_ms) < 0) {
        return;
    }

    /* ---- 抖动统计（环形窗口） ---- */
    if (s_last_ms != 0U) {
        uint32_t dt = now_ms - s_last_ms;
        if (dt > 255U) {
            dt = 255U;                       /* 异常长的间隔饱和到 255ms */
        }
        s_jit_ring[s_jit_idx] = (uint8_t)dt;
        s_jit_idx = (uint16_t)((s_jit_idx + 1U) % JIT_WIN);
        if (s_jit_filled < JIT_WIN) {
            s_jit_filled++;
        }
    }
    s_last_ms = now_ms;
    s_next_ms += MIT_PERIOD_MS;
    if ((int32_t)(now_ms - s_next_ms) > 50) {   /* 落后太多就重新对齐，别补拍 */
        s_next_ms = now_ms + MIT_PERIOD_MS;
    }
    s_ticks++;

#if GRAVITY_FF_ENABLE
    /* DRAG 模式的重力前馈每拍**每侧只算一次**，而不是每个电机各算一遍。 */
    float tau_l[GRAV_DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float tau_r[GRAV_DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    if (s_mode == MIT_MODE_DRAG) {
        float raw7[GRAV_DOF];
        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            const motor_t *mk = MotorMgr_Get(k);
            raw7[k] = (mk != NULL) ? (float)mk->pos : 0.0f;
        }
        Gravity_TorqueRawF(0, raw7, tau_l);
        for (uint8_t k = 0; k < GRAV_DOF; k++) {
            const motor_t *mk = MotorMgr_Get((uint8_t)(ARM_RIGHT_IDX0 + k));
            raw7[k] = (mk != NULL) ? (float)mk->pos : 0.0f;
        }
        Gravity_TorqueRawF(1, raw7, tau_r);
    }
#endif

    /* ---- 控制律 ---- */
    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        const motor_t *m = MotorMgr_Get(i);
        double pos = 0.0, vel = 0.0, trq = 0.0, kp = 0.0, kd = 0.0;

        if (m == NULL || !m->online) {
            continue;
        }

        switch (s_mode) {
            case MIT_MODE_ZERO:
                kp = 0.0;
                kd = s_drag_kd[i];              /* 零力矩 + 逐轴阻尼，防止自由下坠 */
                break;

            case MIT_MODE_HOLD: {
                uint32_t dt = now_ms - s_state[i].ramp_t0;
                if (dt < MIT_RAMP_MS) {
                    kp_scale = (double)dt / (double)MIT_RAMP_MS;
                } else {
                    kp_scale = 1.0;
                }
                pos = s_state[i].pos;
                vel = 0.0;
                kp  = s_kp * kp_scale;
                kd  = s_kd * kp_scale;
                break;
            }

            case MIT_MODE_DRAG:
                kp = 0.0;
                kd = s_drag_kd[i];       /* 逐轴阻尼，初值见 kDragKdDefault */
#if GRAVITY_FF_ENABLE
                /* 左臂 0..6，右臂 8..14（7 是左夹爪、15 是右夹爪；
                 * 每侧每拍只算一次，见上方） */
                if (i < ARM_JOINTS_PER) {
                    trq = (double)tau_l[i - ARM_LEFT_IDX0];
                } else if (i >= ARM_RIGHT_IDX0 && i < ARM_RIGHT_IDX0 + ARM_JOINTS_PER) {
                    trq = (double)tau_r[i - ARM_RIGHT_IDX0];
                }
#endif
                break;

            default:
                break;
        }

        /* 安全：NaN / 力矩钳位 */
        if (isnan(pos)) {
            pos = m->pos;
        }
        if (isnan(kp)) {
            kp = 0.0;
        }
        if (fabs(trq) > 12.0) {          /* 肩 roll 的重力前馈就有 ~6 N·m，5 太紧 */
            trq = (trq > 0.0) ? 12.0 : -12.0;
        }

        MotorMgr_SendMit(i, pos, vel, kp, kd, trq);
    }
}
