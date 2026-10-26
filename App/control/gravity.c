/**
 * @file    gravity.c
 * @brief   精确解析重力补偿：FK + 重力力矩投影（无 pinocchio、无动态内存）
 *
 * 与 `lite_controller/gravity_compensator.cpp` 的 `rnea(q,0,0)` 等价：
 *   τ_k = ∂U/∂q_k，  U = −Σ m_i·g·(p_i + R_i·c_i)
 * 对转动关节：
 *   τ_k = −(R_k a_k) · Σ_{i∈distal(k)} [ (p_i−p_k)×(m_i g) + (R_i m_i c_i)×g ]
 * 再逐关节加上标定的 offset_nm。
 *
 * 只依赖 App/cfg/gravity_model.h（运动学/惯性参数，由
 * Scripts/gen_gravity_model.py 生成）与 App/cfg/joint_map.h（raw→q 的
 * direction/homing_offset/限位，由 Scripts/gen_joint_map.py 生成）。
 */
#include <math.h>

#include "gravity.h"
#include "gravity_model.h"
#include "joint_map.h"

_Static_assert(GRAV_DOF == GRAV_JOINTS, "gravity.h 与 gravity_model.h 关节数不一致");
_Static_assert(GRAV_DOF == JOINT_MAP_DOF, "gravity.h 与 joint_map.h 关节数不一致");
_Static_assert(GRAV_SEGMENTS <= 16, "desc 位掩码是 uint16_t，段数不能超过 16");

/* ------------------------------------------------------------------------- */
/* 小型 3×3 / 3 维运算（行主序 9 个 float）                                    */
/* ------------------------------------------------------------------------- */
static const float kI3[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};
static const float kZ3[3] = {0.0f, 0.0f, 0.0f};

static void mat3_mul(float out[9], const float a[9], const float b[9])
{
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            out[r * 3 + c] = a[r * 3 + 0] * b[0 * 3 + c]
                           + a[r * 3 + 1] * b[1 * 3 + c]
                           + a[r * 3 + 2] * b[2 * 3 + c];
        }
    }
}

/** Rodrigues：绕单位轴 a 转 th。 */
static void rot_axis(float out[9], const float a[3], float th)
{
    const float c = cosf(th);
    const float s = sinf(th);
    const float C = 1.0f - c;
    const float x = a[0], y = a[1], z = a[2];

    out[0] = c + x * x * C;     out[1] = x * y * C - z * s; out[2] = x * z * C + y * s;
    out[3] = y * x * C + z * s; out[4] = c + y * y * C;     out[5] = y * z * C - x * s;
    out[6] = z * x * C - y * s; out[7] = z * y * C + x * s; out[8] = c + z * z * C;
}

/* ------------------------------------------------------------------------- */
/* 主算法                                                                     */
/* ------------------------------------------------------------------------- */
void Gravity_TorqueF(int side, const float q[GRAV_DOF], float tau[GRAV_DOF])
{
    const grav_seg_t *seg = (side == 0) ? kGravChainLeft : kGravChainRight;
    const float *off = (side == 0) ? kGravOffsetLeft : kGravOffsetRight;

    float R[GRAV_SEGMENTS][9];
    float p[GRAV_SEGMENTS][3];

    /* ---- 1) 正运动学：逐段算 R_i, p_i ---- */
    for (int i = 0; i < GRAV_SEGMENTS; i++) {
        const int par = seg[i].parent;
        const float *Rp = (par < 0) ? kI3 : R[par];
        const float *pp = (par < 0) ? kZ3 : p[par];

        float R0q[9];
        if (seg[i].qidx >= 0) {
            float Ra[9];
            rot_axis(Ra, seg[i].axis, q[seg[i].qidx]);
            mat3_mul(R0q, seg[i].R0, Ra);
        } else {
            for (int k = 0; k < 9; k++) {
                R0q[k] = seg[i].R0[k];
            }
        }
        mat3_mul(R[i], Rp, R0q);

        /* p = p_parent + R_parent * t */
        for (int r = 0; r < 3; r++) {
            p[i][r] = pp[r]
                    + Rp[r * 3 + 0] * seg[i].t[0]
                    + Rp[r * 3 + 1] * seg[i].t[1]
                    + Rp[r * 3 + 2] * seg[i].t[2];
        }
    }

    /* ---- 2) 重力矩：对每个转动关节，求远端所有段的力矩和 ---- */
    for (int k = 0; k < GRAV_SEGMENTS; k++) {
        const int qk = seg[k].qidx;
        if (qk < 0) {
            continue;
        }

        const uint16_t mask = seg[k].desc;
        float T0 = 0.0f, T1 = 0.0f;   /* T_z 恒为 0（g 只有 z 分量） */

        for (int i = 0; i < GRAV_SEGMENTS; i++) {
            if ((mask & (uint16_t)(1u << i)) == 0u) {
                continue;
            }
            const float *Ri = R[i];

            /* (p_i - p_k) × (m_i g)，g = (0,0,-G) */
            const float dp0 = p[i][0] - p[k][0];
            const float dp1 = p[i][1] - p[k][1];
            const float fz = -seg[i].mass * GRAV_G;
            T0 += dp1 * fz;
            T1 += -dp0 * fz;

            /* (R_i * mc_i) × g */
            const float mx = Ri[0] * seg[i].mc[0] + Ri[1] * seg[i].mc[1] + Ri[2] * seg[i].mc[2];
            const float my = Ri[3] * seg[i].mc[0] + Ri[4] * seg[i].mc[1] + Ri[5] * seg[i].mc[2];
            T0 += -my * GRAV_G;
            T1 += mx * GRAV_G;
        }

        /* z_k = R_k * axis_k；τ_k = −z_k·T + offset */
        const float *Rk = R[k];
        const float ax = seg[k].axis[0], ay = seg[k].axis[1], az = seg[k].axis[2];
        const float zx = Rk[0] * ax + Rk[1] * ay + Rk[2] * az;
        const float zy = Rk[3] * ax + Rk[4] * ay + Rk[5] * az;

        tau[qk] = -(zx * T0 + zy * T1) + off[qk];
    }
}

void Gravity_TorqueRawF(int side, const float raw[GRAV_DOF], float tau[GRAV_DOF])
{
    const joint_map_t *map = (side == 0) ? kArmMapLeft : kArmMapRight;
    float q[GRAV_DOF];
    float tq[GRAV_DOF];

    /* raw -> URDF 关节角：
     *     q = direction·(raw − homing_offset) + turn·2π
     * turn 取"离关节区间中点最近的一圈"，抵消电机多圈。
     * ⚠️ homing_offset 是模板机实测的固定偏差，不能省：
     *    模板机 raw=0（自然下垂）时肩 roll 的 q ≈ ∓89.5°。 */
    for (int i = 0; i < GRAV_DOF; i++) {
        const int8_t turn = JointMap_TurnFor(&map[i], raw[i]);
        q[i] = JointMap_ToJoint(&map[i], raw[i], turn);
    }

    Gravity_TorqueF(side, q, tq);

    /* 关节力矩 -> raw 力矩（MIT 的 τ_ff 在电机 raw 系） */
    for (int i = 0; i < GRAV_DOF; i++) {
        tau[i] = (float)map[i].direction * tq[i];
    }
}

void Gravity_Torque(int side, const double q[GRAV_DOF], double tau[GRAV_DOF])
{
    float qf[GRAV_DOF];
    float tf[GRAV_DOF];

    for (int i = 0; i < GRAV_DOF; i++) {
        qf[i] = (float)q[i];
    }
    Gravity_TorqueF(side, qf, tf);
    for (int i = 0; i < GRAV_DOF; i++) {
        tau[i] = (double)tf[i];
    }
}
