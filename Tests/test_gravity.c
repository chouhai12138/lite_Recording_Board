/**
 * @file    test_gravity.c
 * @brief   板端重力补偿 vs pinocchio rnea 参考（主机 gcc 单测）
 *
 * 参考由 Scripts/gen_gravity_model.py 生成：
 *   pinocchio rnea(q,0,0) 使用与 gravity_compensator.cpp 相同的参数注入
 *   （mass:null → 1.0，first_moment 照常参与），再逐关节加 offsets_nm。
 *
 * 三条路径都测：
 *   1. Gravity_TorqueF     —— 关节空间（q 直接来自参考）；
 *   2. Gravity_TorqueRawF  —— 板端 raw 空间（含 homing_offset）：
 *          raw = homing_offset + dir·q,  τ_raw = dir·τ_q
 *   3. 多圈：raw + 2π 应得到同样的 τ_raw（整圈折算）。
 *
 * 判据：float 核心与 double 参考的最大误差 < 1e-3 N·m。
 * （标定本身的 holdout RMSE ≈ 0.09 N·m，1e-3 已远小于模型不确定度。）
 */
#include <math.h>
#include <stdio.h>

#include "gravity.h"
#include "gravity_ref.h"
#include "joint_map.h"

#define GRAV_TOL_NM 1.0e-3

static const joint_map_t *map_for(int side)
{
    return (side == 0) ? kArmMapLeft : kArmMapRight;
}

int main(void)
{
    int fails = 0;
    double max_err = 0.0;

    for (int k = 0; k < GRAV_SAMPLES; k++) {
        double tl[GRAV_DOF], tr[GRAV_DOF];

        Gravity_Torque(0, kRefQ[k], tl);
        Gravity_Torque(1, kRefQ[k], tr);

        for (int i = 0; i < GRAV_DOF; i++) {
            const double e1 = fabs(tl[i] - kRefTauLeft[k][i]);
            const double e2 = fabs(tr[i] - kRefTauRight[k][i]);
            if (e1 > max_err) { max_err = e1; }
            if (e2 > max_err) { max_err = e2; }
            if (e1 > GRAV_TOL_NM || e2 > GRAV_TOL_NM) {
                if (fails < 5) {
                    printf("  FAIL q k=%d i=%d  left C=%+.6f ref=%+.6f | right C=%+.6f ref=%+.6f\n",
                           k, i, tl[i], kRefTauLeft[k][i], tr[i], kRefTauRight[k][i]);
                }
                fails++;
            }
        }

        for (int side = 0; side < 2; side++) {
            const joint_map_t *map = map_for(side);
            const double *ref = (side == 0) ? kRefTauLeft[k] : kRefTauRight[k];
            float raw[GRAV_DOF], raw_turn[GRAV_DOF];
            float traw[GRAV_DOF], traw_turn[GRAV_DOF];

            /* raw = homing_offset + dir·q（q 来自参考） */
            for (int i = 0; i < GRAV_DOF; i++) {
                raw[i]      = JointMap_ToRaw(&map[i], (float)kRefQ[k][i]);
                raw_turn[i] = raw[i] + JOINT_MAP_TWO_PI;   /* 人为多转一圈 */
            }
            Gravity_TorqueRawF(side, raw, traw);
            Gravity_TorqueRawF(side, raw_turn, traw_turn);

            for (int i = 0; i < GRAV_DOF; i++) {
                const double want = (double)map[i].direction * ref[i];
                const double e  = fabs((double)traw[i] - want);
                const double et = fabs((double)traw_turn[i] - want);
                if (e > max_err) { max_err = e; }
                if (et > max_err) { max_err = et; }
                if (e > GRAV_TOL_NM || et > GRAV_TOL_NM) {
                    if (fails < 5) {
                        printf("  FAIL raw side=%d k=%d i=%d  C=%+.6f turn=%+.6f ref=%+.6f\n",
                               side, k, i, (double)traw[i], (double)traw_turn[i], want);
                    }
                    fails++;
                }
            }
        }
    }

    printf("gravity: %d samples, max_err=%.3e N·m (tol %.0e), failures=%d\n",
           GRAV_SAMPLES, max_err, GRAV_TOL_NM, fails);
    return (fails == 0) ? 0 : 1;
}
