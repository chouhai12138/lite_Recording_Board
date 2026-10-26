/**
 * @file    gravity.h
 * @brief   STM32 端精确重力补偿（板端前馈力矩）
 *
 * 这是 `/data/lite_motor_ws/src/lite_controller` 里
 * `gravity_compensator.cpp`（pinocchio rnea）的等价 MCU 移植：
 *
 *     τ_k = −(R_k·a_k) · Σ_{i∈distal(k)} [ (p_i − p_k)×(m_i·g) + (R_i·m_i·c_i)×g ]
 *           + offset_k
 *
 * - 参数与离线标定**同源**：`Scripts/model/inertia_params.yaml` 的
 *   (mass, first_moment) 成对使用；`mass: null` 取 1.0 但一阶矩照常参与。
 * - 夹爪两指是 wrist_pitch 的兄弟固定段（q=0），参与远端求和。
 * - 运动学常量由 `Scripts/gen_gravity_model.py` 生成到
 *   `App/cfg/gravity_model.h`；本模块无动态内存、无 pinocchio 依赖。
 * - 板端坐标：MIT/CSP 用电机 raw，而模型在 URDF 关节空间。
 *   `joint_map.h` 固化每关节 `direction` / `homing_offset` / 关节区间：
 *       q_i     = d_i · (raw_i − homing_offset_i) + k_i·2π
 *       τ_raw_i = d_i · τ_q_i
 *   `Gravity_TorqueRawF` 内部完成这两步；`Gravity_TorqueF` 仍收关节角 q。
 *   ⚠️ homing_offset 不能省：模板机 raw=0（自然下垂）时肩 roll 的 q≈∓89.5°。
 *   `k_i` 由 `Gravity_TorqueRawF` 按"离关节区间中点最近的一圈"自动折算。
 * - 精度：float 核心，与 pinocchio 双精度参考差 < 1e-3 N·m
 *   （标定本身的 holdout RMSE ≈ 0.09 N·m，远大于此）。
 *
 * 独立性：只依赖 `<math.h>` + `App/cfg/gravity_model.h` + `App/cfg/joint_map.h`，
 * 可直接复制到任何 Cortex-M 工程。
 */
#ifndef __GRAVITY_H__
#define __GRAVITY_H__

#ifdef __cplusplus
extern "C" {
#endif

/** 每臂转动关节数（= 电机 0..6 / 8..14）。 */
#define GRAV_DOF 7

/**
 * @brief 计算一侧 7 个臂关节的重力前馈力矩（float，MCU 主用）。
 * @param side 0 = 左臂，1 = 右臂
 * @param q    7 个关节角，rad，顺序 = shoulder_pitch..wrist_pitch
 * @param tau  输出 7 个关节空间前馈力矩，N·m（已含每关节 offset）
 */
void Gravity_TorqueF(int side, const float q[GRAV_DOF], float tau[GRAV_DOF]);

/**
 * @brief 板端 raw 版：入参是电机原始位置，输出可直接写进 MIT τ_ff。
 *
 * 内部按 `joint_map.h` 做：
 *     q = dir·(raw − homing_offset) + k·2π
 *     tau_raw = dir·tau_q
 * k 取"离关节区间中点最近的一圈"，抵消电机多圈。
 * ⚠️ raw=0 并不等于 q=0（自然下垂归零时两者差一个固定 homing_offset）。
 *
 * @param raw 7 个电机位置，rad（与 MIT/CSP 同一 raw 坐标）
 * @param tau 输出 7 个 raw 空间前馈力矩，N·m（已含 offset）
 */
void Gravity_TorqueRawF(int side, const float raw[GRAV_DOF], float tau[GRAV_DOF]);

/**
 * @brief 兼容旧接口（double）。内部仍按 float 计算，仅做类型转换。
 */
void Gravity_Torque(int side, const double q[GRAV_DOF], double tau[GRAV_DOF]);

#ifdef __cplusplus
}
#endif

#endif /* __GRAVITY_H__ */
