/* 由 Scripts/gen_joint_map.py 自动生成，勿手改。
 *
 * 电机 raw → URDF 关节角（模板机实测，硬编码）：
 *     q       = direction·(raw − homing_offset) + turn·2π
 *     trq_raw = direction·tau_joint
 *
 * 数据来源：
 *   Scripts/model/calibration.yaml   (homing_offset / direction)
 *   Scripts/model/joint_spec.yaml    (lower/upper，仅用于整圈折算)
 *
 * ⚠️ 换机器人 / 重新归零后必须重新生成，否则重力前馈会用错 q。
 */
#ifndef __JOINT_MAP_H__
#define __JOINT_MAP_H__

#include <math.h>
#include <stdint.h>

#define JOINT_MAP_DOF     7
#define JOINT_MAP_TWO_PI  6.283185307179586f

typedef struct {
    int8_t direction;      /* ±1，URDF direction */
    float  homing_offset;  /* 单圈 raw 零点（raw→q 的固定偏差） */
    float  lower;          /* 关节下限，仅用于整圈折算 */
    float  upper;          /* 关节上限 */
} joint_map_t;

static const joint_map_t kArmMapLeft[JOINT_MAP_DOF] = {
    {-1, +0.055644454f, -3.141592654f, +0.785398163f},   /* left_shoulder_pitch */
    {-1, -1.561794568f, -1.570796327f, +1.919862177f},   /* left_shoulder_roll */
    {+1, -1.153013497f, -1.570796327f, +1.570796327f},   /* left_shoulder_yaw */
    {-1, -0.101785128f, -2.356194490f, +0.000000000f},   /* left_elbow_pitch */
    {+1, +0.996926188f, -1.570796327f, +1.570796327f},   /* left_wrist_yaw */
    {-1, -0.051773432f, -0.698131701f, +0.698131701f},   /* left_wrist_roll */
    {-1, -0.275741462f, -0.785398163f, +0.785398163f},   /* left_wrist_pitch */
};

static const joint_map_t kArmMapRight[JOINT_MAP_DOF] = {
    {+1, -0.046440289f, -3.141592654f, +0.785398163f},   /* right_shoulder_pitch */
    {-1, +1.558918266f, -1.919862177f, +1.570796327f},   /* right_shoulder_roll */
    {+1, -0.310832343f, -1.570796327f, +1.570796327f},   /* right_shoulder_yaw */
    {-1, +0.161036945f, +0.000000000f, +2.356194490f},   /* right_elbow_pitch */
    {+1, +0.214572111f, -1.570796327f, +1.570796327f},   /* right_wrist_yaw */
    {+1, -0.076701380f, -0.698131701f, +0.698131701f},   /* right_wrist_roll */
    {-1, -0.051006418f, -0.785398163f, +0.785398163f},   /* right_wrist_pitch */
};

/* 单圈：q = direction·(raw − homing_offset) */
static inline float JointMap_Naive(const joint_map_t *m, float raw)
{
    return (float)m->direction * (raw - m->homing_offset);
}

/* 选离关节区间中点最近的一圈（与 lite_driver 的 turn folding 同构）。
 * 上电后对每个关节**只锁存一次**，不要逐周期重算。 */
static inline int8_t JointMap_TurnFor(const joint_map_t *m, float raw)
{
    const float naive  = JointMap_Naive(m, raw);
    const float center = 0.5f * (m->lower + m->upper);
    return (int8_t)lroundf((center - naive) / JOINT_MAP_TWO_PI);
}

/* raw → q（带锁存好的整圈数） */
static inline float JointMap_ToJoint(const joint_map_t *m, float raw, int8_t turn)
{
    return JointMap_Naive(m, raw) + (float)turn * JOINT_MAP_TWO_PI;
}

/* q → raw（单圈；回放/命令侧如需 q 反算 raw 用） */
static inline float JointMap_ToRaw(const joint_map_t *m, float q)
{
    return m->homing_offset + (float)m->direction * q;
}

/* 关节系力矩 → 电机 raw 系 effort（MIT 帧里用的就是 raw 系） */
static inline float JointMap_TorqueToRaw(const joint_map_t *m, float tau_joint)
{
    return (float)m->direction * tau_joint;
}

#endif /* __JOINT_MAP_H__ */
