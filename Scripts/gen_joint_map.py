#!/usr/bin/env python3
"""把模板机的 raw→关节角映射固化成 STM32 头文件。

公式（与 ROS 侧 lite_driver 一致）：
    q = direction · (raw − homing_offset)          # 单圈量
    trq_raw = direction · tau_joint                # MIT effort 在 raw 系

多圈：`homing_offset` 是单圈量。运行期用关节区间 [lower, upper] 选
"离区间中点最近的一圈"并锁存（与 lite_driver 的 turn folding 同构）：
    turn = round((center − naive) / 2π)
    q    = naive + turn · 2π

输入（已拷到本工作区，离线可复现）：
    Scripts/model/calibration.yaml    homing_offset / direction
    Scripts/model/joint_spec.yaml     lower / upper（仅用于整圈折算）
产出：
    App/cfg/joint_map.h

用法：
    python3 Scripts/gen_joint_map.py
"""
import argparse
import os
import sys

import yaml

SUF = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
       "elbow_pitch", "wrist_yaw", "wrist_roll", "wrist_pitch")
SIDES = ("left", "right")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=os.path.join(ROOT, "Scripts/model/calibration.yaml"))
    ap.add_argument("--spec", default=os.path.join(ROOT, "Scripts/model/joint_spec.yaml"))
    ap.add_argument("--out", default=os.path.join(ROOT, "App/cfg/joint_map.h"))
    a = ap.parse_args()

    cal = yaml.safe_load(open(a.calib))
    spec = yaml.safe_load(open(a.spec))["joints"]

    rows = {s: [] for s in SIDES}
    warnings = []
    for side in SIDES:
        for s in SUF:
            name = f"{side}_{s}"
            if name not in cal:
                raise KeyError(f"{a.calib} 缺少 {name}")
            if name not in spec:
                raise KeyError(f"{a.spec} 缺少 {name}")
            c = cal[name]
            if "homing_offset" not in c:
                raise KeyError(f"{name} 缺少 homing_offset")
            direction = int(c["direction"])
            if direction not in (-1, 1):
                raise ValueError(f"{name}: direction={direction} 非法")
            # 标定文件的 direction 与 URDF/joint_spec 必须一致
            spec_dir = spec[name].get("direction")
            if spec_dir is not None and int(spec_dir) != direction:
                warnings.append(
                    f"{name}: calibration.direction={direction} != joint_spec.direction={spec_dir}")
            lo = float(spec[name]["lower_limit"])
            hi = float(spec[name]["upper_limit"])
            if not (hi - lo) < 2.0 * 3.141592653589793:
                warnings.append(f"{name}: 行程 {hi - lo:.3f} rad 不窄于 2π，整圈折算不唯一")
            rows[side].append((direction, float(c["homing_offset"]), lo, hi))

    L = []
    L.append("/* 由 Scripts/gen_joint_map.py 自动生成，勿手改。")
    L.append(" *")
    L.append(" * 电机 raw → URDF 关节角（模板机实测，硬编码）：")
    L.append(" *     q       = direction·(raw − homing_offset) + turn·2π")
    L.append(" *     trq_raw = direction·tau_joint")
    L.append(" *")
    L.append(" * 数据来源：")
    L.append(" *   Scripts/model/calibration.yaml   (homing_offset / direction)")
    L.append(" *   Scripts/model/joint_spec.yaml    (lower/upper，仅用于整圈折算)")
    L.append(" *")
    L.append(" * ⚠️ 换机器人 / 重新归零后必须重新生成，否则重力前馈会用错 q。")
    L.append(" */")
    L.append("#ifndef __JOINT_MAP_H__")
    L.append("#define __JOINT_MAP_H__")
    L.append("")
    L.append("#include <math.h>")
    L.append("#include <stdint.h>")
    L.append("")
    L.append("#define JOINT_MAP_DOF     7")
    L.append("#define JOINT_MAP_TWO_PI  6.283185307179586f")
    L.append("")
    L.append("typedef struct {")
    L.append("    int8_t direction;      /* ±1，URDF direction */")
    L.append("    float  homing_offset;  /* 单圈 raw 零点（raw→q 的固定偏差） */")
    L.append("    float  lower;          /* 关节下限，仅用于整圈折算 */")
    L.append("    float  upper;          /* 关节上限 */")
    L.append("} joint_map_t;")
    L.append("")

    for side in SIDES:
        tag = side.capitalize()
        L.append(f"static const joint_map_t kArmMap{tag}[JOINT_MAP_DOF] = {{")
        for (d, h, lo, hi), s in zip(rows[side], SUF):
            L.append("    {%+d, %+.9ff, %+.9ff, %+.9ff},   /* %s_%s */"
                     % (d, h, lo, hi, side, s))
        L.append("};")
        L.append("")

    L += [
        "/* 单圈：q = direction·(raw − homing_offset) */",
        "static inline float JointMap_Naive(const joint_map_t *m, float raw)",
        "{",
        "    return (float)m->direction * (raw - m->homing_offset);",
        "}",
        "",
        "/* 选离关节区间中点最近的一圈（与 lite_driver 的 turn folding 同构）。",
        " * 上电后对每个关节**只锁存一次**，不要逐周期重算。 */",
        "static inline int8_t JointMap_TurnFor(const joint_map_t *m, float raw)",
        "{",
        "    const float naive  = JointMap_Naive(m, raw);",
        "    const float center = 0.5f * (m->lower + m->upper);",
        "    return (int8_t)lroundf((center - naive) / JOINT_MAP_TWO_PI);",
        "}",
        "",
        "/* raw → q（带锁存好的整圈数） */",
        "static inline float JointMap_ToJoint(const joint_map_t *m, float raw, int8_t turn)",
        "{",
        "    return JointMap_Naive(m, raw) + (float)turn * JOINT_MAP_TWO_PI;",
        "}",
        "",
        "/* q → raw（单圈；回放/命令侧如需 q 反算 raw 用） */",
        "static inline float JointMap_ToRaw(const joint_map_t *m, float q)",
        "{",
        "    return m->homing_offset + (float)m->direction * q;",
        "}",
        "",
        "/* 关节系力矩 → 电机 raw 系 effort（MIT 帧里用的就是 raw 系） */",
        "static inline float JointMap_TorqueToRaw(const joint_map_t *m, float tau_joint)",
        "{",
        "    return (float)m->direction * tau_joint;",
        "}",
        "",
        "#endif /* __JOINT_MAP_H__ */",
    ]

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write("\n".join(L) + "\n")

    for w in warnings:
        print("⚠️ " + w, file=sys.stderr)
    print(f"生成：{os.path.relpath(a.out, ROOT)}（{len(rows['left'])} 关节/侧）")
    for side in SIDES:
        print(f"  {side}: " + ", ".join(
            f"dir={d:+d}/off={h:+.4f}" for d, h, _, _ in rows[side]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
