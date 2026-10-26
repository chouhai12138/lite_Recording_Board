#!/usr/bin/env python3
"""从 URDF + 重力辨识参数生成 STM32 端重力补偿模型（精确解析式，非拟合）。

这是 `/data/lite_motor_ws` 里 `lite_controller/gravity_compensator.cpp` 的
**等价移植**：那里把 yaml 的 (mass, first_moment) 成对写回 pinocchio 模型再
`rnea(q,0,0)`；这里把同一条运动链固化成 C 常量表，运行时用解析式算
`τ = ∂U/∂q`，不依赖 pinocchio / 动态内存 / 浮点大库。

算法（与 gravity.c 完全对应）：
  1. FK：沿链算每段的 R_i, p_i（节拍内）。
  2. 对每个转动关节 k：
        T_k = Σ_{i∈distal(k)} [ (p_i−p_k)×(m_i·g) + (R_i·m_i·c_i)×g ]
        τ_k = −(R_k·a_k)·T_k + offset_k
  3. 夹爪两根手指是 wrist_pitch 的**兄弟**固定段（q=0），参与远端力矩求和。

用法：
    python3 Scripts/gen_gravity_model.py
    python3 Scripts/gen_gravity_model.py --urdf ... --yaml ...

产出：
    App/cfg/gravity_model.h   链常量 + 惯性参数 + 每关节 offset（static const，独立）
    Tests/gravity_ref.h       200 组随机位形的 pinocchio rnea 参考（主机单测用）
"""
import argparse
import os
import re
import sys

import numpy as np
import pinocchio as pin
import yaml

# 每臂 7 个臂关节，顺序 = 电机 0..6（左）/ 7..13（右）
JOINT_SUFFIX = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                "elbow_pitch", "wrist_yaw", "wrist_roll", "wrist_pitch")
SIDES = ("left", "right")
G = 9.81          # 与 pinocchio 默认重力一致
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_model(urdf, ypath):
    """按 gravity_compensator.cpp 的方式注入标定参数。

    ⚠️ 三条关键约定（旧生成器在这里踩过坑）：
      · mass: null 的连杆取 1.0，**但 first_moment 仍然必须参与**；
        旧脚本把 m=0 时 mc 也清零，肩 pitch 的一阶矩被丢掉。
      · mc 直接就是关节系下的一阶矩，不要再乘/除 g，也不要加 URDF
        <inertial><origin> 的旋转 —— 标定参数本来就在 pinocchio 关节系。
      · 惯量张量置 0 不影响重力（只与 mass / mc 有关）。
    """
    calib = yaml.safe_load(open(ypath))["links"]
    m = pin.buildModelFromUrdf(urdf)
    for jid, name in enumerate(m.names):
        if name not in calib or calib[name].get("first_moment") is None:
            continue
        mass = calib[name].get("mass")
        mass = 1.0 if mass is None else float(mass)
        mc = np.array(calib[name]["first_moment"], dtype=float)
        m.inertias[jid] = pin.Inertia(mass, mc / mass, np.zeros((3, 3)))
    return m, calib


def axes_from_urdf(path):
    """URDF <axis>（关节系），归一化。"""
    txt = open(path).read()
    out = {}
    for mm in re.finditer(r'<joint name="([^"]+)" type="(revolute|prismatic)">(.*?)</joint>',
                          txt, re.S):
        am = re.search(r'<axis xyz="([^"]+)"', mm.group(3))
        a = np.array([float(v) for v in (am.group(1).split() if am else ("1", "0", "0"))])
        out[mm.group(1)] = a / np.linalg.norm(a)
    return out


def build_chain(model, side):
    """返回 [(jid, parent_seg, qidx)]。

    臂链从 wrist_pitch 沿 parents 往上走再反转；夹爪两段是 wrist_pitch 的
    兄弟（不是串在腕后面的），所以 parent 指回 wrist_pitch 那一段。
    """
    ids = []
    j = model.getJointId(f"{side}_wrist_pitch")
    while j != 0:
        ids.append(j)
        j = int(model.parents[j])
    ids.reverse()

    segs = [(jid, i - 1, i) for i, jid in enumerate(ids)]
    wrist_seg = len(ids) - 1
    for gname in (f"{side}_gripper", f"{side}_gripper_passive_joint"):
        jid = model.getJointId(gname)
        parent_jid = int(model.parents[jid])
        assert parent_jid == ids[wrist_seg], (gname, model.names[parent_jid])
        segs.append((jid, wrist_seg, -1))
    return segs


def desc_masks(segs):
    n = len(segs)
    children = [[] for _ in range(n)]
    for i, (_jid, par, _q) in enumerate(segs):
        if par >= 0:
            children[par].append(i)

    masks = [0] * n

    def walk(i):
        m = 1 << i
        for c in children[i]:
            m |= walk(c)
        masks[i] = m
        return m

    for i in range(n):
        if segs[i][1] < 0:
            walk(i)
    return masks


def analytic_torque(segs, masks, model, axes, calib, q7):
    """与 gravity.c 同构的参考实现（用于生成前自检）。"""
    n = len(segs)
    R = [None] * n
    p = [None] * n
    for i, (jid, par, _qidx) in enumerate(segs):
        R0 = np.array(model.jointPlacements[jid].rotation)
        t = np.array(model.jointPlacements[jid].translation)
        Rp = np.eye(3) if par < 0 else R[par]
        pp = np.zeros(3) if par < 0 else p[par]
        name = model.names[jid]
        if any(name == f"{side_}_{s}" for side_ in SIDES for s in JOINT_SUFFIX):
            k = next(k for k, s in enumerate(JOINT_SUFFIX) if name.endswith("_" + s))
            a = axes[name]
            th = q7[k]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            Ra = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
            R[i] = Rp @ R0 @ Ra
        else:
            R[i] = Rp @ R0
        p[i] = pp + Rp @ t

    g = np.array([0.0, 0.0, -G])
    tau = np.zeros(7)
    for k, (jid, _par, qidx) in enumerate(segs):
        if qidx < 0:
            continue
        T = np.zeros(3)
        for i in range(n):
            if not (masks[k] & (1 << i)):
                continue
            mass = model.inertias[segs[i][0]].mass
            mc = mass * np.array(model.inertias[segs[i][0]].lever)
            T += np.cross(p[i] - p[k], mass * g) + np.cross(R[i] @ mc, g)
        z = R[k] @ axes[model.names[jid]]
        tau[qidx] = -np.dot(z, T)
    return tau


def fmt_row(v, per_line):
    return ", ".join(f"{x:+.6f}f" for x in v)


def emit_header(model, segs_by_side, masks_by_side, calib, out_path, meta):
    L = []
    L.append("/* 由 Scripts/gen_gravity_model.py 自动生成，勿手改。")
    L.append(" *")
    L.append(" * 精确解析重力补偿模型（不是拟合）：")
    L.append(" *   τ_k = −(R_k·a_k)·Σ_{i∈distal(k)}[(p_i−p_k)×(m_i·g)+(R_i·m_i·c_i)×g] + offset_k")
    L.append(" * 参数来源：lite_controller/config/gravity/inertia_params.yaml")
    L.append(" * 运动学：lite_arm_model.urdf（纯 URDF）")
    L.append(" * 注：raw→q 的 direction/homing_offset 在 App/cfg/joint_map.h（gen_joint_map.py）")
    if meta:
        L.append(" * generated: " + meta)
    L.append(" */")
    L.append("#ifndef __GRAVITY_MODEL_H__")
    L.append("#define __GRAVITY_MODEL_H__")
    L.append("")
    L.append("#include <stdint.h>")
    L.append("")
    L.append("#define GRAV_JOINTS   7   /* 每臂转动关节数 */")
    L.append("#define GRAV_SEGMENTS 9   /* 7 转动 + 2 夹爪固定段 */")
    L.append("#define GRAV_G        9.81f")
    L.append("")
    L.append("typedef struct {")
    L.append("    float R0[9];       /* 父系 -> 本关节系(q=0)，行主序 */")
    L.append("    float t[3];        /* 平移，父系 */")
    L.append("    float axis[3];     /* 转轴，关节系（单位向量） */")
    L.append("    float mass;        /* kg */")
    L.append("    float mc[3];       /* 一阶矩 m*c，关节系，kg*m */")
    L.append("    int8_t qidx;       /* 0..6 = 臂关节, -1 = 固定段 */")
    L.append("    int8_t parent;     /* 父段下标, -1 = 世界 */")
    L.append("    uint16_t desc;     /* 后代位掩码（含自身） */")
    L.append("} grav_seg_t;")
    L.append("")

    for side in SIDES:
        tag = side.capitalize()
        L.append(f"static const grav_seg_t kGravChain{tag}[GRAV_SEGMENTS] = {{")
        for i, (jid, par, qidx) in enumerate(segs_by_side[side]):
            R0 = np.array(model.jointPlacements[jid].rotation).reshape(-1)
            t = np.array(model.jointPlacements[jid].translation)
            name = model.names[jid]
            ax = (np.array(axes_model[name]) if name in axes_model else np.zeros(3))
            mass = model.inertias[jid].mass
            mc = mass * np.array(model.inertias[jid].lever)
            L.append("    /* %-2d %-28s */" % (i, name))
            L.append("    {{%s}, {%s}, {%s}, %+.6ff, {%s}, %d, %d, 0x%04X}," % (
                ", ".join(f"{x:+.6f}f" for x in R0),
                ", ".join(f"{x:+.6f}f" for x in t),
                ", ".join(f"{x:+.6f}f" for x in ax),
                mass,
                ", ".join(f"{x:+.6f}f" for x in mc),
                qidx, par, masks_by_side[side][i]))
        L.append("};")
        L.append("")

    for side in SIDES:
        tag = side.capitalize()
        offs = [float(yaml_off[f"{side}_{s}"]) for s in JOINT_SUFFIX]
        L.append(f"static const float kGravOffset{tag}[GRAV_JOINTS] = {{")
        L.append("    " + ", ".join(f"{v:+.6f}f" for v in offs) + ",")
        L.append("};")
        L.append("")

    L.append("#endif /* __GRAVITY_MODEL_H__ */")
    with open(out_path, "w") as f:
        f.write("\n".join(L) + "\n")


def emit_reference(model, calib, qs, refs, out_path):
    L = ["/* 由 Scripts/gen_gravity_model.py 生成：pinocchio rnea(q,0,0)+offset 参考 */",
         "#ifndef __GRAVITY_REF_H__", "#define __GRAVITY_REF_H__", "",
         '#include "gravity.h"', "",
         f"#define GRAV_SAMPLES {len(qs)}", "",
         f"static const double kRefQ[{len(qs)}][GRAV_DOF] = {{"]
    L += ["    {" + ", ".join(f"{v:+.9f}" for v in q) + "}," for q in qs]
    for side in SIDES:
        tag = side.capitalize()
        L += ["};", "", f"static const double kRefTau{tag}[{len(qs)}][GRAV_DOF] = {{"]
        L += ["    {" + ", ".join(f"{v:+.9f}" for v in t) + "}," for t in refs[side]]
    L += ["};", "", "#endif"]
    with open(out_path, "w") as f:
        f.write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(ROOT, "Scripts/model/lite_arm_model.urdf"))
    ap.add_argument("--yaml", default=os.path.join(ROOT, "Scripts/model/inertia_params.yaml"))
    ap.add_argument("--out", default=ROOT)
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--qrange", type=float, default=2.0)
    a = ap.parse_args()

    global axes_model, yaml_off
    model, calib = build_model(a.urdf, a.yaml)
    axes_model = axes_from_urdf(a.urdf)
    yaml_off = yaml.safe_load(open(a.yaml))["offsets_nm"]

    segs_by_side, masks_by_side = {}, {}
    for side in SIDES:
        segs_by_side[side] = build_chain(model, side)
        masks_by_side[side] = desc_masks(segs_by_side[side])

    # 自检：解析式 == pinocchio rnea
    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(300):
        q7 = rng.uniform(-2.0, 2.0, 7)
        for side in SIDES:
            q = pin.neutral(model)
            for k, s in enumerate(JOINT_SUFFIX):
                q[model.idx_qs[model.getJointId(f"{side}_{s}")]] = q7[k]
            r = pin.rnea(model, model.createData(), q, np.zeros(model.nv), np.zeros(model.nv))
            ref = np.array([r[model.idx_vs[model.getJointId(f"{side}_{s}")]]
                            for s in JOINT_SUFFIX])
            ref += np.array([float(yaml_off[f"{side}_{s}"]) for s in JOINT_SUFFIX])
            got = analytic_torque(segs_by_side[side], masks_by_side[side],
                                  model, axes_model, calib, q7)
            got += np.array([float(yaml_off[f"{side}_{s}"]) for s in JOINT_SUFFIX])
            worst = max(worst, float(np.max(np.abs(got - ref))))
    print(f"自检：解析式 vs pinocchio rnea 最大差 = {worst:.3e} N·m")
    if worst > 1e-9:
        print("❌ 解析式与 pinocchio 不一致，拒绝生成", file=sys.stderr)
        return 1

    # 生成参考向量
    d = model.createData()
    rng = np.random.default_rng(a.seed)
    qs, refs = [], {s: [] for s in SIDES}
    for _ in range(a.samples):
        q7 = rng.uniform(-a.qrange, a.qrange, 7)
        qs.append(q7)
        for side in SIDES:
            q = pin.neutral(model)
            for k, s in enumerate(JOINT_SUFFIX):
                q[model.idx_qs[model.getJointId(f"{side}_{s}")]] = q7[k]
            r = pin.rnea(model, d, q, np.zeros(model.nv), np.zeros(model.nv))
            tau = np.array([r[model.idx_vs[model.getJointId(f"{side}_{s}")]]
                            for s in JOINT_SUFFIX])
            tau += np.array([float(yaml_off[f"{side}_{s}"]) for s in JOINT_SUFFIX])
            refs[side].append(tau)

    os.makedirs(os.path.join(a.out, "App/cfg"), exist_ok=True)
    os.makedirs(os.path.join(a.out, "Tests"), exist_ok=True)
    meta = os.path.basename(a.yaml)
    emit_header(model, segs_by_side, masks_by_side, calib,
                os.path.join(a.out, "App/cfg/gravity_model.h"), meta)
    emit_reference(model, calib, qs, refs, os.path.join(a.out, "Tests/gravity_ref.h"))
    print(f"生成：App/cfg/gravity_model.h（{len(segs_by_side['left'])} 段/侧）")
    print(f"生成：Tests/gravity_ref.h（{a.samples} 组）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
