#!/usr/bin/env python3
"""⚠️ 已弃用（DEPRECATED）—— 保留仅作对照。

早期"rnea 采样 + 基函数拟合"方案（max_err ≈ 1.3 N·m），已被
`Scripts/gen_gravity_model.py` 的**精确解析模型**取代；后者与 pinocchio
rnea 一致到 1e-14 N·m，且不依赖基函数拟合。

用 pinocchio rnea 采样，把 τ(q) 拟合成低阶基函数（供 MCU 做乘加）。

基函数（每关节 78 项）：
    1, sin(qk), cos(qk) (k=0..6),
    sin*sin, sin*cos, cos*cos (j<k)

产出：
    App/cfg/gravity_fit.h   系数表（float）
    Tests/gravity_ref.h     留出集的 q 与 pinocchio 期望力矩
"""
import argparse
import itertools
import numpy as np
import pinocchio as pin
import yaml

NJ = 7
NAMES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw",
         "elbow_pitch", "wrist_yaw", "wrist_roll", "wrist_pitch")


def build_model(urdf, ypath):
    calib = yaml.safe_load(open(ypath))["links"]
    m = pin.buildModelFromUrdf(urdf)
    for jid, name in enumerate(m.names):
        if name in calib and calib[name].get("first_moment"):
            mm = calib[name].get("mass") or 0.0
            fm = np.array(calib[name]["first_moment"], dtype=float)
            m.inertias[jid] = pin.Inertia(mm, fm / mm if mm > 1e-9 else np.zeros(3),
                                          np.zeros((3, 3)))
    return m


def chain_idx(m, side):
    ids, j = [], m.getJointId(f"{side}_wrist_pitch")
    while j != 0:
        ids.append(j)
        j = int(m.parents[j])
    ids.reverse()
    return ids


def features(q):
    """基函数顺序必须与 App/control/gravity.c 完全一致。"""
    s = np.sin(q)
    c = np.cos(q)
    f = [1.0]
    f.extend(s)
    f.extend(c)
    for j, k in itertools.combinations(range(NJ), 2):
        f.append(s[j] * s[k]); f.append(s[j] * c[k])
        f.append(c[j] * s[k]); f.append(c[j] * c[k])
    for j, k, l in itertools.combinations(range(NJ), 3):
        for a in (s, c):
            for b in (s, c):
                for d in (s, c):
                    f.append(a[j] * b[k] * d[l])
    return np.array(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--qrange", type=float, default=1.6)
    ap.add_argument("--fit-samples", type=int, default=4000)
    ap.add_argument("--test-samples", type=int, default=200)
    ap.add_argument("--out", default=".")
    a = ap.parse_args()

    m = build_model(a.urdf, a.yaml)
    d = m.createData()
    chains = {side: chain_idx(m, side) for side in ("left", "right")}

    def tau(side, q7):
        q = np.zeros(m.nq)
        for k, jid in enumerate(chains[side]):
            q[m.joints[jid].idx_q] = q7[k]
        r = pin.rnea(m, d, q, np.zeros(m.nv), np.zeros(m.nv))
        return np.array([r[m.joints[jid].idx_v] for jid in chains[side]])

    rng = np.random.default_rng(2024)
    Q = rng.uniform(-a.qrange, a.qrange, (a.fit_samples, NJ))
    coef = {}
    for side in ("left", "right"):
        F = np.array([features(q) for q in Q])
        T = np.array([tau(side, q) for q in Q])
        coef[side] = np.linalg.lstsq(F, T, rcond=None)[0]      # (78, 7)

    # 留出集
    Qt = rng.uniform(-a.qrange, a.qrange, (a.test_samples, NJ))
    err = {"left": 0.0, "right": 0.0}
    ref = {"left": [], "right": []}
    for q in Qt:
        for side in ("left", "right"):
            t = tau(side, q)
            p = features(q) @ coef[side]
            err[side] = max(err[side], float(np.max(np.abs(t - p))))
            ref[side].append(t)
    print(f"拟合残差（max |τ_fit - τ_rnea|）：left={err['left']:.2e}  right={err['right']:.2e} N·m")

    # C 头文件
    lines = ["/* 由 Scripts/fit_gravity.py 生成：τ(q) = Σ coef·basis(q)，基函数见 gravity.c */",
             "#ifndef __GRAVITY_FIT_H__", "#define __GRAVITY_FIT_H__", "",
             f"#define GRAV_DOF    {NJ}", f"#define GRAV_NBASIS {coef['left'].shape[0]}", ""]
    for si, side in enumerate(("left", "right")):
        lines.append(f"static const float kGravFit{side.capitalize()}[GRAV_DOF][GRAV_NBASIS] = {{")
        for j in range(NJ):
            row = ", ".join(f"{v:+.6f}f" for v in coef[side][:, j])
            lines.append("    {" + row + "},")
        lines.append("};")
        lines.append("")
    lines.append("#endif")

    import os
    os.makedirs(os.path.join(a.out, "App/cfg"), exist_ok=True)
    os.makedirs(os.path.join(a.out, "Tests"), exist_ok=True)
    open(os.path.join(a.out, "App/cfg/gravity_fit.h"), "w").write("\n".join(lines) + "\n")

    rl = ["/* 由 Scripts/fit_gravity.py 生成：pinocchio 留出集 */",
          "#ifndef __GRAVITY_REF_H__", "#define __GRAVITY_REF_H__", "",
          f"#define GRAV_SAMPLES {a.test_samples}", "",
          f"static const double kRefQ[{a.test_samples}][{NJ}] = {{"]
    rl += ["    {" + ", ".join(f"{v:+.9f}" for v in q) + "}," for q in Qt]
    rl += ["};", ""]
    for side in ("left", "right"):
        rl.append(f"static const double kRefTau{side.capitalize()}[{a.test_samples}][{NJ}] = {{")
        rl += ["    {" + ", ".join(f"{v:+.9f}" for v in t) + "}," for t in ref[side]]
        rl.append("};")
        rl.append("")
    rl.append("#endif")
    open(os.path.join(a.out, "Tests/gravity_ref.h"), "w").write("\n".join(rl) + "\n")
    print("生成：App/cfg/gravity_fit.h, Tests/gravity_ref.h")


if __name__ == "__main__":
    main()
