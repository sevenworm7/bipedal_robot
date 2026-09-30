"""Offline checks of a model version (no simulator): stance, CoM, static torques, reach, tail effect.

Run after build_parts.py (needs reports/mass_properties.json):
    .venv_cad\\Scripts\\python.exe 01_robot_construction\\model\\generator\\check_static.py --version 1

Writes model/v<N>/reports/static_check.md and static_check.json. Checks:
1. Stance: for the given knee bend, hip_pitch / ankle_pitch with the sole flat and the whole-body CoM
   above the foot centre (the tail pulls the CoM back, so the ankles end up behind the hips).
2. Static joint torques: double support, single support (torso rolled over the stance foot, tail at
   0 / +-30 / +-60 deg), swing leg held up, tail held level; compared with the 6 V stall / rated torque.
3. Reach: step length and foot lift with the sole flat at the stance hip height, within the joint limits.
4. Tail: static lateral CoM shift per tail_yaw angle; dynamic ZMP shift of a sinusoidal tail swing.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402

G = 9.81


def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    x, y, z = axis
    if (x, y, z) == (1, 0, 0):
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if (x, y, z) == (0, 1, 0):
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Body:
    def __init__(self, lay: P.Layout, mp: dict):
        self.lay = lay
        self.links = mp["links"]
        self.origin = {k: np.array(v["origin_world_mm"]) for k, v in self.links.items()}
        self.mass = {k: v["mass_g"] / 1000 for k, v in self.links.items()}   # kg
        self.M = sum(self.mass.values())

    def fk(self, q: dict, base_R=np.eye(3), base_p=None):
        """World rotation / position of each link frame. q in deg; base frame at the hip_roll midpoint."""
        base_p = self.origin["base"] if base_p is None else base_p
        T = {"base": (base_R, np.asarray(base_p, float))}
        for j in self.lay.joints:
            Rp, pp = T[j.parent]
            off = self.origin[j.child] - self.origin[j.parent]
            R = Rp @ rot(j.axis, q.get(j.name, 0.0))
            T[j.child] = (R, pp + Rp @ off)
        return T

    def coms(self, T):
        return {k: T[k][0] @ np.array(self.links[k]["com_mm"]) + T[k][1] for k in self.links}

    def com(self, T):
        c = self.coms(T)
        return sum(self.mass[k] * c[k] for k in c) / self.M

    def descendants(self, link):
        out = {link}
        changed = True
        while changed:
            changed = False
            for j in self.lay.joints:
                if j.parent in out and j.child not in out:
                    out.add(j.child)
                    changed = True
        return out

    def torque(self, T, joint_name, bodies):
        """|moment about the joint axis| (N*m) of gravity acting on `bodies`."""
        j = self.lay.joint(joint_name)
        R, p = T[j.child]
        axis = R @ np.array(j.axis, float)
        c = self.coms(T)
        tau = sum((np.cross((c[k] - p) / 1000, np.array([0, 0, -self.mass[k] * G])) for k in bodies), np.zeros(3))
        return float(abs(axis @ tau))


def sole_height(body: Body, T, side):
    """Lowest z of the sole plate corners of one foot (mm)."""
    fp = body.lay.p["legs"]["foot"]
    R, p = T[f"{side}_foot"]
    z_an = body.lay.joint(f"{side}_ankle_pitch").origin[2]
    zs = []
    for dx in (-fp["length"] / 2, fp["length"] / 2):
        for dy in (-fp["width"] / 2, fp["width"] / 2):
            zs.append((R @ np.array([dx, dy, -z_an]) + p)[2])
    return min(zs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", type=int, required=True)
    ap.add_argument("--knee", type=float, default=None, help="stance knee bend (deg); default params.stance.knee")
    args = ap.parse_args()
    vdir = P.version_dir(args.version)
    lay = P.load(args.version)
    mp = json.loads((vdir / "reports/mass_properties.json").read_text(encoding="utf-8"))
    b = Body(lay, mp)
    p = lay.p
    sv = p["servo"]
    stall, rated = sv["torque_stall_6v"], sv["torque_rated_6v"]
    rep: dict = {"version": args.version, "total_mass_kg": b.M}
    md = [f"# 離線靜態檢查（v{args.version}）", "",
          f"由 `generator/check_static.py` 產生；質量來自 `reports/mass_properties.json`（git {mp['git_commit']}）。"
          f"扭矩比較基準：6 V 堵轉 {stall} N·m／額定 {rated} N·m。", ""]

    # ---------------------------------------------------------- 1. stance
    knee = args.knee if args.knee is not None else p["stance"]["knee"]

    def pose(h, k):
        q = {}
        for s in P.SIDES:
            q.update({f"{s}_hip_pitch": h, f"{s}_knee": k, f"{s}_ankle_pitch": -(h + k)})
        return q

    def com_minus_ankle(h):
        T = b.fk(pose(h, knee))
        return b.com(T)[0] - T["left_foot"][1][0]

    lo, hi = -60.0, 30.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if com_minus_ankle(lo) * com_minus_ankle(mid) <= 0:
            hi = mid
        else:
            lo = mid
    h0 = (lo + hi) / 2
    q0 = pose(h0, knee)
    T0 = b.fk(q0)
    lift = sole_height(b, T0, "left")
    T0 = b.fk(q0, base_p=b.origin["base"] - np.array([0, 0, lift]))
    com0 = b.com(T0)
    base_h = T0["base"][1][2]
    ankle_x = T0["left_foot"][1][0]
    rep["stance"] = {"hip_pitch": h0, "knee": knee, "ankle_pitch": -(h0 + knee), "base_height_mm": base_h,
                     "com_mm": com0.tolist(), "ankle_x_mm": ankle_x}
    tc = math.sqrt(com0[2] / 1000 / G)
    md += ["## 1. 站姿", "",
           f"- 膝彎 {knee:.0f}° 時，腳掌平貼且全身重心在腳掌中心正上方：**hip_pitch {h0:.1f}°、knee {knee:.0f}°、ankle_pitch {-(h0 + knee):.1f}°**。",
           f"- base（兩髖 roll 軸中點）離地 {base_h:.1f} mm；全身重心 ({com0[0]:.1f}, {com0[1]:.1f}, {com0[2]:.1f}) mm；"
           f"腳踝在髖關節後方 {-ankle_x:.1f} mm（恐龍式站姿）。",
           f"- 倒單擺時間常數 T_c = √(z/g) = {tc:.3f} s；單步時間取 2–3 T_c → 步態頻率（左右各一步為一週期）"
           f" {1 / (2 * 3 * tc):.2f}–{1 / (2 * 2 * tc):.2f} Hz。", ""]
    rep["Tc_s"] = tc

    # ---------------------------------------------------------- 2. torques
    rows = []
    legs = {s: b.descendants(f"{s}_hip") for s in P.SIDES}
    all_links = set(b.links)

    def prox(joint):
        return all_links - b.descendants(lay.joint(joint).child)

    # double support: each leg carries roughly half of the moment of the upper body
    for role in P.LEG_ROLES:
        jn = f"left_{role}"
        upper = all_links - legs["left"] - legs["right"]
        t_up = b.torque(T0, jn, upper) / 2
        t_leg = b.torque(T0, jn, prox(jn) & legs["left"])
        rows.append(("雙腳站立", role, t_up + t_leg))
    # single support on the left foot: foot flat, no ankle roll -> the stance leg stays vertical in the
    # frontal plane and the torso rolls about the left hip (both hip_roll joints move together).
    # Static balance needs the CoM inside the sole: at least `margin` mm inboard of its inner edge.
    fw = p["legs"]["foot"]["width"]
    margin = 10.0
    swing = pose(h0, knee)
    swing.update({"right_hip_pitch": h0 - 25, "right_knee": knee + 50, "right_ankle_pitch": -(h0 + knee + 25)})

    def single_pose(ty, r):
        qq = dict(swing, tail_yaw=ty, left_hip_roll=r, right_hip_roll=r)
        Tl = b.fk(qq, base_R=rot((1, 0, 0), -r))
        inner = Tl["left_foot"][1][1] - fw / 2
        return b.com(Tl)[1] - inner, Tl

    single = {}
    for ty in (0, -15, -30, -45, -60):
        need = None
        for r in np.arange(0.0, 25.01, 0.25):          # hip_roll = +r tilts the torso toward the stance (left) side
            m_, Tl = single_pose(ty, r)
            if m_ >= margin:
                need = r
                break
        m0, _ = single_pose(ty, 0.0)
        r_used = need if need is not None else 25.0
        _, Tl = single_pose(ty, r_used)
        single[ty] = {"com_inside_inner_edge_mm_no_roll": m0, "torso_roll_deg": need,
                      "tau": {role: b.torque(Tl, f"left_{role}", prox(f"left_{role}")) for role in P.LEG_ROLES},
                      "swing_hip_pitch": b.torque(Tl, "right_hip_pitch", b.descendants("right_thigh")),
                      "swing_knee": b.torque(Tl, "right_knee", b.descendants("right_shin"))}
    for role in P.LEG_ROLES:
        rows.append(("單腳支撐（尾巴置中）", role, single[0]["tau"][role]))
    rows.append(("單腳支撐（尾巴擺向支撐側 30°）", "hip_roll", single[-30]["tau"]["hip_roll"]))
    rows.append(("擺動腳懸空", "hip_pitch", single[0]["swing_hip_pitch"]))
    rows.append(("擺動腳懸空", "knee", single[0]["swing_knee"]))
    for tp in (0, 30, -30):
        Tt = b.fk(dict(q0, tail_pitch=tp))
        rows.append((f"尾巴 pitch {tp}°", "tail_pitch", b.torque(Tt, "tail_pitch", {"tail_pitch_link"})))
    rep["torques"] = [{"case": c, "joint": j, "Nm": t, "pct_stall": t / stall, "pct_rated": t / rated} for c, j, t in rows]
    rep["single_support"] = {str(k): v for k, v in single.items()}
    md += ["## 2. 靜態扭矩", "",
           "| 情境 | 關節 | 扭矩 (N·m) | 佔堵轉 | 佔額定 |", "|---|---|---|---|---|"]
    md += [f"| {c} | {j} | {t:.3f} | {t / stall:.0%} | {t / rated:.0%} |" for c, j, t in rows]
    md += ["", f"單腳支撐（左腳站、右腳抬起；沒有 ankle roll，腳掌平貼 → 支撐腿在正面維持垂直，靠 hip_roll 讓軀幹往支撐側傾）。"
           f"靜態站穩的條件是全身重心落在腳掌內緣以內至少 {margin:.0f} mm（腳掌內緣距身體中線 "
           f"{p['legs']['hip_spacing'] / 2 - fw / 2:.1f} mm）：", "",
           "| tail_yaw (deg) | 軀幹不傾斜時重心在內緣內側 (mm) | 需要的軀幹側傾 (deg) | 支撐側 hip_roll 扭矩 (N·m) |", "|---|---|---|---|"]
    for ty, v in single.items():
        rr = v["torso_roll_deg"]
        md.append(f"| {ty:+d} | {v['com_inside_inner_edge_mm_no_roll']:+.1f} | {'> 25（做不到）' if rr is None else f'{rr:.1f}'} | "
                  f"{v['tau']['hip_roll']:.3f} |")
    md += ["", "tail_yaw 負值 = 尾尖往左（+Y），也就是擺向左側的支撐腳。軀幹側傾以 hip_roll 做到，上限 ±25°。", ""]

    # ---------------------------------------------------------- 3. reach
    lg = p["legs"]
    L1, L2 = lg["thigh"], lg["shin"]
    jt = p["joints"]
    hip_to_ankle = T0["left_thigh"][1][2] - T0["left_foot"][1][2]
    best_step, best_lift = 0.0, 0.0
    for hp in np.arange(jt["hip_pitch"]["lower"], jt["hip_pitch"]["upper"] + 0.01, 0.5):
        for kn in np.arange(max(5, jt["knee"]["lower"]), jt["knee"]["upper"] + 0.01, 0.5):
            an = -(hp + kn)
            if not jt["ankle_pitch"]["lower"] <= an <= jt["ankle_pitch"]["upper"]:
                continue
            x = -L1 * math.sin(math.radians(hp)) - L2 * math.sin(math.radians(hp + kn))
            z = L1 * math.cos(math.radians(hp)) + L2 * math.cos(math.radians(hp + kn))
            if abs(z - hip_to_ankle) < 0.5:
                best_step = max(best_step, abs(x - (T0["left_foot"][1][0] - T0["left_thigh"][1][0])))
            if abs(x - (T0["left_foot"][1][0] - T0["left_thigh"][1][0])) < 1.0:
                best_lift = max(best_lift, hip_to_ankle - z)
    need = {f"{v} m/s @ {fq} Hz": v / (2 * fq) * 1000 for v in (0.1, 0.2, 0.3) for fq in (1.2, 1.5, 2.0)}
    rep["reach"] = {"hip_to_ankle_mm": hip_to_ankle, "max_step_mm": best_step, "max_lift_mm": best_lift,
                    "needed_step_mm": need}
    md += ["## 3. 可達範圍（腳掌保持水平、站姿髖高）", "",
           f"- 髖 pitch 軸到踝軸的垂直距離 {hip_to_ankle:.1f} mm。",
           f"- 腳掌相對站姿位置可前後移動 **±{best_step:.0f} mm**；原地抬腳最高 **{best_lift:.0f} mm**（v0.1 要求 30–50 mm）。",
           "- 各速度與步頻需要的單步位移（= v ÷ 2f）："]
    md += [f"  - {k}: {v:.0f} mm {'✓' if v <= best_step else '✗ 超出'}" for k, v in need.items()]
    md.append("")

    # ---------------------------------------------------------- 4. tail
    tl_m = b.mass["tail_pitch_link"] + b.mass["tail_yaw_link"]
    rows = []
    for ty in (15, 30, 45, 60):
        Tt = b.fk(dict(q0, tail_yaw=-ty))
        rows.append((ty, b.com(Tt)[1] - com0[1]))
    md += ["## 4. 尾巴", "",
           f"- 尾巴兩節合計 {tl_m * 1000:.0f} g（佔全身 {tl_m / b.M:.0%}）。",
           "- 尾巴往左擺（tail_yaw 負值）時，全身重心的靜態側移："]
    md += [f"  - {ty}°：{dy:+.1f} mm" for ty, dy in rows]
    md += [f"- 以步頻擺動時，ZMP 側移約是靜態值的 (1 + ω²·z/g) 倍：1.5 Hz 時 ×{1 + (2 * math.pi * 1.5) ** 2 * com0[2] / 1000 / G:.1f}。",
           f"- 單腳支撐需要把 ZMP 移到支撐腳中心，也就是 {lg['hip_spacing'] / 2:.0f} mm。", ""]
    rep["tail_static_shift_mm"] = {str(ty): dy for ty, dy in rows}

    (vdir / "reports/static_check.md").write_text("\n".join(md), encoding="utf-8")
    (vdir / "reports/static_check.json").write_text(json.dumps(rep, indent=2, default=float), encoding="utf-8")
    print("\n".join(md))


if __name__ == "__main__":
    main()
