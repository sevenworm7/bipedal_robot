"""Build every part of a model version: print files, per-link visual meshes, mass properties, interference check.

Run (CAD environment, from the project root):
    .venv_cad\\Scripts\\python.exe 01_robot_construction\\model\\generator\\build_parts.py --version 1

Outputs in model/v<N>/:
    parts/step/*.step, parts/stl/*.stl    one file per unique printed part (left-side geometry, link frame)
    meshes/<link>.stl                     visual mesh per URDF link (printed parts + servos), link frame, mm
    reports/mass_properties.json          mass, CoM, inertia per link (consumed by build_urdf.py)
    reports/parts_bom.md                  part list with volume, mass, print notes
    reports/interference.md               joint sweeps incl. screw heads and connector keep-outs
    reports/servo_map.md / .json          servo ID -> joint, horn direction, position-to-angle sign
A version with a FROZEN file (written when it is tagged) is never rebuilt.
"""
from __future__ import annotations

import argparse
import itertools
import math
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np
from build123d import Location, Pos, Rot, export_step, export_stl

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402
from parts import Builder, PartSpec, box  # noqa: E402


# ---------------------------------------------------------------- kinematics
def joint_loc(j: P.Joint, deg: float) -> Location:
    ax = {(1, 0, 0): (deg, 0, 0), (0, 1, 0): (0, deg, 0), (0, 0, 1): (0, 0, deg)}[tuple(j.axis)]
    return Pos(*j.origin) * Rot(*ax) * Pos(*(-c for c in j.origin))


def link_locations(lay: P.Layout, q: dict[str, float]) -> dict[str, Location]:
    """World placement of every link's zero-pose geometry for joint angles q (deg)."""
    loc = {"base": Location()}
    for j in lay.joints:                           # joints are listed parent-before-child
        loc[j.child] = loc[j.parent] * joint_loc(j, q.get(j.name, 0.0))
    return loc


# ---------------------------------------------------------------- mass properties
def part_mass(spec: PartSpec, rho: float) -> float:
    return spec.mass if spec.mass is not None else spec.solid.volume * rho / 1000.0


def inertia_of(spec: PartSpec, mass: float) -> tuple[np.ndarray, np.ndarray]:
    """CoM (mm, world) and inertia about the CoM (g*mm^2), scaled from the uniform-density solid."""
    vol = spec.solid.volume
    com = np.array(tuple(spec.solid.center()))
    inertia = np.array(spec.solid.matrix_of_inertia) * (mass / vol)
    return com, inertia


def box_inertia(size, mass):
    a, b, c = size
    return np.diag([mass * (b * b + c * c) / 12, mass * (a * a + c * c) / 12, mass * (a * a + b * b) / 12])


def combine(items):
    """items: (mass, com, inertia_about_com) -> total mass, com, inertia about the total com."""
    m = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / m
    inertia = np.zeros((3, 3))
    for mi, ci, Ii in items:
        d = ci - com
        inertia += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, inertia


# ---------------------------------------------------------------- interference
def span(lay, joint, n=None, step=5):
    """Sample angles of a joint over its limits: every `step` deg, or n evenly spaced values (ends included)."""
    j = lay.joint(joint)
    if n:
        return tuple(round(j.lower + (j.upper - j.lower) * k / (n - 1), 1) for k in range(n))
    out = list(range(int(math.ceil(j.lower)), int(math.floor(j.upper)) + 1, step))
    return tuple(sorted(set(out) | {j.lower, j.upper}))


def leg_sweeps(lay):
    """(parent link, child link, {joint: angles}) sampled from the joint limits in params.yaml (left leg; the
    right leg is its mirror image)."""
    return [
        ("base", "left_hip", {"left_hip_roll": span(lay, "left_hip_roll")}),
        ("left_hip", "left_thigh", {"left_hip_pitch": span(lay, "left_hip_pitch")}),
        ("left_thigh", "left_shin", {"left_knee": span(lay, "left_knee")}),
        ("left_shin", "left_foot", {"left_ankle_pitch": span(lay, "left_ankle_pitch")}),
        ("base", "left_thigh", {"left_hip_roll": span(lay, "left_hip_roll", 5), "left_hip_pitch": span(lay, "left_hip_pitch", 6)}),
        ("left_hip", "left_shin", {"left_hip_pitch": span(lay, "left_hip_pitch", 4), "left_knee": span(lay, "left_knee", 4)}),
        ("left_thigh", "left_foot", {"left_knee": span(lay, "left_knee", 4), "left_ankle_pitch": span(lay, "left_ankle_pitch", 5)}),
        ("base", "left_shin", {"left_hip_roll": span(lay, "left_hip_roll", 3), "left_hip_pitch": span(lay, "left_hip_pitch", 4),
                               "left_knee": span(lay, "left_knee", 3)}),
        ("base", "left_foot", {"left_hip_pitch": span(lay, "left_hip_pitch", 3), "left_knee": span(lay, "left_knee", 2),
                               "left_ankle_pitch": span(lay, "left_ankle_pitch", 2)}),
        ("base", "tail_yaw_link", {"tail_yaw": span(lay, "tail_yaw")}),
        ("tail_yaw_link", "tail_pitch_link", {"tail_pitch": span(lay, "tail_pitch")}),
        ("base", "tail_pitch_link", {"tail_yaw": span(lay, "tail_yaw", 5), "tail_pitch": span(lay, "tail_pitch", 3)}),
    ]


def bb_overlap(a, b) -> bool:
    p, q = a.bounding_box(), b.bounding_box()
    return (p.min.X < q.max.X and q.min.X < p.max.X and p.min.Y < q.max.Y and q.min.Y < p.max.Y
            and p.min.Z < q.max.Z and q.min.Z < p.max.Z)


def skip_pair(a: PartSpec, b: PartSpec) -> bool:
    """Pairs that may legitimately touch: a servo and its own connector keep-out, two keep-outs."""
    if a.kind == "keepout" and b.kind == "keepout":
        return True
    for x, y in ((a, b), (b, a)):
        if x.kind == "servo" and y.kind == "keepout" and y.name.startswith(x.name + "_connector"):
            return True
    return False


def worst_overlap(items_a, loc_a, items_b, loc_b):
    worst, where = 0.0, ""
    for a in items_a:
        sa = loc_a * a.solid
        for b in items_b:
            if a is b or skip_pair(a, b):
                continue
            sb = loc_b * b.solid
            if not bb_overlap(sa, sb):
                continue
            v = (sa & sb).volume
            if v > worst:
                worst, where = v, f"{a.name} × {b.name}"
    return worst, where


def fmt(v, bad):
    return f"**{v:.1f}**" if bad else f"{v:.1f}"


def check_interference(lay, model) -> tuple[str, int]:
    by_link: dict[str, list] = {}
    for s in model.parts:
        by_link.setdefault(s.link, []).append(s)
    lines = ["# 干涉檢查", "",
             "計算兩個連桿所有實體的重疊體積（列印件、馬達本體與擺臂、**螺絲頭**、**連接器插頭保留空間**），重疊 > 0.5 mm³ 視為干涉。",
             "連接器保留空間假設插頭從馬達側面插入、再往外留 10 mm 給插頭與線材彎曲（實物待確認）。", "",
             "## 1. 關節掃描（左腿；右腿是鏡像）", "",
             "| 父連桿 | 子連桿 | 取樣關節 | 姿態數 | 最大重疊 (mm³) | 發生位置 |", "|---|---|---|---|---|---|"]
    n_bad = 0
    for parent, child, grid in leg_sweeps(lay):
        names = list(grid)
        worst, where, count = 0.0, "", 0
        for vals in itertools.product(*grid.values()):
            q = dict(zip(names, vals))
            loc = link_locations(lay, q)
            v, w = worst_overlap(by_link[parent], loc[parent], by_link[child], loc[child])
            if v > worst:
                worst, where = v, w + " @ " + ", ".join(f"{k}={x}" for k, x in q.items())
            count += 1
        bad = worst > 0.5
        n_bad += bad
        lines.append(f"| {parent} | {child} | {', '.join(names)} | {count} | {fmt(worst, bad)} | {where if bad else '—'} |")

    lines += ["", "## 2. 同一連桿內（零件、插頭保留空間、螺絲頭、其他馬達）", "", "| 連桿 | 最大重疊 (mm³) | 位置 |", "|---|---|---|"]
    zero = link_locations(lay, {})
    for link, items in by_link.items():
        worst, where = 0.0, ""
        for i, a in enumerate(items):
            v, w = worst_overlap([a], zero[link], items[i + 1:], zero[link])
            if v > worst:
                worst, where = v, w
        bad = worst > 0.5
        n_bad += bad
        lines.append(f"| {link} | {fmt(worst, bad)} | {where if bad else '—'} |")

    lines += ["", "## 3. 不相鄰的連桿（零位與站姿，所有連桿兩兩比對）", "", "| 姿態 | 最大重疊 (mm³) | 位置 |", "|---|---|---|"]
    st = lay.p["stance"]
    stance = {f"{s}_{r}": st[r] for s in ("left", "right") for r in ("hip_pitch", "knee", "ankle_pitch")}
    links = list(by_link)
    for pname, q in (("零位", {}), ("站姿", stance)):
        loc = link_locations(lay, q)
        worst, where = 0.0, ""
        for i, la in enumerate(links):
            for lb in links[i + 1:]:
                if any({j.parent, j.child} == {la, lb} for j in lay.joints):
                    continue
                v, w = worst_overlap(by_link[la], loc[la], by_link[lb], loc[lb])
                if v > worst:
                    worst, where = v, w
        bad = worst > 0.5
        n_bad += bad
        lines.append(f"| {pname} | {fmt(worst, bad)} | {where if bad else '—'} |")

    lines += ["", "## 4. 左右腿互撞（兩腳同時內收，參考用，不計入問題數）", "",
              "| 兩側 hip_roll 內收 (deg) | 最大重疊 (mm³) | 位置 |", "|---|---|---|"]
    legs = {s: [f"{s}_hip", f"{s}_thigh", f"{s}_shin", f"{s}_foot"] for s in ("left", "right")}
    for r in (0, 1, 2, 3, 6):
        loc = link_locations(lay, {"left_hip_roll": -r, "right_hip_roll": r})
        worst, where = 0.0, "—"
        for la in legs["left"]:
            for lb in legs["right"]:
                v, w = worst_overlap(by_link[la], loc[la], by_link[lb], loc[lb])
                if v > worst:
                    worst, where = v, w
        lines.append(f"| {r} | {worst:.1f} | {where} |")

    lines += ["", "## 5. 電子部件（佔位方塊）", "", "| 部件 | 對象 | 最大重疊 (mm³) |", "|---|---|---|"]
    for n, _, sz, c, _ in model.boxes:
        if n == "misc":
            continue
        eb = PartSpec(n, "base", box(c[0] - sz[0] / 2, c[0] + sz[0] / 2, c[1] - sz[1] / 2, c[1] + sz[1] / 2,
                                     c[2] - sz[2] / 2, c[2] + sz[2] / 2), "bought")
        worst, who = worst_overlap([eb], Location(), by_link["base"], Location())
        for side in ("left", "right"):
            for r in range(-25, 26, 5):
                loc = link_locations(lay, {f"{side}_hip_roll": r})
                v, w = worst_overlap([eb], Location(), by_link[f"{side}_hip"], loc[f"{side}_hip"])
                if v > worst:
                    worst, who = v, f"{w} @ roll {r}"
        bad = worst > 0.5
        n_bad += bad
        lines.append(f"| {n} | {who or '—'} | {fmt(worst, bad)} |")
    return "\n".join(lines) + "\n", n_bad


# ---------------------------------------------------------------- servo direction table
def servo_name(joint: str) -> str:
    return "servo_" + joint.replace("ankle_pitch", "ankle")


def servo_map(lay, model) -> tuple[str, list]:
    ccw = lay.p["servo"]["position_increase_ccw"]
    rows, data = [], []
    for e in lay.p["servo_bus"]:
        jn = e["joint"]
        sname = servo_name(jn)
        _, horn, body = model.servo_frames[sname]
        axis = lay.joint(jn).axis
        dot = sum(a * b for a, b in zip(horn, axis))
        direction = int(round(dot)) * ccw
        data.append({"id": e["id"], "joint": jn, "bus": e["bus"], "servo": sname, "horn_dir": list(horn),
                     "body_dir": list(body), "direction": direction})
        rows.append(f"| {e['id']} | {jn} | {e['bus']} | {tuple(horn)} | {tuple(body)} | {direction:+d} |")
    md = ["# 馬達方向與安裝表（自動產生）", "",
          "`direction` = 馬達位置值增加時，關節往 + 方向（`params.yaml` joints 的正方向）轉動為 +1，反之 −1。",
          "計算方式：主擺臂朝向 · 關節軸 × `servo.position_increase_ccw`。目前假設位置值增加 = 從主擺臂看逆時針，"
          "**第一次上電前要實測確認**；若實測相反，把 `position_increase_ccw` 改成 −1 重新產生，整張表會一起翻轉。", "",
          "關節角（rad）= direction × (位置值 − 2048) × 2π / 4096。", "",
          "| ID | 關節 | 匯流排 | 主擺臂朝向（世界座標，零位） | 本體朝向 | direction |", "|---|---|---|---|---|---|"] + rows
    return "\n".join(md) + "\n", data


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", type=int, required=True)
    ap.add_argument("--skip-interference", action="store_true")
    args = ap.parse_args()
    vdir = P.version_dir(args.version)
    if (vdir / "FROZEN").exists():
        sys.exit(f"{vdir} is frozen (tagged). Create a new version instead of rebuilding it.")
    t0 = time.time()
    lay = P.load(args.version)
    model = Builder(lay).build()
    rho = lay.p["print"]["effective_density"]
    origin = lay.link_origin
    for d in ("parts/step", "parts/stl", "meshes"):
        if (vdir / d).exists():
            shutil.rmtree(vdir / d)
    for d in ("parts/step", "parts/stl", "meshes", "reports"):
        (vdir / d).mkdir(parents=True, exist_ok=True)

    # ballast = tip total - printed holder
    tip_holder = next(s for s in model.parts if s.name == "tail_tip")
    ballast = next(s for s in model.parts if s.name == "tail_ballast")
    ballast.mass = lay.p["tail"]["tip"]["mass"] - part_mass(tip_holder, rho)
    bal_cap = ballast.solid.volume * 7.8 / 1000
    if ballast.mass > bal_cap:
        print(f"WARNING: ballast {ballast.mass:.1f} g does not fit (steel capacity {bal_cap:.1f} g)")

    # print files: one per unique part (left side), in its link frame
    real = [s for s in model.parts if s.kind not in ("head", "keepout")]
    bom = [(s, part_mass(s, rho)) for s in real]
    for s in real:
        if s.kind in ("print", "tpu") and "right" not in s.name:
            o = origin[s.link]
            local = Pos(-o[0], -o[1], -o[2]) * s.solid
            fname = s.name.replace("_left", "")
            export_step(local, str(vdir / "parts/step" / f"{fname}.step"))
            export_stl(local, str(vdir / "parts/stl" / f"{fname}.stl"), tolerance=0.02, angular_tolerance=0.2)

    # visual meshes + mass properties per link
    links = ["base"] + [j.child for j in lay.joints]
    props = {}
    screw_m = lay.p["fasteners"]["screw_mass"]
    for link in links:
        items, solids = [], []
        for s, m in bom:
            if s.link != link:
                continue
            com, inertia = inertia_of(s, m)
            items.append((m, com, inertia))
            if s.screws:
                items.append((s.screws * screw_m, com, np.zeros((3, 3))))
            solids.append(s.solid)
        for n, l, size, c, m in model.boxes:
            if l == link:
                items.append((m, np.array(c, float), box_inertia(size, m)))
        mass, com, inertia = combine(items)
        o = np.array(origin[link], float)
        props[link] = {"mass_g": mass, "com_mm": (com - o).tolist(), "inertia_gmm2": inertia.tolist(),
                       "origin_world_mm": o.tolist()}
        mesh = solids[0]
        for q in solids[1:]:
            mesh = mesh + q
        export_stl(Pos(*(-o)) * mesh, str(vdir / "meshes" / f"{link}.stl"), tolerance=0.05, angular_tolerance=0.3)
    total = sum(v["mass_g"] for v in props.values())
    meta = {"version": args.version, "git_commit": P.git_commit(), "generated": time.strftime("%Y-%m-%d %H:%M"),
            "total_mass_g": total, "links": props}
    (vdir / "reports/mass_properties.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

    md, data = servo_map(lay, model)
    (vdir / "reports/servo_map.md").write_text(md, encoding="utf-8")
    (vdir / "reports/servo_map.json").write_text(json.dumps(data, indent=1), encoding="utf-8")

    rows = ["# 零件清單（自動產生）", "", f"模型 v{args.version}，git {meta['git_commit']}，{meta['generated']}。"
            f"列印件質量 = CAD 體積 × 等效密度 {rho} g/cm³（實際列印後請秤重回填）。", "",
            "| 零件 | 連桿 | 類型 | 體積 (cm³) | 質量 (g) | 螺絲數 | 數量 | 列印方式 |", "|---|---|---|---|---|---|---|---|"]
    for s, m in bom:
        if "right" in s.name:
            continue
        rows.append(f"| {s.name} | {s.link.replace('left', 'SIDE')} | {s.kind} | {s.solid.volume / 1000:.2f} | {m:.1f} | "
                    f"{s.screws} | {s.qty_note} | {s.print_note} |")
    rows += ["", "| 連桿 | 質量 (g) | 質心（連桿座標，mm） |", "|---|---|---|"]
    for k, v in props.items():
        rows.append(f"| {k} | {v['mass_g']:.1f} | ({', '.join(f'{c:.1f}' for c in v['com_mm'])}) |")
    rows.append(f"| **合計** | **{total:.1f}** | |")
    (vdir / "reports/parts_bom.md").write_text("\n".join(rows) + "\n", encoding="utf-8")
    print(f"total mass {total:.1f} g; exported in {time.time() - t0:.0f} s")
    for k, v in props.items():
        print(f"  {k:16s} {v['mass_g']:7.1f} g  com {np.round(v['com_mm'], 1)}")
    print(md)

    if not args.skip_interference:
        txt, n_bad = check_interference(lay, model)
        (vdir / "reports/interference.md").write_text(txt, encoding="utf-8")
        print(txt)
        print(f"interference: {n_bad} problem(s); total {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
