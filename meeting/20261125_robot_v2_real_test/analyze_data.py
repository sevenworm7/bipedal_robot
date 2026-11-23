"""Extract the numbers of the 2026-11-25 report from the 2026-11-23 sessions (outputs/, not in git) -> report_data.json."""
import csv, json, math, re, statistics as st, sys
from pathlib import Path

S = Path(__file__).resolve().parents[2] / "outputs" / "robot_v2" / "deploy" / "sessions"
J = ["left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "right_hip_roll", "right_hip_pitch",
     "right_knee", "right_ankle_pitch", "tail_yaw", "tail_pitch"]
D = 180 / math.pi


def load(name):
    rows = list(csv.DictReader(open(S / name / "telemetry.csv")))
    for r in rows:
        r["t"] = int(r["t_ms"]) / 1000
        r["st"] = int(r["state"])
        r["v"] = float(r["v_min"])
        r["cur"] = sum(abs(int(r.get(f"current_raw_{j}", 0))) for j in J)
        r["err"] = [abs(float(r[f"q_{j}"]) - float(r[f"target_{j}"])) * D for j in J]
    return rows


def summ(rows):
    return dict(n=len(rows), v_min=min(r["v"] for r in rows), v_max=max(r["v"] for r in rows),
                v_mean=st.mean(r["v"] for r in rows), cur_mean=st.mean(r["cur"] for r in rows),
                cur_max=max(r["cur"] for r in rows), err_max=max(max(r["err"]) for r in rows),
                temp_max=max(int(r["temp_max"]) for r in rows),
                loop_ms=st.median(int(r["loop_us"]) for r in rows) / 1000)


out = {}
# stances: hanging 103932 (after INIT), table 104449 (last 60 s on the table)
h = [r for r in load("20261123_103932") if r["st"] == 2]
out["hang"] = summ(h[len(h) // 3:])
t = [r for r in load("20261123_104449") if r["st"] == 2]
tab = [r for r in t if r["t"] > t[-1]["t"] - 60]
out["table"] = summ(tab)
off = [r for r in load("20261123_104449") if r["st"] == 0]
out["off"] = summ(off) if off else None
out["table_joint_cur"] = {j: st.mean(abs(int(r[f"current_raw_{j}"])) for r in tab) for j in J}
hc = [r for r in t if r["t"] < t[0]["t"] + 8]
out["hang_cur"] = summ(hc)
out["hang_joint_cur"] = {j: st.mean(abs(int(r[f"current_raw_{j}"])) for r in hc) for j in J}
out["table_joint_err"] = {j: max(r["err"][i] for r in tab) for i, j in enumerate(J)}

# WALK 6.5 V guard
w1 = load("20261123_105717")
s1 = [r for r in w1 if r["st"] == 3]
out["walk65"] = summ(s1) if s1 else None
out["walk65_series"] = [[round(r["t"] - s1[0]["t"], 3), r["v"], r["cur"]] for r in s1] if s1 else []
pre = [r for r in w1 if r["st"] == 2][-25:]
out["walk65_pre"] = summ(pre)

# WALK 5.5 V guard
w = load("20261123_110511")
wk = [r for r in w if r["st"] == 3]
t0 = wk[0]["t"]
out["walk"] = summ(wk)
out["walk_pre_stand"] = summ([r for r in w if r["st"] == 2 and r["t"] < t0][-50:])
out["walk_series"] = [[round(r["t"] - t0, 3), r["v"], r["cur"], round(max(r["err"]), 2)] for r in wk]
out["walk_joint"] = {}
for i, j in enumerate(J):
    q = [float(r[f"q_{j}"]) for r in wk]
    tt = [r["t"] for r in wk]
    vel = sorted(abs(q[k + 1] - q[k]) / (tt[k + 1] - tt[k]) * D for k in range(len(q) - 1) if tt[k + 1] > tt[k])
    out["walk_joint"][j] = dict(cur_mean=st.mean(abs(int(r[f"current_raw_{j}"])) for r in wk),
                                cur_max=max(abs(int(r[f"current_raw_{j}"])) for r in wk),
                                err_max=max(r["err"][i] for r in wk),
                                vel_p95=vel[int(0.95 * len(vel))], vel_max=vel[-1])
hi = [max(r["err"]) for r in wk if r["v"] >= 7.2]
lo = [max(r["err"]) for r in wk if r["v"] < 6.6]
out["err_by_v"] = dict(hi_n=len(hi), hi_mean=st.mean(hi), lo_n=len(lo), lo_mean=st.mean(lo))
bins = {}
for r in wk:
    bins.setdefault(r["v"], []).append(max(r["err"]))
out["err_bins"] = {f"{k:.1f}": [len(v), st.mean(v)] for k, v in sorted(bins.items())}

# PLAY 0.5
pl = []
for line in open(S / "20261123_105642" / "console.log", encoding="utf-8"):
    m = re.match(r"[\d.]+ PL (\d+) (-?\d+) ([\d.]+) ([\d.]+) (\w+)", line)
    if m:
        pl.append([int(m[1]) / 1000, int(m[2]), float(m[3]), float(m[4])])
out["play05"] = dict(n=len(pl), v_min=min(p[2] for p in pl), err_max=max(p[3] for p in pl))
json.dump(out, open(Path(__file__).with_name("report_data.json"), "w"), indent=1)
for k, v in out.items():
    if not k.endswith("series"):
        print(k, json.dumps(v)[:400])
