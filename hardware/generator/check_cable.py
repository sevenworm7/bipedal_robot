"""Servo-cable reach check for one leg (pure geometry, no CAD).

The servos are daisy-chained on one bus and the stock cables only reach a limited socket-to-socket distance
(user, 2026-11-18: about 120 mm with two servos side by side). This script places every leg servo's two
connector sockets (params.yaml `servo.connector`, both long side faces) in the world frame, sweeps the joints
each cable crosses through their full range, and reports the worst-case straight socket-to-socket distance of
every segment of a wiring order. Each servo uses a different socket for its incoming and outgoing cable.

The straight distance is a lower bound on the cable needed: the real cable also bends around the joint and the
brackets, so keep a margin below the limit.

Usage (project root, CAD env or any Python with numpy + PyYAML):
    python hardware/generator/check_cable.py --version 1
    python hardware/generator/check_cable.py --version 1 --thigh 60 --shin 55
    python hardware/generator/check_cable.py --version 1 --sweep          # table over thigh x shin
"""
from __future__ import annotations

import argparse
import copy
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from params import load  # noqa: E402

# leg servos, numbered top to bottom as the user does (1 hip_roll, 2 hip_pitch, 3 knee, 4 ankle)
ROLES = {1: "hip_roll", 2: "hip_pitch", 3: "knee", 4: "ankle_pitch"}
# link that holds each servo body, as the index of the last joint that moves it (0 = torso)
MOVED_BY = {1: 0, 2: 2, 3: 2, 4: 3}
# default wiring: fold-back leg 1-3-4-2 (then exits at hip level), terminating leg 1-2-3-4
ORDERS = {"fold_back_1342": [1, 3, 4, 2], "straight_1234": [1, 2, 3, 4]}


def rot(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


class Leg:
    """Left leg kinematics at zero pose from a Layout (mm, deg)."""

    def __init__(self, lay):
        self.lay = lay
        self.j = [lay.joint(f"left_{r}") for r in ("hip_roll", "hip_pitch", "knee", "ankle_pitch")]
        y = self.j[0].origin[1]
        X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        nZ = (0, 0, -1)
        # (axis point, horn dir, body dir), same as parts.Builder._leg
        self.frames = {1: (self.j[0].origin, X, Z), 2: (self.j[1].origin, Y, nZ),
                       3: (self.j[2].origin, Y, Z), 4: (self.j[3].origin, Y, Z)}
        sv = lay.p["servo"]
        cn = sv["connector"]
        xs = sum(cn["x"]) / 2                      # socket centre along the body
        zs = sum(cn["z"]) / 2                      # socket centre across the thickness (secondary-horn half)
        hw = sv["body"]["half_width"]
        self.sockets = {}                          # servo -> [(name, world point at zero pose)]
        for n, (o, hd, bd) in self.frames.items():
            xS, zS = np.array(bd, float), np.array(hd, float)
            yS = np.cross(zS, xS)
            pts = []
            for sgn, tag in ((1, "a"), (-1, "b")):
                p = np.array(o, float) + xs * xS + sgn * hw * yS + zs * zS
                face = "+".join(f"{'+-'[int(v < 0)]}{'XYZ'[i]}" for i, v in enumerate(sgn * yS) if abs(v) > 0.5)
                pts.append((f"{tag}({face})", p))
            self.sockets[n] = pts

    def place(self, p, q, upto):
        """World position of point p (zero pose) carried by joints 0..upto-1 at angles q (deg)."""
        p = np.array(p, float)
        for i in reversed(range(upto)):
            o = np.array(self.j[i].origin, float)
            p = rot(self.j[i].axis, q[i]) @ (p - o) + o
        return p

    def limits(self, i):
        return self.j[i].lower, self.j[i].upper


def segment_max(leg, a, b, sa, sb, steps=13):
    """Worst-case and zero-pose distance between socket sa of servo a and socket sb of servo b."""
    moved = sorted({k for k in range(max(MOVED_BY[a], MOVED_BY[b]))})
    grids = [np.linspace(*leg.limits(i), steps) for i in moved]
    pa, pb = leg.sockets[a][sa][1], leg.sockets[b][sb][1]
    worst, at = -1.0, None
    for vals in itertools.product(*grids) if grids else [()]:
        q = [0.0] * 4
        for i, v in zip(moved, vals):
            q[i] = v
        d = np.linalg.norm(leg.place(pa, q, MOVED_BY[a]) - leg.place(pb, q, MOVED_BY[b]))
        if d > worst:
            worst, at = d, dict(zip((ROLES[i + 1] for i in moved), vals))
    zero = np.linalg.norm(pa - pb)
    return worst, zero, at


def best_chain(leg, order):
    """Pick sockets so that each servo uses different sockets in and out; minimise the longest segment."""
    best = None
    # socket choice per servo: index of the socket used for the OUTGOING cable (incoming uses the other one)
    for choice in itertools.product((0, 1), repeat=len(order)):
        segs = []
        for k in range(len(order) - 1):
            a, b = order[k], order[k + 1]
            sa = choice[k]
            sb = 1 - choice[k + 1]
            w, z, at = segment_max(leg, a, b, sa, sb)
            segs.append((a, b, leg.sockets[a][sa][0], leg.sockets[b][sb][0], w, z, at))
        m = max(s[4] for s in segs)
        if best is None or m < best[0]:
            best = (m, segs)
    return best


def cross_to_other_hip(leg):
    """Hip_pitch servo (2) of this leg to the other leg's hip_roll servo (1) socket, worst case over hip joints."""
    worst = 0.0
    for sa in (0, 1):
        pa = leg.sockets[2][sa][1]
        best_b = None
        for sb in (0, 1):
            pb = leg.sockets[1][sb][1].copy()
            pb[1] = -pb[1]                         # mirror to the right leg
            w = 0.0
            for q0 in np.linspace(*leg.limits(0), 9):
                for q1 in np.linspace(*leg.limits(1), 13):
                    w = max(w, np.linalg.norm(leg.place(pa, [q0, q1, 0, 0], 2) - pb))
            best_b = w if best_b is None else min(best_b, w)
        worst = best_b if worst == 0.0 else min(worst, best_b)
    return worst


def layout(version, thigh=None, shin=None):
    """Layout of this version with the thigh / shin lengths optionally overridden (mm)."""
    lay = load(version)
    p = copy.deepcopy(lay.p)
    if thigh is not None:
        p["legs"]["thigh"] = thigh
    if shin is not None:
        p["legs"]["shin"] = shin
    return load(version, p)


def to_board(leg, servo):
    """Worst distance from either socket of a leg servo (best one) to the control board centre.
    The board (params electronics.board, z relative to the hip_roll axis) is glued under the torso top lid;
    its header position is not modelled, so this is only indicative (+- half the board size)."""
    e = leg.lay.p["electronics"]["board"]
    c = np.array([e["center"][0], e["center"][1], leg.lay.hip_roll_z + e["center"][2]], float)
    upto = MOVED_BY[servo]
    best = None
    for _, p in leg.sockets[servo]:
        w = 0.0
        for q0 in (np.linspace(*leg.limits(0), 9) if upto >= 1 else [0.0]):
            for q1 in (np.linspace(*leg.limits(1), 13) if upto >= 2 else [0.0]):
                w = max(w, np.linalg.norm(leg.place(p, [q0, q1, 0, 0], upto) - c))
        best = w if best is None else min(best, w)
    return best, c


def report(version, thigh, shin):
    lay = layout(version, thigh, shin)
    lg = lay.p["legs"]
    leg = Leg(lay)
    print(f"thigh {lg['thigh']} mm, shin {lg['shin']} mm, hip_roll axis height {lay.hip_roll_z:.0f} mm")
    for n, pts in leg.sockets.items():
        print(f"  servo {n} {ROLES[n]:11s} sockets: " +
              ", ".join(f"{t} ({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f})" for t, p in pts))
    for name, order in ORDERS.items():
        m, segs = best_chain(leg, order)
        print(f"\n  {name}: longest segment {m:.0f} mm")
        for a, b, ta, tb, w, z, at in segs:
            ats = ", ".join(f"{k} {v:.0f}" for k, v in at.items()) if at else "fixed"
            print(f"    {a}->{b}  {ta:10s} -> {tb:10s}  zero pose {z:5.0f}  worst {w:5.0f} mm  ({ats})")
    print(f"\n  2 -> other leg's 1 (cross at hip level), worst {cross_to_other_hip(leg):.0f} mm")
    for n in (1, 2):
        w, c = to_board(leg, n)
        print(f"  {n} -> control board centre ({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f}), worst {w:.0f} mm "
              f"(header position not modelled)")


def sweep(version, thighs, shins, limit):
    print(f"longest fold-back (1-3-4-2) segment, worst case over the joint ranges [mm]; limit {limit}")
    print("thigh\\shin " + " ".join(f"{s:6d}" for s in shins))
    for t in thighs:
        row = []
        for s in shins:
            m, _ = best_chain(Leg(layout(version, t, s)), ORDERS["fold_back_1342"])
            row.append(f"{m:5.0f}{'*' if m <= limit else ' '}")
        print(f"{t:10d} " + " ".join(row))
    print("* = within the limit (straight-line distance; keep a routing margin)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", type=int, default=1)
    ap.add_argument("--thigh", type=float)
    ap.add_argument("--shin", type=float)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--limit", type=float, default=120.0)
    a = ap.parse_args()
    if a.sweep:
        sweep(a.version, list(range(50, 95, 5)), list(range(40, 95, 5)), a.limit)
    else:
        report(a.version, a.thigh, a.shin)
