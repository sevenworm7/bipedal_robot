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
    python hardware/generator/check_cable.py --version 2 --chain          # the whole bus in params `wiring`

`--chain` checks the complete daisy chain of params.yaml `wiring` (board -> ... -> last servo): legs, tail and
the board, with each leg and the tail joint swept independently, and picks the socket each servo uses in and out
so that the longest segment is as short as possible. The board is the centre point of params `electronics.board`.
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


# ---------------------------------------------------------------- whole bus (params `wiring`)
GRID = {"hip_roll": 7, "hip_pitch": 11, "knee": 12, "tail_yaw": 13}
LEG_JOINTS = ("hip_roll", "hip_pitch", "knee")


def _leg_n(role):
    return next(i for i, r in ROLES.items() if r == role)


class Bus:
    """Socket positions of every servo on the bus (and the board) as functions of the joint angles (deg).
    Node names are the joint names of params `servo_bus`, plus "board"."""

    def __init__(self, lay):
        self.lay = lay
        self.leg = Leg(lay)
        p = lay.p
        sv = p["servo"]
        cn = sv["connector"]
        xs, zs, hw = sum(cn["x"]) / 2, sum(cn["z"]) / 2, sv["body"]["half_width"]

        self.faces = {}

        def socks(o, hd, bd, node):                # same frames as parts.Builder._tail
            xS, zS = np.array(bd, float), np.array(hd, float)
            yS = np.cross(zS, xS)
            self.faces[node] = ["+-"[int(s * yS[i] < 0)] + "XYZ"[i] for s in (1, -1) for i in range(3) if abs(yS[i]) > 0.5]
            return [np.array(o, float) + xs * xS + s * hw * yS + zs * zS for s in (1, -1)]

        self.ty = lay.joint("tail_yaw")
        self.tail = {"tail_yaw": socks(self.ty.origin, (0, 0, -1), (1, 0, 0), "tail_yaw"),
                     "tail_pitch": socks(lay.joint("tail_pitch").origin, (0, 1, 0), (1, 0, 0), "tail_pitch")}
        e = p["electronics"]["board"]
        self.board = np.array([e["center"][0], e["center"][1], lay.hip_roll_z + e["center"][2]], float)

    def joints_of(self, node):
        """Joints that move this node's sockets."""
        if node in ("board", "tail_yaw"):
            return []
        if node == "tail_pitch":
            return ["tail_yaw"]
        side, role = node.split("_", 1)
        return [f"{side}_{r}" for r in LEG_JOINTS[:MOVED_BY[_leg_n(role)]]]

    def grid(self, joint):
        j = self.lay.joint(joint)
        role = "tail_yaw" if joint == "tail_yaw" else joint.split("_", 1)[1]
        return np.linspace(j.lower, j.upper, GRID[role])

    def pos(self, node, k, q):
        if node == "board":
            return self.board
        if node == "tail_yaw":
            return self.tail[node][k]
        if node == "tail_pitch":
            o = np.array(self.ty.origin, float)
            return rot(self.ty.axis, q["tail_yaw"]) @ (self.tail[node][k] - o) + o
        side, role = node.split("_", 1)
        n = _leg_n(role)
        qq = [q.get(f"{side}_{r}", 0.0) for r in LEG_JOINTS] + [0.0]
        if side == "right":                        # right leg = mirror image of the left leg
            qq[0] = -qq[0]
        p = self.leg.place(self.leg.sockets[n][k][1], qq, MOVED_BY[n])
        return p if side == "left" else np.array([p[0], -p[1], p[2]])

    def name(self, node, k):
        if node == "board":
            return "board"
        if node.startswith(("left_", "right_")):
            tag = self.leg.sockets[_leg_n(node.split("_", 1)[1])][k][0]
            if node.startswith("right_"):              # mirrored: a +Y face becomes -Y
                tag = tag.replace("+Y", "#").replace("-Y", "+Y").replace("#", "-Y")
            return tag
        return f"{'ab'[k]}({self.faces[node][k]})"   # tail_pitch faces are at tail_yaw = 0

    def worst(self, a, ka, b, kb):
        joints = sorted(set(self.joints_of(a)) | set(self.joints_of(b)))
        best, at = -1.0, {}
        for vals in itertools.product(*(self.grid(j) for j in joints)):
            q = dict(zip(joints, vals))
            d = float(np.linalg.norm(self.pos(a, ka, q) - self.pos(b, kb, q)))
            if d > best:
                best, at = d, q
        return best, at


def check_chain(lay, wiring, limit):
    """Worst distance of every segment of the bus, with the socket assignment that minimises the longest one.
    The board has one socket; every servo uses one socket in and the other out; the last servo's spare stays empty."""
    bus = Bus(lay)
    cache = {}

    def seg(i, ka, kb):
        if (i, ka, kb) not in cache:
            cache[(i, ka, kb)] = bus.worst(wiring[i], ka, wiring[i + 1], kb)
        return cache[(i, ka, kb)]

    n_servo = len(wiring) - 1
    best = None
    for out in itertools.product((0, 1), repeat=n_servo - 1):   # outgoing socket of every servo except the last
        segs, k_out = [], 0
        for i in range(n_servo):
            if i < n_servo - 1:
                k_in = 1 - out[i]
            else:
                k_in = min((0, 1), key=lambda k: seg(i, k_out, k)[0])
            segs.append((i, k_out, k_in, *seg(i, k_out, k_in)))
            if i < n_servo - 1:
                k_out = out[i]
        m = max(s[3] for s in segs)
        if best is None or m < best[0]:
            best = (m, segs)
    m, segs = best
    print(f"bus wiring (params `wiring`), limit {limit:.0f} mm: longest segment {m:.0f} mm")
    print("socket names: the world direction the servo side face points to at zero pose;")
    print("the board is its centre point (header position not modelled).")
    for i, ka, kb, w, at in segs:
        a, b = wiring[i], wiring[i + 1]
        ats = ", ".join(f"{k} {v:.0f}" for k, v in at.items()) if at else "fixed"
        flag = "ok" if w <= limit else "TOO LONG"
        print(f"  {i + 1:2d}. {a:17s} [{bus.name(a, ka):5s}] -> {b:17s} [{bus.name(b, kb):5s}]"
              f"  worst {w:5.0f} mm  {flag:8s} ({ats})")
    return m


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
    ap.add_argument("--chain", action="store_true", help="check the whole bus listed in params `wiring`")
    ap.add_argument("--limit", type=float, default=120.0)
    a = ap.parse_args()
    if a.chain:
        lay = layout(a.version, a.thigh, a.shin)
        if "wiring" not in lay.p:
            sys.exit("params.yaml has no `wiring` list")
        check_chain(lay, lay.p["wiring"], a.limit)
    elif a.sweep:
        sweep(a.version, list(range(50, 95, 5)), list(range(40, 95, 5)), a.limit)
    else:
        report(a.version, a.thigh, a.shin)
