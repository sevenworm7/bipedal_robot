"""build123d geometry of every printed part, servo and bought part, placed in the zero-pose world frame.

Design rules (physical meaning and assembly order: ../v<N>/ASSEMBLY.md):
- Servo frame S: origin on the output axis at the body mid-plane, +z_S = primary horn, +x_S = toward the far end
  of the 34 mm body, y_S = z_S x x_S. The HD-1910 is dual-shaft: a horn on each face.
- Every joint = one link holds the servo BODY (walls on both horn faces, screwed into the two far mounting holes,
  plus a crossbar over the far end), the other link's two ARMS clamp the two HORNS.
- A horn sits in a pocket on the arm's inner face. The pocket is an open channel toward the servo body, so the arms
  slide onto the horns along the body direction without being spread; 4 screws per horn then clamp it.
- A servo body always points away from the link its horns drive, so the arms never sweep over the body screws.
- Connector sockets are on both long side faces (secondary-horn half); keep-out boxes model plug + cable bend.
- Screw heads and connector keep-outs are included in the interference sweeps (kind "head" / "keepout").
- Left-leg parts are built at y = +hip_spacing/2; right-leg parts are their mirror images (every leg part is
  symmetric about its own leg plane, so one STL serves both sides).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Align, Box, Cylinder, Part, Plane, Pos, Rot, Vector, mirror

from params import Layout

MIN = (Align.MIN, Align.MIN, Align.MIN)
C = Align.CENTER
NO_MASS = ("head", "keepout")


def box(x0, x1, y0, y1, z0, z1) -> Part:
    """Axis-aligned box from extents (in whatever frame it is later placed in)."""
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=MIN)


def cyl(r, h, center, axis) -> Part:
    """Cylinder of radius r and length h centred at `center`, axis 'x', 'y' or 'z'."""
    rot = {"x": Rot(0, 90, 0), "y": Rot(90, 0, 0), "z": Rot(0, 0, 0)}[axis]
    return Pos(*center) * rot * Cylinder(r, h)


def union(solids):
    out = solids[0]
    for s in solids[1:]:
        out = out + s
    return out


def add(v, w):
    return tuple(a + b for a, b in zip(v, w))


def scale(v, k):
    return tuple(a * k for a in v)


@dataclass
class PartSpec:
    name: str            # unique part name (STL/STEP file name)
    link: str            # URDF link it belongs to
    solid: Part          # world zero-pose geometry
    kind: str = "print"  # print | tpu | servo | bought | head | keepout
    mass: float | None = None   # g; None -> volume x effective density
    qty_note: str = ""
    print_note: str = ""
    screws: int = 0      # fasteners carried by this part (mass budget)


@dataclass
class Fastener:
    """One screw: head seat point p (world, zero pose), unit direction d into the parts, length (mm).
    target: 'servo' (threads into a servo horn or body hole), 'insert' (into a heat-set insert),
    'rod' (set screw pressing on the carbon tube)."""
    name: str
    link: str
    p: tuple
    d: tuple
    length: float
    spec: str
    target: str
    joint: str = ""      # what it connects, for the report


@dataclass
class Insert:
    """Heat-set insert hole: mouth point p, unit direction d into the part, hole depth."""
    name: str
    link: str
    p: tuple
    d: tuple
    depth: float


@dataclass
class Model:
    lay: Layout
    parts: list[PartSpec] = field(default_factory=list)
    boxes: list[tuple] = field(default_factory=list)   # electronics: name, link, size, center, mass
    servo_frames: dict = field(default_factory=dict)   # servo name -> (axis_pt, horn_dir, body_dir)
    fasteners: list[Fastener] = field(default_factory=list)
    inserts: list[Insert] = field(default_factory=list)


class Builder:
    def __init__(self, lay: Layout):
        self.lay = lay
        p = lay.p
        self.sv = p["servo"]
        self.pr = p["print"]
        self.fs = p["fasteners"]
        b = self.sv["body"]
        self.near, self.far, self.hw, self.ht = b["near"], b["far"], b["half_width"], b["half_thick"]
        self.hn = self.sv["horn"]
        self.horn_out_p = self.ht + self.hn["primary_thick"]
        self.horn_out_s = self.ht + self.hn["secondary_thick"]
        self.arm_in = self.ht + self.pr["horn_clearance"]
        self.arm_out = self.arm_in + self.pr["arm_thick"]
        self.fit = self.pr["servo_fit"]
        self.pad_r = 10.5
        self.arm_w = 21.0
        self.mount_x = self.sv["mount_holes"]["x"][1]
        self.mount_y = self.sv["mount_holes"]["y"][1]
        self.clr = self.fs["m2_clear"] / 2
        self.head_r = self.fs["m2_head_d"] / 2
        self.head_h = self.fs["m2_head_h"]
        self.ins_r = self.fs["insert"]["hole_d"] / 2
        self.ins_h = self.fs["insert"]["hole_depth"]
        self.clear = lay.p["legs"]["link_bottom_clear"]
        self.parts: list[PartSpec] = []
        self.frames: dict = {}
        self.fasteners: list[Fastener] = []
        self.inserts: list[Insert] = []

    # ------------------------------------------------------------ helpers
    @staticmethod
    def sframe(axis_pt, horn_dir, body_dir) -> Plane:
        return Plane(origin=axis_pt, x_dir=body_dir, z_dir=horn_dir)

    @staticmethod
    def to_world(pl: Plane, p_local, d_local):
        p = pl.from_local_coords(Vector(*p_local))
        q = pl.from_local_coords(Vector(*p_local) + Vector(*d_local))
        return tuple(p), tuple(q - p)

    def screw(self, name, link, pl, p_local, d_local, length, spec, target, joint=""):
        p, d = self.to_world(pl, p_local, d_local)
        self.fasteners.append(Fastener(name, link, p, d, length, spec, target, joint))

    def insert(self, name, link, p, d):
        self.inserts.append(Insert(name, link, tuple(p), tuple(d), self.ins_h))

    def head(self, name, link, solid):
        self.parts.append(PartSpec(name, link, solid, "head", 0.0))

    def servo(self, name, link, axis_pt, horn_dir, body_dir):
        """HD-1910 (dual shaft) + its two connector keep-outs."""
        pl = self.sframe(axis_pt, horn_dir, body_dir)
        body = box(-self.near, self.far, -self.hw, self.hw, -self.ht, self.ht)
        hp = Pos(0, 0, self.ht) * Cylinder(self.hn["radius"], self.hn["primary_thick"], align=(C, C, Align.MIN))
        hs = Pos(0, 0, -self.ht) * Cylinder(self.hn["radius"], self.hn["secondary_thick"], align=(C, C, Align.MAX))
        self.parts.append(PartSpec(name, link, pl * (body + hp + hs), "servo", self.sv["mass"]))
        cn = self.sv["connector"]
        k = cn["keepout"]
        for sgn, tag in ((1, "a"), (-1, "b")):
            y0, y1 = sorted((sgn * self.hw, sgn * (self.hw + k)))
            ko = box(cn["x"][0], cn["x"][1], y0, y1, cn["z"][0], cn["z"][1])
            self.parts.append(PartSpec(f"{name}_connector_{tag}", link, pl * ko, "keepout", 0.0))
        self.frames[name] = (axis_pt, horn_dir, body_dir)

    def arm_pair(self, name, link, axis_pt, horn_dir, arm_dir, length, width=None) -> Part:
        """Two arms clamping both horns of the servo at axis_pt (horn_dir = primary side, arm_dir = direction the
        bracket extends at zero pose). The horn pockets open toward -arm_dir, i.e. toward the servo body."""
        width = width or self.arm_w
        t = self.arm_out - self.arm_in
        out = []
        for sgn, horn_out in ((1, self.horn_out_p), (-1, self.horn_out_s)):
            depth = horn_out - self.arm_in
            rp = self.hn["radius"] + self.pr["horn_pocket_clearance"]
            slab = Cylinder(self.pad_r, t, align=(C, C, Align.MIN)) + box(0, length, -width / 2, width / 2, 0, t)
            slab -= Cylinder(rp, depth, align=(C, C, Align.MIN))
            slab -= box(-self.pad_r - 1, 0, -rp, rp, 0, depth)              # open channel toward the body
            slab -= Cylinder(3.0, 3 * t)                                    # horn centre screw access
            heads = []
            for kk in range(self.hn["screw_count"]):
                a = math.radians(self.hn["screw_angle0"] + 360 * kk / self.hn["screw_count"])
                c = (self.hn["screw_radius"] * math.cos(a), self.hn["screw_radius"] * math.sin(a))
                slab -= Pos(c[0], c[1], 0) * Cylinder(self.clr, 3 * t)
                heads.append(Pos(c[0], c[1], t) * Cylinder(self.head_r, self.head_h, align=(C, C, Align.MIN)))
            z = scale(horn_dir, sgn)
            pl = Plane(origin=add(axis_pt, scale(z, self.arm_in)), x_dir=arm_dir, z_dir=z)
            out.append(pl * slab)
            for kk in range(self.hn["screw_count"]):
                a = math.radians(self.hn["screw_angle0"] + 360 * kk / self.hn["screw_count"])
                c = (self.hn["screw_radius"] * math.cos(a), self.hn["screw_radius"] * math.sin(a), t)
                self.screw(f"{name}_horn{'p' if sgn > 0 else 's'}{kk}", link, pl, c, (0, 0, -1), 5.0, "M2×5 盤頭",
                           "servo", "手臂 → 擺臂")
            self.head(f"{name}_hornscrews_{'p' if sgn > 0 else 's'}", link, pl * union(heads))
        return out[0] + out[1]

    def holder(self, name, link, axis_pt, horn_dir, body_dir, x0, wall, half_w=None, crossbar=3.0) -> Part:
        """Walls on both horn faces of a servo from x_S = x0 to its far end, screwed into the far mounting holes,
        plus a crossbar over the far end (with a cable / tool slot). Walls thicker than 2.5 mm are counterbored
        so every body screw is an M2 x 6."""
        half_w = half_w or (self.hw + 0.5)
        zi, zo = self.ht + self.fit, self.ht + self.fit + wall
        xf = self.far + 0.5
        h = box(x0, xf, -half_w, half_w, zi, zo) + box(x0, xf, -half_w, half_w, -zo, -zi)
        if crossbar:
            h += box(xf, xf + crossbar, -half_w, half_w, -zo, zo)
            h -= box(xf - 1, xf + crossbar + 1, -5, 5, -4, 4)
        heads = []
        cb = max(0.0, wall - 2.5)
        for yy in (-self.mount_y, self.mount_y):
            h -= Pos(self.mount_x, yy, 0) * Cylinder(self.clr, 2 * zo + 2)
            for sgn in (1, -1):
                if cb > 0:
                    h -= Pos(self.mount_x, yy, sgn * (zo - cb / 2)) * Cylinder(self.head_r + 0.25, cb + 0.01)
                z_head = (zo - cb) if sgn > 0 else -(zo - cb) - self.head_h
                heads.append(Pos(self.mount_x, yy, z_head) * Cylinder(self.head_r, self.head_h, align=(C, C, Align.MIN)))
        pl = self.sframe(axis_pt, horn_dir, body_dir)
        for yy in (-self.mount_y, self.mount_y):
            for sgn in (1, -1):
                self.screw(f"{name}_body{'+' if sgn > 0 else '-'}{'+' if yy > 0 else '-'}", link, pl,
                           (self.mount_x, yy, sgn * (zo - cb)), (0, 0, -sgn), 6.0, "M2×6 自攻盤頭", "servo",
                           "固定牆 → 馬達本體")
        self.head(f"{name}_bodyscrews", link, pl * union(heads))
        return pl * h

    # ------------------------------------------------------------ model
    def build(self) -> Model:
        lay, m = self.lay, Model(self.lay)
        y0 = lay.p["legs"]["hip_spacing"] / 2
        self.parts = []
        self._leg(y0)
        left = self.parts
        self.parts = []
        for spec in left:
            r = PartSpec(spec.name.replace("left", "right"), spec.link.replace("SIDE", "right"),
                         mirror(spec.solid, Plane.XZ), spec.kind, spec.mass, spec.qty_note, spec.print_note, spec.screws)
            spec.link = spec.link.replace("SIDE", "left")
            self.parts.append(spec)
            self.parts.append(r)
        left_f, self.fasteners = self.fasteners, []
        for f in left_f:                                        # mirrored fasteners
            r = Fastener(f.name.replace("left", "right"), f.link.replace("SIDE", "right"), (f.p[0], -f.p[1], f.p[2]),
                         (f.d[0], -f.d[1], f.d[2]), f.length, f.spec, f.target, f.joint)
            f.link = f.link.replace("SIDE", "left")
            self.fasteners += [f, r]
        for n, (a, hd, bd) in list(self.frames.items()):     # mirrored servo frames (direction table)
            if "left" in n:
                self.frames[n.replace("left", "right")] = ((a[0], -a[1], a[2]), (hd[0], -hd[1], hd[2]), (bd[0], -bd[1], bd[2]))
        self._torso()
        self._tail()
        m.parts = self.parts
        m.servo_frames = self.frames
        m.fasteners = self.fasteners
        m.inserts = self.inserts
        for name, e in lay.p["electronics"].items():
            c = e["center"]
            m.boxes.append((name, "base", tuple(e.get("size", (10, 10, 10))), (c[0], c[1], lay.hip_roll_z + c[2]), e["mass"]))
        return m

    def _leg(self, y):
        lay = self.lay
        z_hr = lay.hip_roll_z
        z_hp = lay.joint("left_hip_pitch").origin[2]
        z_kn = lay.joint("left_knee").origin[2]
        z_an = lay.joint("left_ankle_pitch").origin[2]
        ai, ao, ht = self.arm_in, self.arm_out, self.ht
        X, Y, Z, nZ = (1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, -1)
        wall_leg = ao - ht - self.fit                     # holder walls flush with the arms' outer faces
        R, Pp, K, A = (0, y, z_hr), (0, y, z_hp), (0, y, z_kn), (0, y, z_an)
        hw_arm = self.arm_w / 2

        # servos: hip_roll in the torso; hip_pitch and knee in the thigh; ankle in the shin
        self.servo("servo_left_hip_roll", "base", R, X, Z)
        self.servo("servo_left_hip_pitch", "SIDE_thigh", Pp, Y, nZ)
        self.servo("servo_left_knee", "SIDE_thigh", K, Y, Z)
        self.servo("servo_left_ankle", "SIDE_shin", A, Y, Z)

        # hip block: cross piece. Roll arms (front/back) hang from the hip_roll horns, pitch arms (outer/inner)
        # rise from the hip_pitch horns; a core joins them, clear of both servos' corner sweep (r = 13.8 mm).
        core_top = z_hr - 14.5
        core_bot = z_hp + 14.5
        hb = self.arm_pair("hip_block_left_roll", "SIDE_hip", R, X, nZ, z_hr - core_top + 0.5)
        hb += self.arm_pair("hip_block_left_pitch", "SIDE_hip", Pp, Y, Z, core_bot - z_hp + 0.5)
        hb += box(-ao, ao, y - ao, y + ao, core_bot, core_top)
        for sx in (1, -1):                                 # gussets: roll arms onto the core
            x0, x1 = sorted((sx * ai, sx * ao))
            hb += box(x0, x1, y - ao, y + ao, core_top - 0.5, core_top + 3.0)
        for sy in (1, -1):                                 # gussets: pitch arms onto the core
            y0_, y1_ = sorted((y + sy * ai, y + sy * ao))
            hb += box(-ao, ao, y0_, y1_, core_bot - 3.0, core_bot + 0.5)
        self.parts.append(PartSpec("hip_block_left", "SIDE_hip", hb, screws=16, qty_note="x2（左右同一個檔案）",
                                   print_note="側躺列印（一片 pitch 手臂的外側面貼床）；另一片 pitch 手臂懸空，需要支撐"))

        # thigh: holds the hip_pitch servo (body down) and the knee servo (body up), side plates in between
        th = self.holder("thigh_left_hp", "SIDE_thigh", Pp, Y, nZ, self.clear, wall_leg, half_w=hw_arm)
        th += self.holder("thigh_left_kn", "SIDE_thigh", K, Y, Z, self.clear, wall_leg, half_w=hw_arm)
        join = self.far + 0.5                              # side plates overlap the holders only at the crossbars
        for sy in (1, -1):
            y0_, y1_ = sorted((y + sy * (ht + self.fit), y + sy * ao))
            th += box(-hw_arm, hw_arm, y0_, y1_, z_kn + join, z_hp - join)
        self.parts.append(PartSpec("thigh_left", "SIDE_thigh", th, screws=8, qty_note="x2（左右同一個檔案）",
                                   print_note="側立列印：側板的 21 mm 寬度朝上，層線沿大腿長度"))

        # shin: clamps the knee horns, holds the ankle servo (body up)
        sh = self.arm_pair("shin_left", "SIDE_shin", K, Y, nZ, z_kn - (z_an + self.far + 0.5))   # overlaps only the crossbar
        sh += self.holder("shin_left_an", "SIDE_shin", A, Y, Z, self.clear, wall_leg, half_w=hw_arm)
        self.parts.append(PartSpec("shin_left", "SIDE_shin", sh, screws=12, qty_note="x2（左右同一個檔案）",
                                   print_note="側立列印：手臂的 21 mm 寬度朝上，層線沿小腿長度"))

        # foot
        fp = lay.p["legs"]["foot"]
        L, W = fp["length"], fp["width"]
        plate_bot = fp["pad_thick"] - 0.5
        plate_top = plate_bot + fp["plate"]
        ft = self.arm_pair("foot_left", "SIDE_foot", A, Y, nZ, z_an - plate_top + 0.5)
        ft += box(-L / 2, L / 2, y - W / 2, y + W / 2, plate_bot, plate_top)
        for sy in (1, -1):
            y0_, y1_ = sorted((y + sy * ai, y + sy * ao))
            ft += box(-15, 15, y0_, y1_, plate_top - 0.5, plate_top + 6.5)
        px, py = fp["pad_size"]
        ix, iy = fp["pad_inset"]
        pads = []
        for cx in (-1, 1):
            for cy in (-1, 1):
                c = (cx * (L / 2 - ix - px / 2), y + cy * (W / 2 - iy - py / 2))
                ft -= box(c[0] - px / 2 - 0.1, c[0] + px / 2 + 0.1, c[1] - py / 2 - 0.1, c[1] + py / 2 + 0.1,
                          plate_bot - 1, plate_bot + 0.5)
                pads.append(box(c[0] - px / 2, c[0] + px / 2, c[1] - py / 2, c[1] + py / 2, 0, fp["pad_thick"]))
        self.parts.append(PartSpec("foot_left", "SIDE_foot", ft, screws=8, qty_note="x2（左右同一個檔案）",
                                   print_note="腳底朝下平放列印"))
        self.parts.append(PartSpec("foot_pads_left", "SIDE_foot", union(pads), "tpu",
                                   qty_note="每腳 4 片，TPU（共 8 片，STL 內含 4 片）",
                                   print_note="TPU 列印，用瞬間膠黏在腳底 0.5 mm 的凹槽"))

    def _torso(self):
        lay, p = self.lay, self.lay.p
        t = p["torso"]
        z_hr = lay.hip_roll_z
        x0, x1 = t["x"]
        y0, y1 = t["y"]
        z0, z1 = z_hr + t["z_rel"][0], z_hr + t["z_rel"][1]
        w = t["wall"]
        yh = p["legs"]["hip_spacing"] / 2
        ht, fit = self.ht, self.fit
        sh = box(x0, x1, y0, y1, z0, z1) - box(x0 + w, x1 - w, y0 + w, y1 - w, z0 - 1, z1 - w)
        bx0, bx1 = t["hip_bay_x"]
        cut_top = z_hr + t["hip_bay_top_rel"]
        for sy in (1, -1):
            a, b = sorted((sy * (y1 - w - 0.5), sy * (y1 + 1)))
            sh -= box(bx0, bx1, a, b, z0 - 1, cut_top)
        top_in = z1 - w
        # hip_roll servo holders: walls on the servo's front/back (horn) faces, hanging from the top
        hz0 = 13.0
        z_hole = z_hr + self.mount_x
        for sy in (1, -1):
            yc = sy * yh
            ya, yb = sorted((yc - sy * (self.hw + 0.5), sy * (y1 - w)))
            for sx in (1, -1):                                   # walls reach from the servo to the side wall and top
                xa, xb = sorted((sx * (ht + fit), sx * (ht + fit + 2.5)))
                sh += box(xa, xb, ya, yb, z_hr + hz0, top_in + 0.5)
            sh += self.holder("torso_left_hr" if sy > 0 else "torso_right_hr", "base", (0, yc, z_hr), (1, 0, 0),
                              (0, 0, 1), hz0, 2.5, crossbar=2.5)
            for dy in (-self.mount_y, self.mount_y):
                sh -= cyl(self.clr, 2 * (ht + fit + 3.5), (0, yc + dy, z_hole), "x")       # re-cut through the walls
                sh -= cyl(2.5, 2 * w + 2, (x1 - w / 2, yc + dy, z_hole), "x")          # screwdriver access, front
                sh -= cyl(2.5, 2 * w + 2, (x0 + w / 2, yc + dy, z_hole), "x")          # and rear
        el = p["electronics"]
        bosses = [(bx1 + 3.5, sy * (y1 - w - 3)) for sy in (1, -1)] + [(x1 - w - 3, sy * (y1 - w - 3)) for sy in (1, -1)]
        for bxc, byc in bosses:
            sh += box(bxc - 3, bxc + 3, byc - 3, byc + 3, z0 + 2.5, z0 + 9.5)          # floor plate sits below
            sh -= cyl(self.ins_r, 2 * self.ins_h, (bxc, byc, z0 + 2.5), "z")
            self.insert(f"floor_{bxc:.0f}_{byc:.0f}", "base", (bxc, byc, z0 + 2.5), (0, 0, 1))
            self.fasteners.append(Fastener(f"floor_{bxc:.0f}_{byc:.0f}", "base", (bxc, byc, z0 + self.head_h),
                                           (0, 0, 1), 5.0, "M2×5 盤頭", "insert", "底板 → 殼體"))
        bd = el["board"]
        bc = (bd["center"][0], bd["center"][1], z_hr + bd["center"][2])
        btop = bc[2] + bd["size"][2] / 2
        for dx in (-bd["holes"][0] / 2, bd["holes"][0] / 2):
            for dy in (-bd["holes"][1] / 2, bd["holes"][1] / 2):
                sh += cyl(3.0, top_in - btop + 0.5, (bc[0] + dx, bc[1] + dy, (btop + top_in + 0.5) / 2), "z")
                sh -= cyl(self.ins_r, 2 * self.ins_h, (bc[0] + dx, bc[1] + dy, btop), "z")
                self.insert(f"board_{dx:+.0f}_{dy:+.0f}", "base", (bc[0] + dx, bc[1] + dy, btop), (0, 0, 1))
                self.fasteners.append(Fastener(f"board_{dx:+.0f}_{dy:+.0f}", "base", (bc[0] + dx, bc[1] + dy, btop - 1.6),
                                               (0, 0, 1), 5.0, "M2×5 盤頭", "insert", "開發板（PCB 1.6 mm）→ 殼體"))
        im = el["imu"]
        ic = (im["center"][0], im["center"][1], z_hr + im["center"][2])
        itop = ic[2] + im["size"][2] / 2
        for dy in (-im["holes"] / 2, im["holes"] / 2):
            sh += cyl(3.0, top_in - itop + 0.5, (ic[0], dy, (itop + top_in + 0.5) / 2), "z")
            sh -= cyl(self.ins_r, 2 * self.ins_h, (ic[0], dy, itop), "z")
            self.insert(f"imu_{dy:+.0f}", "base", (ic[0], dy, itop), (0, 0, 1))
            self.fasteners.append(Fastener(f"imu_{dy:+.0f}", "base", (ic[0], dy, itop - 1.6), (0, 0, 1), 5.0,
                                           "M2×5 盤頭", "insert", "IMU（PCB 1.6 mm）→ 殼體"))
        sh -= box(bc[0] - 18, bc[0] + 18, y0 - 1, y0 + w + 1, bc[2] - 5, bc[2] + 5)          # board USB
        sh -= box(x1 - w - 1, x1 + 1, -8, 8, bc[2] - 3, bc[2] + 3)                           # SD card
        zt = z_hr + p["tail"]["z_rel"]
        for dy in (-14, 14):
            for dz in (-11, 11):
                sh -= cyl(self.clr, 2 * w + 2, (x0 + w / 2, dy, zt + dz), "x")             # tail mount screws
        sh -= cyl(3.5, 2 * w + 2, (x0 + w / 2, 0, zt + 18.5), "x")                         # tail cable
        for vx in range(30, 86, 8):
            sh -= box(vx, vx + 3, -25, 25, z1 - w - 1, z1 + 1)
        self.parts.append(PartSpec("torso_shell", "base", sh, qty_note="x1",
                                   print_note="頂面朝下列印（開口朝上）；hip_roll 馬達上方的橫樑是短橋接"))
        fx0, fx1 = bx1, x1 - w - 0.2
        fl = box(fx0, fx1, y0 + w + 0.2, y1 - w - 0.2, z0, z0 + 2.5)
        for bxc, byc in bosses:
            fl -= cyl(self.clr, 10, (bxc, byc, z0), "z")
            fl -= cyl(self.head_r + 0.2, 2 * self.head_h, (bxc, byc, z0), "z")
        for sx in (40, 75):
            for sy in (1, -1):
                fl -= box(sx - 10, sx + 10, sy * 22 - 1.5, sy * 22 + 1.5, z0 - 1, z0 + 4)    # battery strap slots
        fl -= box(fx1 - 12, fx1 - 4, -8, 8, z0 - 1, z0 + 4)                                  # battery lead slot
        self.parts.append(PartSpec("torso_floor", "base", fl, screws=4, qty_note="x1",
                                   print_note="平放列印；4 顆 M2 螺絲鎖進軀幹底部的熱熔螺母，拆下即可換電池"))

    def _tail(self):
        lay, p = self.lay, self.lay.p
        tl = p["tail"]
        T = lay.joint("tail_yaw").origin
        Q = lay.joint("tail_pitch").origin
        ao = self.arm_out
        x0 = p["torso"]["x"][0]
        zt = T[2]
        X, Y, nX, nZ = (1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, 0, -1)
        self.servo("servo_tail_yaw", "base", T, nZ, X)
        self.servo("servo_tail_pitch", "tail_yaw_link", Q, Y, X)

        # tail mount: holder for the tail_yaw servo; the backplate behind its far end bolts to the torso
        bp0 = T[0] + self.far + 0.5
        tm = self.holder("tail_mount", "base", T, nZ, X, 19.0, 3.0, half_w=12.0, crossbar=0)
        tm += box(bp0, x0, -18, 18, zt - 15.5, zt + 22.0)
        for dy in (-14, 14):
            for dz in (-11, 11):
                tm -= cyl(self.ins_r, 2 * self.ins_h, (x0, dy, zt + dz), "x")
                self.insert(f"tail_mount_{dy:+.0f}_{dz:+.0f}", "base", (x0, dy, zt + dz), (-1, 0, 0))
                w = p["torso"]["wall"]
                self.fasteners.append(Fastener(f"tail_mount_{dy:+.0f}_{dz:+.0f}", "base", (x0 + w, dy, zt + dz), (-1, 0, 0),
                                               6.0, "M2×6 盤頭", "insert", "尾巴座 → 殼體後牆（從殼體內側鎖）"))
        tm -= cyl(3.5, 30, (x0, 0, zt + 18.5), "x")                                          # tail cable
        self.parts.append(PartSpec("tail_mount", "base", tm, screws=8, qty_note="x1",
                                   print_note="背板貼床列印（上下夾板朝上長出）"))

        # tail_yaw bracket: clamps the yaw horns, holds the tail_pitch servo
        x_end = Q[0] + 18.0
        yb = self.arm_pair("tail_yaw_bracket", "tail_yaw_link", T, nZ, nX, T[0] - x_end + 0.5)
        yb += self.holder("tail_yaw_bracket_tp", "tail_yaw_link", Q, Y, X, 18.0, 2.5, half_w=ao, crossbar=2.4)
        self.parts.append(PartSpec("tail_yaw_bracket", "tail_yaw_link", yb, screws=12, qty_note="x1",
                                   print_note="側立列印（y 面貼床）"))

        # tail_pitch bracket with the carbon-rod socket
        rod = tl["rod"]
        blk1 = Q[0] - 15.0                        # clears the servo near-end corners (r = 13.8)
        blk0 = blk1 - 6.0 - rod["insert_depth"] - 4.0
        pb = self.arm_pair("tail_pitch_bracket", "tail_pitch_link", Q, Y, nX, Q[0] - blk1 + 6.0)
        pb += box(blk1 - 6.0, blk1, -ao, ao, zt - 10.5, zt + 10.5)
        pb += cyl(rod["diameter"] / 2 + 3.5, blk1 - 6.0 - blk0, ((blk0 + blk1 - 6.0) / 2, 0, zt), "x")
        pb -= cyl(rod["diameter"] / 2 + 0.15, 2 * rod["insert_depth"], (blk0, 0, zt), "x")
        xs = blk0 + rod["insert_depth"] / 2
        top = zt + rod["diameter"] / 2 + 3.5
        pb -= cyl(self.ins_r, 20, (xs, 0, top), "z")          # insert hole, continues into the rod bore
        self.insert("rod_socket", "tail_pitch_link", (xs, 0, top), (0, 0, -1))
        self.fasteners.append(Fastener("rod_socket", "tail_pitch_link", (xs, 0, top), (0, 0, -1), 4.0,
                                       "M2×4 止付螺絲", "rod", "碳管 → pitch 支架（另加 AB 膠）"))
        self.parts.append(PartSpec("tail_pitch_bracket", "tail_pitch_link", pb, screws=9, qty_note="x1",
                                   print_note="側立列印；碳管用 AB 膠固定，M2 止付螺絲（熱熔螺母）輔助"))

        tip = tl["tip"]
        xc = Q[0] - tl["length"]
        tx1 = xc + tip["length"] / 2
        rod_x0, rod_x1 = blk0 + rod["insert_depth"], tx1 - tip["socket"]
        r_o, r_i = rod["diameter"] / 2, rod["inner"] / 2
        tube = cyl(r_o, rod_x0 - rod_x1, ((rod_x0 + rod_x1) / 2, 0, zt), "x") - \
            cyl(r_i, rod_x0 - rod_x1 + 2, ((rod_x0 + rod_x1) / 2, 0, zt), "x")
        self.parts.append(PartSpec("tail_rod", "tail_pitch_link", tube, "bought",
                                   mass=rod["linear_density"] * (rod_x0 - rod_x1),
                                   qty_note=f"碳纖維管 Ø{rod['diameter']:.0f}/{rod['inner']:.0f} mm，長 {rod_x0 - rod_x1:.0f} mm"))
        th = cyl(tip["diameter"] / 2, tip["length"], (xc, 0, zt), "x")
        sk = tip["socket"]
        th -= cyl(r_o + 0.15, 2 * sk, (tx1, 0, zt), "x")
        bal_r = tip["diameter"] / 2 - 3.0
        bal_len = tip["length"] - sk - 2.0
        bal_c = tx1 - tip["length"] + bal_len / 2 - 0.5
        th -= cyl(bal_r, bal_len + 1, (bal_c, 0, zt), "x")
        # set screw: an access counterbore down to an insert seated just above the rod socket
        xs = tx1 - sk / 2
        r_ins = r_o + 0.15 + 4.0                              # insert top, 4 mm above the rod surface
        r_tip = tip["diameter"] / 2
        th -= cyl(2.5, r_tip - r_ins + 1, (xs, 0, zt + (r_tip + r_ins) / 2 + 0.5), "z")
        th -= cyl(self.ins_r, r_ins - r_o + 0.5, (xs, 0, zt + (r_ins + r_o) / 2 - 0.25), "z")
        self.insert("tail_tip", "tail_pitch_link", (xs, 0, zt + r_ins), (0, 0, -1))
        self.fasteners.append(Fastener("tail_tip", "tail_pitch_link", (xs, 0, zt + r_ins), (0, 0, -1), 5.0,
                                       "M2×5 止付螺絲", "rod", "尾端配重座 → 碳管（另加 AB 膠）"))
        self.parts.append(PartSpec("tail_tip", "tail_pitch_link", th, screws=1, qty_note="x1",
                                   print_note="圓柱軸向立著列印；後端 Ø20 × 20 mm 孔放鋼配重，AB 膠封口"))
        self.parts.append(PartSpec("tail_ballast", "tail_pitch_link", cyl(bal_r - 0.1, bal_len, (bal_c + 0.5, 0, zt), "x"),
                                   "bought", mass=None, qty_note="鋼配重，質量 = tip.mass − 列印座質量"))
