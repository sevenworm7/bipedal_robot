"""Inspect a URDF and flag values that commonly break legged-robot simulation.

Stdlib only, so it runs with any Python 3.10+:

    tools\\isaacpy.cmd tools\\urdf_inspect.py path\\to\\robot.urdf
    tools\\isaacpy.cmd tools\\urdf_inspect.py path\\to\\robot.urdf --json

Checks: kinematic tree, per-link mass / total mass, joint axes & limits,
placeholder effort/velocity limits, non-positive-definite inertia, mesh
collisions (slow in PhysX), and chain lengths from base to each end link.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET


def _vec(text: str | None, default: str = "0 0 0") -> list[float]:
    return [float(v) for v in (text or default).split()]


def parse(path: str) -> dict:
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        mass = None
        inertia = None
        if inertial is not None:
            m = inertial.find("mass")
            mass = float(m.get("value")) if m is not None else None
            i = inertial.find("inertia")
            if i is not None:
                inertia = {k: float(i.get(k, 0.0)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        collision_types = []
        for col in link.findall("collision"):
            geom = col.find("geometry")
            if geom is not None and len(geom):
                collision_types.append(geom[0].tag)
        links[link.get("name")] = {"mass": mass, "inertia": inertia, "collision": collision_types}

    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": _vec(origin.get("xyz") if origin is not None else None),
            "rpy": _vec(origin.get("rpy") if origin is not None else None),
            "axis": _vec(joint.find("axis").get("xyz") if joint.find("axis") is not None else None, "1 0 0"),
            "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") else None,
            "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") else None,
            "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
            "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None,
        }
    return {"name": root.get("name"), "links": links, "joints": joints}


def check(model: dict) -> list[str]:
    warnings = []
    links, joints = model["links"], model["joints"]
    children = {j["child"] for j in joints.values()}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        warnings.append(f"expected exactly one root link, found {roots}")

    for name, link in links.items():
        if link["mass"] is None or link["mass"] <= 0:
            warnings.append(f"link {name}: missing or non-positive mass")
        inertia = link["inertia"]
        if inertia:
            ixx, iyy, izz = inertia["ixx"], inertia["iyy"], inertia["izz"]
            if min(ixx, iyy, izz) <= 0:
                warnings.append(f"link {name}: non-positive principal inertia")
            elif ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
                warnings.append(f"link {name}: inertia violates triangle inequality")
        if "mesh" in link["collision"]:
            warnings.append(f"link {name}: mesh collision (prefer box/capsule primitives for RL)")

    total = sum(l["mass"] or 0.0 for l in links.values())
    for name, link in links.items():
        if link["mass"] and link["mass"] > 0.4 * total and name not in roots:
            warnings.append(f"link {name}: {link['mass']:.2f} kg is {link['mass'] / total:.0%} of total mass")

    for name, j in joints.items():
        if j["type"] in ("revolute", "prismatic"):
            if j["lower"] is None or j["upper"] is None:
                warnings.append(f"joint {name}: missing position limits")
            if j["effort"] in (None, 0, 100) or j["velocity"] in (None, 0, 100):
                warnings.append(f"joint {name}: effort/velocity look like placeholders ({j['effort']}, {j['velocity']})")
            if abs(sum(a * a for a in j["axis"]) - 1.0) > 1e-3:
                warnings.append(f"joint {name}: axis not unit length {j['axis']}")
    return warnings


def chains(model: dict) -> list[tuple[list[str], float]]:
    """Return every root-to-leaf joint chain and its summed origin offset length [m]."""
    joints = model["joints"]
    by_parent: dict[str, list[str]] = {}
    for name, j in joints.items():
        by_parent.setdefault(j["parent"], []).append(name)
    children = {j["child"] for j in joints.values()}
    roots = [n for n in model["links"] if n not in children]
    out = []

    def walk(link: str, path: list[str], length: float) -> None:
        if link not in by_parent:
            out.append((path, length))
            return
        for jn in by_parent[link]:
            walk(joints[jn]["child"], path + [jn], length + math.dist(joints[jn]["xyz"], (0, 0, 0)))

    for r in roots:
        walk(r, [], 0.0)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf")
    ap.add_argument("--json", action="store_true", help="dump parsed model as JSON")
    args = ap.parse_args()

    model = parse(args.urdf)
    if args.json:
        json.dump(model, sys.stdout, indent=2)
        return 0

    links, joints = model["links"], model["joints"]
    total = sum(l["mass"] or 0.0 for l in links.values())
    actuated = [n for n, j in joints.items() if j["type"] not in ("fixed",)]
    print(f"robot: {model['name']}   links: {len(links)}   actuated joints: {len(actuated)}   total mass: {total:.3f} kg\n")

    print(f"{'joint':32s} {'type':9s} {'axis':16s} {'lower':>8s} {'upper':>8s} {'effort':>7s} {'vel':>6s}  origin xyz")
    for n, j in joints.items():
        lo = f"{math.degrees(j['lower']):7.1f}d" if j["lower"] is not None else "-"
        hi = f"{math.degrees(j['upper']):7.1f}d" if j["upper"] is not None else "-"
        axis = " ".join(f"{a:+.0f}" for a in j["axis"])
        xyz = " ".join(f"{v:+.4f}" for v in j["xyz"])
        print(f"{n:32s} {j['type']:9s} {axis:16s} {lo:>8s} {hi:>8s} {j['effort'] or '-':>7} {j['velocity'] or '-':>6}  {xyz}")

    print(f"\n{'link':28s} {'mass kg':>8s}  collision")
    for n, l in links.items():
        print(f"{n:28s} {l['mass'] or 0:8.3f}  {','.join(l['collision']) or '-'}")

    print("\nchains (sum of joint offsets, rough length indicator):")
    for path, length in chains(model):
        print(f"  {length:.3f} m  {' -> '.join(path)}")

    warnings = check(model)
    print(f"\nwarnings ({len(warnings)}):")
    for w in warnings:
        print(f"  - {w}")
    return 1 if warnings else 0


if __name__ == "__main__":
    sys.exit(main())
