"""Build the interactive 3D viewer of a model version (three.js page + per-link meshes + robot.json).

Run after build_parts.py and check_static.py (CAD environment):
    .venv_cad\\Scripts\\python.exe 01_robot_construction\\model\\generator\\build_viewer.py --version 1

Output (not in git, rebuild any time): outputs/robot_v<N>/01_model/viewer/
    index.html            the page (template: generator/viewer_template.html)
    robot.json            joints, link frames, masses, stance, report numbers (all from params + reports)
    meshes/<link>_<kind>.stl   printed parts and servos per link, link frame, mm
    meshes.json           the same STL files base64-encoded (what the page loads)
Also writes model/v<N>/viewer.html: a standalone copy with robot.json and the meshes inlined, for opening from disk
(VS Code / browser), fully offline: robot.json, the meshes and three.js r128 are all inlined.
The page is published as a claude.ai Artifact with the meshes and robot.json as supporting files.
"""
from __future__ import annotations

import argparse
import base64
import json
import shutil
import sys
import urllib.request
from pathlib import Path

from build123d import Pos, export_stl

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402
from parts import Builder  # noqa: E402

HERE = Path(__file__).resolve().parent
LIBS = ("https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js",
        "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js",
        "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js")
LIB_CACHE = P.PROJECT_ROOT / "outputs" / "_cache" / "three-r128"


def lib_source(url: str) -> str:
    """Library source for the standalone page, downloaded once into outputs/_cache (not in git)."""
    path = LIB_CACHE / url.rsplit("/", 1)[1]
    if not path.exists():
        LIB_CACHE.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(url, timeout=60) as r:
            path.write_bytes(r.read())
    return path.read_text(encoding="utf-8").replace("</script", "<\\/script")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", type=int, required=True)
    args = ap.parse_args()
    vdir = P.version_dir(args.version)
    lay = P.load(args.version)
    out = P.PROJECT_ROOT / "outputs" / f"robot_v{args.version}" / "01_model" / "viewer"
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    model = Builder(lay).build()
    origin = lay.link_origin

    groups: dict[tuple[str, str], list] = {}
    for s in model.parts:
        if s.kind == "keepout":
            continue
        kind = {"servo": "servo", "tpu": "tpu", "bought": "rod", "head": "rod"}.get(s.kind, "print")
        groups.setdefault((s.link, kind), []).append(s.solid)
    meshes = []
    for (link, kind), solids in groups.items():
        m = solids[0]
        for q in solids[1:]:
            m = m + q
        o = origin[link]
        name = f"{link}_{kind}.stl"
        export_stl(Pos(-o[0], -o[1], -o[2]) * m, str(out / "meshes" / name), tolerance=0.08, angular_tolerance=0.4)
        meshes.append({"link": link, "kind": kind, "file": f"meshes/{name}"})

    mp = json.loads((vdir / "reports/mass_properties.json").read_text(encoding="utf-8"))
    st = json.loads((vdir / "reports/static_check.json").read_text(encoding="utf-8"))
    p = lay.p
    fp = p["legs"]["foot"]
    data = {
        "name": p["robot"]["name"], "version": args.version, "git": mp["git_commit"], "generated": mp["generated"],
        "total_mass_g": mp["total_mass_g"],
        "links": {k: {"origin": v["origin_world_mm"], "mass_g": v["mass_g"], "com": v["com_mm"]} for k, v in mp["links"].items()},
        "joints": [{"name": j.name, "parent": j.parent, "child": j.child, "origin": list(j.origin), "axis": list(j.axis),
                    "lower": j.lower, "upper": j.upper} for j in lay.joints],
        "meshes": meshes,
        "stance": p["stance"],
        "foot": {"length": fp["length"], "width": fp["width"], "ankle_height": p["legs"]["ankle_height"]},
        "electronics": {k: {"size": v.get("size"), "center": [v["center"][0], v["center"][1], lay.hip_roll_z + v["center"][2]],
                            "mass": v["mass"], "model": v["model"]} for k, v in p["electronics"].items() if v.get("size")},
        "servo": {"stall": p["servo"]["torque_stall_6v"], "rated": p["servo"]["torque_rated_6v"]},
        "static": st,
    }
    (out / "robot.json").write_text(json.dumps(data, indent=1, default=float), encoding="utf-8")
    # artifacts do not serve binary STL -> ship the meshes base64-encoded in one JSON file
    packed = {m["file"]: base64.b64encode((out / m["file"]).read_bytes()).decode("ascii") for m in meshes}
    (out / "meshes.json").write_text(json.dumps(packed), encoding="utf-8")
    html = (HERE / "viewer_template.html").read_text(encoding="utf-8").replace("{{VERSION}}", str(args.version))
    (out / "index.html").write_text(html, encoding="utf-8")
    # standalone copy for opening from disk (file:// cannot fetch the JSON files): data inlined
    embed = (f"<script>window.__ROBOT__ = {json.dumps(data, default=float)};\n"
             f"window.__MESHES__ = {json.dumps(packed)};</script>\n")
    # the libraries are inlined too: editor previews (VS Code) may block CDN scripts
    for url in LIBS:
        tag = f'<script src="{url}"></script>'
        assert tag in html, url
        html = html.replace(tag, "<script>\n" + lib_source(url) + "\n</script>", 1)
    first = html.index("<script>")
    head = '<!doctype html>\n<meta charset="utf-8">\n<meta name="viewport" content="width=device-width, initial-scale=1">\n'
    standalone = head + html[:first] + embed + html[first:]
    (vdir / "viewer.html").write_text(standalone, encoding="utf-8")
    print(f"viewer written to {out} ({len(meshes)} meshes)")


if __name__ == "__main__":
    main()
