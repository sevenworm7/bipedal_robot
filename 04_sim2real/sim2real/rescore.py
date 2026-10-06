r"""Re-score saved evaluations (``episodes.pt``) with the sim-to-real success criterion; no simulation needed.

Usage:
    tools\isaacpy.cmd 04_sim2real\sim2real\rescore.py outputs\robot_v1\sim2real\eval_*_teacher_final_*
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import torch  # noqa: E402

from sim2real.scoring import summarize  # noqa: E402


def main(paths: list[str]):
    for p in paths:
        d = Path(p)
        stats = json.loads((d / "stats.json").read_text(encoding="utf-8"))
        ep = torch.load(d / "episodes.pt")
        scen = stats.get("scenario", "")
        res = summarize(ep, stats["env"]["episode_s"], speed_check=not scen.startswith("gait"))
        stats["sim2real_criterion"] = res
        (d / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
        print(f"{d.name:55s} {json.dumps(res)}")


if __name__ == "__main__":
    main(sys.argv[1:])
