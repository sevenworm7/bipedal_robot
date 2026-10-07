"""Evaluation videos for any agent (rule-based teacher or learned policy), ``/eval-video`` format.

``record_class_videos`` runs the robots side by side (slow / medium / fast commands, the rest random),
records one follow-camera stream per robot with the HUD, then keeps one successful video per speed class
plus one failure case (a fall if any, otherwise an unsuccessful episode) and deletes the rest.
Episodes are not restarted: recording of a robot stops when its first episode ends, and the final hold
reuses the last live frame (the env auto-resets on done).

Requires an env created with ``cfg.eval_camera = True`` and ``cfg.record_episodes = True`` and the
app started with ``--enable_cameras``.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import torch

from .metrics import rule_based_success
from .video_overlay import EvalVideoRecorder, HudState

CLASS_NAMES = ("slow", "medium", "fast")


def record_class_videos(env, agent, out_dir: Path, exp: str, label: str, tag: str,
                        speeds: list[float], freq: float, success_fn=None) -> dict:
    """Record and select the videos.

    Args:
        env: ``BipedWalkEnv`` with eval camera and episode recording.
        agent: object with ``reset(env_ids)`` and ``compute_action(obs)``.
        out_dir: run folder; videos go to ``out_dir / "videos"``, the summary to ``out_dir / "videos.json"``.
        exp: experiment prefix of the file names (``exp1``).
        label: HUD version label, e.g. ``exp1 / rule_based`` or ``exp1 / iter2 / ppo``.
        tag: file-name tag, e.g. ``rule_based`` or ``iter2_ppo`` -> ``exp1_iter2_ppo_<class>_eval_<date>.mp4``.
        speeds: slow / medium / fast commands [m/s].
        freq: gait frequency of the class robots [Hz].
        success_fn: ``fn(ep) -> bool tensor`` deciding which episodes count as successful (default: the rule-based
            criterion of 02 / 03; 04_sim2real passes its own criterion that allows corrective steps).
    """
    n = env.num_envs
    per = max(1, n // (len(speeds) + 1))
    cls = [i // per if i // per < len(speeds) else -1 for i in range(n)]  # -1 = random command
    obs = env.reset()[0]["policy"]
    env.completed_episodes.clear()  # see shared/rollout.py: drop summaries of episodes ended by this reset
    for i, c in enumerate(cls):
        if c >= 0:
            env.cmd_vx[i] = speeds[c]
            env.gait_freq[i] = freq
    obs = env._get_observations()["policy"]  # refresh the command / clock in the observation
    agent.reset()

    vid_dir = Path(out_dir) / "videos"
    date = time.strftime("%Y%m%d")
    recs = [EvalVideoRecorder(vid_dir / "all", exp=exp, tag=f"{tag}_env{i:02d}") for i in range(n)]
    trajs = [[] for _ in range(n)]
    last_frame = [None] * n
    active = torch.ones(n, dtype=torch.bool)
    summaries: dict[int, dict] = {}
    names = {-1: "random"} | dict(enumerate(CLASS_NAMES))
    for step in range(int(env.max_episode_length) + 2):
        act = agent.compute_action(obs)
        o, _, term, trunc, _ = env.step(act)
        obs = o["policy"]
        done = (term | trunc).cpu()
        for ep in env.completed_episodes:
            for j, eid in enumerate(ep["env_ids"].tolist()):
                if eid not in summaries:
                    summaries[eid] = {k: v[j].item() if v[j].numel() == 1 else v[j].tolist() for k, v in ep.items()}
        env.completed_episodes.clear()
        t = (step + 1) * env.step_dt
        for i in range(n):
            if not active[i]:
                continue
            tr = env.tracker
            s = summaries.get(i, {})
            if bool(done[i]):
                hud_steps, hud_dist, hud_lat = s.get("steps", 0), s.get("distance", 0.0), s.get("lateral", 0.0)
                status = "FALLEN" if bool(term[i]) else "FINISHED"
                cmd = s.get("cmd_vx", 0.0)
            else:
                hud_steps, hud_dist, hud_lat = float(tr.steps[i]), float(tr.distance[i]), float(tr.lateral[i])
                status = "WALKING"
                cmd = float(env.cmd_vx[i])
            trajs[i].append((hud_dist, hud_lat))
            if recs[i].wants_frame(t) or status != "WALKING":
                now = ""
                if getattr(env, "_gait_var_on", False):  # commands change mid-episode (04_sim2real gait variation)
                    c_now, f_now = float(env.cmd_vx[i]), float(env.gait_freq[i])
                    now = "STAND" if f_now == 0 else ("IN PLACE" if c_now == 0 else f"{c_now:.3f} m/s")
                hud = HudState(label=f"{label} / {names[cls[i]]} cmd {cmd:.3f} m/s", sim_time=t, steps=int(hud_steps),
                               distance=hud_dist, lateral=hud_lat, vx=float(env.base_lin_vel[i, 0]), status=status,
                               trajectory=list(trajs[i]), command=now)
                if status == "WALKING" or last_frame[i] is None:
                    last_frame[i] = env.capture_frame(i)
                recs[i].add_frame(last_frame[i], hud)
                if status != "WALKING":
                    for _ in range(30):  # hold the last frame for 1 s
                        recs[i].add_frame(last_frame[i], hud)
            if bool(done[i]):
                active[i] = False
        if not bool(active.any()):
            break

    paths = [r.close() for r in recs]
    keep, report = {}, []
    for i in range(n):
        s = summaries.get(i)
        if s is None:
            continue
        ep = {k: torch.tensor([v]) for k, v in s.items() if not isinstance(v, list)}
        ok = bool((success_fn or rule_based_success)(ep)[0])
        report.append({"env": i, "class": names[cls[i]], "success": ok, **{k: s[k] for k in
                       ("cmd_vx", "steps", "distance", "lateral", "max_abs_lateral", "time", "fell", "mean_vx")}})
        if cls[i] >= 0 and ok and names[cls[i]] not in keep:
            keep[names[cls[i]]] = i
        if s["fell"] > 0.5 and "failure" not in keep:
            keep["failure"] = i
    if "failure" not in keep:  # no fall: keep an unsuccessful (e.g. too short / too slow) episode instead
        bad = [r["env"] for r in report if not r["success"]]
        if bad:
            keep["failure"] = bad[0]
    final = {}
    for name, i in keep.items():
        dst = vid_dir / f"{exp}_{tag}_{name}_eval_{date}.mp4"
        Path(paths[i]).replace(dst)
        final[name] = {"env": i, "video": str(dst)}
    for p in (vid_dir / "all").glob("*.mp4"):
        p.unlink()
    (vid_dir / "all").rmdir()
    result = {"kept": final, "episodes": report}
    (Path(out_dir) / "videos.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result
