"""Stage 2 / distillation: behaviour cloning of an MLP policy from HDF5 datasets.

* Network: the rsl_rl ``MLPModel`` actor (``policy.make_actor``), so the checkpoint loads directly into PPO.
  Observation normalisation = dataset mean / variance of the training split (frozen afterwards).
* Loss: MSE between the MLP output and the label action (normalised action units).
* Split: by episode (``BCCfg.val_frac`` of the episodes held out), never by step.
* Mixing (distillation): ``--prev`` datasets are subsampled to ``FilterCfg.prev_mix_frac`` of the steps.
* After training, the best-validation model is rolled out on the fixed evaluation set in simulation.

Output: ``iter<N>_<bc|distill>_<time>/{model.pt, stats.json, config.yaml, tb/}``.

Usage:
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\bc.py --iter 0 --data <iter0_collect>/dataset.h5
    tools\\isaacpy.cmd 03_training_pipeline\\training_pipeline\\bc.py --iter 1 --stage distill --data <filtered.h5> --prev <iter0 data>
"""

import argparse
import math
import sys
import time
from pathlib import Path

# project root (shared), 02_rule_based (rule_based teacher), 03_training_pipeline (training_pipeline)
_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_ROOT), str(_ROOT / "02_rule_based"), str(_ROOT / "03_training_pipeline")]

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--iter", type=int, default=0)
parser.add_argument("--stage", type=str, default="bc", choices=("bc", "distill"))
parser.add_argument("--data", type=str, nargs="+", required=True, help="Main dataset(s) (HDF5).")
parser.add_argument("--prev", type=str, nargs="*", default=[], help="Previous-round dataset(s) to mix in.")
parser.add_argument("--epochs", type=int, default=None)
parser.add_argument("--no_eval", action="store_true", help="Skip the rollout evaluation in simulation.")
parser.add_argument("--tag", type=str, default="")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch.utils.tensorboard import SummaryWriter  # noqa: E402

from training_pipeline.common import dump_config, make_env, run_dir, write_json  # noqa: E402
from training_pipeline.config import PipelineCfg  # noqa: E402
from training_pipeline.dataset import load_h5, merge, select_episodes  # noqa: E402
from training_pipeline.evaluation import evaluate_agent, summary_line  # noqa: E402
from training_pipeline.policy import PolicyAgent, actor_mean, make_actor, save_actor, set_normalizer  # noqa: E402


def load_all(paths: list[str]) -> dict:
    parts = [load_h5(Path(p)) for p in paths]
    for p in parts:
        p.pop("attrs", None)
    return merge(parts) if len(parts) > 1 else parts[0]


def mix_previous(main: dict, prev: dict, frac: float, rng: np.random.Generator) -> tuple[dict, dict]:
    """Add whole episodes of ``prev`` until they make up ``frac`` of the steps."""
    n_main = len(main["obs"])
    target = frac / (1 - frac) * n_main
    order = rng.permutation(len(prev["episodes"]["length"]))
    cum = np.cumsum(prev["episodes"]["length"][order])
    idx = order[: int(np.searchsorted(cum, target)) + 1]
    sub = select_episodes(prev, idx)
    return merge([main, sub]), {"prev_episodes": int(len(idx)), "prev_steps": int(len(sub["obs"]))}


def main():
    cfg = PipelineCfg()
    b = cfg.bc
    torch.manual_seed(b.seed + args_cli.iter)
    rng = np.random.default_rng(b.seed + args_cli.iter)
    dev = "cuda:0"
    out = run_dir(cfg, args_cli.iter, args_cli.stage, args_cli.tag)
    dump_config(out, cfg, vars(args_cli))

    data = load_all(args_cli.data)
    mix_info = {}
    if args_cli.prev:
        data, mix_info = mix_previous(data, load_all(args_cli.prev), cfg.filter.prev_mix_frac, rng)
    ep = data["episodes"]
    n_ep = len(ep["length"])
    perm = rng.permutation(n_ep)
    n_val = max(1, int(round(b.val_frac * n_ep)))
    val_idx, train_idx = np.sort(perm[:n_val]), np.sort(perm[n_val:])
    tr, va = select_episodes(data, train_idx), select_episodes(data, val_idx)
    x_tr = torch.as_tensor(tr["obs"], device=dev)
    y_tr = torch.as_tensor(tr["action"], device=dev)
    x_va = torch.as_tensor(va["obs"], device=dev)
    y_va = torch.as_tensor(va["action"], device=dev)
    print(f"[bc] episodes train {len(train_idx)} val {len(val_idx)} | steps train {len(x_tr)} val {len(x_va)} {mix_info}", flush=True)

    actor = make_actor(cfg.actor, x_tr.shape[1], y_tr.shape[1], dev)
    set_normalizer(actor, x_tr.mean(0), x_tr.var(0, unbiased=False), len(x_tr))
    opt = torch.optim.Adam(actor.mlp.parameters(), lr=b.lr, weight_decay=b.weight_decay)
    epochs = args_cli.epochs or b.epochs
    steps_per_epoch = math.ceil(len(x_tr) / b.batch_size)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs * steps_per_epoch, eta_min=b.lr_min)
    writer = SummaryWriter(str(out / "tb"))

    @torch.no_grad()
    def val_loss() -> float:
        tot = 0.0
        for i in range(0, len(x_va), 65536):
            tot += torch.nn.functional.mse_loss(actor_mean(actor, x_va[i:i + 65536]), y_va[i:i + 65536], reduction="sum").item()
        return tot / (len(x_va) * y_va.shape[1])

    best, best_state, history = float("inf"), None, []
    t0 = time.time()
    for epoch in range(epochs):
        actor.train()
        order = torch.randperm(len(x_tr), device=dev)
        tot = 0.0
        for i in range(steps_per_epoch):
            idx = order[i * b.batch_size:(i + 1) * b.batch_size]
            loss = torch.nn.functional.mse_loss(actor_mean(actor, x_tr[idx]), y_tr[idx])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            tot += loss.item() * len(idx)
        actor.eval()
        tl, vl = tot / len(x_tr), val_loss()
        history.append({"epoch": epoch, "train_mse": tl, "val_mse": vl})
        writer.add_scalar("bc/train_mse", tl, epoch)
        writer.add_scalar("bc/val_mse", vl, epoch)
        writer.add_scalar("bc/lr", sched.get_last_lr()[0], epoch)
        if vl < best:
            best, best_state = vl, {k: v.clone() for k, v in actor.state_dict().items()}
        if epoch % 5 == 0 or epoch == epochs - 1:
            print(f"[bc] epoch {epoch:3d} train {tl:.5f} val {vl:.5f} (best {best:.5f})", flush=True)
    writer.close()
    actor.load_state_dict(best_state)
    label_var = float(y_va.var(0, unbiased=False).mean())
    meta = {"stage": args_cli.stage, "iteration": args_cli.iter, "data": args_cli.data, "prev": args_cli.prev,
            "val_mse": best, "label_var": label_var, "actor": cfg.actor.__dict__}
    save_actor(out / "model.pt", actor, meta)
    stats = {
        "train": {"episodes_train": int(len(train_idx)), "episodes_val": int(len(val_idx)), "steps_train": int(len(x_tr)),
                  "steps_val": int(len(x_va)), **mix_info, "best_val_mse": round(best, 6),
                  "val_r2": round(1 - best / max(label_var, 1e-9), 4), "epochs": epochs,
                  "wall_time_s": round(time.time() - t0, 1), "history": history},
    }
    print(f"[bc] best val mse {best:.5f} (label variance {label_var:.4f}, R2 {stats['train']['val_r2']:.3f})", flush=True)
    del x_tr, y_tr, x_va, y_va, data, tr, va

    if not args_cli.no_eval:
        env = make_env(cfg, cfg.eval.episodes, cfg.eval.episode_s, cfg.eval.seed)
        ev, _ = evaluate_agent(env, PolicyAgent(actor), cfg.eval)
        stats["eval"] = ev
        print(f"[bc] eval: {summary_line(ev)}", flush=True)
        env.close()
    write_json(out / "stats.json", stats)
    print(f"[bc] output: {out}")


if __name__ == "__main__":
    main()
    app.close()
