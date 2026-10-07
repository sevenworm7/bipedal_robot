"""Evaluation-video HUD and recorder (spec: /eval-video skill). Shared by rule-based and training.

Usage::

    rec = EvalVideoRecorder(out_dir, exp="exp1", tag="rule_based", label="exp1 / rule_based")
    while running:
        ...env.step(...)
        if rec.wants_frame(sim_time):
            rec.add_frame(env.render(), HudState(...))
    path = rec.close()

Frames are sampled on **simulated** time at ``fps`` so playback speed is real-time regardless of
the control rate. No torch / Isaac imports: this module can be unit-tested standalone.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_FONT_CANDIDATES = ("C:/Windows/Fonts/consolab.ttf", "C:/Windows/Fonts/consola.ttf", "DejaVuSansMono-Bold.ttf")


def _font(size: int) -> ImageFont.ImageFont:
    for f in _FONT_CANDIDATES:
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            continue
    return ImageFont.load_default()


@dataclass
class HudState:
    """Values shown on one frame (all from the same simulation step)."""

    label: str
    """Version tag, e.g. ``"exp1 / rule_based"`` or ``"exp1 / iter2 / ppo"``."""
    sim_time: float
    """[s]"""
    steps: int
    distance: float
    """along the target heading [m]"""
    lateral: float
    """signed lateral offset from the straight line [m]"""
    vx: float
    """forward speed [m/s]"""
    status: str = "WALKING"
    """WALKING / FALLEN / TIMEOUT / DONE"""
    trajectory: list[tuple[float, float]] = field(default_factory=list)
    """(distance, lateral) history for the mini-map."""
    command: str = ""
    """Current command when it can change mid-episode (04_sim2real), e.g. ``0.10 m/s``, ``STAND``, ``IN PLACE``;
    shown on the status line. Empty (02 / 03) keeps the original layout."""


def format_time(t: float) -> str:
    m, s = divmod(max(t, 0.0), 60.0)
    return f"{int(m):02d}:{s:05.2f}"


def draw_hud(frame: np.ndarray, hud: HudState) -> np.ndarray:
    """Return a copy of ``frame`` (H, W, 3 uint8 RGB) with the HUD drawn on it."""
    img = Image.fromarray(np.ascontiguousarray(frame[..., :3]))
    h, w = frame.shape[:2]
    scale = h / 720.0
    font = _font(int(26 * scale))
    lines = [
        f"{hud.label:<20s} t = {format_time(hud.sim_time)}",
        f"steps   {hud.steps:>6d}     dist  {hud.distance:6.2f} m",
        f"lateral {hud.lateral:+6.2f} m   v_x   {hud.vx:6.2f} m/s",
        f"status  {hud.status}" + (f"   cmd now {hud.command}" if hud.command else ""),
    ]
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    pad, line_h = int(12 * scale), int(34 * scale)
    if draw.textlength(lines[0], font=font) + 2 * pad > w:
        # long labels (04_sim2real) would push the time off the frame: give the time its own line; labels that fit
        # (02 / 03) keep the original single-line layout
        lines = [hud.label, f"t = {format_time(hud.sim_time)}"] + lines[1:]
    box_w = int(max(draw.textlength(l, font=font) for l in lines)) + 2 * pad
    box_h = line_h * len(lines) + 2 * pad
    draw.rectangle((0, 0, box_w, box_h), fill=(0, 0, 0, 170))
    for i, line in enumerate(lines):
        color = (255, 90, 90, 255) if (line.startswith("status") and hud.status == "FALLEN") else (255, 255, 255, 255)
        draw.text((pad, pad + i * line_h), line, font=font, fill=color)

    # mini-map (bottom-right): top view of the path, lateral exaggerated x5
    mw, mh = int(220 * scale), int(160 * scale)
    x0, y0 = w - mw - pad, h - mh - pad
    draw.rectangle((x0, y0, x0 + mw, y0 + mh), fill=(0, 0, 0, 150))
    draw.line((x0 + 10, y0 + mh // 2, x0 + mw - 10, y0 + mh // 2), fill=(120, 120, 120, 255), width=1)
    small = _font(int(16 * scale))
    draw.text((x0 + 6, y0 + 4), "path (lateral x5)", font=small, fill=(200, 200, 200, 255))
    if hud.trajectory:
        traj = np.asarray(hud.trajectory, dtype=float)
        span = max(1.0, float(traj[:, 0].max()) * 1.1)
        px = x0 + 10 + (traj[:, 0] / span) * (mw - 20)
        py = y0 + mh / 2 - np.clip(traj[:, 1] * 5.0 / span * (mw - 20), -mh / 2 + 6, mh / 2 - 6)
        pts = list(zip(px.tolist(), py.tolist()))
        if len(pts) > 1:
            draw.line(pts, fill=(80, 200, 255, 255), width=max(2, int(3 * scale)))
        draw.ellipse((pts[-1][0] - 4, pts[-1][1] - 4, pts[-1][0] + 4, pts[-1][1] + 4), fill=(255, 200, 0, 255))

    if hud.status == "FALLEN":
        bw = max(4, int(10 * scale))
        draw.rectangle((0, 0, w - 1, h - 1), outline=(230, 30, 30, 255), width=bw)
    return np.asarray(Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB"))


def video_filename(exp: str, tag: str, date: str | None = None, k: int | None = None) -> str:
    """``<exp>_<tag>_eval_<YYYYMMDD>[_<k>].mp4``, e.g. ``exp1_iter3_ppo_eval_20260918.mp4``."""
    date = date or time.strftime("%Y%m%d")
    return f"{exp}_{tag}_eval_{date}" + (f"_{k}" if k is not None else "") + ".mp4"


class EvalVideoRecorder:
    """Writes an H.264 mp4 at ``fps`` frames per *simulated* second."""

    def __init__(self, out_dir: str | Path, exp: str, tag: str, k: int | None = None, fps: int = 30):
        self.path = Path(out_dir) / video_filename(exp, tag, k=k)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self._next_t = 0.0
        self._writer = imageio.get_writer(
            str(self.path), fps=fps, codec="libx264", quality=8, macro_block_size=1, pixelformat="yuv420p"
        )
        self.frames = 0

    def wants_frame(self, sim_time: float) -> bool:
        return sim_time + 1e-9 >= self._next_t

    def add_frame(self, frame: np.ndarray, hud: HudState) -> None:
        if frame is None:
            raise RuntimeError("env.render() returned None: create the env with render_mode='rgb_array' and --enable_cameras")
        self._writer.append_data(draw_hud(frame, hud))
        self.frames += 1
        self._next_t += 1.0 / self.fps

    def close(self) -> Path:
        self._writer.close()
        return self.path
