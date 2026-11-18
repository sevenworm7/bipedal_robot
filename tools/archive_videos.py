"""Compress selected evaluation videos from outputs/ into docs/videos/ (tracked by git LFS).

Reads docs/videos/manifest.csv (columns: dest, source, note). For each row, re-encodes the
source with H.264 (same resolution, CRF 28, no audio) into docs/videos/<dest>.
Existing destinations are never overwritten: archived videos are append-only.

Usage (project root):
    tools\\isaacpy.cmd tools\\archive_videos.py            # archive missing entries
    tools\\isaacpy.cmd tools\\archive_videos.py --dry_run  # only list what would be done

ffmpeg comes from imageio_ffmpeg (bundled with Isaac Sim) or the FFMPEG environment variable.
"""

import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIDEO_DIR = ROOT / "docs" / "videos"
CRF = 28
PRESET = "slow"


def find_ffmpeg() -> str:
    if os.environ.get("FFMPEG"):
        return os.environ["FFMPEG"]
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        sys.exit("ffmpeg not found: install imageio_ffmpeg or set FFMPEG")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", default=str(VIDEO_DIR / "manifest.csv"))
    ap.add_argument("--dry_run", action="store_true")
    args = ap.parse_args()

    with open(args.manifest, encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    ffmpeg = None if args.dry_run else find_ffmpeg()
    total_in = total_out = 0
    missing = []
    for row in rows:
        src = ROOT / row["source"]
        dst = VIDEO_DIR / row["dest"]
        if dst.exists():
            print(f"skip (exists)  {row['dest']}  {dst.stat().st_size / 1e6:.1f} MB")
            total_out += dst.stat().st_size
            continue
        if not src.exists():
            print(f"MISSING source {row['source']}")
            missing.append(row["source"])
            continue
        if args.dry_run:
            print(f"would encode   {row['dest']}  ({src.stat().st_size / 1e6:.1f} MB)")
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        tmp = dst.with_suffix(".tmp.mp4")
        cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-i", str(src),
               "-c:v", "libx264", "-crf", str(CRF), "-preset", PRESET, "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", "-an", str(tmp)]
        subprocess.run(cmd, check=True)
        tmp.rename(dst)
        total_in += src.stat().st_size
        total_out += dst.stat().st_size
        print(f"encoded        {row['dest']}  {src.stat().st_size / 1e6:.1f} -> {dst.stat().st_size / 1e6:.1f} MB")

    print(f"\n{len(rows)} entries; archived total {total_out / 1e6:.1f} MB"
          + (f" (this run read {total_in / 1e6:.1f} MB)" if total_in else ""))
    if missing:
        sys.exit(f"{len(missing)} source file(s) missing")


if __name__ == "__main__":
    main()
