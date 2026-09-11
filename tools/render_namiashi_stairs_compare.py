#!/usr/bin/env python
"""Render namiashi against three staircase rises, side by side.

Same terrain-blind gait, same run (0.20 m), same 10 steps -- only the rise
height changes: 0.10 m (43% of tuned stance height), 0.05 m, and 0.02 m
(below the fixed swing clearance the gait planner budgets on flat ground).
The point is to see, rather than assume, where that clearance mismatch stops
being the dominant effect.

Camera chases trunk x, y, *and* z per panel -- a climb moves the trunk up to
a metre, and each panel is on its own climb, so a shared or fixed-height
lookat would desync them.

    tools/render_namiashi_stairs_compare.py --root /tmp/nami_stairs \\
        --out stairs_compare.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model

PANELS = [
    ("rise = 0.10 m", "rise_10", "43% of tuned stance height"),
    ("rise = 0.05 m", "rise_05", "21% of tuned stance height"),
    ("rise = 0.02 m", "rise_02", "below the fixed swing clearance (0.035-0.045 m)"),
]


def panel_frames(root, sub, fps, seconds, w, h, period):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 1.9, -14.0, 132.0
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    win = period * math.ceil(0.8 / period)
    t = trace["t"]
    end = min(t[-1], t[0] + seconds)
    stamps = np.arange(t[0], end, 1.0 / fps)
    out = []
    with mujoco.Renderer(model, h, w) as r:
        for ts in stamps:
            i = min(int(np.searchsorted(t, ts)), len(t) - 1)
            data.qpos[0:3] = trace["root"][i, 0:3]
            data.qpos[3:7] = trace["root"][i, 3:7]
            for k, a in enumerate(adr):
                data.qpos[a] = trace["q"][i, k]
            mujoco.mj_forward(model, data)
            cam.lookat[:] = data.qpos[0:3]
            r.update_scene(data, cam, opt)
            out.append((r.render(), i, ts - t[0]))
    return out, trace


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_stairs")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--panel_w", type=int, default=560)
    ap.add_argument("--panel_h", type=int, default=440)
    ap.add_argument("--period", type=float, default=0.320)
    ap.add_argument("--cmd_vx", type=float, default=0.80)
    args = ap.parse_args()

    import imageio.v2 as imageio

    panels = []
    for label, sub, note in PANELS:
        print(f"  rendering {sub}")
        frames, trace = panel_frames(args.root, sub, args.fps, args.seconds,
                                     args.panel_w, args.panel_h, args.period)
        panels.append((label, note, frames, trace))

    n = min(len(p[2]) for p in panels)
    header_h = 84
    pw, ph = args.panel_w, args.panel_h
    W, H = pw * len(PANELS), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(24), font(19), font(15)
    win = args.period * math.ceil(0.8 / args.period)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  staircase rise comparison", font=f_title,
                fill=(240, 244, 250))
        dh.text((16, 46), "same gait, same run=0.20m, same 10 steps -- rise is "
                          "the only thing that changes", font=f_sub,
                fill=(150, 162, 180))

        for k, (label, note, frames, trace) in enumerate(panels):
            rgb, idx, elapsed = frames[i]
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(rgb), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")
            d.rectangle([0, 0, pw, 78], fill=(0, 0, 0, 178))
            d.text((12, 8), label, font=f_big, fill=(255, 255, 255))
            d.text((12, 38), note, font=f_sm, fill=(170, 180, 196))

            vx = body_frame_rates(trace, idx, win)[0]
            z = trace["root"][idx, 2]
            x = trace["root"][idx, 0]
            d.rectangle([0, ph - 30, pw, ph], fill=(0, 0, 0, 160))
            d.text((10, ph - 26), f"x={x:5.2f}m  z={z:.3f}m  vx={vx:+.2f} m/s",
                   font=f_med, fill=(215, 220, 232))
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
