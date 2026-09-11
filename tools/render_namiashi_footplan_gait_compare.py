#!/usr/bin/env python
"""Render namiashi's terrain-footplan attempt on the 5 cm staircase, Trot
against Walk, side by side.

Same terrain-footplan correction (vertical clearance + horizontal snap onto
tread, both computed from ground-truth height -- see TerrainFootplanCfg in
tests/wbc_walk.rs), same 5 cm/0.20 m staircase. Only the gait's own timing
changes: Trot at 0.320 s period / 0.50 duty vs Walk at 0.500 s / 0.75 duty
(more feet down, more of the time, an order of magnitude slower). Both
reach and briefly hold tread-1 height before collapsing -- the point of this
clip is that switching to the slower, more-supported gait does not change
that outcome, even though it visibly damps the lateral drift.

    tools/render_namiashi_footplan_gait_compare.py --root /tmp/nami_stairs \\
        --out footplan_gait_compare.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model
from render_namiashi_stairs import draw_step_progress

PANELS = [
    ("Trot", "rise05_footplan", "T=0.320s duty=0.50", 0.320, 0.80),
    ("Walk", "rise05_footplan_walk", "T=0.500s duty=0.75", 0.500, 0.33),
]
RISE_M, N_STEPS = 0.05, 10


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
    stance_z = trace["root"][0, 2]
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
    return out, trace, stance_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_stairs")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--panel_w", type=int, default=580)
    ap.add_argument("--panel_h", type=int, default=440)
    args = ap.parse_args()

    import imageio.v2 as imageio

    panels = []
    for label, sub, note, period, cmd_vx in PANELS:
        print(f"  rendering {sub}")
        frames, trace, stance_z = panel_frames(
            args.root, sub, args.fps, args.seconds, args.panel_w, args.panel_h, period
        )
        panels.append((label, note, period, cmd_vx, frames, trace, stance_z))

    n = min(len(p[4]) for p in panels)
    header_h = 92
    pw, ph = args.panel_w, args.panel_h
    W, H = pw * len(PANELS), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(24), font(19), font(15)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  5 cm staircase  terrain footplan  Trot vs Walk",
                font=f_title, fill=(240, 244, 250))
        dh.text((16, 46), "same footplan (vertical clearance + horizontal snap, "
                          "ground-truth height) -- only gait timing differs",
                font=f_sub, fill=(150, 162, 180))

        for k, (label, note, period, cmd_vx, frames, trace, stance_z) in enumerate(panels):
            rgb, idx, elapsed = frames[i]
            win = period * math.ceil(0.8 / period)
            z = trace["root"][idx, 2]
            frame = draw_step_progress(rgb, z, stance_z, RISE_M, N_STEPS)
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(frame), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")
            d.rectangle([0, 0, pw, 78], fill=(0, 0, 0, 178))
            d.text((12, 8), label, font=f_big, fill=(255, 255, 255))
            d.text((12, 38), note, font=f_sm, fill=(170, 180, 196))

            vx = body_frame_rates(trace, idx, win)[0]
            x = trace["root"][idx, 0]
            d.rectangle([0, ph - 30, pw, ph], fill=(0, 0, 0, 160))
            d.text((10, ph - 26), f"x={x:5.2f}m  z={z:.3f}m  vx={vx:+.2f} (cmd {cmd_vx:+.2f}) m/s",
                   font=f_med, fill=(215, 220, 232))
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
