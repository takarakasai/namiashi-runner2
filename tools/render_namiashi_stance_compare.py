#!/usr/bin/env python
"""Render the per-leg nominal stance height (`TerrainStanceCfg`) against the
same staircase with the correction off, side by side.

Everything is identical between the two panels -- same Trot tuning, same
5 cm / 10-step staircase, same forward command, same seed pose. The only
difference is whether each leg's `nominal_foot_body.z` is offset by its own
terrain height (as a difference from the four-leg mean, so body height is
untouched and only the front-to-rear support pattern tilts).

Full 20 s, deliberately: the endings are the point. The correction clearly
beats having none (which topples at t~5 and never moves again), but neither
setting HOLDS what it gains -- gain 1.0 climbs, then walks itself back down
below the first riser; gain 1.45 climbs furthest, then collapses flat onto
the stairs. Cutting the clip early would show only the good half.

Note the collapse is terrain-relative: belly-down on step 3 still reads
z=0.25 m in world frame, above the flat-ground fall threshold.

    tools/render_namiashi_stance_compare.py --root /tmp/nami_stairs \\
        --out stance_compare.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model
from render_namiashi_stairs import draw_step_progress

RISE_M, N_STEPS = 0.05, 10
PANELS = [
    ("OFF", "stance_off",
     "all four legs share one nominal -- a flat-ground assumption in the leg geometry"),
    ("ON, gain 1.0", "stance_100",
     "never collapses -- but walks back DOWN, ending at x=0.74 m, below the first riser"),
    ("ON, gain 1.45", "stance_145",
     "climbs furthest (~3.6 steps), then collapses onto the stairs around t=15 s"),
]


def panel_frames(root, sub, fps, seconds, w, h):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 2.0, -13.0, 132.0
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

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
            # Chase x, y AND z: each panel is on its own climb, so a shared
            # or fixed-height lookat would desync them.
            cam.lookat[:] = data.qpos[0:3]
            r.update_scene(data, cam, opt)
            out.append((r.render(), i, ts - t[0]))
    return out, trace, stance_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_stairs")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--panel_w", type=int, default=520)
    ap.add_argument("--panel_h", type=int, default=420)
    ap.add_argument("--period", type=float, default=0.320)
    args = ap.parse_args()

    import imageio.v2 as imageio

    panels = []
    for label, sub, note in PANELS:
        print(f"  rendering {sub}")
        frames, trace, stance_z = panel_frames(
            args.root, sub, args.fps, args.seconds, args.panel_w, args.panel_h
        )
        panels.append((label, note, frames, trace, stance_z))

    # max(), not min(): a panel that runs out just holds its last frame,
    # rather than cutting the whole clip at the shorter run's length.
    n = max(len(p[2]) for p in panels)
    header_h = 92
    pw, ph = args.panel_w, args.panel_h
    W, H = pw * len(PANELS), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(23), font(19), font(15)
    win = args.period * math.ceil(0.8 / args.period)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  5 cm staircase  per-leg nominal stance height",
                font=f_title, fill=(240, 244, 250))
        dh.text((16, 46), "identical gait, staircase and command -- the only difference is whether each leg's "
                          "support height follows the step it is on",
                font=f_sub, fill=(150, 162, 180))

        for k, (label, note, frames, trace, stance_z) in enumerate(panels):
            rgb, idx, elapsed = frames[min(i, len(frames) - 1)]
            frozen = i >= len(frames)
            z = trace["root"][idx, 2]
            x = trace["root"][idx, 0]
            frame = draw_step_progress(rgb, z, stance_z, RISE_M, N_STEPS)
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(frame), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")
            d.rectangle([0, 0, pw, 78], fill=(0, 0, 0, 178))
            d.text((12, 8), label, font=f_big, fill=(255, 255, 255))
            d.text((12, 38), note, font=f_sm, fill=(170, 180, 196))

            vx = body_frame_rates(trace, idx, win)[0]
            d.rectangle([0, ph - 30, pw, ph], fill=(0, 0, 0, 160))
            status = "  [recording ended]" if frozen else ""
            d.text((10, ph - 26),
                   f"t={elapsed:5.2f}s  x={x:5.2f}m  z={z:.3f}m  vx={vx:+.2f} m/s{status}",
                   font=f_med, fill=(215, 220, 232) if not frozen else (150, 150, 150))
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
