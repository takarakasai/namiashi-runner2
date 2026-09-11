#!/usr/bin/env python
"""Render a single namiashi replay against staircase terrain.

Single-clip, not side-by-side -- the point right now is just to look at the
environment (rise, run, approach floor, top platform) before spending compute
on a real climbing comparison. Reuses the same replay-to-video machinery as
every other namiashi clip: the Rust harness wrote the exact MJCF it simulated
plus a per-tick root + joint trace, and this pushes those into `qpos` and
calls `mj_forward` -- nothing here re-derives the motion.

The camera chases the trunk in x, y, *and* z, unlike the flat-ground clips
(which hold a fixed lookat height) -- namiashi's trunk moves up to a metre
over the course of a climb, and a fixed lookat would drift out of frame.

    tools/render_namiashi_stairs.py --root /tmp/nami_stairs --sub smoke \\
        --out stairs.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, overlay, scened_model


def draw_step_progress(frame, z, stance_z, rise_m, n_steps):
    """A bottom-of-frame bar: which riser height the trunk is currently at,
    out of `n_steps`, read from z rather than from x -- so it reflects
    height actually gained, not just horizontal distance travelled (the two
    diverge exactly when climbing fails, which is the case this needs to
    show clearly)."""
    if rise_m <= 0:
        return frame
    climbed = (z - stance_z) / rise_m
    climbed_clamped = max(0.0, min(n_steps, climbed))
    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    w, h = img.size
    label_w, bar_h = 210, 26
    bar_w = w - 44 - label_w
    x0, y0 = 22, h - 70
    d.rectangle([x0, y0, x0 + bar_w, y0 + bar_h], fill=(0, 0, 0, 150))
    fill_w = bar_w * climbed_clamped / n_steps
    d.rectangle([x0, y0, x0 + fill_w, y0 + bar_h], fill=(120, 200, 130, 220))
    for k in range(1, n_steps):
        xk = x0 + bar_w * k / n_steps
        d.line([(xk, y0), (xk, y0 + bar_h)], fill=(20, 22, 28, 200), width=1)
    f = font(17)
    label = f"step {climbed_clamped:4.1f} / {n_steps}"
    d.text((x0 + bar_w + 10, y0 + 3), label, font=f, fill=(220, 225, 235))
    return np.asarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_stairs")
    ap.add_argument("--sub", default="smoke")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--period", type=float, default=0.320)
    ap.add_argument("--cmd_vx", type=float, default=0.80)
    ap.add_argument("--rise_m", type=float, default=0.10)
    ap.add_argument("--run_m", type=float, default=0.20)
    ap.add_argument("--n_steps", type=int, default=10)
    args = ap.parse_args()
    caption = (f"rise={args.rise_m:.2f}m  run={args.run_m:.2f}m  {args.n_steps} steps  "
               f"({args.rise_m * args.n_steps:.1f} m total)")

    import imageio.v2 as imageio

    d = Path(args.root) / args.sub
    model = scened_model(d / "model.xml", args.width, args.height)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 2.2, -16.0, 132.0

    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    win = args.period * math.ceil(0.8 / args.period)
    t = trace["t"]
    # First tick's z, before the robot has moved -- the flat-approach stance
    # height the step count is measured up from.
    stance_z = trace["root"][0, 2]
    end = min(t[-1], t[0] + args.seconds)
    stamps = np.arange(t[0], end, 1.0 / args.fps)
    frames = []
    with mujoco.Renderer(model, args.height, args.width) as r:
        for ts in stamps:
            i = min(int(np.searchsorted(t, ts)), len(t) - 1)
            data.qpos[0:3] = trace["root"][i, 0:3]
            data.qpos[3:7] = trace["root"][i, 3:7]
            for k, a in enumerate(adr):
                data.qpos[a] = trace["q"][i, k]
            mujoco.mj_forward(model, data)

            cam.lookat[:] = data.qpos[0:3]
            r.update_scene(data, cam, opt)
            z = trace["root"][i, 2]
            frame = overlay(r.render(), "namiashi  staircase",
                             caption,
                             (args.cmd_vx, 0.0, 0.0),
                             body_frame_rates(trace, i, win), ts - t[0],
                             settling=(ts - t[0]) < 1.15,
                             z=z)
            frame = draw_step_progress(frame, z, stance_z, args.rise_m, args.n_steps)
            frames.append(frame)

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(frames)} frames, {len(frames)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
