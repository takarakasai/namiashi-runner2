#!/usr/bin/env python
"""Render namiashi's three tuned gaits side by side, with their footfall patterns.

What separates Trot, Walk and Crawl is not how they look from the side -- at a
glance all three are a small quadruped walking. It is which feet are on the
ground at the same time, and how much of the cycle each one spends there. So
every panel carries a footfall diagram built from the *measured* normal force
in the replay trace.

That force is recorded by the Rust harness rather than inferred here, and the
reason is worth stating: foot height is not a usable stand-in. Checked against
the harness's own force-based duty at the same stance, a height threshold
overstates duty by 0.07 to 0.24 of the cycle -- worst on Walk's front pair,
which is exactly the foot that gets unloaded and exactly what the comparison is
supposed to show. Read from the trace, the diagram matches the harness to
within 0.015.

    tools/render_namiashi_gaits.py --root /tmp/nami_g6 --out gaits.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model

# gait, directory, period, duty, max_step, command
GAITS = [
    ("Crawl", "crawl", 0.800, 0.85, 0.145, 0.17),
    ("Walk", "walk", 0.500, 0.75, 0.145, 0.33),
    ("Trot", "trot", 0.320, 0.50, 0.145, 0.80),
]

FZ_COLS = ["fz_FL_foot", "fz_FR_foot", "fz_RL_foot", "fz_RR_foot"]
FOOT_LABELS = ["FL", "FR", "RL", "RR"]
# The harness's own support-census threshold, so the diagram and the numbers
# in `namiashi_tuned_gaits_hold` mean the same thing.
FZ_ON_N = 1.0

STRIP_S = 2.5      # seconds of history in the footfall diagram
STRIP_H = 118
CAM_DIST, CAM_ELEV, CAM_AZIM, LOOKAT_Z = 1.10, -12.0, 148.0, 0.17


def load_forces(path):
    import csv
    t, fz = [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t.append(float(r["t"]))
            fz.append([float(r[c]) for c in FZ_COLS])
    return np.array(t), np.array(fz)


def panel_frames(root, sub, fps, seconds, w, h):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    _, fz = load_forces(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = CAM_DIST, CAM_ELEV, CAM_AZIM
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    stamps = np.arange(t[0], min(t[-1], t[0] + seconds), 1.0 / fps)
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
            cam.lookat[2] = LOOKAT_Z
            r.update_scene(data, cam, opt)
            out.append((r.render(), i, ts - t[0]))
    return out, trace, fz


def draw_strip(d, x0, y0, w, h, t, fz, i, period, label_f):
    """Footfall raster: four rows, time running left to right, filled where
    that foot is carrying load. One gait period is marked so the reader can
    see how many footfalls fit in a cycle without counting."""
    rows, pad = 4, 4
    row_h = (h - 2 * pad - 16) / rows
    t_now = t[i]
    t0 = t_now - STRIP_S
    d.rectangle([x0, y0, x0 + w, y0 + h], fill=(16, 18, 24, 230))

    lab_w = 30
    px0 = x0 + lab_w + 6
    px_w = w - lab_w - 16

    # Cycle boundaries, so the reader can count footfalls per cycle.
    k = math.ceil(t0 / period)
    while k * period <= t_now:
        x = px0 + (k * period - t0) / STRIP_S * px_w
        d.line([(x, y0 + 2), (x, y0 + h - 2)], fill=(96, 106, 124), width=1)
        k += 1

    lo = int(np.searchsorted(t, t0))
    colours = [(120, 200, 255), (120, 200, 255), (255, 190, 120), (255, 190, 120)]
    for rr in range(rows):
        yc = y0 + pad + rr * row_h
        d.text((x0 + 6, yc + row_h / 2 - 8), FOOT_LABELS[rr], font=label_f,
               fill=(150, 160, 175))
        on = fz[lo:i + 1, rr] > FZ_ON_N
        if len(on) < 2:
            continue
        tt = t[lo:i + 1]
        start = None
        for j, v in enumerate(on):
            if v and start is None:
                start = j
            elif not v and start is not None:
                xa = px0 + (tt[start] - t0) / STRIP_S * px_w
                xb = px0 + (tt[j] - t0) / STRIP_S * px_w
                d.rectangle([xa, yc + 2, max(xb, xa + 1), yc + row_h - 4],
                            fill=colours[rr])
                start = None
        if start is not None:
            xa = px0 + (tt[start] - t0) / STRIP_S * px_w
            xb = px0 + (tt[-1] - t0) / STRIP_S * px_w
            d.rectangle([xa, yc + 2, max(xb, xa + 1), yc + row_h - 4],
                        fill=colours[rr])
    d.line([(px0 + px_w, y0), (px0 + px_w, y0 + h)], fill=(230, 236, 245), width=2)
    d.text((x0 + 6, y0 + h - 15), f"measured load > {FZ_ON_N:.0f} N   "
                                  f"grid = one cycle   last {STRIP_S:.1f} s",
           fill=(120, 130, 148), font=label_f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_g6")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=11.0)
    ap.add_argument("--panel_w", type=int, default=600)
    ap.add_argument("--panel_h", type=int, default=380)
    args = ap.parse_args()

    import imageio.v2 as imageio

    data = []
    for label, sub, period, duty, step, cmd in GAITS:
        print(f"  rendering {sub}")
        frames, trace, fz = panel_frames(args.root, sub, args.fps, args.seconds,
                                         args.panel_w, args.panel_h)
        data.append((label, period, duty, step, cmd, frames, trace, fz))

    n = min(len(d[5]) for d in data)
    header_h = 84
    pw, ph = args.panel_w, args.panel_h + STRIP_H
    W, H = pw * len(GAITS), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(26), font(19), font(15)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  gait comparison", font=f_title,
                fill=(240, 244, 250))
        dh.text((16, 46), "3.30 kg   stance 0.235 m   each gait at its own "
                          "tuned period, duty and command",
                font=f_sub, fill=(150, 162, 180))

        for k, (label, period, duty, step, cmd, frames, trace, fz) in enumerate(data):
            rgb, idx, elapsed = frames[i]
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(rgb), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")

            d.rectangle([0, 0, pw, 74], fill=(0, 0, 0, 175))
            d.text((14, 8), label, font=f_big, fill=(255, 255, 255))
            d.text((14, 44), f"T={period:.3f}s  duty={duty:.2f}  "
                             f"step={step:.3f}m", font=f_sm,
                   fill=(175, 185, 200))

            win = period * math.ceil(0.8 / period)
            vx = body_frame_rates(trace, idx, win)[0]
            settling = elapsed < 1.15
            good = (not settling) and abs(vx - cmd) < 0.12 * max(abs(cmd), 0.30)
            d.text((pw - 250, 10), "cmd", font=f_sm, fill=(150, 160, 175))
            d.text((pw - 250, 32), f"{cmd:+.2f} m/s", font=f_med,
                   fill=(220, 225, 235))
            d.text((pw - 130, 10), "measured", font=f_sm, fill=(150, 160, 175))
            d.text((pw - 130, 28), f"{vx:+.2f}", font=f_big,
                   fill=(120, 225, 130) if good
                   else (140, 148, 160) if settling else (240, 200, 110))

            draw_strip(d, 0, args.panel_h, pw, STRIP_H, trace["t"], fz, idx,
                       period, f_sm)
            n_down = int((fz[idx] > FZ_ON_N).sum())
            d.rectangle([14, args.panel_h - 30, 210, args.panel_h - 6],
                        fill=(0, 0, 0, 150))
            d.text((22, args.panel_h - 27), f"support  {n_down} / 4 feet",
                   font=f_med,
                   fill=(120, 225, 130) if n_down >= 3 else (240, 200, 110))
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
