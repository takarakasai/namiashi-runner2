#!/usr/bin/env python
"""Speed mode against torque mode following a command that keeps changing.

Forward slow, forward fast, turn in place, strafe, backward, stop -- six
targets, stepped rather than ramped, because a step is what exposes how an
interface hands over.

The commands are read from the replay trace, not restated here. The Rust
harness records what it actually applied each tick, so the overlay cannot
drift out of step with the run the way a schedule written down twice would.

Measured values are body-frame, averaged over a whole number of gait cycles,
as everywhere else. The strip along the bottom plots commanded against
measured for all three axes over the whole run, so the handover at each step
is visible rather than inferred from a number that has already settled.

    tools/render_namiashi_schedule.py --root /tmp/nami_rob --out schedule.mp4
"""
import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model

PERIOD = 0.320
CAM_DIST, CAM_ELEV, CAM_AZIM, LOOKAT_Z = 1.45, -14.0, 148.0, 0.17
PLOT_H = 150


def load_cmd(path):
    out = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            out.append((float(r["cmd_vx"]), float(r["cmd_vy"]),
                        float(r["cmd_wz"])))
    return np.array(out)


def clip(root, sub, fps, w, h):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    cmd = load_cmd(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = CAM_DIST, CAM_ELEV, CAM_AZIM
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    win = PERIOD * math.ceil(0.8 / PERIOD)
    stamps = np.arange(t[0], t[-1], 1.0 / fps)
    meas, frames, idxs = [], [], []
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
            frames.append(r.render())
            idxs.append(i)
            meas.append(body_frame_rates(trace, i, win))
    return frames, idxs, np.array(meas), cmd, stamps - t[0], t


def draw_plot(d, x0, y0, w, h, ts, cmd_series, meas, now, f_sm):
    """Commanded (dashed) against measured (solid), three axes, whole run."""
    d.rectangle([x0, y0, x0 + w, y0 + h], fill=(16, 18, 24, 235))
    rows = [
        ("vx  m/s", 0, 0, (120, 200, 255), 1.0),
        ("vy  m/s", 1, 1, (150, 235, 170), 1.0),
        ("wz deg/s", 2, 2, (255, 190, 120), 1.0 / 40.0),
    ]
    lab_w, pad = 74, 5
    px0, px_w = x0 + lab_w, w - lab_w - 14
    row_h = (h - 2 * pad) / len(rows)
    span = max(ts[-1], 1e-6)
    for r_i, (name, ci, mi, col, scale) in enumerate(rows):
        yc = y0 + pad + r_i * row_h
        mid = yc + row_h / 2
        d.text((x0 + 6, mid - 8), name, font=f_sm, fill=(150, 160, 175))
        d.line([(px0, mid), (px0 + px_w, mid)], fill=(60, 66, 78), width=1)
        # A shared vertical scale per row, from whatever that row reaches.
        c = cmd_series[:, ci] * (1.0 if ci < 2 else 1.0)
        m = meas[:, mi] * (1.0 if mi < 2 else scale)
        cs = c * (scale if ci == 2 else 1.0)
        amp = max(np.abs(cs).max(), np.abs(m).max(), 1e-3) * 1.25
        to_y = lambda v: mid - (v / amp) * (row_h / 2 - 3)
        to_x = lambda tt: px0 + (tt / span) * px_w
        for j in range(1, len(ts)):
            if ts[j] > now:
                break
            d.line([(to_x(ts[j - 1]), to_y(cs[j - 1])),
                    (to_x(ts[j]), to_y(cs[j]))],
                   fill=(215, 220, 232), width=2)
            d.line([(to_x(ts[j - 1]), to_y(m[j - 1])),
                    (to_x(ts[j]), to_y(m[j]))], fill=col, width=2)
    x_now = px0 + (now / span) * px_w
    d.line([(x_now, y0), (x_now, y0 + h)], fill=(240, 244, 250), width=2)
    d.text((x0 + 6, y0 + h - 15), "white = commanded    colour = measured",
           font=f_sm, fill=(120, 130, 148))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_rob")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--panel_w", type=int, default=660)
    ap.add_argument("--panel_h", type=int, default=400)
    args = ap.parse_args()

    import imageio.v2 as imageio

    panels = []
    for mode in ("speed", "torque"):
        print(f"  rendering {mode}")
        panels.append((mode,) + clip(args.root, f"schedule_{mode}", args.fps,
                                     args.panel_w, args.panel_h))

    n = min(len(p[1]) for p in panels)
    header_h = 84
    pw, ph = args.panel_w, args.panel_h + PLOT_H
    W, H = pw * 2, ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(28), font(20), font(15)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  Trot  following a changing command",
                font=f_title, fill=(240, 244, 250))
        dh.text((16, 46), "forward slow -> forward fast -> turn -> strafe -> "
                          "backward -> stop,  stepped every 3 s   "
                          "(host 400 Hz)", font=f_sub, fill=(150, 162, 180))

        for k, (mode, frames, idxs, meas, cmd, ts, _t) in enumerate(panels):
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(frames[i]), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")
            d.rectangle([0, 0, pw, 96], fill=(0, 0, 0, 178))
            d.text((14, 8), "Speed mode" if mode == "speed" else "Torque mode",
                   font=f_big, fill=(255, 255, 255))

            cvx, cvy, cwz = cmd[idxs[i]]
            mvx, mvy, mwz = meas[i]
            cols = [("vx", cvx, mvx, "m/s"), ("vy", cvy, mvy, "m/s"),
                    ("wz", math.degrees(cwz), mwz, "deg/s")]
            for j, (nm, c, m, unit) in enumerate(cols):
                x = 210 + j * 150
                d.text((x, 8), nm, font=f_sm, fill=(150, 160, 175))
                d.text((x, 28), f"{c:+.2f}", font=f_med, fill=(215, 220, 232))
                ref = max(abs(c), 0.20 if unit == "m/s" else 8.0)
                ok = abs(m - c) < 0.15 * ref
                d.text((x, 54), f"{m:+.2f}", font=f_med,
                       fill=(120, 225, 130) if ok else (240, 200, 110))
                d.text((x + 66, 58), unit, font=f_sm, fill=(140, 150, 165))

            draw_plot(d, 0, args.panel_h, pw, PLOT_H, ts, cmd[idxs],
                      meas, ts[i], f_sm)
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
