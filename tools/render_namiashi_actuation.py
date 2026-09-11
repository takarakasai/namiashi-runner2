#!/usr/bin/env python
"""Speed mode against torque mode, side by side, one video per gait.

Both are interfaces the LKMTech MG4005 actually offers. At the same update
rate they are close to the same control law -- the torque path is the driver's
PD computed host-side -- so what this compares is not two algorithms but two
places to put the loop. A speed-mode driver closes its inner loop internally
at several kHz on fresh encoder data, whatever the host is doing; in torque
mode there is no inner loop, and the last torque the host sent is held until
the next arrives. Both clips run the host at 400 Hz, the rate the bus is designed for, and the
speed side models the driver as the velocity source an 8-16 kHz loop is from
a 400 Hz host -- limited by torque, not by bandwidth.

Each panel carries the footfall diagram from the measured normal force, since
what degrades first is the contact pattern rather than the speed.

    tools/render_namiashi_actuation.py --root /tmp/nami_act --gait trot \\
        --out namiashi_speed_vs_torque_trot.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, body_frame_rates, font, load_trace, scened_model
from render_namiashi_gaits import (
    GAITS, FZ_ON_N, STRIP_H, draw_strip, load_forces,
)

# Period and command per gait, taken from the single source in
# render_namiashi_gaits so the two renderers cannot disagree about what they
# are showing.
PERIOD = {g[1]: g[2] for g in GAITS}
COMMAND = {g[1]: g[5] for g in GAITS}

# label, directory suffix, one-line description of the interface
MODES = [
    ("Speed mode", "speed",
     "driver is an 8-16 kHz velocity source, torque-limited;"
     "host sends qd = dq*/dt + 40*(q*-q)"),
    ("Torque mode", "torque",
     "no driver loop at all;"
     "host sends kp(q*-q) - kd*qd + gravity + WBC tau"),
]

HOST_HZ = 400

CAM_DIST, CAM_ELEV, CAM_AZIM, LOOKAT_Z = 1.10, -12.0, 148.0, 0.17


def panel_frames(root, gait, sub, fps, seconds, w, h):
    d = Path(root) / f"{gait}_{sub}"
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_act")
    ap.add_argument("--gait", default="trot", choices=sorted(PERIOD))
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=11.0)
    ap.add_argument("--panel_w", type=int, default=640)
    ap.add_argument("--panel_h", type=int, default=400)
    args = ap.parse_args()

    import imageio.v2 as imageio

    period, cmd = PERIOD[args.gait], COMMAND[args.gait]
    panels = []
    for label, sub, blurb in MODES:
        print(f"  rendering {args.gait} {sub}")
        panels.append((label, blurb) + panel_frames(
            args.root, args.gait, sub, args.fps, args.seconds,
            args.panel_w, args.panel_h))

    n = min(len(p[2]) for p in panels)
    header_h = 88
    pw, ph = args.panel_w, args.panel_h + STRIP_H
    W, H = pw * len(MODES), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(26), font(19), font(15)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), f"namiashi  {args.gait.capitalize()}  "
                          f"MG4005 interface comparison", font=f_title,
                fill=(240, 244, 250))
        dh.text((16, 46), f"host {HOST_HZ} Hz   command {cmd:+.2f} m/s   "
                          f"stance 0.235 m   identical gait and gains -- "
                          f"only where the loop runs differs",
                font=f_sub, fill=(150, 162, 180))

        for k, (label, blurb, frames, trace, fz) in enumerate(panels):
            rgb, idx, elapsed = frames[i]
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(rgb), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")

            d.rectangle([0, 0, pw, 96], fill=(0, 0, 0, 178))
            d.text((14, 8), label, font=f_big, fill=(255, 255, 255))
            # Two short lines rather than one long one -- at this panel width
            # a single line runs under the speed readout.
            for j, line in enumerate(blurb.split(";")):
                d.text((14, 42 + j * 20), line.strip(), font=f_sm,
                       fill=(170, 180, 196))

            win = period * math.ceil(0.8 / period)
            vx = body_frame_rates(trace, idx, win)[0]
            settling = elapsed < 1.15
            good = (not settling) and abs(vx - cmd) < 0.12 * max(abs(cmd), 0.30)
            d.text((pw - 128, 10), "vx  m/s", font=f_sm, fill=(150, 160, 175))
            d.text((pw - 128, 34), f"{vx:+.2f}", font=f_big,
                   fill=(120, 225, 130) if good
                   else (140, 148, 160) if settling else (240, 200, 110))

            n_down = int((fz[idx] > FZ_ON_N).sum())
            d.rectangle([14, args.panel_h - 30, 210, args.panel_h - 6],
                        fill=(0, 0, 0, 150))
            d.text((22, args.panel_h - 27), f"support  {n_down} / 4 feet",
                   font=f_med,
                   fill=(120, 225, 130) if n_down >= 3 else (240, 200, 110))

            draw_strip(d, 0, args.panel_h, pw, STRIP_H, trace["t"], fz, idx,
                       period, f_sm)
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
