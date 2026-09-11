#!/usr/bin/env python
"""Speed mode against torque mode under a sideways push, over a whole stride.

One push time proves nothing. `namiashi_push_phase_dependence` measured the
same 12 N x 0.12 s impulse at eight phases of Trot's 0.320 s cycle: speed mode
falls at three of them, torque mode at three, and they are not the same three.
Which foot pair is loaded when the impulse lands decides the outcome, so a
video built on a single push would be presenting an accident of timing as a
property of the interface.

This plays all eight phases in sequence, both modes side by side, each clip
labelled with which phase it is and whether that run survived. The verdict is
the tally at the end, not any one clip.

    tools/render_namiashi_push.py --root /tmp/nami_rob --out push.mp4
"""
import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import JOINTS, font, load_trace, scened_model

PERIOD = 0.320
COMMAND = 0.80
PUSH_T0 = 6.0
FALL_ROLL_DEG = 45.0
CAM_DIST, CAM_ELEV, CAM_AZIM, LOOKAT_Z = 1.30, -12.0, 148.0, 0.17


def load_push(path):
    t, fy = [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t.append(float(r["t"]))
            fy.append(float(r["push_fy"]))
    return np.array(t), np.array(fy)


def roll_of(quat):
    w, x, y, z = quat
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def clip(root, sub, fps, t_from, t_to, w, h):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    _, fy = load_push(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = CAM_DIST, CAM_ELEV, CAM_AZIM
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    # Worst roll after the push decides the survived/fell label, and it is read
    # off the trace rather than restated from the sweep.
    post = t >= PUSH_T0
    worst_roll = max(abs(roll_of(q)) for q in trace["root"][post, 3:])

    stamps = np.arange(t_from, min(t[-1], t_to), 1.0 / fps)
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
            out.append((r.render(), ts, fy[i], math.degrees(roll_of(trace["root"][i, 3:])),
                        trace["root"][i, 2]))
    return out, math.degrees(worst_roll)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_rob")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--panel_w", type=int, default=640)
    ap.add_argument("--panel_h", type=int, default=420)
    args = ap.parse_args()

    import imageio.v2 as imageio

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(30), font(20), font(15)
    header_h, W = 92, args.panel_w * 2
    H = args.panel_h + header_h + 34
    frames, tally = [], {"speed": 0, "torque": 0}

    for step in range(8):
        t_push = PUSH_T0 + PERIOD * step / 8.0
        panels = []
        for mode in ("speed", "torque"):
            fr, worst = clip(args.root, f"push{step}_{mode}", args.fps,
                             t_push - 1.0, t_push + 3.0,
                             args.panel_w, args.panel_h)
            panels.append((mode, fr, worst))
            if worst < FALL_ROLL_DEG:
                tally[mode] += 1

        n = min(len(p[1]) for p in panels)
        for i in range(n):
            sheet = Image.new("RGB", (W, H), (13, 15, 20))
            dh = ImageDraw.Draw(sheet)
            dh.text((16, 10), "namiashi  Trot  sideways push, "
                              f"stride phase {step + 1}/8",
                    font=f_title, fill=(240, 244, 250))
            dh.text((16, 46), "12 N for 0.12 s = 1.44 N*s on 3.3 kg -- a "
                              "0.44 m/s kick, the same size as the forward "
                              "command", font=f_sub, fill=(150, 162, 180))
            dh.text((16, 68), f"push at t = {t_push:.3f} s   "
                              f"(cycle 0.320 s, so this is {step/8:.3f} of a "
                              f"stride after the last)",
                    font=f_sub, fill=(150, 162, 180))

            for k, (mode, fr, worst) in enumerate(panels):
                rgb, ts, fy, roll, z = fr[i]
                tile = Image.new("RGB", (args.panel_w, args.panel_h + 34),
                                 (13, 15, 20))
                tile.paste(Image.fromarray(rgb), (0, 0))
                d = ImageDraw.Draw(tile, "RGBA")
                d.rectangle([0, 0, args.panel_w, 46], fill=(0, 0, 0, 178))
                d.text((14, 8), "Speed mode" if mode == "speed" else "Torque mode",
                       font=f_big, fill=(255, 255, 255))

                # The push itself, while it is being delivered.
                if abs(fy) > 0.1:
                    d.rectangle([0, 0, args.panel_w, args.panel_h],
                                outline=(255, 120, 110), width=6)
                    d.text((args.panel_w - 190, 10), f"PUSH {fy:+.0f} N",
                           font=f_med, fill=(255, 140, 130))

                fell = worst >= FALL_ROLL_DEG
                col = (245, 120, 110) if fell else (120, 225, 130)
                d.rectangle([0, args.panel_h, args.panel_w, args.panel_h + 34],
                            fill=(0, 0, 0, 190))
                d.text((14, args.panel_h + 8),
                       f"roll {roll:+6.1f} deg    trunk z {z:.3f} m",
                       font=f_sm, fill=(200, 210, 225))
                d.text((args.panel_w - 210, args.panel_h + 6),
                       "FELL" if fell else "recovered", font=f_med, fill=col)
                sheet.paste(tile, (k * args.panel_w, header_h))
            frames.append(np.asarray(sheet))

    # Tally card: the actual result, since no single phase is one.
    card = Image.new("RGB", (W, H), (13, 15, 20))
    d = ImageDraw.Draw(card)
    d.text((60, 100), "Over one full stride of push phases",
           font=font(40), fill=(240, 244, 250))
    for k, mode in enumerate(("speed", "torque")):
        surv = tally[mode]
        d.text((60, 200 + k * 70),
               f"{('Speed' if mode == 'speed' else 'Torque'):>7} mode:  "
               f"{surv} of 8 recovered,  {8 - surv} fell",
               font=font(32),
               fill=(120, 225, 130) if surv >= 6 else (240, 200, 110))
    d.text((60, 380), "Same impulse, same gait, same gains.  Which feet were "
                      "loaded when it landed", font=font(21),
           fill=(150, 162, 180))
    d.text((60, 408), "is what decides it -- neither interface is robust to "
                      "this on its own.", font=font(21),
           fill=(150, 162, 180))
    frames += [np.asarray(card)] * (args.fps * 4)

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8,
                    macro_block_size=1)
    print(f"wrote {args.out}  ({len(frames)} frames, "
          f"{len(frames)/args.fps:.1f}s)   tally={tally}")


if __name__ == "__main__":
    main()
