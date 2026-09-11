#!/usr/bin/env python
"""Render namiashi's tuned gaits at four stance heights as a 2x2 comparison.

Same replay machinery as `render_namiashi.py` -- the Rust harness writes the
MJCF it simulated plus a per-tick root pose and joint trace, and this pushes
those into `qpos`. All four panels advance on the same clock, so a difference
on screen is a difference in the run and not in the playback.

Each panel carries two kinds of number, and they are labelled differently on
purpose. Speed is live, averaged over a whole number of gait cycles. Actuator
saturation is a whole-run statistic -- the fraction of ticks a joint spent
clamped at its `effort` limit -- and it is the reason this comparison exists:
on Trot the thigh is clamped 26.7% of the time standing tall and 10.7% at 6 cm
of crouch, while the knee goes the other way, 6.0% to 17.0%.

    tools/render_namiashi_height.py --root /tmp/namiashi_height \\
        --gait trot --out namiashi_trunk_height.mp4
"""
import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import (
    JOINTS, body_frame_rates, font, load_trace, scened_model,
)

ALL_DROPS = [0.0, 0.02, 0.04, 0.06]

# Whole-run actuator saturation, % of ticks at the effort limit, measured by
# `namiashi_trunk_height_sweep`. Restated here rather than recomputed because
# the replay trace carries pose, not torque -- keep in step with that test.
SATURATION = {
    # gait: {drop_cm: (hip, thigh, calf)}
    "trot": {0: (2.9, 26.7, 6.0), 2: (5.6, 25.3, 10.2),
             4: (4.2, 19.6, 19.2), 6: (3.2, 10.7, 17.0)},
    "walk": {0: (0.6, 6.3, 11.3), 2: (0.7, 6.6, 2.8),
             4: (0.6, 6.1, 1.8), 6: (0.9, 5.3, 1.9)},
}

SATURATION["crawl"] = {0: (0.1, 3.4, 9.8), 2: (0.1, 1.3, 4.4),
                       4: (0.1, 1.2, 0.5), 6: (0.1, 2.1, 0.5)}

PERIOD = {"trot": 0.320, "walk": 0.500, "crawl": 0.800}
COMMAND = {"trot": 0.80, "walk": 0.33, "crawl": 0.17}


# Camera constants, shared by every panel so a screen row means the same world
# height in all of them.
CAM_DIST, CAM_ELEV, CAM_AZIM = 1.05, -11.0, 148.0
LOOKAT_Z = 0.16
REFERENCE_Z = 0.295  # the stance height the harness used before the crouch


def screen_y_of(world_z, h, fovy_deg):
    """Screen row for a world height, at the camera's lookat x,y.

    The camera is a fixed-distance tracker with fixed elevation, so the
    mapping is the same in every panel -- which is the point. Without a shared
    reference, 6 cm on a 0.3 m robot is about twenty pixels and reads as
    nothing.
    """
    dz = world_z - LOOKAT_Z
    up = dz * math.cos(math.radians(CAM_ELEV))
    half = CAM_DIST * math.tan(math.radians(fovy_deg) / 2.0)
    return h / 2.0 - (up / half) * (h / 2.0)


def panel_frames(root, gait, drop, fps, seconds, w, h):
    d = Path(root) / f"{gait}_{drop*100:.0f}cm"
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    # Tighter and squarer on than the single-clip view: at quarter size the
    # thing to see is the trunk height and how much the body rolls, not the
    # scenery.
    cam.distance, cam.elevation, cam.azimuth = CAM_DIST, CAM_ELEV, CAM_AZIM
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    win = PERIOD[gait] * math.ceil(0.8 / PERIOD[gait])
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
            out.append((r.render(), trace["root"][i, 2],
                        body_frame_rates(trace, i, win), ts - t[0]))
    return out, float(model.vis.global_.fovy)


def annotate(rgb, gait, drop, z, rates, elapsed, fovy):
    img = Image.fromarray(rgb)
    d = ImageDraw.Draw(img, "RGBA")
    w, h = img.size
    f_big, f_med, f_sm = font(26), font(18), font(15)

    d.rectangle([0, 0, w, 74], fill=(0, 0, 0, 170))
    title = "baseline" if drop == 0 else f"crouch {drop*100:.0f} cm"
    d.text((14, 8), title, font=f_big, fill=(255, 255, 255))
    # Live from the trace. This is the number the whole comparison is about,
    # so it must not be something typed into a caption.
    d.text((14, 40), f"trunk z = {z:.3f} m", font=font(21), fill=(215, 222, 235))

    settling = elapsed < 1.15
    vx = rates[0]
    cmd = COMMAND[gait]
    good = (not settling) and abs(vx - cmd) < 0.12 * max(abs(cmd), 0.30)
    d.text((w - 190, 8), "vx", font=f_sm, fill=(150, 160, 175))
    d.text((w - 160, 4), f"{vx:+.2f} m/s", font=f_big,
           fill=(120, 225, 130) if good
           else (140, 148, 160) if settling else (240, 200, 110))

    # Reference rule at the pre-crouch stance height, drawn identically in
    # every panel. The baseline robot's trunk rides on it; a crouched one
    # rides visibly under it, and the gap is the thing being compared.
    y_ref = screen_y_of(REFERENCE_Z, h, fovy)
    if 80 < y_ref < h - 60:
        for x in range(20, w - 20, 22):
            d.line([(x, y_ref), (x + 12, y_ref)], fill=(255, 210, 120, 190),
                   width=2)
        d.text((22, y_ref - 24), f"{REFERENCE_Z:.3f} m  old stance",
               font=f_sm, fill=(255, 210, 120))
        drop_mm = (REFERENCE_Z - z) * 1000.0
        if drop_mm > 3.0:
            y_now = screen_y_of(z, h, fovy)
            xb = w - 132
            d.line([(xb, y_ref), (xb, y_now)], fill=(120, 225, 130, 220),
                   width=3)
            for yy in (y_ref, y_now):
                d.line([(xb - 7, yy), (xb + 7, yy)], fill=(120, 225, 130, 220),
                       width=2)
            d.text((xb + 12, (y_ref + y_now) / 2 - 10), f"-{drop_mm:.0f} mm",
                   font=font(17), fill=(120, 225, 130))

    hip, thigh, calf = SATURATION[gait][int(round(drop * 100))]
    d.rectangle([0, h - 50, w, h], fill=(0, 0, 0, 175))
    d.text((14, h - 40), "torque-limited", font=f_sm, fill=(150, 160, 175))
    d.text((14, h - 24), "(% of run)", font=f_sm, fill=(150, 160, 175))
    for k, (name, val) in enumerate((("thigh", thigh), ("calf", calf))):
        x = w - 250 + k * 130
        d.text((x, h - 42), name, font=f_sm, fill=(160, 170, 185))
        # Red as the joint approaches being clamped a quarter of the time --
        # the point of the comparison is which joint pays, not the total.
        col = (245, 120, 110) if val > 18 else (245, 205, 120) if val > 8 \
            else (140, 210, 150)
        d.text((x, h - 25), f"{val:.1f}%", font=font(22), fill=col)
    return np.asarray(img)


def layout(n):
    """Panels across, panels down. Two heights read better side by side than
    stacked -- the thing being compared is a vertical distance."""
    return (2, 1) if n == 2 else (2, (n + 1) // 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/namiashi_height")
    ap.add_argument("--gait", default="trot", choices=sorted(PERIOD))
    ap.add_argument("--drops", default="0,2,4,6",
                    help="crouch depths in cm, comma separated")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=11.0)
    ap.add_argument("--panel_w", type=int, default=560)
    ap.add_argument("--panel_h", type=int, default=340)
    args = ap.parse_args()

    import imageio.v2 as imageio

    drops = [int(x) / 100.0 for x in args.drops.split(",")]
    panels = []
    panel_fovy = 45.0
    for drop in drops:
        print(f"  rendering {args.gait} {drop*100:.0f}cm")
        fr, fovy = panel_frames(args.root, args.gait, drop, args.fps,
                                args.seconds, args.panel_w, args.panel_h)
        panels.append(fr)
        panel_fovy = fovy

    n = min(len(p) for p in panels)
    cols, rows = layout(len(drops))
    header_h = 84
    W, H = args.panel_w * cols, args.panel_h * rows + header_h
    frames = []

    header_f, sub_f = font(28), font(17)
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), f"namiashi  {args.gait.capitalize()}  "
                          f"stance height comparison", font=header_f,
                fill=(240, 244, 250))
        dh.text((16, 46), f"3.30 kg   command {COMMAND[args.gait]:+.2f} m/s   "
                          f"identical gait parameters -- only the stance "
                          f"height differs", font=sub_f, fill=(150, 162, 180))
        for k, p in enumerate(panels):
            rgb, z, rates, elapsed = p[i]
            tile = Image.fromarray(
                annotate(rgb, args.gait, drops[k], z, rates, elapsed,
                         panel_fovy))
            sheet.paste(tile, ((k % cols) * args.panel_w,
                               header_h + (k // cols) * args.panel_h))
        frames.append(np.asarray(sheet))

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8,
                    macro_block_size=1)
    print(f"wrote {args.out}  ({len(frames)} frames, "
          f"{len(frames)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
