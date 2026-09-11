#!/usr/bin/env python
"""Render namiashi's WBC gait replays as a single annotated MP4.

Kinematic replay, not a second physics rollout: `tests/wbc_walk.rs` writes the
exact MJCF it simulated plus a per-tick root pose and joint trace under
`WbcParams::replay_dir`, and this pushes those straight into `qpos` and calls
`mj_forward`. Nothing here re-derives the motion, so what you watch is the run
the regression measured, not an approximation of it.

The overlay reports the same body-frame quantities the harness asserts on --
speed in the instantaneous heading frame, not the run-start one. That
distinction is not cosmetic: measured from the starting heading, a robot that
yaws while walking straight appears to slide sideways at `v*sin(yaw)`, which
on an early namiashi run read as 0.23 m/s of crab that was not happening.

    tools/render_namiashi.py --root /tmp/namiashi_replay --out namiashi.mp4
"""
import argparse
import csv
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# label, directory, caption, commanded (vx, vy, wz), gait period
#
# The period is here because the overlay averages over a whole number of gait
# cycles. A fixed window beats against the gait -- 1.0 s against Walk's 0.500
# is exactly 2 cycles and against Crawl's 0.800 exactly 1.25, so every window
# boundary lands on one of two or four gait phases and the average never
# settles. The harness has the same fix for the same reason.
CLIPS = [
    ("Trot", "trot", "T=0.320s  duty=0.50  step=0.145m", (0.80, 0.0, 0.0), 0.320),
    ("Walk", "walk", "T=0.500s  duty=0.75  step=0.145m", (0.33, 0.0, 0.0), 0.500),
    ("Crawl", "crawl", "T=0.800s  duty=0.85  step=0.145m", (0.17, 0.0, 0.0), 0.800),
    ("Trot / forward", "cmd_fwd", "command coverage", (0.80, 0.0, 0.0), 0.320),
    ("Trot / backward", "cmd_back", "the one that does not track", (-0.80, 0.0, 0.0), 0.320),
    ("Trot / strafe", "cmd_strafe", "command coverage", (0.0, 0.45, 0.0), 0.320),
    ("Trot / turn", "cmd_turn", "yaw arm fix: 76% -> 100%", (0.0, 0.0, 0.60), 0.320),
]

JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


# The exported MJCF is built for physics, not for looking at: no lights, no
# skybox, and a flat grey ground plane that renders as an unreadable dark
# field. Injected here rather than in the Rust exporter, which has no business
# knowing about rendering.
SCENERY = """
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
    <map znear="0.02" zfar="30"/>
    <quality shadowsize="4096"/>
  </visual>
  <asset>
    <texture name="skybox" type="skybox" builtin="gradient"
             rgb1="0.32 0.38 0.48" rgb2="0.06 0.07 0.10" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.30 0.32 0.36" rgb2="0.22 0.24 0.28" width="512" height="512"/>
    <material name="gridmat" texture="grid" texrepeat="24 24" reflectance="0.08"/>
  </asset>
  <worldbody>
    <light name="key" pos="1.2 -1.6 2.4" dir="-0.4 0.55 -0.9" directional="true"
           diffuse="0.75 0.75 0.75" specular="0.25 0.25 0.25" castshadow="true"/>
    <light name="fill" pos="-2.0 1.4 2.0" dir="0.6 -0.45 -0.9" directional="true"
           diffuse="0.30 0.32 0.36" castshadow="false"/>
  </worldbody>
"""


def scened_model(path, width, height):
    """Load the exported MJCF with lighting and a floor you can see move.

    The checker floor is the point: against a plain plane a walking robot and a
    robot shuffling in place look identical, which is the exact confusion this
    whole exercise has been about.
    """
    xml = Path(path).read_text()
    xml = xml.replace(
        'rgba="0.5 0.5 0.55 1"', 'material="gridmat"', 1
    ).replace("</mujoco>", SCENERY + "</mujoco>", 1)
    model = mujoco.MjModel.from_xml_string(xml)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    return model


def load_trace(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    n = len(rows)
    out = {
        "t": np.empty(n),
        "root": np.empty((n, 7)),
        "q": np.empty((n, len(JOINTS))),
    }
    for i, r in enumerate(rows):
        out["t"][i] = float(r["t"])
        out["root"][i] = [
            float(r[k]) for k in
            ("root_x", "root_y", "root_z", "root_qw", "root_qx", "root_qy", "root_qz")
        ]
        out["q"][i] = [float(r[j]) for j in JOINTS]
    return out


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_frame_rates(trace, i, dt_win):
    """(vx, vy, yaw_rate) over a window ending at `i`, in that window's own
    heading frame. Rotating by the yaw at the window's start rather than the
    run's start is what separates sliding sideways from having turned."""
    t = trace["t"]
    j = int(np.searchsorted(t, t[i] - dt_win))
    if j >= i:
        return 0.0, 0.0, 0.0
    a, b = trace["root"][j], trace["root"][i]
    dt = t[i] - t[j]
    ya = yaw_of(a[3:])
    c, s = math.cos(-ya), math.sin(-ya)
    dx, dy = b[0] - a[0], b[1] - a[1]
    dyaw = yaw_of(b[3:]) - ya
    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
    return (c * dx - s * dy) / dt, (s * dx + c * dy) / dt, math.degrees(dyaw) / dt


def font(size):
    for p in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ):
        if Path(p).exists():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def overlay(frame, label, caption, cmd, meas, t, settling, z):
    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    w, _ = img.size
    f_big, f_med, f_small = font(34), font(23), font(19)

    d.rectangle([0, 0, w, 132], fill=(0, 0, 0, 165))
    d.text((22, 14), label, font=f_big, fill=(255, 255, 255))
    d.text((22, 56), caption, font=f_small, fill=(175, 185, 200))
    # Read off the trace, not typed in. A hardcoded stance height is exactly
    # the kind of caption that keeps saying 0.235 m after someone changes the
    # default, and there is no way to catch it by looking.
    d.text((22, 84), f"namiashi  3.30 kg   trunk z = {z:.3f} m", font=f_small,
           fill=(175, 185, 200))

    cvx, cvy, cwz = cmd
    mvx, mvy, mwz = meas
    col = 620
    rows = [
        ("vx", f"{cvx:+.2f}", f"{mvx:+.2f}", "m/s"),
        ("vy", f"{cvy:+.2f}", f"{mvy:+.2f}", "m/s"),
        ("wz", f"{math.degrees(cwz):+.1f}", f"{mwz:+.1f}", "deg/s"),
    ]
    d.text((col, 12), "cmd", font=f_small, fill=(150, 160, 175))
    d.text((col + 96, 12), "measured", font=f_small, fill=(150, 160, 175))
    for k, (name, c, m, unit) in enumerate(rows):
        y = 38 + k * 30
        d.text((col - 46, y), name, font=f_med, fill=(160, 170, 185))
        d.text((col, y), c, font=f_med, fill=(220, 225, 235))
        # Green when the measured value is close to what was asked for. The
        # tolerance is on the same scale as the command so a zero command is
        # not held to an impossible relative error.
        ref = max(abs(float(c)), 0.30)
        good = (not settling) and abs(float(m) - float(c)) < 0.12 * ref
        d.text((col + 96, y), m, font=f_med,
                  fill=(120, 225, 130) if good
               else (140, 148, 160) if settling else (240, 200, 110))
        d.text((col + 190, y + 3), unit, font=f_small, fill=(150, 160, 175))

    d.text((22, img.size[1] - 34), f"t = {t:5.2f} s", font=f_small,
           fill=(190, 200, 215))
    if settling:
        # The command is only issued after the burn-in; without saying so, the
        # first second reads as the controller ignoring it.
        note = "settling  (command issued at t = 1.0 s)"
        tw = d.textlength(note, font=f_small)
        d.text(((w - tw) / 2, img.size[1] - 34), note, font=f_small,
               fill=(245, 205, 120))
    return np.asarray(img)


def render_clip(root, label, sub, caption, cmd, period, fps, seconds, width, height):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", width, height)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")

    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 1.35, -18.0, 132.0

    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    model.vis.scale.contactwidth = 0.04
    model.vis.scale.contactheight = 0.02
    model.vis.map.force = 0.012

    # Whole number of gait cycles, at least ~0.8 s so the number is readable.
    win = period * math.ceil(0.8 / period)

    t = trace["t"]
    end = min(t[-1], t[0] + seconds)
    stamps = np.arange(t[0], end, 1.0 / fps)
    frames = []
    with mujoco.Renderer(model, height, width) as r:
        for ts in stamps:
            i = int(np.searchsorted(t, ts))
            i = min(i, len(t) - 1)
            data.qpos[0:3] = trace["root"][i, 0:3]
            data.qpos[3:7] = trace["root"][i, 3:7]
            for k, a in enumerate(adr):
                data.qpos[a] = trace["q"][i, k]
            mujoco.mj_forward(model, data)

            # Chase the trunk so the robot stays framed however far it travels.
            cam.lookat[:] = data.qpos[0:3]
            cam.lookat[2] = 0.20
            r.update_scene(data, cam, opt)
            frames.append(
                overlay(r.render(), label, caption, cmd,
                        body_frame_rates(trace, i, win), ts - t[0],
                        settling=(ts - t[0]) < 1.15,
                        z=trace["root"][i, 2])
            )
    return frames


def title_card(text, sub, width, height, fps, seconds=1.6):
    img = Image.new("RGB", (width, height), (13, 15, 20))
    d = ImageDraw.Draw(img)
    f1, f2 = font(46), font(24)
    for txt, f, y, col in ((text, f1, height // 2 - 46, (240, 244, 250)),
                           (sub, f2, height // 2 + 16, (150, 162, 180))):
        tw = d.textlength(txt, font=f)
        d.text(((width - tw) / 2, y), txt, font=f, fill=col)
    return [np.asarray(img)] * int(fps * seconds)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/namiashi_replay")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=9.0)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    args = ap.parse_args()

    import imageio.v2 as imageio

    frames = []
    frames += title_card("namiashi  WBC gaits",
                         "3.3 kg model",
                         args.width, args.height, args.fps, 2.2)
    for label, sub, caption, cmd, period in CLIPS:
        if not (Path(args.root) / sub / "trace.csv").exists():
            print(f"  skip {sub} (no trace)")
            continue
        print(f"  rendering {sub}")
        if sub == "cmd_fwd":
            frames += title_card("Command coverage",
                                 "forward / backward / strafe / turn",
                                 args.width, args.height, args.fps)
        frames += render_clip(args.root, label, sub, caption, cmd, period,
                              args.fps, args.seconds, args.width, args.height)

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8,
                    macro_block_size=1)
    print(f"wrote {args.out}  ({len(frames)} frames, "
          f"{len(frames)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
