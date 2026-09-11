#!/usr/bin/env python
"""Render the self-righting recovery from a trace written by
`examples/namiashi_self_righting_video`.

Two things this has to get right that are easy to get wrong:

  - The `<hfield>` is declared with nrow/ncol and no `file=`, so loading the
    XML on its own gets an all-zero grid and draws a perfectly flat ring.
    It looks entirely plausible. The elevation is filled here from the PGM,
    exactly as the Rust side fills it at runtime.
  - The camera tracks the trunk in xy but not in z. The whole point of the
    clip is the body coming up off the ground, and a camera that follows
    that removes it from the picture.

    tools/render_namiashi_self_righting.py --root /tmp/nami_right \\
        --out self_righting.mp4
"""
import argparse
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import font, load_trace as _lt  # noqa: F401  (font only)

JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "arm_pitch_joint",
]

PANELS = [
    ("gentle", "V  -- unhurried",
     "commanded joint rate held low, so the rocking cannot build momentum"),
    ("fast", "Shift+V  -- fast",
     "rocks up momentum and throws the body over; 12/15 conditions against 5/15"),
]


def load_trace(path):
    import csv
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    n = len(rows)
    out = {"t": np.empty(n), "root": np.empty((n, 7)), "q": np.empty((n, len(JOINTS)))}
    for i, r in enumerate(rows):
        out["t"][i] = float(r["t"])
        out["root"][i] = [float(r[k]) for k in
                          ("root_x", "root_y", "root_z",
                           "root_qw", "root_qx", "root_qy", "root_qz")]
        out["q"][i] = [float(r[j]) for j in JOINTS]
    return out


def load_outcome(path):
    vals = {}
    for line in Path(path).read_text().splitlines():
        k, _, v = line.partition(" ")
        vals[k] = float(v)
    return vals


def ring_model(d, width, height):
    """The exported MJCF with its heightfield actually filled in."""
    model = mujoco.MjModel.from_xml_path(str(d / "model.xml"))
    tok = (d / "ring_elevation.pgm").read_text().split()
    w, h = int(tok[1]), int(tok[2])
    a = np.array([int(v) for v in tok[4:4 + w * h]], dtype=np.float32).reshape(h, w) / 255.0
    # Written top-row-is-+y so the file reads like a plan view; MuJoCo's grid
    # starts at -y.
    a = a[::-1, :]
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "kawasaki")
    adr = model.hfield_adr[hid]
    nrow, ncol = model.hfield_nrow[hid], model.hfield_ncol[hid]
    assert (nrow, ncol) == (h, w), f"grid {nrow}x{ncol} vs pgm {h}x{w}"
    model.hfield_data[adr:adr + nrow * ncol] = a.reshape(-1)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    return model


def trunk_up(quat):
    """The trunk's own +z in world coordinates: +1 upright, -1 on its back."""
    w, x, y, z = quat
    return 1.0 - 2.0 * (x * x + y * y)


def overlay(frame, title, caption, t, up, o, upright_from):
    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    W, H = img.size
    d.rectangle([0, 0, W, 78], fill=(0, 0, 0, 150))
    d.text((14, 8), title, font=font(26), fill=(255, 255, 255))
    d.text((14, 44), caption, font=font(15), fill=(190, 200, 215))

    # Attitude bar: the one number the whole clip is about. Bottom-right,
    # not in the header -- the caption needs that width.
    bx, by, bw, bh = W - 250, H - 62, 220, 16
    d.rectangle([bx, by, bx + bw, by + bh], outline=(140, 150, 165), width=1)
    mid = bx + bw / 2
    frac = (up + 1.0) / 2.0
    col = (110, 210, 120) if up > 0.9 else (225, 190, 90) if up > 0 else (215, 110, 100)
    x0, x1 = sorted((mid, mid + (frac - 0.5) * bw))
    d.rectangle([x0, by + 1, x1, by + bh - 1], fill=col)
    d.line([mid, by, mid, by + bh], fill=(200, 200, 210), width=1)
    d.text((bx, by - 20), f"trunk +z {up:+.2f}    -1 back, +1 up",
           font=font(13), fill=(215, 220, 230))

    lines = [
        f"t {t:5.2f} s",
        f"RMS body rate {o['rms_omega']:.2f} rad/s",
        f"RMS joint rate {o['rms_joint']:.2f} rad/s",
        f"cmd rate limit {o['max_rate']:.1f} rad/s"
        if o["max_rate"] < 100 else "cmd rate limit  none",
    ]
    y = H - 22 * len(lines) - 12
    d.rectangle([0, y - 8, 310, H], fill=(0, 0, 0, 130))
    for s in lines:
        d.text((14, y), s, font=font(16), fill=(225, 230, 240))
        y += 22

    if upright_from is not None and t >= upright_from:
        d.text((W - 250, H - 36), f"upright at {upright_from:.2f} s",
               font=font(18), fill=(120, 220, 130))
    return np.asarray(img)


def panel_frames(root, sub, title, caption, fps, width, height):
    d = Path(root) / sub
    model = ring_model(d, width, height)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    o = load_outcome(d / "outcome.txt")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 1.15, -17.0, 118.0
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    upright_from = o["t_right_s"] if o["t_right_s"] == o["t_right_s"] else None
    # Hold past standing up, then stop. Long enough to show the legs come
    # back to a normal stance -- at a 1.5 rad/s command limit that transition
    # takes about 2.5 s on its own, and cutting at "upright" ends the clip
    # on a splayed crouch. The trace runs to 12 s so the scoring has room to
    # catch a late topple; six seconds of a robot standing still is not
    # footage.
    t_end = min(t[-1], upright_from + 4.5) if upright_from else t[-1]
    frames = []
    with mujoco.Renderer(model, height, width) as r:
        for k in range(int(t_end * fps) + 1):
            i = int(np.searchsorted(t, k / fps))
            i = min(i, len(t) - 1)
            data.qpos[0:7] = trace["root"][i]
            for a, q in zip(adr, trace["q"][i]):
                data.qpos[a] = q
            mujoco.mj_forward(model, data)
            # Follow in xy only. Following z as well would hide the one thing
            # the clip exists to show.
            cam.lookat[:] = [trace["root"][i, 0], trace["root"][i, 1], 0.10]
            r.update_scene(data, cam, opt)
            frames.append(overlay(r.render(), title, caption, t[i],
                                  trunk_up(trace["root"][i, 3:7]), o, upright_from))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_right")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--only", default=None,
                    help="render just one variant (gentle | fast)")
    args = ap.parse_args()

    import imageio.v2 as imageio

    frames = []
    for sub, title, caption in PANELS:
        if args.only and sub != args.only:
            continue
        if not (Path(args.root) / sub / "trace.csv").exists():
            print(f"  skip {sub} (no trace)")
            continue
        print(f"  rendering {sub}")
        frames += panel_frames(args.root, sub, title, caption,
                               args.fps, args.width, args.height)

    imageio.mimsave(args.out, frames, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(frames)} frames, {len(frames)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
