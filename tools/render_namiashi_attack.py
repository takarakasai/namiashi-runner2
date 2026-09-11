#!/usr/bin/env python
"""Render the arm attack from a trace written by
`examples/namiashi_attack_video`.

Two clips back to back: the move, then the same move with the arm left up.
The second is not padding. At the 0.38 m range where this works the robot is
close enough that "it walked into it and knocked it over" and "the arm
levered it" look similar, and the only way to show which one happened is to
run it again without the arm.

Three things this has to get right:

  - The `<hfield>` is declared with nrow/ncol and no `file=`, so loading the
    XML alone gets an all-zero grid and draws a flat ring. Filled from the
    PGM here, as the Rust side fills it at runtime.
  - The opponent is a free body, so its seven-dof pose has to be written
    into its own qpos slot -- `mj_forward` will not put it anywhere.
  - The camera looks at the midpoint of the two robots rather than tracking
    either. The subject is what happens between them.

    tools/render_namiashi_attack.py --root /tmp/nami_attack \\
        --out attack.mp4
"""
import argparse
import csv
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_namiashi import font

JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "arm_pitch_joint",
]

PANELS = [
    ("attack", "X  -- arm attack",
     "crouch and drop the arm, creep in, then extend the front legs as the arm swings up"),
    ("no_arm", "the same move, arm left up",
     "the control: if the opponent goes over anyway, the arm was not what did it"),
]


def load_trace(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    n = len(rows)
    out = {
        "t": np.empty(n),
        "root": np.empty((n, 7)),
        "q": np.empty((n, len(JOINTS))),
        "foe": np.empty((n, 7)),
    }
    for i, r in enumerate(rows):
        out["t"][i] = float(r["t"])
        out["root"][i] = [float(r[k]) for k in
                          ("root_x", "root_y", "root_z",
                           "root_qw", "root_qx", "root_qy", "root_qz")]
        out["q"][i] = [float(r[j]) for j in JOINTS]
        out["foe"][i] = [float(r[k]) for k in
                         ("foe_x", "foe_y", "foe_z",
                          "foe_qw", "foe_qx", "foe_qy", "foe_qz")]
    return out


def load_outcome(path):
    vals = {}
    for line in Path(path).read_text().splitlines():
        k, _, v = line.partition(" ")
        vals[k] = float(v)
    return vals


def ring_model(d, width, height):
    model = mujoco.MjModel.from_xml_path(str(d / "model.xml"))
    tok = (d / "ring_elevation.pgm").read_text().split()
    w, h = int(tok[1]), int(tok[2])
    a = np.array([int(v) for v in tok[4:4 + w * h]], dtype=np.float32).reshape(h, w) / 255.0
    a = a[::-1, :]
    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "kawasaki_ring")
    adr = model.hfield_adr[hid]
    nrow, ncol = model.hfield_nrow[hid], model.hfield_ncol[hid]
    assert (nrow, ncol) == (h, w), f"grid {nrow}x{ncol} vs pgm {h}x{w}"
    model.hfield_data[adr:adr + nrow * ncol] = a.reshape(-1)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    return model


def up_of(quat):
    """The trunk's own +z in world coordinates: +1 as placed, -1 on its back."""
    _, x, y, _ = quat
    return 1.0 - 2.0 * (x * x + y * y)


def phase_of(t, o):
    """Which part of the move is running at `t`."""
    t0 = o["attack_at_s"]
    if t < t0:
        return "standing by"
    if t < t0 + o["set_s"]:
        return "1. crouch, arm down"
    if t < t0 + o["set_s"] + o["creep_s"]:
        return "2. creep in"
    if t < t0 + o["set_s"] + o["creep_s"] + o["lift_s"]:
        return "3. extend front, arm up"
    return "done"


def overlay(frame, title, caption, t, foe_up, arm, o):
    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img, "RGBA")
    W, H = img.size
    d.rectangle([0, 0, W, 78], fill=(0, 0, 0, 150))
    d.text((14, 8), title, font=font(26), fill=(255, 255, 255))
    d.text((14, 44), caption, font=font(15), fill=(190, 200, 215))

    # The opponent's attitude, which is the whole question.
    bx, by, bw, bh = W - 250, H - 62, 220, 16
    d.rectangle([bx, by, bx + bw, by + bh], outline=(140, 150, 165), width=1)
    mid = bx + bw / 2
    frac = (foe_up + 1.0) / 2.0
    col = (110, 210, 120) if foe_up < -0.7 else (225, 190, 90) if foe_up < 0.7 else (215, 110, 100)
    x0, x1 = sorted((mid, mid + (frac - 0.5) * bw))
    d.rectangle([x0, by + 1, x1, by + bh - 1], fill=col)
    d.line([mid, by, mid, by + bh], fill=(200, 200, 210), width=1)
    d.text((bx, by - 20), f"opponent +z {foe_up:+.2f}    -1 on its back",
           font=font(13), fill=(215, 220, 230))

    lines = [
        f"t {t:5.2f} s",
        phase_of(t, o),
        f"arm {arm:+.2f} rad  ({'down' if arm > 0.2 else 'up' if arm < -0.2 else 'level'})",
    ]
    y = H - 22 * len(lines) - 12
    d.rectangle([0, y - 8, 330, H], fill=(0, 0, 0, 130))
    for s in lines:
        d.text((14, y), s, font=font(16), fill=(225, 230, 240))
        y += 22

    # Keyed on the attitude right now, not on how the run ends: an earlier
    # version announced "opponent flipped" the moment the phases finished,
    # while the bar beside it still read +0.44 and the opponent was only
    # halfway over.
    if foe_up < -0.7:
        d.text((W - 250, H - 36), "opponent flipped",
               font=font(18), fill=(120, 220, 130))
    elif t > o["attack_at_s"] + o["set_s"] + o["creep_s"] + o["lift_s"] + 2.0:
        d.text((W - 250, H - 36), f"still up ({foe_up:+.2f})",
               font=font(18), fill=(230, 150, 140))
    return np.asarray(img)


def panel_frames(root, sub, title, caption, fps, width, height):
    d = Path(root) / sub
    model = ring_model(d, width, height)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    o = load_outcome(d / "outcome.txt")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]
    # The opponent's own free joint: first (and only) joint under foe_trunk.
    foe_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foe_trunk")
    foe_adr = model.jnt_qposadr[model.body_jntadr[foe_body]]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    # Side on. The approach runs along +x, so a camera off to -y sees both
    # robots in profile and, crucially, the gap the arm has to get into --
    # from behind the attacker the two overlap and the arm is hidden by the
    # thing it is going under.
    cam.distance, cam.elevation, cam.azimuth = 1.25, -8.0, 90.0
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    t = trace["t"]
    # Start just before the move and stop a couple of seconds after it, so
    # the clip is the move rather than the standing about either side of it.
    t_start = max(0.0, o["attack_at_s"] - 1.0)
    t_end = min(t[-1], o["attack_at_s"] + o["set_s"] + o["creep_s"] + o["lift_s"] + 2.5)
    frames = []
    with mujoco.Renderer(model, height, width) as r:
        for k in range(int((t_end - t_start) * fps) + 1):
            i = min(int(np.searchsorted(t, t_start + k / fps)), len(t) - 1)
            data.qpos[0:7] = trace["root"][i]
            for a, q in zip(adr, trace["q"][i]):
                data.qpos[a] = q
            data.qpos[foe_adr:foe_adr + 7] = trace["foe"][i]
            mujoco.mj_forward(model, data)
            # Between the two of them, not on either: the subject is the gap.
            cam.lookat[:] = [
                0.5 * (trace["root"][i, 0] + trace["foe"][i, 0]),
                0.5 * (trace["root"][i, 1] + trace["foe"][i, 1]),
                0.10,
            ]
            r.update_scene(data, cam, opt)
            frames.append(overlay(r.render(), title, caption, t[i],
                                  up_of(trace["foe"][i, 3:7]), trace["q"][i, 12], o))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_attack")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--only", default=None, help="just one variant (attack | no_arm)")
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
