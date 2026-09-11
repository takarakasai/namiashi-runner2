#!/usr/bin/env python
"""Render namiashi's model-based WBC/MPC climb against the RL policy's
sim-to-sim climb, side by side, on the SAME 5 cm / 10-step staircase.

Two different pipelines feed the same trace.csv/model.xml schema
render_namiashi.py already knows how to play back:
  - WBC panel: tests/wbc_walk.rs's own replay_dir dump (e.g. rise_05/),
    the model-based investigation's own measurement.
  - RL panel: go2_rl/sim2sim_namiashi_mujoco.py --trace-dir, a genuine
    second MuJoCo physics rollout (not the training env) driving the
    exported ONNX policy, logged into the identical CSV schema.

Same reasoning as render_namiashi_footplan_gait_compare.py: this is a
video, not a fresh simulation -- both panels replay their own already-
recorded qpos trace via mj_forward.

    tools/render_namiashi_rl_vs_wbc.py --root /tmp/nami_stairs \
        --out rl_vs_wbc.mp4
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
    ("WBC / MPC (swing 0.04m, default)", "rise_05", "open-loop Raibert + WBC, same as every model-based number this session", 0.320, 0.80),
    ("WBC / MPC (swing 0.10m)", "rise05_swing_0.100", "swing-clearance sweep: raising lift height alone did not fix the climb", 0.320, 0.80),
    ("RL policy (sim-to-sim)", "rl_policy_9393_fixed_5cm", "PPO, trained in Isaac Sim, replayed in a genuine second MuJoCo rollout -- climbs all 10 steps", 0.320, 0.80),
]


def panel_frames(root, sub, fps, seconds, w, h, period):
    d = Path(root) / sub
    model = scened_model(d / "model.xml", w, h)
    data = mujoco.MjData(model)
    trace = load_trace(d / "trace.csv")
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JOINTS]

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance, cam.elevation, cam.azimuth = 2.1, -13.0, 132.0
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
            cam.lookat[:] = data.qpos[0:3]
            r.update_scene(data, cam, opt)
            out.append((r.render(), i, ts - t[0]))
    return out, trace, stance_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/tmp/nami_stairs")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--panel_w", type=int, default=640)
    ap.add_argument("--panel_h", type=int, default=480)
    args = ap.parse_args()

    import imageio.v2 as imageio

    panels = []
    for label, sub, note, period, cmd_vx in PANELS:
        print(f"  rendering {sub}")
        frames, trace, stance_z = panel_frames(
            args.root, sub, args.fps, args.seconds, args.panel_w, args.panel_h, period
        )
        panels.append((label, note, period, cmd_vx, frames, trace, stance_z))

    # Play out to the LONGEST panel's full duration, not the shortest --
    # rise_05's own WBC recording is only 6s (its own test's recorded
    # length), well short of the RL policy's ~7.3s climb; capping to the
    # shorter one used to cut the video off before the RL panel ever
    # visibly finished. Panels that run out of frames first just hold
    # their last one (a frozen final state, not a loop or blank).
    n = max(len(p[4]) for p in panels)
    header_h = 92
    pw, ph = args.panel_w, args.panel_h
    W, H = pw * len(PANELS), ph + header_h

    f_title, f_sub = font(28), font(17)
    f_big, f_med, f_sm = font(24), font(19), font(15)
    out = []
    for i in range(n):
        sheet = Image.new("RGB", (W, H), (13, 15, 20))
        dh = ImageDraw.Draw(sheet)
        dh.text((16, 10), "namiashi  5 cm staircase  model-based WBC/MPC vs. RL policy",
                font=f_title, fill=(240, 244, 250))
        dh.text((16, 46), "same 10-step / 0.05 m rise / 0.20 m run staircase, same forward command -- "
                          "only the controller differs", font=f_sub, fill=(150, 162, 180))

        for k, (label, note, period, cmd_vx, frames, trace, stance_z) in enumerate(panels):
            rgb, idx, elapsed = frames[min(i, len(frames) - 1)]
            frozen = i >= len(frames)
            z = trace["root"][idx, 2]
            frame = draw_step_progress(rgb, z, stance_z, RISE_M, N_STEPS)
            tile = Image.new("RGB", (pw, ph), (13, 15, 20))
            tile.paste(Image.fromarray(frame), (0, 0))
            d = ImageDraw.Draw(tile, "RGBA")
            d.rectangle([0, 0, pw, 78], fill=(0, 0, 0, 178))
            d.text((12, 8), label, font=f_big, fill=(255, 255, 255))
            d.text((12, 38), note, font=f_sm, fill=(170, 180, 196))

            win = period * math.ceil(0.8 / period)
            vx = body_frame_rates(trace, idx, win)[0]
            x = trace["root"][idx, 0]
            d.rectangle([0, ph - 30, pw, ph], fill=(0, 0, 0, 160))
            status = "  [recording ended]" if frozen else ""
            d.text((10, ph - 26), f"t={elapsed:5.2f}s  x={x:5.2f}m  z={z:.3f}m  vx={vx:+.2f} (cmd {cmd_vx:+.2f}) m/s{status}",
                   font=f_med, fill=(215, 220, 232) if not frozen else (150, 150, 150))
            sheet.paste(tile, (k * pw, header_h))
        out.append(np.asarray(sheet))

    imageio.mimsave(args.out, out, fps=args.fps, quality=8, macro_block_size=1)
    print(f"wrote {args.out}  ({len(out)} frames, {len(out)/args.fps:.1f}s)")


if __name__ == "__main__":
    main()
