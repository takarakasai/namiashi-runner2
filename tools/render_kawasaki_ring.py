"""Render the ring WITH its elevation loaded.

The <hfield> is declared with nrow/ncol and no file=, so a bare
MjModel.from_xml_path gets an all-zero grid -- a perfectly flat ring. The
Rust side fills it at runtime via set_hfield_data; anything rendering the
XML on its own has to do the same, or it draws a field with no obstacles on
it and looks entirely plausible while doing so.
"""
import sys, mujoco, numpy as np
from PIL import Image, ImageDraw

xml, pgm_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
m = mujoco.MjModel.from_xml_path(xml)
d = mujoco.MjData(m)

tok = open(pgm_path).read().split()
w, h = int(tok[1]), int(tok[2])
a = np.array([int(v) for v in tok[4:4 + w * h]], dtype=np.float32).reshape(h, w) / 255.0
# The PGM is written top-row-is-+y so it reads like the rulebook's plan
# view; MuJoCo's grid starts at -y, so it goes back the other way up.
a = a[::-1, :]
hid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_HFIELD, "kawasaki")
adr, nrow, ncol = m.hfield_adr[hid], m.hfield_nrow[hid], m.hfield_ncol[hid]
assert (nrow, ncol) == (h, w), f"grid {nrow}x{ncol} vs pgm {h}x{w}"
m.hfield_data[adr:adr + nrow * ncol] = a.reshape(-1)
print(f"filled hfield {nrow}x{ncol}, max={a.max():.3f}")
mujoco.mj_forward(m, d)

cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
opt = mujoco.MjvOption(); mujoco.mjv_defaultOption(opt)
outs = []
for label, (dist, elev, azi) in {
    "3/4 overview": (4.2, -30.0, 40.0),
    "near-top: obstacles": (2.9, -70.0, 20.0),
    "low angle: stand, border, barrier": (3.6, -7.0, 15.0),
}.items():
    cam.distance, cam.elevation, cam.azimuth = dist, elev, azi
    cam.lookat[:] = [0, 0, 0]
    with mujoco.Renderer(m, 460, 900) as r:
        r.update_scene(d, cam, opt); img = r.render()
    im = Image.fromarray(img); ImageDraw.Draw(im).text((10, 10), label, fill=(255, 255, 150))
    outs.append(np.asarray(im))
Image.fromarray(np.vstack(outs)).save(out)
print("wrote", out)
