#!/usr/bin/env python3
"""Rewrite namiashi's link masses to a measured total, keeping geometry fixed.

The shipped `namiashi.misa` was converted from a CAD export that totals
2.400 kg, with 0.869 kg of that in the legs (36%). The built robot weighs
3.3 kg, and each leg is 600 g including its motors -- so the legs are 2.4 kg
(73%) and everything else is 0.9 kg. That is not a small correction: it
roughly inverts where the mass lives.

Geometry is unchanged, so for a link whose mass scales by `f` the inertia
tensor scales by `f` too (I = integral r^2 dm, and r is untouched). Scaling
mass without scaling inertia would leave a link that weighs three times more
but resists rotation the same, which is not any physical object.

Within a leg the 600 g has to go somewhere, and the spec does not say where.
The two variants below bracket the plausible range:

  hip   -- all the added mass at the hip link, where a quadruped's abduction
           and hip motors normally sit. Leg inertia about the hip stays low,
           so the swing leg is cheap to move and the trunk sees most of it.
  prop  -- every leg link scaled by the same factor, i.e. the added mass
           follows the CAD distribution. Puts mass out at the thigh and calf,
           which is the pessimistic case for swing cost and for the massless
           -leg assumption in the SRBD MPC.

Which one is right is a fact about the hardware. Which one *matters* is a
question the simulator can answer, so both get built and measured.
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
SRC = HERE / "namiashi.misa"

TOTAL_KG = 3.3
PER_LEG_KG = 0.600
LEG_PREFIXES = ("FL_", "FR_", "RL_", "RR_")
INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")

# The knee runs through an extra 9:14 reduction that the hip and thigh do not.
# The source URDF half-reflects it: calf `effort` is 2.205 against the others'
# 1.5, a ratio of 1.470 where 14/9 is 1.5556 -- 5.5% short. And a reduction
# does three things, not one:
#
#   torque   x 14/9        1.5   -> 2.3333 N*m   (file has 2.205)
#   speed    x  9/14      33.5   -> 21.536 rad/s (file has 33.5, unchanged)
#   rotor    x (14/9)^2   0.0014 -> 0.003388     (file has 0.0014, unchanged)
#
# The last two are not reflected at all, and they are the ones that bite. The
# sim currently lets the knee spin 1.56x faster than the hardware can and
# gives it 2.4x too little reflected inertia -- and armature is the dominant
# term in this leg's swing inertia, so that is not a rounding error.
KNEE_GEAR = 14.0 / 9.0
KNEE_SUFFIX = "calf"

# Actuator torque. The CAD-derived model shipped 1.5 N*m on the hip and thigh
# and 2.205 on the calf; the real part is 2.5 N*m peak, and the knee's
# reduction puts it at 2.5 * 14/9 = 3.889.
#
# `effort` in MuJoCo is a hard clamp, so it is the *peak*. The continuous
# rating is 1.0 N*m, which the clamp cannot express -- a gait that sits at
# 2 N*m indefinitely never clamps and still cooks the motor. The harness
# reports time-above-rated separately for that reason.
PEAK_TORQUE_NM = 2.5
RATED_TORQUE_NM = 1.0


def apply_torque_rating(text):
    """Set peak torque on the leg joints: `PEAK_TORQUE_NM`, knee geared up."""
    out = text
    for name, st, en in reversed(list(_blocks(text, "joint"))):
        if not name.startswith(LEG_PREFIXES):
            continue
        target = PEAK_TORQUE_NM * (
            KNEE_GEAR if name.endswith(f"_{KNEE_SUFFIX}_joint") else 1.0
        )
        blk = re.sub(
            r"^(\s*)effort = [0-9.eE+-]+",
            lambda m: f"{m.group(1)}effort = {target:.6g}",
            text[st:en],
            count=1,
            flags=re.M,
        )
        out = out[:st] + blk + out[en:]
    print(f"  peak torque -> hip/thigh {PEAK_TORQUE_NM:.3f}, "
          f"knee {PEAK_TORQUE_NM * KNEE_GEAR:.3f} N*m "
          f"(rated {RATED_TORQUE_NM:.1f})")
    return out


def _blocks(text, kind):
    """Yield (name, start, end) for each [[kind]] block.

    The end is the next top-level `[[...]]` header of ANY kind, not the next
    header of this kind -- otherwise the last `[[link]]` runs to EOF and
    swallows every joint, actuator and collision-pair entry after it.
    """
    tops = [m.start() for m in re.finditer(r"^\[\[[a-z_]+\]\]", text, re.M)]
    for m in re.finditer(rf"^\[\[{kind}\]\]", text, re.M):
        st = m.start()
        en = next((t for t in tops if t > st), len(text))
        name = re.search(r'^name = "([^"]+)"', text[st:en], re.M)
        yield (name.group(1) if name else "", st, en)


def link_blocks(text):
    return _blocks(text, "link")


def apply_knee_gear(text):
    """Referred torque, speed and rotor inertia through the knee's reduction.

    Applied to the ratios the other joints have, not to the calf's own
    numbers, because the calf's `effort` is already partly geared (1.470x
    rather than 14/9) and compounding it would double-count.
    """
    base = {}
    for name, st, en in _blocks(text, "joint"):
        if name.endswith("_thigh_joint"):
            blk = text[st:en]
            for key in ("effort", "velocity", "armature"):
                m = re.search(rf"^\s*{key} = ([0-9.eE+-]+)", blk, re.M)
                if m:
                    base[key] = float(m.group(1))
            break
    missing = {"effort", "velocity", "armature"} - base.keys()
    if missing:
        sys.exit(f"no reference joint for {sorted(missing)}")

    target = {
        "effort": base["effort"] * KNEE_GEAR,
        "velocity": base["velocity"] / KNEE_GEAR,
        "armature": base["armature"] * KNEE_GEAR**2,
    }

    out = text
    for name, st, en in reversed(list(_blocks(text, "joint"))):
        if not name.endswith(f"_{KNEE_SUFFIX}_joint"):
            continue
        blk = text[st:en]
        for key, val in target.items():
            blk = re.sub(
                rf"^(\s*){key} = [0-9.eE+-]+",
                lambda m, k=key, v=val: f"{m.group(1)}{k} = {v:.6g}",
                blk,
                count=1,
                flags=re.M,
            )
        out = out[:st] + blk + out[en:]
    print(
        "  knee 9:14 -> effort {effort:.4f} N*m  velocity {velocity:.3f} rad/s  "
        "armature {armature:.6f}".format(**target)
    )
    return out


def mass_of(block):
    m = re.search(r"^mass = ([0-9.eE+-]+)", block, re.M)
    return float(m.group(1)) if m else None


def scale_block(block, f):
    """Scale a link's mass and inertia by `f`, leaving origin/geometry alone."""
    if f == 1.0:
        return block

    def sub_mass(m):
        return f"mass = {float(m.group(1)) * f:.6g}"

    out = re.sub(r"^mass = ([0-9.eE+-]+)", sub_mass, block, count=1, flags=re.M)
    for key in INERTIA_KEYS:
        def sub_i(m, k=key):
            return f"{k} = {float(m.group(1)) * f:.6g}"

        out = re.sub(
            rf"^{key} = ([0-9.eE+-]+)", sub_i, out, count=1, flags=re.M
        )
    return out


def build(variant):
    text = SRC.read_text()
    blocks = list(link_blocks(text))

    leg = {n: mass_of(text[s:e]) for n, s, e in blocks if n.startswith(LEG_PREFIXES)}
    leg = {n: m for n, m in leg.items() if m is not None}
    body = {
        n: mass_of(text[s:e])
        for n, s, e in blocks
        if not n.startswith(LEG_PREFIXES) and mass_of(text[s:e])
    }

    # One leg's worth, read off the FL_ links.
    fl = {n: m for n, m in leg.items() if n.startswith("FL_")}
    leg_now = sum(fl.values())
    body_now = sum(body.values())
    body_target = TOTAL_KG - 4 * PER_LEG_KG
    if body_target <= 0:
        sys.exit(f"4 x {PER_LEG_KG} kg of legs leaves nothing for a {TOTAL_KG} kg robot")

    f_body = body_target / body_now
    if variant == "prop":
        factors = {suffix: PER_LEG_KG / leg_now for suffix in ("hip", "thigh", "calf", "foot")}
    elif variant == "hip":
        non_hip = sum(m for n, m in fl.items() if not n.endswith("_hip"))
        hip_now = leg_now - non_hip
        factors = {"hip": (PER_LEG_KG - non_hip) / hip_now, "thigh": 1.0, "calf": 1.0, "foot": 1.0}
    else:
        sys.exit(f"unknown variant {variant!r}")

    # Rebuild back-to-front so the offsets stay valid.
    out = text
    for name, st, en in reversed(blocks):
        block = text[st:en]
        if name.startswith(LEG_PREFIXES):
            suffix = name.split("_", 1)[1]
            f = factors.get(suffix)
            if f is None:
                sys.exit(f"leg link {name!r} has no factor -- unexpected link name")
        elif mass_of(block):
            f = f_body
        else:
            continue
        out = out[:st] + scale_block(block, f) + out[en:]

    out = apply_knee_gear(out)
    out = apply_torque_rating(out)

    dst = HERE / f"namiashi_3p3_{variant}.misa"
    dst.write_text(out)

    check = {n: mass_of(out[s:e]) for n, s, e in link_blocks(out) if mass_of(out[s:e])}
    total = sum(check.values())
    legs = sum(m for n, m in check.items() if n.startswith(LEG_PREFIXES))
    print(f"{dst.name}: total={total:.4f} kg  legs={legs:.4f} ({100*legs/total:.0f}%)  "
          f"body={total-legs:.4f}")
    print("   per-leg (FL): " + "  ".join(
        f"{n.split('_',1)[1]}={check[n]:.4f}" for n in sorted(check) if n.startswith("FL_")))
    return dst


if __name__ == "__main__":
    for v in ("hip", "prop"):
        build(v)
