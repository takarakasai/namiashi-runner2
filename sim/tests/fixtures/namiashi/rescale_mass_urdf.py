#!/usr/bin/env python3
"""URDF counterpart of `rescale_mass.py`, for the IsaacLab/USD import pipeline.

`rescale_mass.py` corrects `namiashi.misa`'s mass (2.400 kg CAD export -> the
built robot's measured 3.3 kg, 600 g/leg) and the knee's 9:14 gear reduction
(effort, velocity, and rotor armature). That correction was never ported to
`urdf/namiashi.urdf` -- the Rust/MuJoCo side loads the `.misa` files, so the
URDF simply went stale. Anything importing the URDF directly (as IsaacLab's
`UrdfConverter` must, since there is no `.misa` importer for USD) would get
the wrong 2.4 kg robot.

Only the "prop" mass variant is built here: `namiashi_3p3_prop.misa` is
`DEFAULT_MISA` in `tests/wbc_walk.rs` and what every staircase measurement
this session used, so it is the one to match for an apples-to-apples RL
comparison against the model-based baseline.

Two things `rescale_mass.py` corrects have no URDF representation and are
deliberately left out here, to be applied later at the IsaacLab
`ArticulationCfg` actuator level (see the plan's Milestone 2):

  - rotor armature (reflected inertia) -- standard URDF `<dynamics>` has no
    armature field at all. Reference values, read directly out of
    `namiashi_3p3_prop.misa` (`[joint.dynamics] armature = ...`) rather than
    re-derived, since they are exactly the numbers to carry into the
    IsaacLab actuator config: hip/thigh 0.0014, knee 0.0014 * (14/9)^2 =
    0.00338765.
  - continuous (rated) torque, 1.0 N*m knee-referred (1.0 * 14/9 = 1.5556 on
    the knee) -- URDF's `effort` is a hard clamp (the peak, 2.5 / 2.5*14/9),
    with no second field for a continuous rating. The Rust harness handles
    this by reporting time-above-rated separately rather than clamping to
    it; an IsaacLab actuator has the same shape of problem and needs the
    same kind of side-channel accounting, not a URDF field that doesn't
    exist.

Also rewrites `package://namiashi_description/meshes/*.stl` mesh URIs to
absolute filesystem paths under `meshes/` (a sibling of `urdf/`, matching
case exactly). IsaacLab's URDF importer is not ROS-integrated and has no
package:// resolver; the CAD-exported package this URDF's meshes came from
has *upper*-case `.STL` filenames, which would not resolve on a
case-sensitive filesystem even if `ROS_PACKAGE_PATH` happened to be set --
resolving to an absolute path here sidesteps both problems.

Usage:
    python3 rescale_mass_urdf.py
Writes urdf/namiashi_3p3_prop.urdf next to the source URDF.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).parent
SRC = HERE / "urdf" / "namiashi.urdf"
DST = HERE / "urdf" / "namiashi_3p3_prop.urdf"

TOTAL_KG = 3.3
PER_LEG_KG = 0.600
LEG_PREFIXES = ("FL_", "FR_", "RL_", "RR_")
INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")

# Same numbers `rescale_mass.py` uses; effort/velocity, the two fields URDF
# can actually express. See the module docstring for what is deliberately
# left out.
KNEE_GEAR = 14.0 / 9.0
PEAK_TORQUE_NM = 2.5

# For the record (applied at the ArticulationCfg level, not here):
ARMATURE_BASE = 0.0014
ARMATURE_KNEE = ARMATURE_BASE * KNEE_GEAR**2  # 0.00338765
RATED_TORQUE_NM = 1.0
RATED_TORQUE_KNEE_NM = RATED_TORQUE_NM * KNEE_GEAR  # 1.5556


def mass_of(link):
    m = link.find("inertial/mass")
    return float(m.get("value")) if m is not None else None


def scale_link(link, f):
    """Scale a link's mass and inertia by `f`, leaving geometry alone --
    same physical reasoning as `rescale_mass.py::scale_block`: I = integral
    r^2 dm, r untouched, so inertia scales with mass."""
    if f == 1.0:
        return
    inertial = link.find("inertial")
    if inertial is None:
        return
    mass_el = inertial.find("mass")
    if mass_el is not None:
        mass_el.set("value", f"{float(mass_el.get('value')) * f:.6g}")
    inertia_el = inertial.find("inertia")
    if inertia_el is not None:
        for key in INERTIA_KEYS:
            v = inertia_el.get(key)
            if v is not None:
                inertia_el.set(key, f"{float(v) * f:.6g}")


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    links = root.findall("link")

    leg = {l.get("name"): mass_of(l) for l in links if l.get("name", "").startswith(LEG_PREFIXES)}
    leg = {n: m for n, m in leg.items() if m is not None}
    body = {
        l.get("name"): mass_of(l)
        for l in links
        if not l.get("name", "").startswith(LEG_PREFIXES) and mass_of(l)
    }

    fl = {n: m for n, m in leg.items() if n.startswith("FL_")}
    leg_now = sum(fl.values())
    body_now = sum(body.values())
    body_target = TOTAL_KG - 4 * PER_LEG_KG
    if body_target <= 0:
        sys.exit(f"4 x {PER_LEG_KG} kg of legs leaves nothing for a {TOTAL_KG} kg robot")

    f_body = body_target / body_now
    # "prop": every leg link (hip/thigh/calf/foot) scaled by the same
    # factor, i.e. the added mass follows the CAD distribution -- matches
    # `rescale_mass.py`'s "prop" branch exactly.
    f_leg = PER_LEG_KG / leg_now

    for l in links:
        name = l.get("name", "")
        if name.startswith(LEG_PREFIXES):
            scale_link(l, f_leg)
        elif mass_of(l):
            scale_link(l, f_body)

    # Knee gear ratio, URDF-representable fields only. Net effect, cross-
    # checked directly against namiashi_3p3_prop.misa's [joint.limit] blocks
    # rather than re-derived from rescale_mass.py's two-pass logic (its
    # apply_torque_rating() runs after apply_knee_gear() and overwrites
    # every leg joint's `effort` unconditionally, so apply_knee_gear's own
    # effort scaling never survives to the output file -- only velocity and
    # armature do):
    #   hip/thigh  effort=2.5      velocity=33.5 (unchanged)
    #   calf       effort=3.88889  velocity=21.5357
    base_velocity = None
    for j in root.findall("joint"):
        if j.get("name", "").endswith("_thigh_joint"):
            limit = j.find("limit")
            base_velocity = float(limit.get("velocity"))
            break
    if base_velocity is None:
        sys.exit("no _thigh_joint found to read a reference velocity from")

    for j in root.findall("joint"):
        name = j.get("name", "")
        if not name.startswith(LEG_PREFIXES):
            continue
        limit = j.find("limit")
        if limit is None:
            continue
        is_knee = name.endswith("_calf_joint")
        limit.set("effort", f"{PEAK_TORQUE_NM * (KNEE_GEAR if is_knee else 1.0):.6g}")
        if is_knee:
            limit.set("velocity", f"{base_velocity / KNEE_GEAR:.6g}")
        # hip/thigh velocity is left exactly as the source URDF has it.

    # package:// -> absolute path, see module docstring for why.
    mesh_dir = HERE / "meshes"
    n_rewritten = 0
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        prefix = "package://namiashi_description/meshes/"
        if fn.startswith(prefix):
            resolved = mesh_dir / fn[len(prefix):]
            if not resolved.is_file():
                sys.exit(f"mesh not found: {resolved} (from {fn!r})")
            mesh.set("filename", str(resolved))
            n_rewritten += 1

    tree.write(DST, xml_declaration=False)

    # Verify against the same checks rescale_mass.py prints.
    tree2 = ET.parse(DST)
    links2 = tree2.getroot().findall("link")
    total = sum(mass_of(l) for l in links2 if mass_of(l))
    legs = sum(
        mass_of(l) for l in links2 if l.get("name", "").startswith(LEG_PREFIXES) and mass_of(l)
    )
    print(
        f"{DST.name}: total={total:.4f} kg  legs={legs:.4f} ({100 * legs / total:.0f}%)  "
        f"body={total - legs:.4f}"
    )
    fl2 = {
        l.get("name"): mass_of(l)
        for l in links2
        if l.get("name", "").startswith("FL_") and mass_of(l)
    }
    print(
        "   per-leg (FL): "
        + "  ".join(f"{n.split('_', 1)[1]}={m:.4f}" for n, m in sorted(fl2.items()))
    )
    print(
        f"   knee gear (URDF fields): effort {PEAK_TORQUE_NM:.3f} -> "
        f"{PEAK_TORQUE_NM * KNEE_GEAR:.5f} N*m   velocity {base_velocity:.1f} -> "
        f"{base_velocity / KNEE_GEAR:.4f} rad/s"
    )
    print(
        f"   knee gear (apply at ArticulationCfg, not URDF): armature "
        f"{ARMATURE_BASE:.4f} -> {ARMATURE_KNEE:.6f}   rated torque "
        f"{RATED_TORQUE_NM:.1f} -> {RATED_TORQUE_KNEE_NM:.4f} N*m"
    )
    print(f"   mesh URIs rewritten to absolute paths: {n_rewritten}")
    return DST


if __name__ == "__main__":
    main()
