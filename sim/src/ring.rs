//! The かわさきロボット競技大会 ring, and the joint-locked opponent that
//! stands on it. Moved out of articara's `mjcf.rs` (branch `namiashi-wbc`)
//! -- the escape hatches it splices into (`extra_asset_xml` /
//! `extra_worldbody_xml`) stayed there, this file is the ring-specific
//! content that goes through them.

use articara::mjcf::{export_mjcf_with_options, scene_lighting_worldbody_xml, MjcfExportOptions};
use articara::robot::RobotModel;

/// The 第31回かわさきロボット競技大会 competition ring, as a heightfield.
///
/// A heightfield rather than composed primitives because every feature here
/// is single-valued in z -- holes are the plate's thickness being *absent*,
/// the central bowl is a dished surface, the edge banks are bumps. Building
/// a 100 mm circular hole out of boxes needs hundreds of them per plate and
/// still stair-steps the rim; one grid expresses all of it.
///
/// # Where the dimensions come from
///
/// Everything except the bank height was recovered from the rulebook PDF's
/// own vector geometry (`31th_ring_0401.pdf`), not scaled off a raster by
/// eye: the drawing's line work was extracted, the ring square used as the
/// scale reference (538.578 pt = 190 cm), and every feature's corners read
/// out of it. Values land on exact centimetres, which is the check that the
/// scale is right. An earlier version of this type guessed the obstacle
/// centres at +/-48 cm; they are +/-65 cm, and the two start platforms are
/// the same size rather than the 30/45 asymmetry the raster suggested.
///
/// `[pdf]` is measured from that geometry. `[assumed]` is not in the drawing
/// at all -- only the edge bank's height, since the rulebook gives its
/// profile as "断面が半楕円形" with no dimension. Every measurement is still
/// a field rather than a constant, because the rulebook itself notes
/// "安全対策及び加工・配置に起因する寸法、形状誤差があります".
#[derive(Clone, Debug)]
pub struct KawasakiRingCfg {
    /// `[pdf]` Ring plate, square, 190 cm on a side.
    pub ring_m: f64,
    /// `[pdf]` Start platform footprint `(width_along_edge, depth_outward)`.
    /// Both platforms measure 45 x 35 cm: the drawing's 30 cm dimension
    /// belongs to something else, and the apparent red/blue asymmetry was an
    /// artifact of reading the raster. Kept as two fields anyway so an
    /// actual asymmetry stays expressible.
    pub blue_platform_m: (f64, f64),
    /// `[pdf]` See `blue_platform_m` -- same size.
    pub red_platform_m: (f64, f64),
    /// Start platform slab THICKNESS. Its top is flush with the ring
    /// surface, which is how the isometric reads -- a start zone the robot
    /// drives off, not a step it has to descend -- so this only sets how far
    /// the slab hangs below. Must be positive; MuJoCo rejects a zero-size
    /// box, and a flush platform is expressed by where the slab sits, not by
    /// giving it no thickness.
    pub platform_h_m: f64,
    /// `[pdf]` Central bowl obstacle: 45 cm square.
    pub bowl_m: f64,
    /// `[pdf]` Width of the bowl's flat outer frame.
    pub bowl_frame_m: f64,
    /// `[pdf]` Bowl rim height. The drawing shows 2.8 cm on the side view
    /// and 2.5 cm on section A-A; taken as the rim, with the difference
    /// most likely frame-vs-lip.
    pub bowl_rim_h_m: f64,
    /// `[pdf]` Height at the bowl's centre, i.e. how far the dish drops.
    pub bowl_centre_h_m: f64,
    /// `[pdf]` Hole-plate obstacles are 30 cm square, 1.5 cm thick.
    pub plate_m: f64,
    pub plate_h_m: f64,
    /// `[pdf]` Single-hole plate: one 180 mm hole, centred.
    pub round_hole_d_m: f64,
    /// `[pdf]` Four-hole plate: 100 mm holes on a 150 mm square pitch.
    pub quad_hole_d_m: f64,
    pub quad_hole_pitch_m: f64,
    /// `[pdf]` Round-hole plate centres, metres from the ring centre.
    pub round_plate_centres: Vec<(f64, f64)>,
    /// `[pdf]` Four-hole plate centres. Drawn rotated 45 deg (diamond).
    pub quad_plate_centres: Vec<(f64, f64)>,
    /// `[pdf]` Half-width of the edge bank's semi-elliptical section. The
    /// banks measure 2 cm across and sit flush against the ring edge.
    pub bank_half_w_m: f64,
    /// `[photo]` Safety barrier around the field: `(height, gap_outside_ring,
    /// thickness)` in metres. Transparent panels on frames, visible in every
    /// competition photo, and the one piece of the surroundings that is
    /// physical rather than scenery -- a robot that leaves the ring stops
    /// here instead of walking out of the world. `None` omits it.
    ///
    /// The gap is bounded by `floor_size_m`: the panels stand ON the venue
    /// floor, so `ring_m/2 + gap + thickness` has to stay inside it. At the
    /// 5 m default that caps the gap near 1.5 m.
    ///
    /// `[photo]` throughout: read off event photographs, not a drawing.
    pub barrier: Option<(f64, f64, f64)>,
    /// `[photo]` Coloured border framing the ring on the two start sides,
    /// as `(width, height)`. Scenery -- the red and blue frames that make
    /// which end is which readable at a glance. `None` omits it.
    pub border: Option<(f64, f64)>,
    /// Whether to emit lighting, sky and the two fill lights. Off leaves
    /// MuJoCo's default dim headlight and black void.
    ///
    /// Note the colours here are deliberately LIGHTER than the real field,
    /// which is near-black in every photograph. At the real contrast the
    /// 15 mm obstacle plates disappear into the surface they sit on, and a
    /// simulation you cannot read is not more realistic in any useful sense.
    pub lighting: bool,
    /// `[assumed]` Bank height -- the ONE number the rulebook never gives.
    /// "断面が半楕円形" only says the section is a半楕円, so this picks a
    /// height that is not simply the半円 that a 1 cm half-width would imply.
    pub bank_h_m: f64,
    /// `[pdf]` Edge bank segments as `(x0, y0, x1, y1)` centrelines, metres
    /// from the ring centre. Each runs 1 cm in from its own ring edge.
    pub bank_segments: Vec<(f64, f64, f64, f64)>,
    /// Heightfield grid pitch. 5 mm resolves a 100 mm hole across 20 cells.
    pub cell_m: f64,
    /// Flat floor under the ring, as the side length of a square slab.
    /// `None` omits it, leaving a robot that walks off the edge to fall
    /// indefinitely -- which reads as a diverging simulation rather than as
    /// the ring-out it actually is.
    pub floor_size_m: Option<f64>,
    /// Height of the stand under the ring: the distance from the ring
    /// slab's UNDERSIDE down to the venue floor.
    ///
    /// Measured from the underside on purpose -- quoting it from the ring
    /// surface makes the visible gap depend on `plate_thickness_m`, so the
    /// same number looks different every time the slab changes.
    ///
    /// Competition photos put the ring at about table height, so ~0.7 is
    /// what the real setup looks like; the default stays at the 0.20 that
    /// was asked for, since it is one number to change and guessing at a
    /// height nobody specified is not an improvement.
    pub floor_gap_m: f64,
    /// Thickness of the ring slab itself. Not in the rulebook -- the drawing
    /// dimensions the ring's top face and its obstacles, never how deep the
    /// plate is -- so `[assumed]`, and exposed because it eats into
    /// `floor_drop_m`'s visible gap.
    pub plate_thickness_m: f64,
}

impl Default for KawasakiRingCfg {
    /// Measured from `31th_ring_0401.pdf`; see the type's doc comment.
    fn default() -> Self {
        Self {
            ring_m: 1.90,
            blue_platform_m: (0.45, 0.35),
            red_platform_m: (0.45, 0.35),
            platform_h_m: 0.05,
            bowl_m: 0.45,
            bowl_frame_m: 0.05,
            bowl_rim_h_m: 0.028,
            bowl_centre_h_m: 0.012,
            plate_m: 0.30,
            plate_h_m: 0.015,
            round_hole_d_m: 0.18,
            quad_hole_d_m: 0.10,
            quad_hole_pitch_m: 0.15,
            round_plate_centres: vec![(-0.65, 0.0), (0.65, 0.0)],
            quad_plate_centres: vec![(0.0, -0.65), (0.0, 0.65)],
            bank_half_w_m: 0.01,
            barrier: Some((0.60, 1.50, 0.02)),
            border: Some((0.08, 0.03)),
            lighting: true,
            bank_h_m: 0.015,
            // Each bank is 2 cm across and flush with its own ring edge, so
            // the centreline sits 1 cm in from +/-0.95. Top and bottom run
            // 150 cm centred; the two side banks run 100 cm and are offset
            // in opposite directions, which is what makes the layout
            // rotationally symmetric about the ring centre rather than
            // mirror-symmetric.
            bank_segments: vec![
                (-0.75, 0.94, 0.75, 0.94),
                (-0.75, -0.94, 0.75, -0.94),
                (-0.94, -0.25, -0.94, 0.75),
                (0.94, -0.75, 0.94, 0.25),
            ],
            cell_m: 0.005,
            floor_size_m: Some(5.0),
            floor_gap_m: 0.20,
            plate_thickness_m: 0.05,
        }
    }
}

impl KawasakiRingCfg {
    /// Depth of the floor's top surface below the ring surface, metres --
    /// the slab's own thickness plus the air gap under it.
    pub fn floor_top_z(&self) -> f64 {
        self.plate_thickness_m + self.floor_gap_m
    }

    /// Grid dimensions of the heightfield, `(nrow, ncol)`.
    pub fn grid(&self) -> (usize, usize) {
        let n = (self.ring_m / self.cell_m).round().max(2.0) as usize;
        (n, n)
    }

    /// Tallest feature, metres. MuJoCo scales the normalised grid by this.
    pub fn z_top_m(&self) -> f64 {
        self.bowl_rim_h_m
            .max(self.plate_h_m)
            .max(self.bank_h_m)
            .max(1e-6)
    }

    /// Elevation grid, row-major, normalised to `[0, 1]` for
    /// [`articara::mujoco_sim::MujocoSim::set_hfield_data`].
    ///
    /// Row 0 is `-y`, column 0 is `-x`, matching MuJoCo's own hfield layout.
    pub fn heights(&self) -> Vec<f32> {
        let (nrow, ncol) = self.grid();
        let z_top = self.z_top_m();
        let half = self.ring_m / 2.0;
        let mut out = vec![0.0_f32; nrow * ncol];
        for r in 0..nrow {
            // Cell centres, so a feature edge never lands exactly on a
            // sample and alias to whichever side floating point picks.
            let y = -half + (r as f64 + 0.5) * self.ring_m / nrow as f64;
            for c in 0..ncol {
                let x = -half + (c as f64 + 0.5) * self.ring_m / ncol as f64;
                out[r * ncol + c] = (self.height_at(x, y) / z_top) as f32;
            }
        }
        out
    }

    /// Surface height at a ring-frame point, metres above the plate.
    ///
    /// Features are applied highest-wins rather than in sequence: they do
    /// not overlap in the nominal layout, but a mistyped centre should show
    /// as two obstacles intersecting, not as one silently erasing the other.
    pub fn height_at(&self, x: f64, y: f64) -> f64 {
        let mut z: f64 = 0.0;

        // Central bowl: flat frame at the rim, dishing to the centre.
        let hb = self.bowl_m / 2.0;
        if x.abs() <= hb && y.abs() <= hb {
            let inner = hb - self.bowl_frame_m;
            // Chebyshev radius, so the dish is square-symmetric like the
            // drawing's four triangular faces rather than a circular bowl.
            let t = (x.abs().max(y.abs()) - 0.0) / inner.max(1e-9);
            z = z.max(if x.abs() <= inner && y.abs() <= inner {
                self.bowl_centre_h_m
                    + (self.bowl_rim_h_m - self.bowl_centre_h_m) * t.clamp(0.0, 1.0)
            } else {
                self.bowl_rim_h_m
            });
        }

        let hp = self.plate_m / 2.0;
        for &(cx, cy) in &self.round_plate_centres {
            let (dx, dy) = (x - cx, y - cy);
            if dx.abs() <= hp && dy.abs() <= hp && dx.hypot(dy) > self.round_hole_d_m / 2.0 {
                z = z.max(self.plate_h_m);
            }
        }
        for &(cx, cy) in &self.quad_plate_centres {
            // Drawn as a diamond: rotate the query into the plate's frame.
            let (dx, dy) = (x - cx, y - cy);
            let s = std::f64::consts::FRAC_1_SQRT_2;
            let (px, py) = (s * (dx + dy), s * (dy - dx));
            if px.abs() > hp || py.abs() > hp {
                continue;
            }
            let q = self.quad_hole_pitch_m / 2.0;
            let in_hole = [(-q, -q), (q, -q), (-q, q), (q, q)]
                .iter()
                .any(|&(ox, oy)| (px - ox).hypot(py - oy) <= self.quad_hole_d_m / 2.0);
            if !in_hole {
                z = z.max(self.plate_h_m);
            }
        }

        // Edge banks: semi-elliptical across the segment, flat along it.
        for &(x0, y0, x1, y1) in &self.bank_segments {
            let (vx, vy) = (x1 - x0, y1 - y0);
            let len2 = vx * vx + vy * vy;
            if len2 < 1e-12 {
                continue;
            }
            let t = (((x - x0) * vx + (y - y0) * vy) / len2).clamp(0.0, 1.0);
            let d = (x - (x0 + t * vx)).hypot(y - (y0 + t * vy));
            if d < self.bank_half_w_m {
                let u = d / self.bank_half_w_m;
                z = z.max(self.bank_h_m * (1.0 - u * u).max(0.0).sqrt());
            }
        }
        z
    }

    /// The `<hfield>` declaration for
    /// [`MjcfExportOptions::extra_asset_xml`]. No `file=`, so MuJoCo
    /// allocates the grid and the host fills it.
    pub fn asset_xml(&self, name: &str) -> String {
        let (nrow, ncol) = self.grid();
        // The 4th size component is how far the field extends BELOW z=0,
        // i.e. the ring slab itself -- MuJoCo builds that solid for free.
        // A separate box for the plate would duplicate exactly this volume
        // and put two coplanar faces at z=0, which renders as a speckled
        // mess of z-fighting across the whole field.
        format!(
            "    <hfield name=\"{name}\" nrow=\"{nrow}\" ncol=\"{ncol}\" \
             size=\"{} {} {} {}\"/>\n",
            self.ring_m / 2.0,
            self.ring_m / 2.0,
            self.z_top_m(),
            self.plate_thickness_m,
        )
    }

    /// Ring plate, start platforms and the heightfield geom, for
    /// [`MjcfExportOptions::extra_worldbody_xml`].
    ///
    /// The plate is a solid box under the heightfield rather than the
    /// heightfield's own base thickness, so the ring has sides a robot can
    /// fall off -- driving off the edge is part of this field, unlike the
    /// staircase where it was only ever a test-track artifact.
    pub fn worldbody_xml(&self, name: &str) -> String {
        let half = self.ring_m / 2.0;
        let plate_h = self.plate_thickness_m;
        let floor_top = -self.floor_top_z();
        let mut xml = String::new();

        if self.lighting {
            xml += &scene_lighting_worldbody_xml();
        }

        if let Some(side) = self.floor_size_m {
            // Venue floor. A slab, not an infinite plane: the ring is what
            // the robot is meant to stay on, and a floor with a visible edge
            // makes that read at a glance.
            const FLOOR_H: f64 = 0.05;
            xml += &format!(
                "    <geom name=\"ring_floor\" type=\"box\" pos=\"0 0 {}\" size=\"{} {} {}\" rgba=\"0.42 0.44 0.47 1\"/>\n",
                floor_top - FLOOR_H / 2.0,
                side / 2.0,
                side / 2.0,
                FLOOR_H / 2.0,
            );
            // The stand the ring sits on, filling floor-to-underside. Inset
            // slightly so the ring reads as a top plate on a base rather
            // than as one solid block.
            let inset = 0.06;
            let h = -plate_h - floor_top;
            if h > 1e-6 {
                xml += &format!(
                    "    <geom name=\"ring_stand\" type=\"box\" pos=\"0 0 {}\" size=\"{} {} {}\" rgba=\"0.20 0.22 0.28 1\"/>\n",
                    floor_top + h / 2.0,
                    half - inset,
                    half - inset,
                    h / 2.0,
                );
            }
        }

        xml += &format!(
            "    <geom name=\"ring_field\" type=\"hfield\" hfield=\"{name}\" pos=\"0 0 0\" rgba=\"0.36 0.37 0.40 1\"/>\n"
        );

        for (label, (w, d), sx, sy, rgba) in [
            ("start_red", self.red_platform_m, -1.0, -1.0, "0.72 0.12 0.12 1"),
            ("start_blue", self.blue_platform_m, 1.0, 1.0, "0.12 0.20 0.72 1"),
        ] {
            // Butted against the outside of the ring edge, on opposite
            // corners, matching the isometric.
            xml += &format!(
                "    <geom name=\"{label}\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} {}\" rgba=\"{rgba}\"/>\n",
                sx * (half + d / 2.0),
                sy * (self.ring_m / 2.0 - w / 2.0),
                -self.platform_h_m / 2.0,
                d / 2.0,
                w / 2.0,
                self.platform_h_m / 2.0,
            );
        }

        // Coloured border along each start side, so which end is which reads
        // at a glance the way the red/blue frames do in the photographs.
        if let Some((bw, bh)) = self.border {
            for (label, sx, rgba) in
                [("border_red", -1.0_f64, "0.72 0.12 0.12 1"), ("border_blue", 1.0, "0.12 0.20 0.72 1")]
            {
                xml += &format!(
                    "    <geom name=\"{label}\" type=\"box\" pos=\"{} 0 {}\" size=\"{} {half} {}\" rgba=\"{rgba}\"/>\n",
                    sx * (half + bw / 2.0),
                    bh / 2.0 - plate_h,
                    bw / 2.0,
                    (bh + plate_h) / 2.0,
                );
            }
        }

        // Safety barrier: four transparent panels, standing on the venue
        // floor and clear of the start platforms. Collidable -- this is the
        // one part of the surroundings a robot can actually hit.
        if let Some((bh, gap, bt)) = self.barrier {
            let r = half + gap;
            for (label, x, y, sx, sy) in [
                ("barrier_xp", r, 0.0, bt / 2.0, r + bt),
                ("barrier_xn", -r, 0.0, bt / 2.0, r + bt),
                ("barrier_yp", 0.0, r, r + bt, bt / 2.0),
                ("barrier_yn", 0.0, -r, r + bt, bt / 2.0),
            ] {
                xml += &format!(
                    "    <geom name=\"{label}\" type=\"box\" pos=\"{x} {y} {}\" size=\"{sx} {sy} {}\" rgba=\"0.55 0.70 0.80 0.25\"/>\n",
                    floor_top + bh / 2.0,
                    bh / 2.0,
                );
            }
        }

        xml
    }
}


/// A second copy of `model`, joints welded at `pose`, as worldbody XML.
///
/// For the opponent robot: it has to be a physical body that can be tipped
/// over, but it must not move under its own power. So the root keeps its
/// free joint and every hinge is deleted, with the angle it was holding
/// baked into the child body's orientation instead.
///
/// Built by re-exporting `model` and rewriting the result rather than by
/// assembling primitives, so the opponent has the real robot's masses,
/// inertias, collision shapes and meshes -- flipping something over is
/// entirely a question of where its mass is, and a stand-in built out of
/// boxes would answer a different question.
///
/// `pose` is `[hip, thigh, calf]` applied to all four legs plus the arm,
/// matching [`crate::self_righting::STANCE`]'s layout with the arm appended.
/// Mesh assets are NOT re-emitted: the caller's own export already declares
/// them and this XML refers to the same ones.
///
/// # Panics
/// Never; on a shape it does not recognise it returns an empty string and
/// logs, which shows up as an opponent that is simply not there.
pub fn locked_robot_worldbody_xml(
    model: &RobotModel,
    prefix: &str,
    pos: [f64; 3],
    pose: LockedPose,
) -> String {
    let xml = export_mjcf_with_options(
        model,
        MjcfExportOptions {
            base_pos: Some(pos),
            ground_plane: None,
            add_actuators: false,
            ..MjcfExportOptions::default()
        },
    );
    let Some(start) = xml.find("<body name=\"") else {
        log::error!("locked robot: no <body> in the re-exported MJCF");
        return String::new();
    };
    let Some(end) = xml.rfind("</worldbody>") else {
        log::error!("locked robot: no </worldbody> in the re-exported MJCF");
        return String::new();
    };
    let subtree = &xml[start..end];

    let mut out = String::new();
    for line in subtree.lines() {
        let t = line.trim_start();

        // Hinges become welds. The joint sits at the body frame's origin and
        // rotates that frame about its own axis, so welding it at `q` is
        // exactly a body `quat` of that rotation -- the child geoms and the
        // rest of the subtree are expressed in the frame and follow it.
        if t.starts_with("<joint ") {
            continue;
        }
        if t.starts_with("<freejoint") {
            // Kept: the opponent has to be able to fall over.
            out.push_str(line);
            out.push('\n');
            continue;
        }
        if t.starts_with("<body ") {
            let name = attr(t, "name").unwrap_or_default();
            let q = pose.angle_for(&name);
            let mut l = line.to_string();
            if let Some(axis) = joint_axis_for(subtree, &name) {
                if q != 0.0 {
                    let quat = quat_about(axis, q);
                    let ins = l.rfind('>').unwrap_or(l.len());
                    l.insert_str(
                        ins,
                        &format!(
                            " quat=\"{:.9} {:.9} {:.9} {:.9}\"",
                            quat[0], quat[1], quat[2], quat[3]
                        ),
                    );
                }
            }
            l = l.replace(
                &format!("name=\"{name}\""),
                &format!("name=\"{prefix}{name}\""),
            );
            out.push_str(&l);
            out.push('\n');
            continue;
        }
        // Sites carry names too and would collide with the caller's.
        if t.starts_with("<site ") {
            if let Some(name) = attr(t, "name") {
                out.push_str(&line.replace(
                    &format!("name=\"{name}\""),
                    &format!("name=\"{prefix}{name}\""),
                ));
                out.push('\n');
                continue;
            }
        }
        // Tint the visual geoms so the opponent reads as the opponent. Only
        // group 1 -- group 3 is the collision shell, drawn translucent.
        if t.starts_with("<geom ") && t.contains("group=\"1\"") {
            if let Some(rgba) = attr(t, "rgba") {
                out.push_str(&line.replace(
                    &format!("rgba=\"{rgba}\""),
                    "rgba=\"0.85 0.25 0.22 1\"",
                ));
                out.push('\n');
                continue;
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// Joint angles to weld a [`locked_robot_worldbody_xml`] copy at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LockedPose {
    /// `[hip, thigh, calf]`, applied to all four legs.
    pub leg: [f64; 3],
    pub arm: f64,
}

impl Default for LockedPose {
    /// Crouched: lower than the walking stance, which is what an opponent
    /// waiting to be attacked looks like and also what makes it possible to
    /// get an arm underneath.
    fn default() -> Self {
        Self { leg: [0.0, 1.60, -2.55], arm: 0.85 }
    }
}

impl LockedPose {
    fn angle_for(&self, body: &str) -> f64 {
        if body.ends_with("_hip") {
            // Left and right hips mirror, so a common sign in the trunk
            // frame needs opposite signs per side.
            let s = if body.starts_with("FL") || body.starts_with("RL") { 1.0 } else { -1.0 };
            s * self.leg[0]
        } else if body.ends_with("_thigh") {
            self.leg[1]
        } else if body.ends_with("_calf") {
            self.leg[2]
        } else if body == "arm" {
            self.arm
        } else {
            0.0
        }
    }
}

fn attr(line: &str, key: &str) -> Option<String> {
    let pat = format!("{key}=\"");
    let i = line.find(&pat)? + pat.len();
    let j = line[i..].find('"')? + i;
    Some(line[i..j].to_string())
}

/// The axis of the hinge declared inside `<body name="{body}">`, if any.
fn joint_axis_for(subtree: &str, body: &str) -> Option<[f64; 3]> {
    let i = subtree.find(&format!("<body name=\"{body}\""))?;
    // The body's own joint is the first one after its opening tag and before
    // any nested body, which the exporter always emits in that order.
    let rest = &subtree[i..];
    let stop = rest[1..].find("<body ").map(|k| k + 1).unwrap_or(rest.len());
    let head = &rest[..stop];
    let j = head.find("<joint ")?;
    let axis = attr(&head[j..head[j..].find('>')? + j], "axis")?;
    let v: Vec<f64> = axis.split_whitespace().filter_map(|s| s.parse().ok()).collect();
    (v.len() == 3).then(|| [v[0], v[1], v[2]])
}

/// Unit quaternion `[w, x, y, z]` for `angle` about `axis`.
fn quat_about(axis: [f64; 3], angle: f64) -> [f64; 4] {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let (s, c) = ((angle * 0.5).sin(), (angle * 0.5).cos());
    [c, s * axis[0] / n, s * axis[1] / n, s * axis[2] / n]
}
