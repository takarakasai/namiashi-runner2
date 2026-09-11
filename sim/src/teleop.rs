//! Shared keyboard bindings for the interactive namiashi demos.
//!
//! One place, so `examples/namiashi_wbc_teleop.rs` (model-based WBC/MPC,
//! driven through [`crate::wbc_harness::run_wbc_sim`]) and
//! `examples/namiashi_rl_teleop.rs` (the trained RL policy) cannot drift
//! apart -- the whole point of having both demos is that the only thing
//! differing between them is the controller, so the same keys must mean
//! the same thing on each.
//!
//! | keys                      | axis                        |
//! |---------------------------|-----------------------------|
//! | `W`/`S` or `Up`/`Down`    | forward/back speed (vx)     |
//! | `A`/`D` or `Left`/`Right` | turn rate (wz)              |
//! | `Q`/`E` or `PgUp`/`PgDn`  | strafe (vy)                 |
//! | `Shift` (held)            | full speed instead of half  |
//! | `1`/`2`/`3`               | Crawl / Walk / Trot         |
//! | `R`/`F`                   | swing height +/- 5 mm       |
//! | `=`/`-`                   | trunk height +/- 5 mm       |
//! | `B`                       | trunk levelling on/off      |
//! | `N`                       | respawn at the start pose   |
//! | `Shift`+`N`               | respawn upside down         |
//! | `X`                       | tip the opponent over       |
//! | `V`                       | self-right, unhurried       |
//! | `Shift`+`V`               | self-right, fast            |
//!
//! `V` is the unhurried one and gets up in roughly 11 of 27 measured
//! conditions; `Shift`+`V` throws the body over and manages 23.
//! | `Y`/`G` (held)            | arm pitch up / down         |
//! | `K`                       | show / hide this list       |
//! | `O`/`L`                   | ground friction mu +/- 0.05 |
//! | `P`/`.`                   | controller's assumed mu     |
//!
//! Hold-to-move, release-to-stop: a released key means zero on that axis
//! that same frame, not a latched command that keeps running. The two
//! discrete controls (gait, swing height) are edge-triggered instead, so
//! holding the key doesn't repeat.
//!
//! Two of these overlap with `MjViewer`'s own built-in hotkeys, both
//! harmless but worth knowing: `E` also toggles constraint visualisation,
//! and `Q` quits **only** with Ctrl held (plain `Q` is ours). `PgUp`/`PgDn`
//! are the conflict-free way to strafe. The viewer's `X` (hide the side
//! panel) does NOT stop these bindings -- mujoco-rs runs detached UI
//! callbacks outside its `ViewerStatusBit::UI` gate.

use mujoco::viewer::egui;
use quadruped_gait::GaitType;

/// What the viewer's key callback writes and the physics loop reads, once
/// per tick, behind a mutex.
#[derive(Clone, Copy, Debug)]
pub struct LiveTeleop {
    /// Body-frame velocity command `[vx, vy, wz]`.
    pub cmd: [f64; 3],
    /// Requested gait. The WBC/MPC loop applies a change by swapping the
    /// whole `GaitConfig`; the RL demo ignores this (a learned policy has
    /// no gait to switch).
    pub gait: GaitType,
    /// Requested peak swing-foot lift, metres -- `GaitConfig::swing_height_m`.
    /// The same knob `namiashi_staircase_5cm_swing_clearance_sweep` swept
    /// from 0.040 to 0.200 m trying to clear a 5 cm riser, exposed live so
    /// the effect can be felt against the staircase directly.
    ///
    /// Carried across a gait switch rather than snapping back to the new
    /// gait's tuned value: a raised foot is usually raised deliberately
    /// (to clear something), and silently undoing that on a gait change
    /// would be the more surprising behaviour. The tuned values are within
    /// 5 mm of each other anyway (0.035-0.040 m).
    pub swing_height_m: f64,
    /// Trunk height relative to the tuned stance, metres. Positive raises.
    ///
    /// Sign chosen for the driver, not for the internals: `trunk_drop_m` and
    /// `nominal_foot_body.z` both increase to CROUCH, which is the right
    /// convention for leg geometry and the wrong one for a key that says
    /// "up". The conversion happens once, where it is applied.
    pub body_lift_m: f64,
    /// Whether the proprioceptive trunk levelling is engaged
    /// ([`crate::wbc_harness::ProprioStanceCfg`]). A toggle rather than a
    /// build-time choice because its value is most obvious as a difference:
    /// standing astride a step and switching it off tips the trunk by the
    /// step's own geometry, live.
    pub level_enabled: bool,
    /// Arm pitch rate, -1 / 0 / +1. A RATE rather than an angle because the
    /// key callback runs at frame rate and the physics loop knows dt: having
    /// the loop integrate keeps the arm's speed the same whether the display
    /// is managing 60 fps or 6.
    pub arm_rate: f64,
    /// Commanded arm pitch, radians -- integrated by the physics loop from
    /// `arm_rate` and clamped to the joint's own limits, then echoed here
    /// for the HUD. Telemetry, not input.
    pub arm_angle_rad: f64,
    /// Whether the controls list is up. Off by default: the HUD's job is to
    /// show what the robot is DOING, and a permanent key legend crowds that
    /// out for something you need once. The one-line `[K] controls` footer
    /// stays visible so it is still findable without reading the source.
    pub show_help: bool,
    /// Set by the `N` key, cleared by the physics loop once it has acted.
    /// A request rather than a state, because respawning is an edge: leaving
    /// it latched would teleport the robot home every tick.
    pub respawn_requested: bool,
    /// Whether that request is for the upside-down pose (`Shift`+`N`).
    /// Only meaningful while `respawn_requested` is set.
    pub respawn_inverted: bool,
    /// Set by `V`: run the self-righting trajectory. Also an edge, for the
    /// same reason as `respawn_requested`.
    pub recover_requested: bool,
    /// Whether that request is for the fast trajectory (`Shift`+`V`) rather
    /// than the unhurried one. Only meaningful while `recover_requested` is
    /// set.
    pub recover_fast: bool,
    /// Set by `X`: swing the arm under the opponent and tip it over.
    pub attack_requested: bool,
    /// Fire the attack at this sim time, once, and clear. Same reason as
    /// [`Self::recover_at_sim_s`].
    pub attack_at_sim_s: Option<f64>,
    /// Fire the recovery at this sim time, once, and clear.
    ///
    /// Nothing on the keyboard sets this -- it exists so a scripted run can
    /// press `V` at a defined moment. Setting `recover_requested` from
    /// another thread cannot: the physics loop runs thousands of ticks per
    /// wall-clock millisecond, so a request posted "half a second in" lands
    /// at a different sim time on every run, and a test built that way
    /// disagrees with itself about whether the robot got up.
    pub recover_at_sim_s: Option<f64>,
    /// Simulated sliding friction of every geom -- the actual slipperiness
    /// of the world. Applied via `MujocoSim::set_slide_friction_all`.
    pub ground_mu: f64,
    /// The friction the CONTROLLER believes it has: the half-angle of the
    /// WBC QP's friction cone and the MPC's own force-planning limit.
    ///
    /// Deliberately separate from `ground_mu`, because the two were found
    /// to disagree (0.5 assumed against 0.7 simulated, with a stale
    /// comment in wbc_pipeline.rs claiming they matched). Being able to
    /// drive them apart, or together, is the point.
    pub controller_mu: f64,

    // ── Telemetry: written by the physics loop, read by the HUD. These
    // flow the OPPOSITE way from the fields above -- the key callback
    // must never write them, the loop must never read them as input.
    /// Trunk world x, metres. On the staircase this is the progress
    /// readout: risers start at 1.5 m, treads are 0.20 m apart.
    pub body_x_m: f64,
    /// Trunk world z, metres.
    pub body_z_m: f64,
    /// Measured world-frame forward speed, m/s -- what the robot is
    /// actually doing, against the `cmd[0]` it was asked for.
    pub measured_vx_mps: f64,
    /// Simulated time since the run started, seconds.
    pub sim_time_s: f64,
    /// Wall-clock time since the run started, seconds. Shown against
    /// `sim_time_s` as a real-time factor: both demos pace themselves to
    /// real time, so a factor under 1.0 means the physics could not keep
    /// up and everything on screen is running slow.
    pub wall_time_s: f64,
    /// Smoothed render rate, from [`FpsMeter`].
    pub fps: f64,
}

/// Exponentially-smoothed frame-rate meter for the HUD. Lives in the
/// render loop rather than the UI callback so it measures the rate frames
/// are actually produced at, not how often egui happens to run.
#[derive(Default)]
pub struct FpsMeter {
    last: Option<std::time::Instant>,
    fps: f64,
}

impl FpsMeter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Call once per rendered frame; returns the smoothed rate. The first
    /// call has no interval to measure and reports 0.
    pub fn tick(&mut self) -> f64 {
        let now = std::time::Instant::now();
        if let Some(prev) = self.last.replace(now) {
            let dt = now.duration_since(prev).as_secs_f64();
            if dt > 0.0 {
                let instant = 1.0 / dt;
                // ~0.2 s time constant at 60 Hz: settles fast enough to
                // show a stall, slow enough not to flicker.
                self.fps = if self.fps == 0.0 {
                    instant
                } else {
                    self.fps * 0.9 + instant * 0.1
                };
            }
        }
        self.fps
    }
}

impl LiveTeleop {
    /// Stopped, in the given gait, at that gait's tuned swing height.
    pub fn new(gait: GaitType) -> Self {
        Self {
            cmd: [0.0; 3],
            gait,
            swing_height_m: crate::wbc_harness::namiashi_tuned_swing_height_m(gait),
            body_lift_m: 0.0,
            level_enabled: true,
            show_help: false,
            arm_rate: 0.0,
            arm_angle_rad: 0.0,
            respawn_requested: false,
            respawn_inverted: false,
            recover_requested: false,
            recover_fast: false,
            recover_at_sim_s: None,
            attack_requested: false,
            attack_at_sim_s: None,
            // Seeded from the live sim/controller at startup (see
            // run_wbc_sim); these are only a placeholder until then.
            ground_mu: 0.0,
            controller_mu: 0.0,
            body_x_m: 0.0,
            body_z_m: 0.0,
            measured_vx_mps: 0.0,
            sim_time_s: 0.0,
            wall_time_s: 0.0,
            fps: 0.0,
        }
    }
}

/// Per-axis ceiling for a full-deflection (Shift-held) command.
#[derive(Clone, Copy, Debug)]
pub struct SpeedEnvelope {
    pub vx: f64,
    pub vy: f64,
    pub wz: f64,
}

impl SpeedEnvelope {
    /// The tuned envelope for one gait.
    ///
    /// `vx` is that gait's own tuned command from
    /// [`crate::wbc_harness::NAMIASHI_TUNED`] -- NOT a single number shared
    /// across gaits. Each gait's reachable speed is
    /// `max_step_length_m / (cycle_period_s * duty_factor)`, so commanding
    /// Trot's 0.80 m/s in Crawl would just saturate against a ceiling under
    /// a quarter of that and read as a broken controller rather than a slow
    /// gait.
    ///
    /// `vy`/`wz` have no equivalent tuned reference anywhere in this repo,
    /// so they are scaled off the Trot values by that gait's own vx ratio
    /// -- a heuristic that keeps a crawl from being asked to spin at a
    /// trot's yaw rate, not a measured limit.
    pub fn for_gait(gait: GaitType) -> Self {
        const TROT_VX: f64 = 0.800;
        const TROT_VY: f64 = 0.30;
        const TROT_WZ: f64 = 1.00;
        let vx = crate::wbc_harness::namiashi_tuned_cmd_vx(gait);
        let r = vx / TROT_VX;
        Self { vx, vy: TROT_VY * r, wz: TROT_WZ * r }
    }
}

/// Fraction of [`SpeedEnvelope`] used when Shift is *not* held.
const SLOW_FRACTION: f64 = 0.5;

/// Read the current velocity command straight off which keys are down
/// right now. Nothing is integrated or latched: releasing a key zeroes
/// that axis on the next poll, which is what makes this stop-on-release
/// rather than a cruise control.
pub fn poll_cmd(ctx: &egui::Context, env: SpeedEnvelope) -> [f64; 3] {
    ctx.input(|r| {
        let scale = if r.modifiers.shift { 1.0 } else { SLOW_FRACTION };
        let axis = |pos: bool, neg: bool, max: f64| {
            ((pos as i32 - neg as i32) as f64) * max * scale
        };
        [
            axis(
                r.key_down(egui::Key::W) || r.key_down(egui::Key::ArrowUp),
                r.key_down(egui::Key::S) || r.key_down(egui::Key::ArrowDown),
                env.vx,
            ),
            axis(
                r.key_down(egui::Key::Q) || r.key_down(egui::Key::PageUp),
                r.key_down(egui::Key::E) || r.key_down(egui::Key::PageDown),
                env.vy,
            ),
            axis(
                r.key_down(egui::Key::A) || r.key_down(egui::Key::ArrowLeft),
                r.key_down(egui::Key::D) || r.key_down(egui::Key::ArrowRight),
                env.wz,
            ),
        ]
    })
}

/// Was the controls-list toggle pressed this frame? Edge-triggered.
pub fn poll_help_toggle(ctx: &egui::Context) -> bool {
    ctx.input(|r| r.key_pressed(egui::Key::K))
}

/// The bindings, as `(keys, what)`. One list, rendered by the help panel and
/// mirrored by this module's own header table -- so a key that moves has one
/// place to move in.
pub const BINDINGS: &[(&str, &str)] = &[
    ("W / S", "forward, back        (Up / Down too)"),
    ("A / D", "turn                 (Left / Right too)"),
    ("Q / E", "strafe               (PgUp / PgDn too)"),
    ("Shift", "full speed while held (otherwise half)"),
    ("1 / 2 / 3", "gait: Crawl / Walk / Trot"),
    ("R / F", "swing height +/- 5 mm"),
    ("= / -", "trunk height +/- 5 mm"),
    ("B", "trunk levelling on / off"),
    ("Y / G", "arm pitch up / down (held)"),
    ("N", "respawn at the start pose, 10 cm up"),
    ("Shift+N", "respawn upside down"),
    ("X", "arm attack: tip the opponent over"),
    ("V", "self-right, unhurried (~11/27 conditions)"),
    ("Shift+V", "self-right, fast (~23/27)"),
    ("O / L", "ground friction mu"),
    ("P / .", "controller's assumed mu"),
    ("K", "show / hide this list"),
];

/// Arm pitch direction held this frame: +1 up, -1 down, 0 neither.
/// Hold-to-move, since an arm is aimed rather than stepped.
pub fn poll_arm_rate(ctx: &egui::Context) -> f64 {
    ctx.input(|r| {
        (r.key_down(egui::Key::Y) as i32 - r.key_down(egui::Key::G) as i32) as f64
    })
}

/// Radians per second the arm sweeps at full deflection.
pub const ARM_RATE_RAD_S: f64 = 0.8;

/// Was a respawn asked for this frame? Edge-triggered.
///
/// Deliberately not the viewer's own Reset button, which cannot be hooked:
/// it resets the viewer's private copy of the data -- which the physics
/// loop overwrites on the next sync -- to `qpos0`, every hinge straight.
/// For a robot whose spawn height was computed for a bent stance, that is
/// a pose with its feet through the floor.
/// `X`, edge-triggered: start the arm attack.
pub fn poll_attack(ctx: &egui::Context) -> bool {
    ctx.input(|r| r.key_pressed(egui::Key::X))
}

/// `V`, edge-triggered: start the self-righting trajectory. `Some(true)` for
/// the fast one (`Shift` held), `Some(false)` for the unhurried default.
pub fn poll_recover(ctx: &egui::Context) -> Option<bool> {
    ctx.input(|r| r.key_pressed(egui::Key::V).then_some(r.modifiers.shift))
}

pub fn poll_respawn(ctx: &egui::Context) -> Option<bool> {
    ctx.input(|r| {
        r.key_pressed(egui::Key::N).then_some(r.modifiers.shift)
    })
}

/// Was the levelling toggle pressed this frame? Edge-triggered.
pub fn poll_level_toggle(ctx: &egui::Context) -> bool {
    ctx.input(|r| r.key_pressed(egui::Key::B))
}

/// One `=`/`-` press worth of trunk-height change.
pub const BODY_LIFT_STEP_M: f64 = 0.005;

/// Clamp on the live trunk height, metres either side of the tuned stance.
/// The lower bound is what stops a crouch from folding the legs past the
/// nominal the whole gait is built around.
pub const BODY_LIFT_RANGE_M: (f64, f64) = (-0.05, 0.06);

/// Trunk-height change requested this frame, metres. Edge-triggered.
pub fn poll_body_lift_delta(ctx: &egui::Context) -> f64 {
    ctx.input(|r| {
        // Equals rather than Plus: `+` needs Shift, which is already the
        // full-speed modifier, so pressing it would also make the robot run.
        let up = r.key_pressed(egui::Key::Equals);
        let down = r.key_pressed(egui::Key::Minus);
        ((up as i32 - down as i32) as f64) * BODY_LIFT_STEP_M
    })
}

/// One `R`/`F` press worth of swing-height change.
pub const SWING_HEIGHT_STEP_M: f64 = 0.005;

/// Clamp on the live swing height, metres. The upper bound is the top of
/// the range `namiashi_staircase_5cm_swing_clearance_sweep` already
/// established the leg can actually reach; the lower one keeps the foot
/// clearing its own stance compression.
pub const SWING_HEIGHT_RANGE_M: (f64, f64) = (0.010, 0.200);

/// Swing-height change requested this frame, metres (0.0 if neither key
/// was pressed). Edge-triggered, so holding `R` steps once rather than
/// ramping at frame rate.
pub fn poll_swing_height_delta(ctx: &egui::Context) -> f64 {
    ctx.input(|r| {
        let up = r.key_pressed(egui::Key::R);
        let down = r.key_pressed(egui::Key::F);
        ((up as i32 - down as i32) as f64) * SWING_HEIGHT_STEP_M
    })
}

/// One key-press worth of friction change.
pub const FRICTION_STEP: f64 = 0.05;

/// Clamp on both friction coefficients. The floor is well below any real
/// surface (deliberately -- being able to make the world skating-rink
/// slippery is the point of having the control); the ceiling is past
/// rubber-on-dry-concrete.
pub const FRICTION_RANGE: (f64, f64) = (0.05, 1.50);

/// `(d_ground_mu, d_controller_mu)` requested this frame. `O`/`L` move the
/// world's friction, `P`/`.` move what the controller thinks it has.
/// Edge-triggered, like the other discrete controls.
pub fn poll_friction_deltas(ctx: &egui::Context) -> (f64, f64) {
    ctx.input(|r| {
        let step = |up: egui::Key, down: egui::Key| {
            ((r.key_pressed(up) as i32 - r.key_pressed(down) as i32) as f64) * FRICTION_STEP
        };
        (
            step(egui::Key::O, egui::Key::L),
            step(egui::Key::P, egui::Key::Period),
        )
    })
}

/// The gait requested this frame, if any. Edge-triggered (`key_pressed`,
/// not `key_down`) so holding `1` doesn't re-apply Crawl every frame.
pub fn poll_gait(ctx: &egui::Context) -> Option<GaitType> {
    ctx.input(|r| {
        if r.key_pressed(egui::Key::Num1) {
            Some(GaitType::Crawl)
        } else if r.key_pressed(egui::Key::Num2) {
            Some(GaitType::Walk)
        } else if r.key_pressed(egui::Key::Num3) {
            Some(GaitType::Trot)
        } else {
            None
        }
    })
}

/// Always-on overlay of everything the keys control, plus what the robot
/// is actually doing about it. Anchored top-right so it clears
/// `MjViewer`'s own left side panel.
///
/// `gaited` is false for a learned policy: it has no gait schedule and no
/// swing-height parameter, so showing either would be showing a knob that
/// does nothing. Drawn from the shared [`LiveTeleop`] alone -- no MjData
/// needed, so this works from the cheap detached UI callback.
pub fn draw_hud(
    ctx: &egui::Context,
    st: &LiveTeleop,
    env: SpeedEnvelope,
    controller: &str,
    gaited: bool,
) {
    let fast = ctx.input(|r| r.modifiers.shift);
    egui::Window::new("teleop_hud")
        .title_bar(false)
        .resizable(false)
        .movable(false)
        .anchor(egui::Align2::RIGHT_TOP, [-8.0, 8.0])
        .show(ctx, |ui| {
            ui.label(egui::RichText::new(controller).strong());
            ui.separator();

            let row = |ui: &mut egui::Ui, k: &str, v: String| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("{k:<10}")).monospace().weak());
                    ui.label(egui::RichText::new(v).monospace());
                });
            };

            if gaited {
                row(ui, "gait", format!("{:?}", st.gait));
                row(
                    ui,
                    "arm",
                    format!("{:+.2} rad ({:+.0} deg)", st.arm_angle_rad, st.arm_angle_rad.to_degrees()),
                );
                row(
                    ui,
                    "levelling",
                    format!(
                        "{}",
                        if st.level_enabled { "ON  (IMU + encoders)" } else { "off" }
                    ),
                );
                row(
                    ui,
                    "body h",
                    format!("{:+.3} m from tuned", st.body_lift_m),
                );
                row(
                    ui,
                    "swing h",
                    format!(
                        "{:.3} m (tuned {:.3})",
                        st.swing_height_m,
                        crate::wbc_harness::namiashi_tuned_swing_height_m(st.gait),
                    ),
                );
            }
            // Ground friction is physics, not control -- it applies to the
            // learned policy exactly as much as to the WBC.
            row(ui, "ground mu", format!("{:.2}", st.ground_mu));
            if gaited {
                // Flagged when the controller's friction cone disagrees
                // with the floor it is standing on. Assuming LESS than the
                // world provides is merely conservative; assuming MORE
                // means planning forces the ground cannot deliver, so the
                // two directions are called out differently.
                let note = if st.controller_mu > st.ground_mu + 1e-9 {
                    "  OVER-ASSUMED"
                } else if st.controller_mu < st.ground_mu - 1e-9 {
                    "  (conservative)"
                } else {
                    "  = matched"
                };
                row(
                    ui,
                    "ctrl mu",
                    format!("{:.2}{note}", st.controller_mu),
                );
            }
            ui.separator();

            row(
                ui,
                "speed",
                format!(
                    "{}",
                    if fast { "FULL" } else { "half" },
                ),
            );
            row(ui, "vx cmd", format!("{:+.3} / {:.3} m/s", st.cmd[0], env.vx));
            row(ui, "vy cmd", format!("{:+.3} / {:.3} m/s", st.cmd[1], env.vy));
            row(ui, "wz cmd", format!("{:+.3} / {:.3} rad/s", st.cmd[2], env.wz));

            ui.separator();
            row(ui, "vx meas", format!("{:+.3} m/s", st.measured_vx_mps));
            row(ui, "trunk x", format!("{:+.3} m", st.body_x_m));
            row(ui, "trunk z", format!("{:+.3} m", st.body_z_m));

            ui.separator();
            row(ui, "fps", format!("{:.1}", st.fps));
            // Real-time factor, not decoration: both demos sleep to pace
            // themselves to real time, so anything below ~1.0 means the
            // physics is the bottleneck and the run is playing back slow.
            let rt = if st.wall_time_s > 1e-9 {
                st.sim_time_s / st.wall_time_s
            } else {
                0.0
            };
            row(ui, "time", format!("{:.1} s  (rt x{rt:.2})", st.sim_time_s));

            ui.separator();
            ui.label(
                egui::RichText::new(if st.show_help {
                    "[K] hide controls"
                } else {
                    "[K] controls"
                })
                .monospace()
                .weak(),
            );
        });

    if !st.show_help {
        return;
    }
    // Opposite corner from the readout, so the two never fight for the same
    // space on a small window.
    egui::Window::new("teleop_help")
        .title_bar(false)
        .resizable(false)
        .movable(false)
        .anchor(egui::Align2::LEFT_BOTTOM, [8.0, -8.0])
        .show(ctx, |ui| {
            ui.label(egui::RichText::new("controls").strong());
            ui.separator();
            for (keys, what) in BINDINGS {
                // A learned policy has no gait, stance or friction cone to
                // set, so listing those keys there would be listing keys
                // that do nothing.
                if !gaited
                    && matches!(*keys, "1 / 2 / 3" | "R / F" | "= / -" | "B" | "Y / G" | "P / .")
                {
                    continue;
                }
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("{keys:<10}")).monospace());
                    ui.label(egui::RichText::new(*what).monospace().weak());
                });
            }
        });
}
