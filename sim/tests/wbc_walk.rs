//! End-to-end MuJoCo regression for the Hierarchical WBC pipeline.
//!
//! Mirrors [`gait_walk_stability`](./gait_walk_stability.rs)'s harness
//! (URDF → MujocoSim → trot loop) but routes the gait controller's
//! output through [`articara::wbc_pipeline::WbcPipeline`] instead of
//! the per-joint Position-PD path. Asserts on two invariants:
//!
//! 1. **Gravity balance** — at hold (zero command), the sum of
//!    contact-force z-components should approximately equal `m·g`.
//!    The WBC's friction-cone + EoM tasks are responsible for this:
//!    if the QP fails or hands over a wrong-sign GRF, the body would
//!    fall through the floor or float.
//! 2. **Forward motion** — under a steady forward command, the trunk
//!    must translate by at least `MIN_DISPLACEMENT_M` over the run.
//!    Catches the "WBC bobs in place" regression where the
//!    direction-preserving floor in `compute_mpc_footstep` collapses
//!    to zero or the WBC output is dominated by gravity comp.
//!
//! ## Known limitations (documented per-test)
//!
//! - The WBC pipeline keeps the misarta floating base at neutral
//!   orientation each tick (`q[3..7] = identity`). On flat ground
//!   that approximation is fine; large body tilt would skew the
//!   gravity-comp direction. We use mild commands so the body stays
//!   roughly upright.
//! - Friction enforcement happens in the WBC's QP, but the actual
//!   MuJoCo contact solver may still allow micro-slip. Thresholds
//!   are loose enough to absorb that.

#![cfg(feature = "mujoco")]

use std::path::PathBuf;

use articara::gait::{
    auto_detect_centroidal_mpc_config, auto_detect_kinematics_config,
    auto_detect_srbd_mpc_config, GaitController, DEFAULT_FOOT_LINKS,
};
use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions, StaircaseCfg};
use articara::mujoco_sim::MujocoSim;
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;
use articara::wbc_pipeline::WbcPipeline;
use nalgebra::Vector3;
use quadruped_gait::wbc;
use quadruped_gait::{
    solve_leg_ik, ContactDrivenPhase, GaitConfig, GaitMode, GaitType,
    KinematicsConfig, LegIkSolution, VelocityCmd,
};
use namiashi_sim::wbc_harness::*;

/// Compute the total robot mass by summing every link's `mass` field.
/// `m·g` is the reference for the static-balance assertion.
fn robot_mass(robot: &RobotModel) -> f64 {
    robot.links.iter().map(|l| l.inertial.mass).sum()
}

/// Static stand under WBC torque control. The Phase A integration
/// went through several rounds of tightening before this could pass:
///
/// 1. `q[3..7]` quaternion sync from MuJoCo (resolved at adbc9f1).
/// 2. SE(3)-correct `compute_joint_jacobian_time_derivative` so a
///    moving floating base doesn't blow up `J̇·v` (Phase 1.2).
/// 3. Per-task LSQ weights so EoM/no_contact_motion dominate
///    contact_force/τ_grav (Phase 1.4).
/// 4. `a_base_des` derived from the **MPC's predicted GRFs** via
///    SRBD physics, instead of a hand-tuned PD on body velocity
///    (Phase 1.5-A) — this is the single biggest fix; without it
///    the WBC drives the body to a different equilibrium than the
///    MPC chose and the two fight to body collapse.
///
/// The avg Σf_z tolerance is ±60% rather than the ideal ±10% because
/// MuJoCo's contact bouncing produces large per-tick swings even when
/// the body is mean-stable around the target z. The *min_z fall*
/// check catches the real failure mode (body slumping below 0.18 m).
#[test]
fn wbc_static_stand_balances_gravity() {
    let params = WbcParams::static_stand();
    let misa = params.misa_file;
    let Some(samples) = run_wbc_sim(params) else {
        return;
    };
    assert_static_stand_balances_gravity(&samples, misa);
}

/// Same invariants as [`wbc_static_stand_balances_gravity`], but
/// routed through the misa-wbc `Dynamics` path in the `ForceSpace`
/// formulation (GID's decision-variable layout, `x = [τ; f]`) with
/// the `ActiveSet` backend — the fastest combination per the
/// `misa-wbc` formulation benchmarks (see `ref/wbc_comparison.md`).
/// Confirms the equivalence holds under real MuJoCo contact dynamics,
/// not just on synthetic matrices.
#[test]
fn wbc_static_stand_balances_gravity_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let params = WbcParams::static_stand_misa_wbc(wbc::Formulation::ForceSpace, cfg);
    let misa = params.misa_file;
    let Some(samples) = run_wbc_sim(params) else {
        return;
    };
    assert_static_stand_balances_gravity(&samples, misa);
}

/// NOTE: takes the model path because the m*g reference has to come from the
/// same robot the samples came from. Before `WbcParams::misa_file` existed
/// there was only one model and this read a hardcoded `namiashi.misa`; once a
/// 3.3 kg variant could be passed in, that silently compared 32.4 N of
/// measured contact force against a 23.5 N reference -- a 38% error sitting
/// inside a +/-60% tolerance, so it would have passed while being wrong.
fn assert_static_stand_balances_gravity(samples: &[WbcSample], misa_file: &str) {
    // No fall.
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "static stand: trunk fell, min_z = {min_z:.3} m (threshold {:.2})",
        TRUNK_Z_FALL_THRESHOLD_M,
    );

    // Burn-in window done; sample the last 0.5 s for the f_z average.
    // Window derived from the samples, not from hardcoded run lengths: these
    // used to be literal 1.5/0.5 s constants that happened to match
    // `WbcParams::static_stand()` and would have silently sliced the wrong
    // window the moment anyone changed it.
    let t_end = samples[samples.len() - 1].t;
    let start = samples
        .iter()
        .position(|s| s.t >= t_end - STATIC_AVG_WINDOW_S)
        .unwrap_or(0);
    let avg_fz: f64 = samples[start..]
        .iter()
        .map(|s| s.total_fz_world)
        .sum::<f64>()
        / (samples.len() - start) as f64;

    // Reference m·g. Recompute by reloading the .misa (cheap).
    let path = namiashi_misa_named(misa_file);
    let robot = RobotModel::from_misa(&path).unwrap();
    let mg = robot_mass(&robot) * 9.81;

    let pct_err = ((avg_fz - mg) / mg).abs();
    eprintln!(
        "[wbc] static_stand: avg Σf_z = {avg_fz:.2} N, m·g = {mg:.2} N \
         (err = {:.1}%)",
        pct_err * 100.0,
    );
    // ±60% tolerance — generous because:
    //   - per-tick MuJoCo contact force has large bouncing components
    //     (the integrator's substeps see sharp normal-force spikes
    //     around touchdown which the 0.5 s sample window doesn't fully
    //     average out),
    //   - friction-cone clipping in the QP can re-allocate force
    //     among feet, making instantaneous totals oscillate,
    //   - the WBC's static-stand z drifts within a few cm because
    //     there's no explicit z-position regulator (the SRBD MPC's
    //     z weight handles it but its 30 ms horizon discretisation
    //     leaves residual oscillation).
    // The accompanying min_z assertion (above) is the real fall
    // detector; this avg-Σf_z test mostly guards against pathological
    // gravity-direction errors.
    assert!(
        pct_err < 0.60,
        "static stand: Σf_z = {avg_fz:.2} N deviates from m·g = {mg:.2} N \
         by {:.1}% — friction cone + EoM may be inconsistent",
        pct_err * 100.0,
    );
}

/// Forward walk under WBC + Position-PD hybrid joint command.
/// Asserts the trunk advances at least `MIN_DISPLACEMENT_M` over the
/// run without falling (`min_z > TRUNK_Z_FALL_THRESHOLD_M`).
///
/// The test sequence — MPC-driven `a_base_des` (Phase 1.5-A), the
/// joint-space swing_leg task (G2, sharing q* with Position-PD), and
/// `W_CONTACT_FORCE = 5` (H4 tuning, lets the WBC track MPC's
/// predicted GRF tightly enough that stance forward thrust flows
/// into joint torque) — together produce ~17 cm of forward
/// displacement under a 0.15 m/s command over 2.5 s.
#[test]
fn wbc_forward_command_advances_body() {
    let Some(samples) = run_wbc_sim(WbcParams::forward_walk()) else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// Same invariants as [`wbc_forward_command_advances_body`], routed
/// through misa-wbc's `ForceSpace` + `ActiveSet` — see
/// [`wbc_static_stand_balances_gravity_force_space_active_set`] for
/// the rationale.
#[test]
fn wbc_forward_command_advances_body_force_space_active_set() {
    let cfg = wbc::SolveConfig { backend: wbc::QpSolver::ActiveSet, ..Default::default() };
    let Some(samples) =
        run_wbc_sim(WbcParams::forward_walk_misa_wbc(wbc::Formulation::ForceSpace, cfg))
    else {
        return;
    };
    assert_forward_command_advances_body(&samples);
}

/// One line per run: speed, attitude, support pattern.
///
/// The original assertions here checked trunk height and net displacement.
/// Neither can tell a gait that is walking from one that is shuffling in
/// place, or a robot standing on three legs from one standing on four --
/// both of which turned out to be happening on the Go2 side and were only
/// found by measuring contact directly.
/// What a run is judged on. Body-frame throughout -- see the note in
/// `report_walk_cmd` for why the run-start frame is not trustworthy.
#[derive(Debug, Default, Clone, Copy)]
struct WalkMetrics {
    /// Forward speed in the instantaneous heading frame, m/s.
    body_vx: f64,
    /// Sideways speed in the instantaneous heading frame, m/s.
    body_vy: f64,
    /// deg/s, unwrapped.
    yaw_rate_deg_s: f64,
    z_min: f64,
}

fn report_walk_cmd(
    label: &str,
    samples: &[WbcSample],
    cmd_vx: f64,
    cmd_vy: f64,
    cmd_wz: f64,
    burn_in_s: f64,
) -> WalkMetrics {
    let t0 = samples[0].t + burn_in_s;
    let walk: Vec<&WbcSample> = samples.iter().filter(|s| s.t >= t0).collect();
    if walk.len() < 10 {
        eprintln!("=== {label}: too few samples ===");
        return WalkMetrics::default();
    }
    let n = walk.len() as f64;
    let (a, b) = (walk[0], walk[walk.len() - 1]);
    let span = b.t - a.t;

    // Displacement projected on the heading the robot had when the command
    // arrived, and its normal. Reported for the path shape only -- do NOT
    // read `lat` as sideways motion. If the robot yaws while walking
    // straight ahead, forward travel leaks into this frame's lateral axis:
    // at Trot's -15 deg of yaw drift, 0.89 m/s forward shows up here as
    // 0.89*sin(15 deg) = 0.23 m/s of "crab" that is not happening. Body-frame
    // vx/vy below is the honest measure.
    let (c0, s0) = ((-a.yaw).cos(), (-a.yaw).sin());
    let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
    let fwd = c0 * dx - s0 * dy;
    let lat = s0 * dx + c0 * dy;
    // Accumulate wrapped per-sample increments rather than differencing the
    // endpoints. Endpoint differencing wraps at +/-180 deg, so it cannot see
    // more than half a turn: a clean 0.76-gain response to wz=0.90 rad/s over
    // 10 s is 392 deg of real rotation and reads back as +32 deg, which looks
    // like the turn collapsing to 7% when nothing collapsed.
    let mut dyaw = 0.0;
    for w in walk.windows(2) {
        let mut d = w[1].yaw - w[0].yaw;
        while d > std::f64::consts::PI {
            d -= 2.0 * std::f64::consts::PI;
        }
        while d < -std::f64::consts::PI {
            d += 2.0 * std::f64::consts::PI;
        }
        dyaw += d;
    }

    let z_mean = walk.iter().map(|s| s.body_z).sum::<f64>() / n;
    let z_min = walk.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    let pitch_pk = walk.iter().map(|s| s.pitch.abs()).fold(0.0_f64, f64::max);
    let roll_pk = walk.iter().map(|s| s.roll.abs()).fold(0.0_f64, f64::max);

    // Support census at a 1 N threshold -- namiashi is 2.4 kg, so its
    // per-foot loads are roughly a sixth of Go2's and the 5 N threshold used
    // there would read a loaded foot as airborne.
    let mut hist = [0usize; 5];
    let mut duty = [0usize; 4];
    for s in walk.iter() {
        let mut n_down = 0;
        for fi in 0..4 {
            if s.foot_fz[fi] > 1.0 {
                n_down += 1;
                duty[fi] += 1;
            }
        }
        hist[n_down] += 1;
    }
    let f = |k: usize| hist[k] as f64 / n;

    eprintln!(
        "=== {label} (cmd_vx={cmd_vx:.3}) ===\n\
         fwd={fwd:+.3}m lat={lat:+.3}m over {span:.1}s  speed={:+.3} m/s  \
         track={:.0}%  yaw_drift={:+.1}deg\n\
         trunk z: mean={z_mean:.3} min={z_min:.3}m   peak roll={:.1}deg pitch={:.1}deg\n\
         support@1N: n_down 0/1/2/3/4 = {:.3}/{:.3}/{:.3}/{:.3}/{:.3}   \
         duty per foot = {:.2} {:.2} {:.2} {:.2}",
        fwd / span,
        if cmd_vx.abs() > 1e-9 { 100.0 * (fwd / span) / cmd_vx } else { 0.0 },
        dyaw.to_degrees(),
        roll_pk.to_degrees(), pitch_pk.to_degrees(),
        f(0), f(1), f(2), f(3), f(4),
        duty[0] as f64 / n, duty[1] as f64 / n,
        duty[2] as f64 / n, duty[3] as f64 / n,
    );

    // Per-second velocity in each window's OWN heading frame, plus that
    // window's yaw rate. Rotating by the yaw at the start of the window
    // instead of the start of the run is the whole point: it separates "the
    // robot is sliding sideways" from "the robot is pointing somewhere else
    // by now", which the run-start frame silently mixes together.
    // Round the averaging window up to a whole number of gait cycles. A fixed
    // 1.0 s window beats against the gait: 1.0/0.400 = 2.5 and 1.0/0.800 =
    // 1.25, so every Walk window boundary lands on one of two gait phases and
    // every Crawl boundary on one of four. Averaging many such windows does
    // not average the phase out, it averages two or four clusters.
    let period = walk[0].cycle_period_s;
    let win_s = period * (1.0 / period).ceil();
    // Torque headroom. `mujoco_sim` clamps to `joint.effort` without a word,
    // so a gait can look fine here while asking the hardware for torque it
    // does not have. Reported per joint role because the roles have different
    // limits (hip and thigh 1.5 N*m, calf 2.205) and very different jobs.
    const ROLE: [&str; 3] = ["hip", "thigh", "calf"];
    // Net fore-aft ground force, the thing that actually accelerates the
    // robot. Averaged over the window and separately over stance ticks, since
    // a mean over the whole cycle hides which phase supplies it.
    let fx_mean: f64 =
        walk.iter().map(|s| s.foot_fx.iter().sum::<f64>()).sum::<f64>() / n;
    let fx_pos: f64 = walk
        .iter()
        .map(|s| s.foot_fx.iter().filter(|v| **v > 0.0).sum::<f64>())
        .sum::<f64>()
        / n;
    let fx_neg: f64 = walk
        .iter()
        .map(|s| s.foot_fx.iter().filter(|v| **v < 0.0).sum::<f64>())
        .sum::<f64>()
        / n;
    let mpc_fx: f64 = walk.iter().map(|s| s.mpc_fx).sum::<f64>() / n;
    let wbc_fx: f64 = walk.iter().map(|s| s.wbc_fx).sum::<f64>() / n;
    let mpc_fz: f64 = walk.iter().map(|s| s.mpc_fz).sum::<f64>() / n;
    eprintln!(
        "  fore-aft N: mpc plan={mpc_fx:+.3}  wbc solve={wbc_fx:+.3}  \
         ground={fx_mean:+.3}  (push {fx_pos:+.3} / brake {fx_neg:+.3})   \
         mpc fz={mpc_fz:+.2} (mg=32.4)"
    );

    let mut role_line = String::from("  tau/limit:");
    for j in 0..3 {
        let mut peak = 0.0f64;
        let mut sat = 0usize;
        for s in walk.iter() {
            for leg in 0..4 {
                let f = s.tau_frac[leg * 3 + j];
                peak = peak.max(f);
                if f > 0.99 {
                    sat += 1;
                }
            }
        }
        let mut qd_peak = 0.0f64;
        let mut qd_over = 0usize;
        for s in walk.iter() {
            for leg in 0..4 {
                let f = s.qd_frac[leg * 3 + j];
                qd_peak = qd_peak.max(f);
                if f > 1.0 {
                    qd_over += 1;
                }
            }
        }
        // Split by gait phase. Both numbers are a fraction of the ticks that
        // joint spent in that phase, not of the whole run, so a gait with a
        // short swing does not look better than it is.
        let (mut st_sat, mut st_n, mut sw_sat, mut sw_n) = (0usize, 0usize, 0usize, 0usize);
        for s in walk.iter() {
            for leg in 0..4 {
                let hit = s.tau_frac[leg * 3 + j] > 0.99;
                if s.stance_mask[leg] {
                    st_n += 1;
                    st_sat += hit as usize;
                } else {
                    sw_n += 1;
                    sw_sat += hit as usize;
                }
            }
        }
        let pct = |a: usize, b: usize| if b > 0 { 100.0 * a as f64 / b as f64 } else { 0.0 };
        // Time above the continuous rating, which the peak clamp cannot show.
        let rated = NAMIASHI_RATED_TORQUE_NM
            * if j == 2 { 14.0 / 9.0 } else { 1.0 };
        let mut over_rated = 0usize;
        let mut nm_peak = 0.0f64;
        for s in walk.iter() {
            for leg in 0..4 {
                let t = s.tau_nm[leg * 3 + j];
                nm_peak = nm_peak.max(t);
                if t > rated {
                    over_rated += 1;
                }
            }
        }
        role_line += &format!(
            "  {}: pk={nm_peak:.2}Nm clamp={:.1}% (st {:.1}/sw {:.1}) \
             over-rated={:.1}% | qd pk={qd_peak:.2}",
            ROLE[j],
            100.0 * sat as f64 / (4.0 * n),
            pct(st_sat, st_n),
            pct(sw_sat, sw_n),
            100.0 * over_rated as f64 / (4.0 * n),
        );
        let _ = peak;
    }
    eprintln!("{role_line}");

    let mut line = format!("  per-{win_s:.2}s vx/vy/wz:");
    let (mut vx_sum, mut vy_sum, mut nw) = (0.0, 0.0, 0usize);
    let mut w0 = 0usize;
    while w0 < walk.len() {
        let w1 = walk
            .iter()
            .position(|s| s.t >= walk[w0].t + win_s)
            .unwrap_or(walk.len() - 1);
        if w1 <= w0 {
            break;
        }
        let (p, q) = (walk[w0], walk[w1]);
        let dt = q.t - p.t;
        let (cw, sw) = ((-p.yaw).cos(), (-p.yaw).sin());
        let (ddx, ddy) = (q.body_x - p.body_x, q.body_y - p.body_y);
        let (fx, fy) = ((cw * ddx - sw * ddy) / dt, (sw * ddx + cw * ddy) / dt);
        let mut dy = q.yaw - p.yaw;
        while dy > std::f64::consts::PI {
            dy -= 2.0 * std::f64::consts::PI;
        }
        while dy < -std::f64::consts::PI {
            dy += 2.0 * std::f64::consts::PI;
        }
        line += &format!("  {fx:+.2}/{fy:+.2}/{:+.1}", dy.to_degrees() / dt);
        vx_sum += fx;
        vy_sum += fy;
        nw += 1;
        w0 = w1;
    }
    eprintln!("{line}");
    let mut metrics = WalkMetrics {
        z_min,
        yaw_rate_deg_s: dyaw.to_degrees() / span,
        ..WalkMetrics::default()
    };
    if nw > 0 {
        let (bvx, bvy) = (vx_sum / nw as f64, vy_sum / nw as f64);
        metrics.body_vx = bvx;
        metrics.body_vy = bvy;
        eprintln!(
            "  body-frame: vx={bvx:+.3} m/s ({:.0}% of cmd)  vy={bvy:+.3} m/s  \
             yaw rate={:+.2} deg/s",
            if cmd_vx.abs() > 1e-9 { 100.0 * bvx / cmd_vx } else { 0.0 },
            dyaw.to_degrees() / span,
        );
        if cmd_vy.abs() > 1e-9 || cmd_wz.abs() > 1e-9 {
            eprintln!(
                "  cmd vy={cmd_vy:+.2} -> {bvy:+.3} m/s ({:.0}%)   \
                 cmd wz={:+.1} -> {:+.1} deg/s ({:.0}%)",
                if cmd_vy.abs() > 1e-9 { 100.0 * bvy / cmd_vy } else { 0.0 },
                cmd_wz.to_degrees(),
                dyaw.to_degrees() / span,
                if cmd_wz.abs() > 1e-9 {
                    100.0 * (dyaw / span) / cmd_wz
                } else {
                    0.0
                },
            );
        }
    }
    metrics
}

fn report_walk(
    label: &str,
    samples: &[WbcSample],
    cmd_vx: f64,
    burn_in_s: f64,
) -> WalkMetrics {
    report_walk_cmd(label, samples, cmd_vx, 0.0, 0.0, burn_in_s)
}

/// WHERE namiashi ACTUALLY IS (2026-08-02).
///
/// This file had four tests, all on Trot, all asking only "did the trunk stay
/// up and did x increase" over 3 s at 0.15 m/s. Nothing here had ever run
/// Walk, Crawl or Pace, and nothing measured speed, heading or contact.
///
/// This is the baseline sweep before any tuning: each gait at its own default
/// period and duty, over a range of commands, reporting what actually
/// happens. Failures are expected and are the point -- they say which gait
/// needs what.
///
/// SCALE. namiashi is 2.400 kg with a 0.306 m leg; Go2 is 15.606 kg with
/// 0.426 m. Under Froude similarity (T ~ sqrt(L/g), stride ~ L) Go2's tuned
/// 0.18 s / 0.20 m map to 0.152 s / 0.143 m here, and its 1.63 m/s to
/// 1.38 m/s. The commands below bracket that, but the gait defaults are left
/// alone in this first pass so the starting point is the library's own.
#[test]
#[ignore = "exploratory sweep -- run with --ignored"]
fn namiashi_gait_baseline_sweep() {
    for gait in [GaitType::Trot, GaitType::Walk, GaitType::Crawl] {
        for cmd_vx in [0.15, 0.30, 0.50] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} default"), &samples, cmd_vx, 1.0);
        }
    }
}

/// SCALING THE THREE GAITS TO namiashi'S GEOMETRY.
///
/// The baseline sweep says all three gaits are step-length starved, not
/// broken. Each one sits at (or below) its own geometric ceiling
///
///     v_max = max_step / (T * duty)
///
/// and those ceilings are 0.50 / 0.18 / 0.04 m/s for the library defaults --
/// so Walk saturating at 0.17 m/s under a 0.50 m/s command is arithmetic, not
/// a controller failure.
///
/// Normalised by leg length (0.306 m) the default steps are 33% / 26% / 20%.
/// The Go2 Bound work settled on 0.20 m over a 0.426 m leg = 47%, and that
/// robot walks. So the defaults here are conservative by roughly a factor of
/// two across the board.
///
/// This sweep raises max_step toward that ratio and shortens the period
/// (Froude: T ~ sqrt(L/g), so namiashi's equivalent of Go2's 0.18 s is
/// 0.152 s), asking each gait for a speed its geometry can actually deliver.
/// Two settings per gait: "reach" moves the ceiling up mostly via step
/// length, "quick" mostly via period. Which one holds tells us whether the
/// limit is swing reach or swing time.
#[test]
#[ignore = "exploratory sweep -- run with --ignored"]
fn namiashi_gait_scaled_sweep() {
    // (gait, label, T, duty, max_step, cmd)
    // cmd is set to ~80% of that row's ceiling so the command is inside what
    // the geometry allows -- commanding past the ceiling only measures the
    // ceiling again.
    let rows: &[(GaitType, &str, f64, f64, f64, f64)] = &[
        // Trot: default already reaches 0.5. Push on both axes.
        (GaitType::Trot, "reach", 0.400, 0.50, 0.145, 0.58),
        (GaitType::Trot, "quick", 0.260, 0.50, 0.100, 0.61),
        (GaitType::Trot, "both", 0.260, 0.50, 0.145, 0.89),
        // Walk: duty 0.75 costs a lot of ceiling, so it needs the most step.
        (GaitType::Walk, "reach", 0.600, 0.75, 0.145, 0.25),
        (GaitType::Walk, "quick", 0.400, 0.75, 0.100, 0.26),
        (GaitType::Walk, "both", 0.400, 0.75, 0.145, 0.38),
        // Crawl: duty 0.85 and a 1.67 s period leave almost nothing.
        (GaitType::Crawl, "reach", 1.667, 0.85, 0.145, 0.08),
        (GaitType::Crawl, "quick", 0.800, 0.85, 0.060, 0.07),
        (GaitType::Crawl, "both", 0.800, 0.85, 0.145, 0.17),
    ];
    for &(gait, tag, t, duty, step, cmd_vx) in rows {
        let params = WbcParams {
            total_time_s: 6.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(gait),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
    }
}

/// TWO CANDIDATE CAUSES FOR WHAT THE SCALED SWEEP LEFT BROKEN.
///
/// After scaling, Trot and Walk overshoot their command by 3-16% and Trot
/// crabs sideways (+0.89 m in 5 s at one setting, -0.70 m at another -- the
/// sign flips, so it is not a fixed bias). Crawl instead *under*shoots, at
/// 66-74%.
///
/// H1 (both symptoms): `DEFAULT_CAPTURE_POINT_GAIN_S = 0.05`. That gain is
/// meant to be the LIP model's sqrt(h/g); for a 0.30 m trunk that is
/// 0.175 s, so the library ships a value 3.5x too small, for every robot.
/// The footstep feedback is pure proportional (`k_capture_pulse` and
/// `v_capture_deadband` both default to 0), and proportional feedback that
/// weak leaves exactly this: a standing error in x, and a lateral velocity
/// nothing pulls back to zero.
///
/// H2 (Crawl only): `GaitConfig::crawl()` sets `swing_height_m = 0.005`.
/// Five millimetres of clearance is defensible over the default 0.06 m step;
/// over the 0.145 m step the scaled sweep uses, the swing foot is dragging.
///
/// H1 predicts the sweep over k moves both the overshoot and the drift. H2
/// predicts swing height moves Crawl and nothing else.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_capture_gain_and_swing_height() {
    // H1: gain sweep on the two gaits that overshoot.
    for (gait, t, duty, step, cmd_vx) in [
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.89),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.38),
    ] {
        for k in [0.05, 0.10, 0.175, 0.25] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} k={k:.3}"), &samples, cmd_vx, 1.0);
        }
    }

    // H2: swing clearance on Crawl, at both the old and the LIP gain, so a
    // clearance effect cannot be confused with a gain effect.
    for k in [0.05, 0.175] {
        for h in [0.005, 0.020, 0.040] {
            let params = WbcParams {
                total_time_s: 6.0,
                burn_in_s: 1.0,
                cmd_vx: 0.17,
                gait_type: Some(GaitType::Crawl),
                cycle_period_s: Some(0.800),
                duty_factor: Some(0.85),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(h),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Crawl k={k:.3} h={h:.3}"), &samples, 0.17, 1.0);
        }
    }
}

/// H1 WAS BACKWARDS: THE FOOTSTEP FEEDBACK IS ALREADY AT ITS LIMIT.
///
/// The prediction was that `k_capture = 0.05` is too weak (LIP sqrt(h/g) is
/// 0.175 for this trunk) and that raising it would pull in both the 16%
/// speed overshoot and the lateral crab. Raising it did the opposite: Trot
/// at k=0.175 makes 2% of its command and yaws 43 deg; Walk at k=0.175
/// drifts 2.07 m sideways. Every increase made both symptoms worse, and the
/// drift *flipped sign* between k=0.05 and k=0.10.
///
/// A sign flip under a gain change is a closed-loop property, not a plant
/// bias -- so the lateral drift is the footstep feedback going unstable, and
/// 0.05 is already near the edge rather than 3.5x below where it belongs.
/// (The sqrt(h/g) reasoning assumes the foothold takes effect within one
/// step; here it is filtered through the MPC's own horizon, which adds lag
/// the LIP formula does not know about.)
///
/// So sweep the other way, down to and including k=0 -- pure open-loop
/// Raibert. If the drift is feedback-driven, k=0 has the least of it.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_capture_gain_low_side() {
    for (gait, t, duty, step, cmd_vx) in [
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.89),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.38),
    ] {
        for k in [0.0, 0.015, 0.030, 0.050] {
            let params = WbcParams {
                total_time_s: 8.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} k={k:.3}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// DOES THE LATERAL MODE SATURATE, OR DOES IT DIVERGE?
///
/// The low-side sweep settled two things. Speed tracking is best with the
/// capture-point feedback OFF (98-100% at k=0, 120% at k=0.05) -- the
/// overshoot was the feedback, not the plan. And Trot's lateral velocity
/// grows monotonically (0.00 -> -0.13 m/s over 7 s) *even at k=0*, so the
/// crab is not a feedback artefact either; it is in the gait.
///
/// -0.02 m/s^2 over seven seconds is far too slow to call from a 6 s run.
/// A mode that saturates at some small lateral rate is a robot that walks
/// slightly sideways -- annoying, correctable later by a heading loop. A
/// mode that keeps growing is a robot that eventually falls over, and that
/// has to be fixed before any of this goes on hardware.
///
/// So: 25 s, the same duration the Go2 Bound runs were judged on, on each of
/// the three gaits at its best-so-far setting. This is also the first real
/// endurance test any of them has had -- the four original tests ran 3 s.
#[test]
#[ignore = "long run -- run with --ignored"]
fn namiashi_gait_endurance() {
    let rows: &[(GaitType, &str, f64, f64, f64, f64, f64, f64)] = &[
        // gait, tag, T, duty, step, swing_h, k, cmd
        (GaitType::Trot, "k0", 0.260, 0.50, 0.145, 0.040, 0.0, 0.89),
        (GaitType::Trot, "k.03", 0.260, 0.50, 0.145, 0.040, 0.030, 0.89),
        (GaitType::Walk, "k0", 0.400, 0.75, 0.145, 0.035, 0.0, 0.38),
        (GaitType::Crawl, "h.04", 0.800, 0.85, 0.145, 0.040, 0.050, 0.17),
    ];
    for &(gait, tag, t, duty, step, h, k, cmd_vx) in rows {
        let params = WbcParams {
            total_time_s: 26.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(gait),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(h),
            k_capture_s: Some(k),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} {tag} 25s"), &samples, cmd_vx, 1.0);
    }
}

/// CRAWL'S YAW DRIFT IS THE ONE REAL DEFECT LEFT.
///
/// Measured in each window's own heading frame, all three gaits track
/// forward speed to 101-103% and slide sideways by 2-19 mm/s. What looked
/// like a lateral instability was the run-start reporting frame; what looked
/// like Crawl decaying was Crawl turning.
///
/// What survives that correction is yaw: -0.60 deg/s on Trot, +0.41 on Walk,
/// and +2.34 on Crawl -- four to six times the others, 58 deg over 25 s.
/// Nothing commands a turn, so this is the wz=0 reference not being held.
///
/// Trot's real lateral drift went away at k=0 (-0.145 m/s at k=0.03 -> +0.002
/// at k=0), so the same question is worth asking of Crawl's yaw.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_crawl_yaw_drift() {
    for k in [0.0, 0.025, 0.050] {
        let params = WbcParams {
            total_time_s: 26.0,
            burn_in_s: 1.0,
            cmd_vx: 0.17,
            gait_type: Some(GaitType::Crawl),
            cycle_period_s: Some(0.800),
            duty_factor: Some(0.85),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(k),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Crawl k={k:.3}"), &samples, 0.17, 1.0);
    }
}

/// CAN THESE GAITS DO ANYTHING BUT WALK FORWARD?
///
/// Every run so far has commanded (vx>0, 0, 0). That is the one case the
/// four original tests covered too, so nothing in this file has ever asked
/// namiashi to back up, strafe or turn -- and a gait that only goes forward
/// is not a controller.
///
/// Each gait is asked for all four at ~half its geometric ceiling
/// `max_step/(T*duty)`, since a reversing or strafing foothold is drawn from
/// the same step-length budget as a forward one.
#[test]
#[ignore = "coverage sweep -- run with --ignored"]
fn namiashi_command_coverage() {
    // gait, T, duty, step, swing_h, k, ceiling
    let gaits: &[(GaitType, f64, f64, f64, f64, f64, f64)] = &[
        (GaitType::Trot, 0.260, 0.50, 0.145, 0.040, 0.0, 1.115),
        (GaitType::Walk, 0.400, 0.75, 0.145, 0.035, 0.0, 0.483),
        (GaitType::Crawl, 0.800, 0.85, 0.145, 0.040, 0.0, 0.213),
    ];
    for &(gait, t, duty, step, h, k, ceil) in gaits {
        let v = 0.5 * ceil;
        let cases: [(&str, f64, f64, f64); 4] = [
            ("fwd", v, 0.0, 0.0),
            ("back", -v, 0.0, 0.0),
            ("strafe", 0.0, v, 0.0),
            ("turn", 0.0, 0.0, 0.35),
        ];
        for (tag, vx, vy, wz) in cases {
            let params = WbcParams {
                total_time_s: 11.0,
                burn_in_s: 1.0,
                cmd_vx: vx,
                cmd_vy: vy,
                cmd_wz: wz,
                gait_type: Some(gait),
                cycle_period_s: Some(t),
                duty_factor: Some(duty),
                max_step_length_m: Some(step),
                swing_height_m: Some(h),
                k_capture_s: Some(k),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk_cmd(&format!("{gait:?} {tag}"), &samples, vx, vy, wz, 1.0);
        }
    }
}

/// TURN RATE IS THE ONE THING THAT DOES NOT TRACK.
///
/// Command coverage came back clean on translation -- forward, backward and
/// sideways all land within 98-104% on all three gaits, and all twelve runs
/// keep the trunk between 0.293 and 0.296 m. Turning does not: 0.35 rad/s
/// (20.1 deg/s) commanded produces 15.2 / 16.3 / 15.7 deg/s on
/// Trot / Walk / Crawl -- 76-81%, and near enough the same deficit on three
/// gaits whose duty factors are 0.50, 0.75 and 0.85.
///
/// A shortfall that ignores duty that completely is not in the footstep
/// plan. (The rotational part of the plan is tiny anyway: at 0.35 rad/s and
/// a ~0.10 m hip offset the yaw contribution to the half-stride is ~2 mm,
/// two orders below the 0.145 m clamp, so nothing is being truncated.)
///
/// Which of two things it is decides whether it matters:
///   - a constant gain (~0.78) -- an outer heading loop absorbs it, and any
///     real robot has one;
///   - saturation -- the ratio falls as wz rises, and there is a turn rate
///     above which namiashi simply cannot comply.
/// Only a sweep over wz separates them.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_turn_rate_linearity() {
    for wz in [0.10, 0.20, 0.35, 0.60, 0.90] {
        let params = WbcParams {
            total_time_s: 11.0,
            burn_in_s: 1.0,
            cmd_vx: 0.0,
            cmd_wz: wz,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(0.260),
            duty_factor: Some(0.50),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk_cmd(&format!("Trot wz={wz:.2}"), &samples, 0.0, 0.0, wz, 1.0);
    }
}

// NAMIASHI_TUNED / namiashi_tuned_params moved to src/wbc_harness.rs (pub,
// re-imported by the glob above) once the interactive teleop demo needed
// the same per-gait tuning to switch between Crawl/Walk/Trot at runtime --
// two copies of these numbers would be two things to keep in sync.

/// REGRESSION: Trot, Walk and Crawl each hold for 25 s.
///
/// What the tuning came down to, in order of how much it mattered:
///
/// 1. **The step lengths were half what the geometry allows.** Every gait
///    was pinned to its own ceiling `v_max = max_step/(T*duty)` -- 0.50,
///    0.18 and 0.04 m/s for the library defaults. Walk answering a 0.50 m/s
///    command with 0.17 m/s was arithmetic, not a controller failure.
///    Normalised by the 0.306 m leg the defaults are 33/26/20%; 0.145 m
///    (47%) is the ratio the Go2 work settled on, and it holds here.
///
/// 2. **`k_capture` had to come down hard.** The library default of 0.05 s was
///    the single cause of three separate symptoms: forward speed overshooting
///    by up to 20%, Trot sliding sideways at 0.145 m/s, and Crawl yawing at
///    2.34 deg/s. All three vanish at k=0, where the open-loop Raibert plan
///    alone tracks to 101-103%. The initial guess was the opposite -- that
///    0.05 was 3.5x *below* the LIP value sqrt(h/g)=0.175 -- and raising it
///    made everything worse (Trot at k=0.175 makes 2% of its command and
///    yaws 43 deg). The sqrt(h/g) formula assumes the foothold acts within
///    one step; here it is filtered through the MPC horizon, which adds lag
///    the formula does not model.
///
///    It went to exactly zero for a long time, and that turned out to be one
///    step too far -- see [`NAMIASHI_CAPTURE_GAIN_S`]. Zero was chosen with
///    nothing pushing the robot, which is the one condition under which
///    footstep feedback cannot be judged.
///
/// 3. **Crawl's swing clearance was 5 mm.** Fine over its default 0.06 m
///    step, a scuff over 0.145 m: at fixed gain, 0.005 -> 0.020 m took Crawl
///    from 74% to 94% of command, and 0.040 m to 104%.
///
/// Not fixed, and deliberately so: turning tracks at a flat 76-77% of
/// command from 0.10 to 0.90 rad/s (see `namiashi_turn_rate_linearity`).
/// It is a constant gain with no saturation in range, which any outer
/// heading loop absorbs -- unlike the three items above, it does not stop
/// the gait from working.
#[test]
#[ignore = "25 s per gait -- run with --ignored"]
fn namiashi_tuned_gaits_hold() {
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let Some(samples) = run_wbc_sim(namiashi_tuned_params(i)) else {
            return;
        };
        let m = report_walk(&format!("{gait:?} tuned"), &samples, cmd_vx, 1.0);

        assert!(
            m.z_min > TRUNK_Z_FALL_THRESHOLD_M,
            "{gait:?}: trunk fell to {:.3} m",
            m.z_min
        );
        let track = m.body_vx / cmd_vx;
        assert!(
            (0.90..=1.10).contains(&track),
            "{gait:?}: tracked {:.0}% of {cmd_vx:.2} m/s (got {:.3})",
            100.0 * track,
            m.body_vx
        );
        // Body frame, so this is real sideways sliding and not the robot
        // having turned -- the run-start frame reported 0.23 m/s of "crab"
        // for Trot that was entirely -15 deg of yaw drift leaking in.
        assert!(
            m.body_vy.abs() < 0.05,
            "{gait:?}: slid sideways at {:.3} m/s",
            m.body_vy
        );
        // Nothing commands a turn. 1.5 deg/s is loose enough for the
        // 0.29-0.66 deg/s these settings produce and tight enough to catch
        // the 2.34 deg/s that k_capture=0.05 caused on Crawl.
        assert!(
            m.yaw_rate_deg_s.abs() < 1.5,
            "{gait:?}: yawed at {:.2} deg/s with wz=0",
            m.yaw_rate_deg_s
        );
    }
}

/// THE TUNING WAS DONE ON A ROBOT THAT WEIGHS 0.9 kg LESS THAN THE REAL ONE.
///
/// `namiashi.misa` came from a CAD export totalling 2.400 kg, 36% of it in
/// the legs. The built robot is 3.3 kg with 600 g per leg, so the legs are
/// 2.4 kg and the body is 0.9 kg: 73% of the machine is leg. That is not a
/// scale factor, it is an inversion of where the mass lives, and it lands on
/// the two assumptions this controller rests on.
///
/// The SRBD MPC treats the legs as massless -- all mass in one rigid trunk,
/// contact forces the only thing acting on it. At 36% leg that is a stretch;
/// at 73% the swinging legs carry more momentum than the body they are
/// supposed to be steering, and every swing reacts on the trunk directly.
/// The Raibert foothold plan has the same blind spot: `v * stance/2` says
/// where to put a massless foot, and says nothing about the impulse of
/// throwing 600 g forward and stopping it.
///
/// Three models, same tuned settings, so the comparison is only about mass:
///   - `namiashi.misa`      2.400 kg, 36% leg -- what the tuning was done on
///   - `namiashi_3p3_hip`   3.300 kg, 73% leg, added mass at the hip
///   - `namiashi_3p3_prop`  3.300 kg, 73% leg, added mass spread down the leg
///
/// hip-vs-prop is the same total and the same leg fraction, differing only in
/// how far out the mass sits. If the controller cares about mass at all, it
/// cares through leg inertia, and those two bracket it.
#[test]
#[ignore = "9 x 25 s runs -- run with --ignored"]
fn namiashi_mass_variants() {
    for model in [
        "namiashi.misa",
        "namiashi_3p3_hip.misa",
        "namiashi_3p3_prop.misa",
    ] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                misa_file: model,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let tag = model.trim_end_matches(".misa");
            report_walk(&format!("{gait:?} @{tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// WHY prop BREAKS WALK, AND WHETHER SWING TIME BUYS IT BACK.
///
/// Correcting the mass to 3.3 kg costs nothing on its own: with the extra
/// mass at the hip, all three gaits track 100-101% and the yaw drift
/// actually halves. Spreading the same extra mass down the leg is what
/// hurts, and the two models differ by only 74 g per leg (thigh 20.7 -> 57.3
/// g, calf 21.3 -> 58.9 g). Walk falls to 82% of command and its three-foot
/// support collapses from 70.7% to 14.2% -- it stops being a walk and starts
/// running on two supports at an effective duty of 0.53 against a commanded
/// 0.75.
///
/// The ranking points at swing time. Required swing speed at each gait's
/// tuned command -- ground travel per stance, over the swing window -- is
/// Walk 1.14 m/s, Crawl 0.97, Trot 0.89, and they broke in exactly that
/// order. Walk has the shortest swing window of the three (0.400 s * 0.25 =
/// 0.100 s) despite being the slowest of the two fast gaits, because duty
/// 0.75 spends the cycle on the ground. A leg with three times the distal
/// mass has to be accelerated and stopped inside that window, and if it
/// arrives late the foot touches down late, which is exactly the low
/// measured duty.
///
/// If that is the mechanism, buying swing time fixes it, and the two ways to
/// buy it are a longer period and a lower duty. Both cost something: the
/// ceiling `max_step/(T*duty)` moves with each, so each row is checked
/// against its own ceiling rather than a fixed command.
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_walk_swing_time() {
    // T, duty -- swing window is T*(1-duty)
    let rows: &[(f64, f64)] = &[
        (0.400, 0.75), // the current tuning: 0.100 s of swing
        (0.500, 0.75), // 0.125 s, via period
        (0.600, 0.75), // 0.150 s, via period
        (0.400, 0.65), // 0.140 s, via duty
        (0.400, 0.55), // 0.180 s, via duty -- close to a trot
        (0.500, 0.65), // 0.175 s, both
    ];
    for &(t, duty) in rows {
        let step = 0.145;
        let ceil = step / (t * duty);
        let cmd_vx = 0.80 * ceil;
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Walk),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(0.035),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(
            &format!("Walk T={t:.3} d={duty:.2} swing={:.3}s", t * (1.0 - duty)),
            &samples,
            cmd_vx,
            1.0,
        );
    }
}

/// SWING TIME WAS THE WRONG VARIABLE, AND THE SWEEP THAT SAID SO WAS RIGGED.
///
/// `namiashi_prop_walk_swing_time` set each row's command to 80% of that
/// row's ceiling, so the command moved with the parameters. The two rows
/// that worked were the two slowest commands (0.309 and 0.258 m/s); every
/// row at 0.357 m/s or above failed, whatever its swing window. The row with
/// the most swing time of all (T=0.400, duty 0.55, 0.180 s) failed at
/// 0.527 m/s while a row with less (T=0.500, duty 0.75, 0.125 s) succeeded
/// at 0.309. That is a speed threshold wearing a parameter's clothes.
///
/// Two sweeps that do not confound:
///   A. hold T and duty at the tuning, walk the command up -- where does the
///      three-foot support actually go away?
///   B. hold the command at 0.38 m/s, vary T and duty among settings whose
///      ceiling clears it -- does any of them survive a speed that the
///      tuned setting cannot?
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_walk_speed_vs_shape() {
    let base = |t: f64, duty: f64, cmd_vx: f64| WbcParams {
        misa_file: "namiashi_3p3_prop.misa",
        total_time_s: 16.0,
        burn_in_s: 1.0,
        cmd_vx,
        gait_type: Some(GaitType::Walk),
        cycle_period_s: Some(t),
        duty_factor: Some(duty),
        max_step_length_m: Some(0.145),
        swing_height_m: Some(0.035),
        k_capture_s: Some(0.0),
        ..WbcParams::forward_walk()
    };

    eprintln!("---- A: speed sweep at the tuned shape (T=0.400, duty=0.75) ----");
    for cmd_vx in [0.20, 0.26, 0.31, 0.34, 0.38, 0.44] {
        let Some(samples) = run_wbc_sim(base(0.400, 0.75, cmd_vx)) else { return };
        report_walk(&format!("Walk v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- B: shape sweep at a fixed 0.38 m/s ----");
    // Every row's ceiling 0.145/(T*duty) clears 0.38.
    for &(t, duty) in &[
        (0.400, 0.75),
        (0.500, 0.75),
        (0.400, 0.65),
        (0.500, 0.65),
        (0.400, 0.55),
        (0.300, 0.75),
    ] {
        let Some(samples) = run_wbc_sim(base(t, duty, 0.38)) else { return };
        report_walk(&format!("Walk T={t:.3} d={duty:.2}"), &samples, 0.38, 1.0);
    }
}

/// IS THE BAD BAND REAL, AND IS THE PHASE OVERRIDE WHAT DIGS IT?
///
/// The speed sweep found a hole, not a limit: on the prop model at T=0.400 /
/// duty 0.75, Walk tracks 103/101/97/96% at 0.20-0.34 m/s, drops to 82% at
/// 0.38, and is back to 94% at 0.44. Every setting that failed anywhere
/// failed into the same state -- effective duty ~0.53 with 81-87% of the
/// time on two feet. Duty 0.53 on two supports is a trot. Walk is not
/// degrading, it is being captured by a different gait.
///
/// The only thing in this loop that can rewrite the gait pattern at runtime
/// is `ContactDrivenPhase`, which flips a leg to stance the moment its foot
/// reads above 5 N. That threshold was chosen for a 2.4 kg robot whose whole
/// leg weighed 217 g. A 600 g leg with 74 g more of it out at the thigh and
/// calf lands harder, so contacts that never reached 5 N now do -- and each
/// spurious flip shortens that leg's stance, which is the low measured duty.
///
/// Two questions, one sweep: is the band narrow (fine speed steps), and does
/// raising the override threshold out of reach close it (5 N vs 15 N vs off)?
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_prop_walk_contact_override() {
    for thr in [5.0, 15.0, 1.0e9] {
        let label = if thr > 1.0e6 {
            "off".to_string()
        } else {
            format!("{thr:.0}N")
        };
        eprintln!("---- early-contact override = {label} ----");
        for cmd_vx in [0.34, 0.36, 0.38, 0.40, 0.42, 0.44] {
            let params = WbcParams {
                misa_file: "namiashi_3p3_prop.misa",
                total_time_s: 16.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(GaitType::Walk),
                cycle_period_s: Some(0.400),
                duty_factor: Some(0.75),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(0.035),
                k_capture_s: Some(0.0),
                early_contact_n: thr,
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Walk v={cmd_vx:.2} thr={label}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// NOT THE PHASE OVERRIDE EITHER -- AND IT IS A BAND, NOT A HOLE.
///
/// `ContactDrivenPhase` was the only thing in the loop that can rewrite the
/// gait pattern at runtime, so the guess was that heavier legs trip its 5 N
/// early-touchdown threshold spuriously. Disabling it entirely changes
/// nothing: at 0.38 m/s the prop model tracks 82% with the override at 5 N,
/// 83% at 15 N and 83% with it off. Every speed matches to within 1-3 points
/// across all three settings. The override is not involved.
///
/// The finer sweep also corrected the shape of the failure. It is not a hole
/// at 0.38 -- it is a band: 96% at 0.34, 95/88/89% at 0.36 (the edge),
/// 79-83% from 0.38 to 0.42, and back to 94-95% at 0.44. Roughly 0.37 to
/// 0.43 m/s, about 0.06 m/s wide, with clean walking on both sides.
///
/// Which leaves the question that actually matters for the design, since two
/// readings of the evidence so far are still alive:
///   - distal leg mass CREATES a band that the light model does not have; or
///   - a band is always there and the parameters only move it, and the 2.4 kg
///     tuning happened to sit beside it.
/// Those imply opposite fixes -- redistribute mass in hardware, versus pick a
/// period. Sweeping the same speeds on the hip model (same total mass, same
/// leg fraction, mass held proximal) and on prop at the period that escaped
/// (T=0.500) separates them.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_walk_band_is_mass_or_tuning() {
    let cases: &[(&str, &str, f64)] = &[
        ("namiashi_3p3_hip.misa", "hip T=0.400", 0.400),
        ("namiashi_3p3_prop.misa", "prop T=0.500", 0.500),
        ("namiashi.misa", "2.4kg T=0.400", 0.400),
    ];
    for &(model, label, t) in cases {
        eprintln!("---- {label} ----");
        for cmd_vx in [0.34, 0.36, 0.38, 0.40, 0.42, 0.44] {
            let params = WbcParams {
                misa_file: model,
                total_time_s: 16.0,
                burn_in_s: 1.0,
                cmd_vx,
                gait_type: Some(GaitType::Walk),
                cycle_period_s: Some(t),
                duty_factor: Some(0.75),
                max_step_length_m: Some(0.145),
                swing_height_m: Some(0.035),
                k_capture_s: Some(0.0),
                ..WbcParams::forward_walk()
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{label} v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// RE-TUNE TROT AND CRAWL ON THE CORRECTED MASS.
///
/// The band belongs to distal leg mass: at T=0.400 the 2.4 kg model tracks
/// 101% flat from 0.34 to 0.44 m/s and the hip model 99-100%, while prop
/// drops to 79-82% across 0.38-0.42. Moving prop to T=0.500 removes it --
/// what looks like decay there (99 -> 87% as speed rises) is just the
/// ceiling: 0.145/(0.500*0.75) = 0.387 m/s, and 0.387/0.44 = 88% against 87%
/// measured, with three-foot support holding at 0.67.
///
/// So Walk moves to T=0.500 and a command safely under 0.387. Trot needs
/// checking too: on prop at the old tuning it overshot to 111% and was
/// airborne 2.6% of the time with 4.5 deg of roll, which the 2.4 kg model
/// never did. Crawl was 102% and untouched, but gets swept anyway rather
/// than assumed.
#[test]
#[ignore = "re-tune sweep -- run with --ignored"]
fn namiashi_prop_retune_trot_crawl() {
    eprintln!("---- Trot on prop: speed at T=0.260, and a longer period ----");
    for &(t, cmd_vx) in &[
        (0.260, 0.70),
        (0.260, 0.80),
        (0.260, 0.89),
        (0.320, 0.70),
        (0.320, 0.80),
        (0.320, 0.89),
    ] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(t),
            duty_factor: Some(0.50),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot T={t:.3} v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- Crawl on prop ----");
    for cmd_vx in [0.13, 0.17, 0.20] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Crawl),
            cycle_period_s: Some(0.800),
            duty_factor: Some(0.85),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.040),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Crawl v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }
}

/// IS THE DYNAMIC LAYER DOING ANYTHING?
///
/// An architecture review of this stack argued that the measured locomotion
/// is produced by the joint position-PD tracking IK targets, and that the MPC
/// and WBC contribute almost nothing. Its evidence is checkable and checks
/// out:
///
///   - `WbcPipeline::new` hardcodes `mass_kg: 9.0` and an inertia diagonal
///     for a 9 kg machine (`articara/src/wbc_pipeline.rs:250`), and this
///     harness never overrides either. namiashi is 3.3 kg. The `base_accel`
///     task carries the largest soft weight in the QP.
///   - The MPC's horizon contact schedule is `self.cfg.duty_factor > 0.5`
///     for every node past the first (`mpc_controller.rs:600`). Trot's duty
///     is exactly 0.50, so that is false, and the MPC plans all four legs
///     airborne for nine of its ten nodes. Walk and Crawl plan all four in
///     stance for all nine. Neither resembles the gait being walked.
///   - `wbc_pipeline.rs:515` is `let _ = (v_cmd_body, wz_cmd, omega_obs_world);`
///     and every attitude PD gain defaults to (0.0, 0.0).
///   - During stance `Footstep::stance_at` sweeps the foot from
///     `nominal + half` to `nominal - half` over the stance window, so under
///     no-slip `v_body = 2*half/T_st` identically. With open-loop Raibert
///     that is `v_cmd` exactly -- which is what 101-103% tracking and an
///     exactly-obeyed geometric ceiling look like.
///
/// The claim is falsifiable in one run: zero the WBC's torque feedforward and
/// leave the position-PD alone. If the gait is unchanged, every number in
/// this file describes the PD and IK layer, and the tuning conclusions are
/// statements about kinematics.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_is_the_dynamic_layer_load_bearing() {
    for kinematic_only in [false, true] {
        let tag = if kinematic_only { "PD only" } else { "WBC on" };
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                kinematic_only,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// THE BAND WAS AN ARTEFACT OF AN UNDER-MODELLED KNEE.
///
/// This test was written before the knee's 9:14 reduction was referred
/// through the model. The source URDF raised the calf's `effort` to 2.205
/// against the others' 1.5 -- a ratio of 1.470 where 14/9 is 1.5556 -- and
/// left `velocity` and `armature` at the same values as every other joint.
/// A reduction does all three: torque x14/9, speed x9/14, and reflected
/// rotor inertia x(14/9)^2. So the knee was carrying 2.4x too little
/// inertia.
///
/// With that corrected the speed dependence disappears. Walk on prop at
/// T=0.400 now holds three-foot support of only 0.16-0.21 at *every* speed
/// from 0.30 to 0.48 m/s, where before it held 0.61-0.64 below the band and
/// 0.65 above it. There is no band because there is no good region: T=0.400
/// simply cannot walk this leg. The tuned setting had already moved to
/// T=0.500, which holds 0.79.
///
/// The speed limit is not what binds, either -- knee velocity peaks at
/// 0.63-0.70 of its (now correct) 21.5 rad/s rating and never exceeds it.
/// The knee is torque-limited, and referring the inertia properly is what
/// made that visible: calf saturation went from 0-1% to 9-11% on every gait
/// despite the torque limit going *up* by 5.8%.
///
/// Original question, kept because the answer still stands:
///
/// DOES TORQUE SATURATION EXPLAIN THE WALK BAND?
///
/// `mujoco_sim` clamps every joint to its `effort` limit silently
/// (`src/mujoco_sim.rs:1121`), so a saturated actuator has been invisible in
/// every run in this file. Measuring it changes the picture: on the tuned
/// settings the thigh joint is clamped 22.5% of the time on Trot with the
/// corrected mass, 11.1% even on the original 2.4 kg model, and every joint
/// role reaches its limit at some point on every gait. namiashi's hip and
/// thigh are rated 1.5 N*m and its calf 2.205.
///
/// That is a candidate mechanism for the Walk band that neither swing time
/// nor the contact override could account for. If it is the cause, the
/// saturation fraction should spike inside 0.38-0.42 and fall away on both
/// sides, tracking the failure rather than rising monotonically with speed.
/// If it rises smoothly through the band, saturation is a separate (and
/// separately serious) problem and the band is still unexplained.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_walk_band_torque_saturation() {
    for cmd_vx in [0.30, 0.34, 0.38, 0.40, 0.42, 0.44, 0.48] {
        let params = WbcParams {
            misa_file: "namiashi_3p3_prop.misa",
            total_time_s: 16.0,
            burn_in_s: 1.0,
            cmd_vx,
            gait_type: Some(GaitType::Walk),
            cycle_period_s: Some(0.400),
            duty_factor: Some(0.75),
            max_step_length_m: Some(0.145),
            swing_height_m: Some(0.035),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk v={cmd_vx:.2}"), &samples, cmd_vx, 1.0);
    }
}

/// STEP 1 OF MAKING THE DYNAMIC LAYER REAL: GIVE THE WBC THE RIGHT ROBOT.
///
/// `namiashi_is_the_dynamic_layer_load_bearing` established that zeroing the
/// WBC's torque feedforward changes nothing -- Trot 106 -> 102%, Walk
/// 100 -> 101%, Crawl 101 -> 100%, and Walk's three-foot support actually
/// improves. That is a controller that is not controlling.
///
/// The most likely reason is that it is solving for the wrong machine.
/// `WbcPipeline::new` hardcodes `mass_kg: 9.0` and `inertia_diag_body:
/// (0.07, 0.26, 0.242)` -- a 9 kg robot -- and this harness has never
/// overridden either. namiashi is 3.3 kg. The `base_accel` task those feed
/// carries the largest soft weight in the QP, so at a static stand where the
/// MPC hands over roughly m*g, the WBC reads that as 32.4/9.0 - 9.81 =
/// -6.2 m/s^2 and spends its largest weight asking the trunk to accelerate
/// downward at 0.63 g, continuously.
///
/// One variable at a time: mass, CoM offset and composite inertia from the
/// same source the centroidal MPC config already uses, nothing else touched.
/// Then the same falsification test, to see whether the dynamic layer has
/// become load-bearing or merely better informed.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_wbc_real_inertia() {
    for real in [false, true] {
        for kinematic_only in [false, true] {
            let tag = match (real, kinematic_only) {
                (false, false) => "9kg WBC on",
                (false, true) => "9kg PD only",
                (true, false) => "real WBC on",
                (true, true) => "real PD only",
            };
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    wbc_real_inertia: real,
                    kinematic_only,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
            }
        }
    }
}

/// STEP 3: STOP THE QP ANSWERING A THREE-CONTACT PLAN WITH TWO CONTACTS.
///
/// With the horizon schedule and the WBC's inertia both corrected, the
/// dynamic layer finally helps Trot -- peak roll 3.8 deg with its torque
/// zeroed against 2.0 deg with it on. Walk went the other way: three-foot
/// support 0.842 zeroed against 0.608 applied, and thigh saturation doubled.
/// Walk is duty 0.75, so three feet are down by construction; a controller
/// that costs three-foot support on that gait is doing something specific.
///
/// The friction pyramid constrains stance feet to push and not pull, and
/// nothing more. With three or four contacts the GRF allocation is redundant
/// and a two-contact solution satisfies every constraint in the QP -- the
/// only thing arguing against it is a low-weight regulariser toward the
/// MPC's plan. 0.608 is what choosing that vertex looks like from outside.
///
/// So demand a floor. Expressed as a fraction of the static per-foot share
/// `m*g/4` so it means the same on every model. It is a hard constraint at
/// priority 0: too large and the touchdown transient, where the commanded
/// contact set and the physical one disagree for a tick, becomes infeasible
/// rather than merely wrong. The sweep is there to find where that starts.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_min_stance_force() {
    for frac in [0.0, 0.05, 0.10, 0.20, 0.35] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                wbc_real_inertia: true,
                f_min_stance_frac: frac,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} fmin={frac:.2}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// STEP 4: IS THE BASE-ACCEL REFERENCE WHAT COSTS WALK ITS SUPPORT?
///
/// The minimum-normal-force floor did nothing. Walk's three-foot support sits
/// at 0.606-0.610 whether the floor is 0% or 35% of the static per-foot
/// share, so the QP was never picking a two-contact vertex -- it was already
/// loading every stance foot, and the constraint never binds. The support is
/// being lost in the physics, not in the solution.
///
/// Which moves the suspicion up one level, to what the torque is being asked
/// to achieve. `base_accel` carries the MPC's predicted base acceleration and
/// is the largest soft weight in the QP at 200, against 1 for swing-leg and 5
/// for contact-force and tau-gravity. If that reference is wrong for Walk,
/// the WBC spends most of its authority chasing it and unloads a foot doing
/// so.
///
/// Sweeping the weight to zero turns the WBC into gravity compensation plus
/// the low-weight terms, without disabling it the way `kinematic_only` does.
/// If Walk's support climbs back toward the 0.842 it reaches with the torque
/// zeroed, the reference is the problem. If it stays at 0.61, the reference
/// is innocent and something in the priority-0 block is responsible.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_base_accel_weight() {
    for w in [200.0, 50.0, 10.0, 0.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                wbc_real_inertia: true,
                base_accel_weight: Some(w),
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} w={w:.0}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// STEP 5: THE FLAGS THE WBC IS GIVEN DISAGREE WITH EACH OTHER.
///
/// Two suspects down. A minimum normal force on stance feet moves Walk's
/// three-foot support from 0.608 to 0.610 across 0-35% of the static share,
/// so the QP was never picking a two-contact vertex. Dropping `base_accel`
/// from its default weight of 200 to zero moves it to 0.626, so the MPC's
/// predicted base acceleration is not what the torque is being spent on
/// either. Neither comes near the 0.842 the same gait reaches with the WBC's
/// torque zeroed outright.
///
/// That leaves priority 0, and there is a specific inconsistency there.
/// `ContactDrivenPhase::apply_correction` is called with a late-liftoff
/// threshold of 0, so it is monotone: it can add stance legs, never remove
/// them. When a foot touches down early, `contact_flag[i]` flips to true
/// while `gait_out.legs[i].phase.is_stance` stays false. That leg then gets,
/// simultaneously:
///
///   - `no_contact_motion`, a priority-0 hard equality gated on
///     `contact_flag`, nailing the foot in place;
///   - the swing-tracking cost, gated on the *nominal* phase, still asking
///     it to follow an advancing swing arc;
///   - the joint position-PD at kp=100, still driving q* along that arc.
///
/// Priority 0 wins. The foot is pinned while two other terms drag it, which
/// unloads it -- and an unloaded foot is exactly what a support census reads
/// as airborne.
///
/// This needs no code change to test: raising the early-touchdown threshold
/// out of reach removes the correction, and with it the disagreement.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_contact_flag_consistency() {
    for (thr, tag) in [(5.0, "5N"), (1.0e9, "off")] {
        for kinematic_only in [false, true] {
            let k = if kinematic_only { "PD only" } else { "WBC on" };
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    wbc_real_inertia: true,
                    early_contact_n: thr,
                    kinematic_only,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(&format!("{gait:?} {tag} {k}"), &samples, cmd_vx, 1.0);
            }
        }
    }
}

/// STEP 6: THE WBC IS UNLOADING THE FRONT FEET SPECIFICALLY.
///
/// Removing `ContactDrivenPhase` moved Walk's three-foot support from 0.608
/// to 0.585 -- the wrong direction, so the flag disagreement is not it
/// either. But the per-foot duty in that run says what is happening:
///
///     Walk, WBC torque on   0.54 0.53 | 0.79 0.78
///     Walk, WBC torque off  0.68 0.66 | 0.79 0.79
///
/// The rear pair is identical to two decimal places. Only the front pair
/// drops. The WBC is not degrading the gait, it is shifting load rearward.
///
/// Of the terms that can distribute force front-to-rear, `base_accel` is
/// already ruled out (weight 200 -> 0 moved support by 0.018). `contact_force`
/// is the other one: at weight 5 it pulls the QP's own GRF solution toward
/// the MPC's prediction, and the MPC computes its moment arms from the body
/// root while the WBC -- now that `centroidal_inertia_body` is set -- computes
/// its predicted acceleration from the CoM, 15.9 mm below the root on this
/// model. If the MPC's allocation is front-light, the WBC is faithfully
/// reproducing it.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_contact_force_weight() {
    for w in [5.0, 1.0, 0.0] {
        let params = WbcParams {
            wbc_real_inertia: true,
            contact_force_weight: Some(w),
            total_time_s: 16.0,
            ..namiashi_tuned_params(1)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk cf={w:.0}"), &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
    // Both off: what is left is gravity comp plus the swing-leg term.
    let params = WbcParams {
        wbc_real_inertia: true,
        contact_force_weight: Some(0.0),
        base_accel_weight: Some(0.0),
        total_time_s: 16.0,
        ..namiashi_tuned_params(1)
    };
    if let Some(samples) = run_wbc_sim(params) {
        report_walk("Walk cf=0 ba=0", &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
}

/// STEP 7: WHICH THIRD OF THE "REAL INERTIA" CHANGE COSTS WALK ITS FRONT FEET?
///
/// Correcting the MPC's own inertia -- it was using the heaviest link's
/// tensor, 12 to 24 times under the composite -- moved Walk's three-foot
/// support from 0.725 to 0.745 with the WBC still on its 9 kg placeholder,
/// and its front duty from 0.54/0.53 back to 0.63/0.59. But with the WBC
/// corrected too it sits at 0.592. Something in the WBC-side correction is
/// undoing it.
///
/// `wbc_real_inertia` is three changes at once, and they are not equivalent:
///   - mass 9.0 -> 3.3 kg scales `a_base_des` by 2.7 across the board;
///   - the CoM offset moves where moments are taken about;
///   - setting `centroidal_inertia_body` also switches the pipeline from
///     `predicted_base_accel_world` to its centroidal variant, which is a
///     different function, not just a different constant.
///
/// Bundling them is how the earlier one-at-a-time sweeps kept missing:
/// `base_accel` and `contact_force` turned out to be jointly responsible
/// (0.608 with both on, 0.655 and 0.626 with either alone, 0.819 with both
/// off) because they carry the same MPC prediction down two paths. Same
/// discipline applies here.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_wbc_inertia_decomposition() {
    let cases: &[(&str, bool, bool, bool)] = &[
        ("none", false, false, false),
        ("mass", true, false, false),
        ("com", false, true, false),
        ("inertia", false, false, true),
        ("all", true, true, true),
    ];
    for &(tag, m, c, i) in cases {
        let params = WbcParams {
            wbc_real_mass_only: m,
            wbc_real_com_only: c,
            wbc_real_inertia_only: i,
            total_time_s: 16.0,
            ..namiashi_tuned_params(1)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Walk {tag}"), &samples, NAMIASHI_TUNED[1].5, 1.0);
    }
}

/// VIDEO SOURCE: the three tuned gaits, and the four commands.
///
/// Writes a replay trace per run under `NAMIASHI_REPLAY_OUT/<label>/`, which
/// `render_namiashi.py` turns into an MP4. Nothing here asserts -- the
/// assertions live in `namiashi_tuned_gaits_hold`; this exists so the video
/// and the regression are demonstrably the same configuration, taken from
/// `NAMIASHI_TUNED` rather than restated.
///
/// Run as:
///   NAMIASHI_REPLAY_OUT=/tmp/namiashi_replay cargo test --release \
///     --no-default-features --features mujoco --test wbc_walk \
///     namiashi_video_source -- --ignored --nocapture
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };

    // Part 1: each tuned gait walking forward.
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let label = format!("{gait:?}").to_lowercase();
        let params = WbcParams {
            total_time_s: 13.0,
            replay_dir: Some(format!("{root}/{label}")),
            ..namiashi_tuned_params(i)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} video"), &samples, cmd_vx, 1.0);
    }

    // Part 2: Trot answering all four commands, so the turn is visible. The
    // yaw moment arm fix took turning from 76% of command to 100%, and that
    // is the one result a still number does not convey.
    let (_, t, duty, step, h, fwd) = NAMIASHI_TUNED[0];
    for (tag, vx, vy, wz) in [
        ("cmd_fwd", fwd, 0.0, 0.0),
        ("cmd_back", -fwd, 0.0, 0.0),
        ("cmd_strafe", 0.0, 0.45, 0.0),
        ("cmd_turn", 0.0, 0.0, 0.60),
    ] {
        let params = WbcParams {
            replay_dir: Some(format!("{root}/{tag}")),
            total_time_s: 11.0,
            burn_in_s: 1.0,
            cmd_vx: vx,
            cmd_vy: vy,
            cmd_wz: wz,
            gait_type: Some(GaitType::Trot),
            cycle_period_s: Some(t),
            duty_factor: Some(duty),
            max_step_length_m: Some(step),
            swing_height_m: Some(h),
            k_capture_s: Some(0.0),
            ..WbcParams::forward_walk()
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk_cmd(&format!("Trot {tag}"), &samples, vx, vy, wz, 1.0);
    }
}

/// HOW LOW CAN namiashi STAND AND STILL WALK?
///
/// Every result so far is at one stance height, the 0.295 m the harness has
/// always used. Crouching is not a free parameter: it rotates the leg
/// Jacobian, and the same foot force then needs a different split between
/// thigh and knee. On this robot that matters more than usual, because the
/// thigh is already the joint that saturates -- 24% of the time on Trot
/// against a 1.5 N*m limit.
///
/// It also eats the step-length budget from the other end. `max_step_length_m`
/// is 0.145 m against a 0.306 m leg, and a crouched leg has less horizontal
/// reach before it runs out of extension, so a drop that leaves the gait
/// geometrically fine can still leave it kinematically infeasible.
///
/// Four heights, three gaits, tuned settings otherwise unchanged. `drop` here
/// is absolute, measured from the harness's original ~0.295 m stance -- it
/// overrides `NAMIASHI_STANCE_DROP_M` rather than adding to it, so the 0 cm
/// row is the old baseline and the 6 cm row is the current one.
#[test]
#[ignore = "12 runs -- run with --ignored"]
fn namiashi_trunk_height_sweep() {
    for drop in [0.0, 0.02, 0.04, 0.06] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                trunk_drop_m: drop,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("{gait:?} drop={:.0}cm", drop * 100.0),
                &samples,
                cmd_vx,
                1.0,
            );
        }
    }
}

/// VIDEO SOURCE: the same gait at four stance heights.
///
/// Records all three tuned gaits at 0 / 2 / 4 / 6 cm of crouch for
/// `render_namiashi_height.py` to show as a 2x2 grid. Same settings as
/// `NAMIASHI_TUNED` otherwise, so the comparison is only about height.
///
/// Run as:
///   NAMIASHI_REPLAY_OUT=/tmp/namiashi_height cargo test --release \
///     --no-default-features --features mujoco --test wbc_walk \
///     namiashi_height_video_source -- --ignored --nocapture
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_height_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        for drop in [0.0, 0.02, 0.04, 0.06] {
            let tag = format!(
                "{}_{:.0}cm",
                format!("{gait:?}").to_lowercase(),
                drop * 100.0
            );
            let params = WbcParams {
                trunk_drop_m: drop,
                total_time_s: 12.0,
                replay_dir: Some(format!("{root}/{tag}")),
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// DOES THE CONTROLLER NEED A BOUNDED ABSOLUTE BASE POSITION?
///
/// It reads one -- `WbcPipeline::solve` takes `body_world_position` straight
/// from the simulator and writes it into the floating base's `q[0..3]`. On
/// hardware there is nothing to read it from. IMU integration drifts without
/// bound, and a legged state estimator gives good orientation and velocity
/// but a position that walks away, so if this were a real dependency it would
/// be a blocker.
///
/// It should not be. Rigid-body dynamics is translation invariant: `M(q)`,
/// the nonlinear terms and the contact Jacobians depend on base *orientation*
/// and joint angles, not on where the base is. The two places the position
/// appears downstream are moment arms, `foot_pos_world - body_pos_w` and the
/// CoM variant, where it cancels.
///
/// Also worth noting: nothing calls `set_body_pose_observed`, so the MPC's
/// own `world_position` is never updated from observation at all, and its
/// reference trajectory is built from that -- self-referential, hence
/// invariant too.
///
/// Reasoning is not measurement. A constant 1 km offset tests invariance; a
/// 0.10 m/s drift is what an unaided estimator actually does, and over 16 s
/// that is 1.6 m of accumulated error.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_absolute_position_dependence() {
    let cases: &[(&str, [f64; 3], [f64; 3])] = &[
        ("none", [0.0; 3], [0.0; 3]),
        ("bias 1km", [1000.0, -1000.0, 0.0], [0.0; 3]),
        ("bias 1km + z", [1000.0, -1000.0, 5.0], [0.0; 3]),
        ("drift 0.1m/s", [0.0; 3], [0.1, -0.1, 0.0]),
        ("drift 0.5m/s", [0.0; 3], [0.5, -0.5, 0.05]),
    ];
    for &(tag, bias, drift) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                base_pos_bias_m: bias,
                base_pos_drift_mps: drift,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// HOW MUCH VELOCITY SENSING DOES THIS CONTROLLER ACTUALLY NEED?
///
/// Every result in this file was measured with simulator ground truth for the
/// body velocity: exact, noiseless, zero lag. namiashi will have an IMU and
/// nothing else, and an IMU cannot give a bounded velocity. The horizontal
/// acceleration comes out as `R(q_hat) * a_meas - g`, so an attitude error
/// theta leaks `g*sin(theta)` of phantom acceleration -- 0.17 m/s^2 at one
/// degree, which integrates to 0.85 m/s in five seconds against a 0.80 m/s
/// command. Accelerometer bias, at 0.005-0.02 m/s^2 for a good part, is the
/// smaller problem. Better hardware does not fix this; it is the structure.
///
/// The standard answer is leg odometry -- a loaded foot is stationary in the
/// world, so `v_body = -J(q) q_dot` is a velocity measurement, and the
/// 18-state KF in `legged-estimation` fuses exactly that with the IMU. But it
/// wants a per-foot contact flag, which is the sensor namiashi is not going
/// to carry.
///
/// Before building any of that, the cheaper question: what breaks without it?
/// `k_capture` is already 0, so footstep placement does not read the velocity
/// at all. What is left is the MPC's state feedback, and zeroing both paths
/// its prediction reaches the torque by cost Walk almost nothing.
///
/// Five conditions, from ground truth down to no velocity sensing whatsoever.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_velocity_observation_dependence() {
    let cases: &[(&str, VelObs)] = &[
        ("truth", VelObs::Truth),
        ("lag 20ms", VelObs::Lag(0.020)),
        ("lag 50ms", VelObs::Lag(0.050)),
        ("bias 0.3m/s", VelObs::Bias(0.3, 0.15)),
        ("open loop", VelObs::Command),
        ("zero", VelObs::Zero),
    ];
    for &(tag, vo) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                vel_obs: vo,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// THE SENSOR SET namiashi WILL ACTUALLY HAVE.
///
/// Two dependencies have already been measured away. Absolute base position
/// is not used at all -- a 1 km offset and an 8 m accumulated drift leave
/// every reported figure bit-identical, because rigid-body dynamics is
/// translation invariant and the only place the position appears downstream
/// is a moment arm where it cancels. Body velocity is barely used -- with
/// `k_capture` at 0 the footstep plan does not read it, and feeding a
/// constant zero still tracks 101-102%.
///
/// One is left. `ContactDrivenPhase` takes a per-foot normal force and flips
/// a leg to stance the moment it reads above 5 N, and that flag decides the
/// WBC's priority-0 constraints. namiashi will not carry foot force sensors.
///
/// So run the configuration the hardware can actually supply: joint encoders
/// and an IMU, nothing else. Contact state from the gait schedule's nominal
/// phase, no velocity estimate at all. Against the same settings with ground
/// truth, at the tuned stance, for 25 s.
#[test]
#[ignore = "long -- run with --ignored"]
fn namiashi_hardware_sensor_set() {
    // label, contact-force threshold, velocity observation
    let cases: &[(&str, f64, VelObs)] = &[
        ("truth + contact", 5.0, VelObs::Truth),
        ("truth, no contact", 1.0e9, VelObs::Truth),
        ("no vel, contact", 5.0, VelObs::Zero),
        ("IMU+enc only", 1.0e9, VelObs::Zero),
    ];
    for &(tag, thr, vo) in cases {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                early_contact_n: thr,
                vel_obs: vo,
                total_time_s: 26.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} | {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// CAN THIS RUN THROUGH A SPEED-MODE DRIVER?
///
/// The LKMTech MG4005 has no MIT mode. Its CAN protocol offers closed-loop
/// position, closed-loop speed and closed-loop iq, so the
/// position-target-plus-torque-feedforward interface every result in this
/// file was produced with does not exist on the part. Speed mode is the first
/// choice over iq because torque mode moves the whole stabilising loop onto
/// the host, where bus latency and host rate set the stability margin
/// directly.
///
/// Two gains decide whether it works, and they are different things:
///
///   `loop_kv`  stands in for the driver's own speed loop. The model ships
///              `actuator_kv = 1.2`, which is a position-mode damping value:
///              reaching the 1.5 N*m limit would need 1.25 rad/s of velocity
///              error. A real speed loop is far stiffer, and has integral
///              action this proportional stand-in does not.
///   `k_track`  is the outer position loop the host has to add. A speed loop
///              has no position feedback, so trajectory velocity alone tracks
///              the right rate while absolute position walks away.
///
/// Swept against the position path at the same stance and settings.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_velocity_actuation_sweep() {
    eprintln!("---- reference: position target + torque feedforward ----");
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let params = WbcParams {
            total_time_s: 16.0,
            ..namiashi_tuned_params(i)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?} pos+tau"), &samples, cmd_vx, 1.0);
    }

    eprintln!("---- speed mode: loop stiffness x position-loop gain ----");
    for loop_kv in [1.2, 5.0, 20.0] {
        for k_track in [0.0, 20.0, 60.0] {
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    actuation: Actuation::Velocity { k_track, loop_kv, loop_ki: 0.0 },
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// SPEED MODE, ZOOMED IN ON WHAT THE FIRST SWEEP FOUND.
///
/// Three things came out of it. Trajectory velocity alone does not stand the
/// robot up -- at `k_track = 0` all three gaits end the run at a trunk height
/// of 0.041 m, which is lying down. A speed loop has no position feedback, so
/// the outer loop is not optional.
///
/// `k_track = 60` gets Walk and Crawl to 103% of command, but Trot only to
/// 93% and with 3 deg/s of yaw drift against 0.39 on the position path.
///
/// And a *stiffer* driver loop is worse, not better: at `k_track = 60`,
/// `loop_kv` of 1.2 / 5 / 20 gives 93 / 91 / 90% on Trot and 103 / 97 / 90%
/// on Walk. Worth understanding rather than just noting -- a stiff
/// proportional speed loop reaches the 1.5 N*m clamp on a small velocity
/// error, so it spends most of its time saturated and stops being
/// proportional at all.
///
/// So: hold the loop soft and push the outer gain. `k_track` is in 1/s -- 60
/// means a 0.01 rad position error asks for 0.6 rad/s -- so this is also
/// asking how much outer-loop bandwidth the host has to supply, which on a
/// CAN bus is a real constraint and not just a number.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_velocity_actuation_zoom() {
    for loop_kv in [1.2, 2.5] {
        for k_track in [60.0, 100.0, 150.0, 220.0] {
            for i in 0..NAMIASHI_TUNED.len() {
                let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
                let params = WbcParams {
                    actuation: Actuation::Velocity { k_track, loop_kv, loop_ki: 0.0 },
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// SPEED MODE AGAINST TORQUE MODE, AT HOST RATES A CAN BUS CAN ACTUALLY HOLD.
///
/// Both are on the MG4005. The choice between them is not about which control
/// law is better -- at the same update rate they are close to the same law,
/// since the torque path is just the driver's PD computed host-side. The
/// choice is about *where the loop lives*.
///
/// A speed-mode driver closes its inner loop internally, on fresh encoder
/// data, at several kHz, whatever the host is doing. The host only has to
/// shape the trajectory and supply the outer position term. In torque mode
/// there is no inner loop at all: the last torque the host sent is held until
/// the next one arrives, so host rate and bus latency set the stability
/// margin directly.
///
/// Everything measured in this file so far ran the controller at 500 Hz, the
/// physics rate. A CAN bus with twelve motors on it does not. So sweep the
/// host rate and watch which path degrades first.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_vs_torque_host_rate() {
    for hz in [500.0, 200.0, 100.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("speed", Actuation::Velocity { k_track: 100.0, loop_kv: 1.2, loop_ki: 0.0 }),
                // kp/kd are the model's own actuator gains, so the torque path
                // reproduces the driver's PD exactly and only the rate differs.
                ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(hz),
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} {hz:.0}Hz"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// VIDEO SOURCE: speed mode against torque mode, per gait.
///
/// Both interfaces the MG4005 offers, at the 400 Hz the bus is designed for.
/// The speed side uses the ideal-velocity-source model, since that is what an
/// 8-16 kHz driver loop looks like from a 400 Hz host, and `k_track = 40`,
/// which is the value that suits all three gaits under that model.
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_actuation_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        for (tag, act) in [
            ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
            ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ] {
            let g = format!("{gait:?}").to_lowercase();
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 12.0,
                replay_dir: Some(format!("{root}/{g}_{tag}")),
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} {tag}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// WHAT HOST RATE DOES TORQUE MODE NEED, PER GAIT?
///
/// The coarse sweep put the breakdown between 200 Hz, where torque mode holds
/// 97-102% on all three gaits, and 100 Hz, where it falls to 73-90%. The bus
/// is expected to reach 400 Hz, so the question is which gaits have margin
/// there and which are running near their edge.
///
/// Torque mode has no inner loop: the last torque the host sent is held until
/// the next arrives, so the host period is dead time inside the only loop
/// there is. The rate a gait needs should therefore scale with how fast that
/// gait moves -- Trot's 0.320 s period against Crawl's 0.800 s -- and that is
/// worth checking rather than assuming, because the position loop's own
/// bandwidth (kp=100, kd=1.2) is the same for all three and may be what binds
/// instead.
///
/// Speed mode is swept alongside at the same rates, since the comparison at
/// 400 Hz is the decision actually being made.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_host_rate_requirement() {
    // 2 kHz physics so every rate below is an exact integer divisor. At the
    // usual 0.002 s the gate quantises hard -- 400 Hz became 500 and 200
    // became 167 -- and the sweep silently reported duplicate columns.
    const SWEEP_DT: f64 = 0.0005;
    for hz in [500.0, 400.0, 333.0, 250.0, 200.0, 143.0, 125.0, 100.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
                ("speed", Actuation::Velocity { k_track: 100.0, loop_kv: 1.2, loop_ki: 0.0 }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(hz),
                    dt: SWEEP_DT,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} {hz:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// THE SPEED-LOOP MODEL WAS PROPORTIONAL, AND A REAL ONE IS NOT.
///
/// Every speed-mode figure in this file so far came from a driver model of
/// `tau = kv * (qd* - qd)`, with `kv = 1.2`. That leaves a standing error
/// proportional to load: reaching the 1.5 N*m limit needs 1.25 rad/s of
/// velocity error, so the commanded speed is never actually achieved. Trot's
/// 89-94% ceiling on that path was the model, not the interface.
///
/// An MG4005 closes a PI loop internally at 8-16 kHz. The integral term is
/// what removes the standing error and rejects load torque, and it is the
/// bigger of the two things the model was missing -- the rate matters less
/// than it sounds, since even the 2 kHz the simulator can offer is already
/// five times the 400 Hz host.
///
/// So sweep the integral gain. `loop_ki` is in N*m per (rad/s * s); with the
/// knee's 0.0034 kg*m^2 of reflected inertia, the proportional part alone
/// puts the loop corner near 56 Hz, so an integral corner in the tens to low
/// hundreds of rad/s is the region worth looking at.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_loop_integral() {
    for loop_ki in [0.0, 20.0, 80.0, 300.0, 1000.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                actuation: Actuation::Velocity {
                    k_track: 100.0,
                    loop_kv: 1.2,
                    loop_ki,
                },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} ki={loop_ki:.0}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// SO WHAT IS ACTUALLY LIMITING SPEED MODE ON TROT?
///
/// The proportional-only speed loop was a real modelling defect, and adding
/// the integral a real driver has does not fix Trot: 89 / 89 / 85 / 81 / 83%
/// across `loop_ki` of 0 / 20 / 80 / 300 / 1000. It makes it worse, and the
/// saturation column says why -- the thigh goes 29 -> 40% clamped as the
/// integral is turned up. The loop is not short of authority. It is already
/// spending more than it has.
///
/// Which rules out the standing-error story and leaves the other half of the
/// interface. In speed mode the host does not command a position at all; it
/// commands `qd = dq*/dt + k_track*(q* - q)`, and everything the WBC computed
/// is discarded, because a speed-mode driver has nowhere to put a torque.
/// Trot is the gait with the shortest stance and the highest foot loads, so
/// it is the one that misses that most.
///
/// Two things are still confounded in every speed-mode figure so far, and
/// they pull opposite ways: `k_track` sets how hard the outer loop chases
/// position error, and `loop_kv` sets how hard the driver resists. Sweeping
/// them together at the 400 Hz the bus will actually run, with the integral
/// at a value a real driver would have, is what separates them.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_speed_mode_trot_limit() {
    for loop_kv in [1.2, 4.0, 12.0] {
        for k_track in [40.0, 100.0, 200.0] {
            let i = 0; // Trot -- the only gait that does not already work
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                actuation: Actuation::Velocity {
                    k_track,
                    loop_kv,
                    loop_ki: 80.0,
                },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("{gait:?} kv={loop_kv:.1} kt={k_track:.0}"),
                &samples,
                cmd_vx,
                1.0,
            );
        }
    }
}

/// THE DRIVER MODELLED AS WHAT IT IS: A VELOCITY SOURCE.
///
/// Every speed-mode figure before this one came from a driver model that was
/// a P or PI controller evaluated at the *physics* rate. That imports a
/// slowness the real part does not have. An MG4005 closes its speed loop at
/// 8-16 kHz; against a host at 400 Hz that is twenty to forty times faster
/// than anything being asked of it, so from the controller's side it simply
/// delivers the commanded velocity.
///
/// `VelocityIdeal` models it that way -- deadbeat, `M_ii*(qd* - qd)/dt` plus
/// the joint's bias forces, clamped to `effort`. The clamp stays because
/// torque is what actually binds on this robot; the loop bandwidth was never
/// the constraint.
///
/// Swept against the PI model at both `k_track` values the earlier sweeps
/// disagreed about -- 40 suited Trot, 100 suited Walk and Crawl -- at the
/// 400 Hz the bus is designed for.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_ideal_velocity_source() {
    for k_track in [40.0, 100.0, 200.0] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            for (tag, act) in [
                ("PI kv1.2", Actuation::Velocity {
                    k_track,
                    loop_kv: 1.2,
                    loop_ki: 80.0,
                }),
                ("ideal", Actuation::VelocityIdeal { k_track }),
            ] {
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(400.0),
                    dt: 0.0005,
                    total_time_s: 16.0,
                    ..namiashi_tuned_params(i)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk(
                    &format!("{gait:?} {tag} kt={k_track:.0}"),
                    &samples,
                    cmd_vx,
                    1.0,
                );
            }
        }
    }
}

/// What a push did, measured against the same run's own pre-push behaviour.
///
/// Reported rather than asserted, because the interesting quantity is not
/// "did it survive" -- both interfaces do -- but how far it went and how long
/// it took to come back, which is what separates them.
fn report_push(label: &str, samples: &[WbcSample], t_push: f64, cmd_vx: f64) {
    let period = samples[0].cycle_period_s;
    let win = period * (0.8 / period).ceil();

    // Body-frame rates over a whole number of gait cycles, same convention as
    // everywhere else in this file.
    let rate_at = |i: usize| -> (f64, f64, f64) {
        let t0 = samples[i].t - win;
        let j = samples.iter().position(|s| s.t >= t0).unwrap_or(0);
        if j >= i {
            return (0.0, 0.0, 0.0);
        }
        let (a, b) = (&samples[j], &samples[i]);
        let dt = b.t - a.t;
        let (c, sn) = ((-a.yaw).cos(), (-a.yaw).sin());
        let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
        let mut dyaw = b.yaw - a.yaw;
        while dyaw > std::f64::consts::PI {
            dyaw -= 2.0 * std::f64::consts::PI;
        }
        while dyaw < -std::f64::consts::PI {
            dyaw += 2.0 * std::f64::consts::PI;
        }
        (
            (c * dx - sn * dy) / dt,
            (sn * dx + c * dy) / dt,
            dyaw.to_degrees() / dt,
        )
    };

    let idx_at = |t: f64| samples.iter().position(|s| s.t >= t).unwrap_or(0);
    let i_push = idx_at(t_push);
    let after: Vec<usize> = (i_push..samples.len()).collect();

    let mut vy_peak = 0.0_f64;
    let mut roll_peak = 0.0_f64;
    let mut z_min = f64::INFINITY;
    let mut t_recover = None;
    for &i in &after {
        let (_, vy, _) = rate_at(i);
        vy_peak = vy_peak.max(vy.abs());
        roll_peak = roll_peak.max(samples[i].roll.abs());
        z_min = z_min.min(samples[i].body_z);
        // Recovered: sideways rate back under 5 cm/s and staying there.
        if t_recover.is_none() && samples[i].t > t_push + 0.3 && vy.abs() < 0.05 {
            t_recover = Some(samples[i].t - t_push);
        }
    }
    // Lateral offset left behind, in the heading frame at the push.
    let a = &samples[i_push];
    let b = &samples[samples.len() - 1];
    let (c, sn) = ((-a.yaw).cos(), (-a.yaw).sin());
    let (dx, dy) = (b.body_x - a.body_x, b.body_y - a.body_y);
    let lat_left = sn * dx + c * dy;
    let fwd_after = (c * dx - sn * dy) / (b.t - a.t);

    eprintln!(
        "=== {label} (push at {t_push:.1}s) ===\n\
         peak |vy|={vy_peak:.3} m/s  recovered after {}  \
         lateral offset left={lat_left:+.3}m\n\
         peak roll={:.1}deg  min z={z_min:.3}m  \
         forward after push={fwd_after:+.3} m/s ({:.0}% of cmd)",
        t_recover.map_or("never".into(), |t| format!("{t:.2}s")),
        roll_peak.to_degrees(),
        if cmd_vx.abs() > 1e-9 { 100.0 * fwd_after / cmd_vx } else { 0.0 },
    );
}

/// SPEED MODE AGAINST TORQUE MODE, ACROSS EVERYTHING THE ROBOT IS ASKED TO DO.
///
/// Both at 400 Hz, both on the tuned configuration, differing only in the
/// interface: speed mode as an ideal torque-limited velocity source with
/// `k_track = 40`, torque mode as the host computing the whole loop.
///
/// Forward is the case every earlier result covered. The other five are the
/// ones that decide whether an interface is usable rather than merely
/// demonstrable -- a gait that only walks forward on flat ground with a
/// constant command is not a controller.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_interface_full_comparison() {
    const HOST_HZ: f64 = 400.0;
    const SWEEP_DT: f64 = 0.0005;
    // 12 N for 0.12 s on a 3.3 kg robot is 1.44 N*s, a 0.44 m/s kick
    // sideways -- comparable to the fastest gait's own command, so it is a
    // real disturbance and not a nudge.
    const PUSH_T: f64 = 7.0;

    let modes: [(&str, Actuation); 2] = [
        ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
        ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
    ];

    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd) = NAMIASHI_TUNED[i];
        let lat = 0.5 * cmd;
        for (mtag, act) in modes {
            let base = |p: WbcParams| WbcParams {
                actuation: act,
                host_rate_hz: Some(HOST_HZ),
                dt: SWEEP_DT,
                ..p
            };

            // Steady commands: forward, backward, turn, strafe both ways.
            for (tag, vx, vy, wz) in [
                ("forward", cmd, 0.0, 0.0),
                ("backward", -cmd, 0.0, 0.0),
                ("turn", 0.0, 0.0, 0.60),
                ("strafe_L", 0.0, lat, 0.0),
                ("strafe_R", 0.0, -lat, 0.0),
            ] {
                let params = base(WbcParams {
                    cmd_vx: vx,
                    cmd_vy: vy,
                    cmd_wz: wz,
                    total_time_s: 14.0,
                    ..namiashi_tuned_params(i)
                });
                let Some(samples) = run_wbc_sim(params) else { return };
                report_walk_cmd(
                    &format!("{gait:?} {mtag} {tag}"),
                    &samples,
                    vx,
                    vy,
                    wz,
                    1.0,
                );
            }

            // Speed regulation: a command that moves, in steps.
            let params = base(WbcParams {
                cmd_vx: 0.0,
                cmd_schedule: vec![
                    (1.0, 0.35 * cmd, 0.0, 0.0),
                    (5.0, cmd, 0.0, 0.0),
                    (9.0, 0.6 * cmd, 0.0, 0.0),
                    (13.0, 0.0, 0.0, 0.0),
                ],
                total_time_s: 17.0,
                ..namiashi_tuned_params(i)
            });
            if let Some(samples) = run_wbc_sim(params) {
                // Reported per segment, since a whole-run average over a
                // moving command means nothing.
                for (t0, want) in
                    [(2.0, 0.35 * cmd), (6.0, cmd), (10.0, 0.6 * cmd), (14.0, 0.0)]
                {
                    let seg: Vec<WbcSample> = samples
                        .iter()
                        .filter(|s| s.t >= t0 && s.t < t0 + 3.0)
                        .map(|s| WbcSample { ..*s })
                        .collect();
                    if seg.len() > 10 {
                        report_walk(
                            &format!("{gait:?} {mtag} ramp@{want:.2}"),
                            &seg,
                            want,
                            0.0,
                        );
                    }
                }
            } else {
                return;
            }

            // Disturbance: walk forward, get pushed sideways.
            let params = base(WbcParams {
                cmd_vx: cmd,
                push: Some((PUSH_T, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 14.0,
                ..namiashi_tuned_params(i)
            });
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("{gait:?} {mtag} push"), &samples, PUSH_T, cmd);
        }
    }
}

/// VIDEO SOURCE: the two cases that separate the interfaces, on Trot.
///
/// Steady commands and speed regulation came out close between speed and
/// torque mode, so neither makes a useful film. These two do.
///
/// The push clips record a whole stride's worth of push phases, because
/// `namiashi_push_phase_dependence` showed a single push time says nothing:
/// over eight phases of Trot's 0.320 s cycle, speed mode falls at three of
/// them and torque mode at three, and they are not the same three. A video
/// built on one push time would be showing an accident of phase as though it
/// were a property of the interface.
///
/// The schedule clip steps through forward slow, forward fast, turn, strafe,
/// backward and stop. Steps rather than ramps, because a step is what exposes
/// handover behaviour.
#[test]
#[ignore = "video source -- needs NAMIASHI_REPLAY_OUT"]
fn namiashi_robustness_video_source() {
    let Ok(root) = std::env::var("NAMIASHI_REPLAY_OUT") else {
        eprintln!("NAMIASHI_REPLAY_OUT unset -- nothing to record");
        return;
    };
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let modes: [(&str, Actuation); 2] = [
        ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
        ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
    ];

    for (mtag, act) in modes {
        let base = |p: WbcParams| WbcParams {
            actuation: act,
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            ..p
        };

        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = base(WbcParams {
                cmd_vx: cmd,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 10.0,
                replay_dir: Some(format!("{root}/push{step}_{mtag}")),
                ..namiashi_tuned_params(I)
            });
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {mtag} p{step}"), &samples, t_push, cmd);
        }

        let params = base(WbcParams {
            cmd_vx: 0.0,
            cmd_schedule: vec![
                (1.0, 0.35 * cmd, 0.0, 0.0),
                (4.0, cmd, 0.0, 0.0),
                (7.0, 0.0, 0.0, 0.60),
                (10.0, 0.0, 0.5 * cmd, 0.0),
                (13.0, -cmd, 0.0, 0.0),
                (16.0, 0.0, 0.0, 0.0),
            ],
            total_time_s: 19.0,
            replay_dir: Some(format!("{root}/schedule_{mtag}")),
            ..namiashi_tuned_params(I)
        });
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {mtag} schedule"), &samples, cmd, 1.0);
    }
}

/// THE PUSH OUTCOME DEPENDS ON WHEN IN THE STRIDE IT LANDS.
///
/// `namiashi_interface_full_comparison` pushed at 7.0 s and found Trot
/// shrugging it off in both modes -- 6.3 and 12.2 degrees of roll, 95% of
/// command afterwards. Moving the same push to 6.0 s knocks both down: 17 and
/// 99 degrees of roll, trunk to 0.124 m, 10-13% of command afterwards.
///
/// Nothing about the interfaces changed. What changed is which feet were
/// loaded when the impulse arrived, and Trot spends most of its cycle on a
/// diagonal pair, so a sideways kick lands somewhere between "into the
/// support line" and "across it" depending on phase.
///
/// So a single push time says nothing about an interface, and the videos
/// should not be built on one. Sweep a whole stride -- eight phases over
/// Trot's 0.320 s -- in both modes, and see what the spread actually is.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_push_phase_dependence() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    for step in 0..8 {
        let t_push = 6.0 + period * step as f64 / 8.0;
        for (mtag, act) in [
            ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
            ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ] {
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(
                &format!("Trot {mtag} phase{step}"),
                &samples,
                t_push,
                cmd,
            );
        }
    }
}

/// THE ONE REACTIVE MECHANISM IN THE STACK IS SWITCHED OFF.
///
/// Nothing in this controller replans a gait. `PhaseGenerator::advance` is
/// `cycle_phase += dt/period`, a free-running clock with no input.
/// `ContactDrivenPhase::apply_correction` is stateless -- it flips
/// `is_stance` for the WBC's constraints and never writes back, so a foot
/// landing early does not move the schedule. The only thing that ever reacted
/// to where the body actually was is the capture-point term in
/// `compute_mpc_footstep`, which shifts the foothold by `k_capture * v_err`.
///
/// And `namiashi_capture_gain_low_side` set `k_capture` to 0, because at
/// 0.05 it cost 20% of speed overshoot, 0.145 m/s of lateral drift and
/// 2.34 deg/s of yaw. That was measured on flat ground, at a constant
/// command, with no disturbance -- which is precisely the condition under
/// which footstep feedback has nothing to do and can only add noise. The
/// commit said as much and left it open.
///
/// `namiashi_push_phase_dependence` now gives the missing condition: three of
/// eight push phases end on the floor. So ask the question properly -- does
/// the term that was removed buy any of them back?
///
/// Both interfaces, all eight phases, four gains. The nominal cost is
/// re-measured alongside, since a gain that saves a push and ruins the walk
/// is not a trade worth making silently.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_capture_gain_under_push() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    for k in [0.0, 0.015, 0.030, 0.050] {
        for (mtag, act) in [
            ("speed", Actuation::VelocityIdeal { k_track: 40.0 }),
            ("torque", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ] {
            // What it costs when nothing is pushing.
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                k_capture_s: Some(k),
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Trot {mtag} k{k:.3} nominal"), &samples, cmd, 1.0);

            for step in 0..8 {
                let t_push = 6.0 + period * step as f64 / 8.0;
                let params = WbcParams {
                    actuation: act,
                    host_rate_hz: Some(400.0),
                    dt: 0.0005,
                    cmd_vx: cmd,
                    k_capture_s: Some(k),
                    push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                    total_time_s: 11.0,
                    ..namiashi_tuned_params(I)
                };
                let Some(samples) = run_wbc_sim(params) else { return };
                report_push(
                    &format!("Trot {mtag} k{k:.3} p{step}"),
                    &samples,
                    t_push,
                    cmd,
                );
            }
        }
    }
}

/// THE OCS2-DERIVED FOOTSTEP PLANNER, UNDER THE SAME PUSH.
///
/// `quadruped-gait` carries two mechanisms taken from `legged_control` /
/// OCS2, and namiashi has been using neither.
///
/// `SrbdMpcConfig::enable_foot_offset` extends the MPC's input to include a
/// per-foot landing offset, so the optimiser itself can ask a foot to land
/// further outboard to catch a lateral disturbance. It is implemented but
/// deliberately not read in `GaitMode::Mpc`: `mpc_controller.rs:440` records
/// that the offset never enters the MPC's own dynamics, so optimiser and
/// controller would disagree about what was planned. Dead infrastructure on
/// this path.
///
/// `use_mpc_predicted_footstep` is the `SwingTrajectoryPlanner` analogue --
/// take the foothold correction from the MPC's predicted base displacement
/// over one swing duration, rather than extrapolating `k_capture * v_err`
/// linearly from the present. The MPC has already planned how its GRFs pull
/// the body back, so where the base will be at touchdown is a better answer
/// than where it is heading now. It lives only in `FullCentroidal`.
///
/// So this compares three planners on the same eight push phases: the current
/// Mpc path with the capture gain that `namiashi_capture_gain_under_push`
/// found (0.015, which took survival from 5/8 to 6/8), FullCentroidal with
/// the same capture term, and FullCentroidal with the predicted-footstep
/// planner instead. Nominal walking is re-measured for each, since a planner
/// that survives pushes and cannot walk is not a planner.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_ocs2_footstep_planner() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let variants: [(&str, GaitMode, bool, f64); 3] = [
        ("mpc k.015", GaitMode::Mpc, false, 0.015),
        ("fullcent k.015", GaitMode::FullCentroidal, false, 0.015),
        ("fullcent predicted", GaitMode::FullCentroidal, true, 0.015),
    ];
    for (tag, mode, predicted, k) in variants {
        let make = |push: Option<(f64, [f64; 3], f64)>, secs: f64| WbcParams {
            actuation: Actuation::VelocityIdeal { k_track: 40.0 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            k_capture_s: Some(k),
            gait_mode: mode,
            mpc_predicted_footstep: predicted,
            push,
            total_time_s: secs,
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(make(None, 12.0)) else { return };
        report_walk(&format!("Trot {tag} nominal"), &samples, cmd, 1.0);

        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = make(Some((t_push, [0.0, 12.0, 0.0], 0.12)), 11.0);
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

/// SHOULD THE TUNED CONFIGURATION CARRY A CAPTURE GAIN AGAIN?
///
/// `namiashi_capture_gain_low_side` set it to 0 and that has stood since.
/// The conditions it was chosen under are all gone: 0.295 m stance,
/// position-plus-torque actuation, 500 Hz, no disturbance. Under the current
/// ones a small gain is better on both counts -- 90 -> 97% of command with
/// yaw drift 5.28 -> 0.85 deg/s, and push survival 5/8 -> 6/8.
///
/// Before changing the default, check it against the thing the default has to
/// keep passing: all three gaits, 25 s, on the regression's own actuation
/// path rather than the one the push sweep used.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_capture_gain_recheck_regression() {
    for k in [0.0, 0.015, 0.030] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                k_capture_s: Some(k),
                total_time_s: 26.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("{gait:?} k={k:.3}"), &samples, cmd_vx, 1.0);
        }
    }
}

/// GIVING THE MPC namiashi'S REAL INERTIA.
///
/// `auto_detect_srbd_mpc_config` hands the SRBD MPC the heaviest link's own
/// tensor. On this model that is the 0.872 kg trunk, (0.00111, 0.00504,
/// 0.00529), against a composite of (0.02722, 0.07575, 0.06584) -- 12 to 24
/// times too small. The heuristic also degrades as the model improves:
/// correcting namiashi's mass moved weight into the legs, so the heaviest
/// link got lighter while the composite roughly doubled.
///
/// Fixing it inside `auto_detect` was tried and reverted, because it costs Go2
/// most of its top speed and Go2's tuning was built against the old value.
/// Per robot is what the function's own doc recommends, and that is what this
/// measures.
///
/// Three things at once, because they are the same question at different
/// depths. Does the walk improve? Does push survival improve -- in particular
/// phases 6 and 7, which fall at every capture gain? And does the OCS2-derived
/// predicted-footstep planner become usable, given it was measured at 1 of 8
/// and the suspicion was that it inherits the MPC's bad prediction?
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_mpc_composite_inertia() {
    const I: usize = 0; // Trot for the push work
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];

    eprintln!("---- nominal walk, all three gaits ----");
    for composite in [false, true] {
        for i in 0..NAMIASHI_TUNED.len() {
            let (gait, .., c) = NAMIASHI_TUNED[i];
            let params = WbcParams {
                mpc_composite_inertia: composite,
                total_time_s: 16.0,
                ..namiashi_tuned_params(i)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let tag = if composite { "composite" } else { "heaviest" };
            report_walk(&format!("{gait:?} {tag}"), &samples, c, 1.0);
        }
    }

    eprintln!("---- push survival, Trot, 8 phases ----");
    let variants: [(&str, bool, bool); 3] = [
        ("heaviest", false, false),
        ("composite", true, false),
        ("composite+pred", true, true),
    ];
    for (tag, composite, predicted) in variants {
        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = WbcParams {
                actuation: Actuation::VelocityIdeal { k_track: 40.0 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                mpc_composite_inertia: composite,
                mpc_predicted_footstep: predicted,
                gait_mode: if predicted {
                    GaitMode::FullCentroidal
                } else {
                    GaitMode::Mpc
                },
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 11.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

/// THE INERTIA COMPARISON, REDONE ON A PATH THAT CAN CARRY IT.
///
/// `namiashi_mpc_composite_inertia` swept the push phases with
/// `Actuation::VelocityIdeal`, and a speed-mode driver takes a speed and
/// nothing else -- the WBC's torque is never delivered on that path. So the
/// MPC's output could not reach the physics by construction, and the
/// bit-identical results proved nothing about inertia. That was an
/// experimental design error, not a finding.
///
/// The MPC's output does change, and substantially. Logging the predicted
/// GRF: at the same tick the heaviest-link config gives per-foot 15.40 / 0 /
/// 0 / 17.54 N with a moment of (+0.077, -0.181, -0.015), and the composite
/// gives 12.79 / 0 / 0 / 20.58 N with (+0.459, -0.042, +0.084). A sixfold
/// change in the roll moment.
///
/// Redone on `Actuation::Torque`, where the WBC's torque is the command.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_mpc_inertia_on_torque_path() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    for composite in [false, true] {
        let tag = if composite { "composite" } else { "heaviest" };
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            mpc_composite_inertia: composite,
            total_time_s: 12.0,
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag} nominal"), &samples, cmd, 1.0);

        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = WbcParams {
                actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                mpc_composite_inertia: composite,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 11.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

/// IS THE POSITION SERVO WHY THE MPC DOES NOT MATTER HERE?
///
/// Four measurements have found the MPC and WBC to contribute almost
/// nothing -- zeroing the WBC's torque, swapping FullCentroidal for Mpc,
/// sweeping the two weights its prediction reaches the torque by, and
/// correcting a 12-24x inertia error. In `legged_control` the same
/// architecture plainly does matter, so something structural differs.
///
/// The candidate is where the stance legs get their torque. `legged_control`
/// sends Unitree `(q, dq, kp, kd, tau)` with `kp = kd = 0` on stance legs:
/// the WBC's contact force *is* the joint torque, nothing else acts on them.
/// Swing legs get position gains because they are tracking a trajectory.
///
/// This file has run kp=100 on all twelve joints throughout, with the WBC's
/// torque added as feedforward. Under a stiff servo tracking an IK
/// trajectory, a force allocation is a small correction to a much larger
/// command -- which would explain every one of those four results at once.
///
/// The test is not whether it walks, but whether the WBC becomes necessary.
/// If stance legs are pure torque and zeroing the WBC still leaves the robot
/// walking, the position servo was never the reason.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_legged_control_gain_split() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    let cases: [(&str, Actuation, bool); 6] = [
        ("all-kp100 WBC on", Actuation::Torque { kp: 100.0, kd: 1.2 }, false),
        ("all-kp100 WBC off", Actuation::Torque { kp: 100.0, kd: 1.2 }, true),
        (
            "stance-free WBC on",
            Actuation::TorqueLeggedControl {
                swing_kp: 100.0,
                swing_kd: 1.2,
                stance_kp: 0.0,
                stance_kd: 0.0,
                bias_ff: 0.0,
            },
            false,
        ),
        (
            "stance-free WBC off",
            Actuation::TorqueLeggedControl {
                swing_kp: 100.0,
                swing_kd: 1.2,
                stance_kp: 0.0,
                stance_kd: 0.0,
                bias_ff: 0.0,
            },
            true,
        ),
        // A little stance damping, since kd=0 leaves the joint with nothing
        // resisting velocity at all and MuJoCo's own joint damping is 0.1.
        (
            "stance-damped WBC on",
            Actuation::TorqueLeggedControl {
                swing_kp: 100.0,
                swing_kd: 1.2,
                stance_kp: 0.0,
                stance_kd: 0.5,
                bias_ff: 0.0,
            },
            false,
        ),
        (
            "stance-damped WBC off",
            Actuation::TorqueLeggedControl {
                swing_kp: 100.0,
                swing_kd: 1.2,
                stance_kp: 0.0,
                stance_kd: 0.5,
                bias_ff: 0.0,
            },
            true,
        ),
    ];
    for (tag, act, no_wbc) in cases {
        let params = WbcParams {
            actuation: act,
            kinematic_only: no_wbc,
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 12.0,
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// DOES AN EXPLICIT BIAS FEEDFORWARD RESCUE THE STANCE-FREE PATH?
///
/// With stance legs at kp=0 -- `legged_control`'s scheme -- the WBC becomes
/// necessary (trunk holds at 0.225 m with it, 0.134 without) but not
/// sufficient: the robot still goes down, and the thigh is clamped half the
/// run.
///
/// The WBC's `tau` solves the full equation of motion, so in principle it
/// carries the nonlinear bias already. Measured at a static stand it carries
/// far more than that: `tau_wbc` on a stance leg is (-0.369, +0.008, +0.559)
/// against a leg gravity of (+0.074, +0.048, -0.033), because the support
/// load dominates. So a duplicate bias term is a 10-20% error rather than a
/// doubling, and whether adding it helps or hurts is not something to reason
/// about from the sign.
///
/// `h(q, q̇)` rather than gravity alone, since the swing leg reaches 13-15
/// rad/s and the velocity-dependent part is not obviously negligible there.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_bias_feedforward() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    for stance_kd in [0.0, 0.5] {
        for bias_ff in [0.0, 0.5, 1.0] {
            let params = WbcParams {
                actuation: Actuation::TorqueLeggedControl {
                    swing_kp: 100.0,
                    swing_kd: 1.2,
                    stance_kp: 0.0,
                    stance_kd,
                    bias_ff,
                },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("Trot skd={stance_kd:.1} bias={bias_ff:.1}"),
                &samples,
                cmd,
                1.0,
            );
        }
    }
}

/// MAKING THE MPC LOAD-BEARING: STANCE GAIN AGAINST CONTACT-FORCE WEIGHT.
///
/// Five measurements say the MPC's plan does not reach the ground. The
/// three-point trace says where it stops: freed of the position servo the MPC
/// asks for 6-12 N of propulsion and the WBC passes 1% of it, because
/// `contact_force` -- the task that tracks the MPC's GRF -- is weight 5 at
/// priority 2, under `no_contact_motion` and `floating_base_eom` as hard
/// constraints at priority 0.
///
/// Two knobs have to move together and neither works alone. Dropping the
/// stance gain removes the propulsion the servo was supplying, and the robot
/// stops walking (6% of command). Raising `contact_force` with the servo
/// still there changes little, because the servo has already decided the
/// motion and the WBC is only reconciling force with it.
///
/// So sweep the pair. Every run reports what the MPC planned, what the WBC
/// solved and what the ground delivered, so "the MPC is load-bearing" is read
/// off the numbers rather than inferred from the gait working.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_make_the_mpc_load_bearing() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    for stance_kp in [100.0, 40.0, 10.0, 0.0] {
        for cf_w in [5.0, 50.0, 300.0] {
            let params = WbcParams {
                actuation: Actuation::TorqueLeggedControl {
                    swing_kp: 100.0,
                    swing_kd: 1.2,
                    stance_kp,
                    stance_kd: 0.5,
                    bias_ff: 0.0,
                },
                contact_force_weight: Some(cf_w),
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(
                &format!("Trot skp={stance_kp:.0} cf={cf_w:.0}"),
                &samples,
                cmd,
                1.0,
            );
        }
    }
}

/// legged_control's OWN GAINS.
///
/// Its task stack turns out to be identical to this one -- floating-base EoM,
/// torque limits, friction cone and no-contact-motion hard at priority 0,
/// base-accel and swing-leg at 1, contact-force at 2. So the port is faithful
/// and `contact_force` being lowest is not the problem, which is consistent
/// with weighting it 5 to 300 changing nothing.
///
/// What differs is the command. `legged_controller.cpp:142`:
///
/// ```text
/// setCommand(pos_des, vel_des, 5, 3, torque)
/// ```
///
/// kp = 5, not the 100 this file has used throughout; uniform, with no
/// stance/swing split; and the damping tracks a velocity target rather than
/// damping toward zero. A twentieth of the gain, and not fighting the swing.
///
/// The stance-gain sweep already showed the MPC starting to matter as the
/// gain comes down -- the ratio of WBC solution to MPC plan goes -9.9 at
/// kp=100 (opposite sign; the WBC is reconciling force with motion the servo
/// already produced) to +1.55 at 40. It stopped at 10 and 0, where the MPC
/// asks for 5-12 N and the walk falls apart. 5 is below all of that, so
/// whether it works is not obvious from the trend.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_legged_control_gains() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    let cases: &[(&str, f64, f64)] = &[
        ("lc 5/3", 5.0, 3.0),
        ("lc 10/3", 10.0, 3.0),
        ("lc 20/3", 20.0, 3.0),
        ("lc 40/3", 40.0, 3.0),
        ("lc 100/3", 100.0, 3.0),
        // The current path for reference: same uniform gains but damping
        // toward zero instead of tracking the trajectory velocity.
        ("current 100/1.2", 100.0, 1.2),
    ];
    for &(tag, kp, kd) in cases {
        let act = if tag.starts_with("current") {
            Actuation::Torque { kp, kd }
        } else {
            Actuation::LeggedControl { kp, kd }
        };
        let params = WbcParams {
            actuation: act,
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 12.0,
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// DOES MAKING THE MPC LOAD-BEARING BUY ANYTHING UNDER DISTURBANCE?
///
/// It costs something on flat ground. Dropping the uniform gain from 100 to 5
/// takes the WBC-solution-to-MPC-plan ratio from +10.1 to +0.88 -- the WBC
/// stops overriding the plan and starts following it -- and costs 107% -> 83%
/// of command, 0.231 -> 0.198 m of trunk height and 4.3 -> 8.0 deg of roll.
///
/// Flat ground at a constant command is exactly where a predictive layer has
/// nothing to predict, so that trade says little on its own. The reason to
/// want the MPC load-bearing is what happens when something unexpected
/// arrives, and the push test is the one condition in this file that supplies
/// that. If a lower gain buys nothing here, chasing further fidelity to
/// legged_control is chasing fidelity for its own sake.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_mpc_authority_under_push() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let cases: &[(&str, Actuation)] = &[
        ("current 100/1.2", Actuation::Torque { kp: 100.0, kd: 1.2 }),
        ("lc 100/3", Actuation::LeggedControl { kp: 100.0, kd: 3.0 }),
        ("lc 20/3", Actuation::LeggedControl { kp: 20.0, kd: 3.0 }),
        ("lc 5/3", Actuation::LeggedControl { kp: 5.0, kd: 3.0 }),
    ];
    for &(tag, act) in cases {
        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let params = WbcParams {
                actuation: act,
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                push: Some((t_push, [0.0, 12.0, 0.0], 0.12)),
                total_time_s: 11.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

/// FULLCENTROIDAL WITH ITS legged_control-PARITY OPTIONS.
///
/// The reason for coming here was to take `pos_des` from the MPC's optimised
/// state, the way `legged_controller.cpp:142` does, since at kp=5 the IK
/// trajectory and the MPC's plan disagree and the two fight.
///
/// That is not available. `FullCentroidalMpcGaitController`'s own header says
/// so: "Reference joint_q is held at the controller's current IK output" and
/// "swing leg foot tracking is still driven by the CHAMP layer's joint
/// target". The 24-state MPC carries joint angles so that the per-node moment
/// arm `r = R (foot_body(q) - com_offset)` updates within the horizon -- a
/// coupling the 12-state SRBD cannot see -- but they never become a command.
/// There is no `getJointAngles(optimized_state)` here.
///
/// What is available is `legged_control_parity`, which builds the per-step
/// contact schedule from a per-leg phase projection and reshapes the swing
/// reference, and `dynamic_joint_q_reference`, which samples the foot curve
/// at each horizon step's projected phase instead of holding joint_q flat.
/// Both change what the MPC plans against rather than what the joints are
/// told, so they should show up as the MPC's plan getting better rather than
/// as the servo being bypassed.
///
/// Measured at the gain where the MPC has authority (kp=5, ratio +0.88) and
/// at the one that walks best (kp=100), so a change in the plan's quality is
/// visible at both ends.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_fullcentroidal_parity() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    let opts: [(&str, bool, bool); 3] = [
        ("plain", false, false),
        ("parity", true, false),
        ("parity+dynq", true, true),
    ];
    for (otag, parity, dynq) in opts {
        for (gtag, kp) in [("kp100", 100.0), ("kp5", 5.0)] {
            let params = WbcParams {
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: parity,
                dynamic_joint_q_reference: dynq,
                actuation: Actuation::LeggedControl { kp, kd: 3.0 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Trot {otag} {gtag}"), &samples, cmd, 1.0);
        }
    }
}

/// THE 24-STATE MPC SHARES ONE COST SCALAR ACROSS TWO UNIT SYSTEMS.
///
/// Its plan is wrong in a specific way: 40-50 N of vertical against m*g of
/// 32.4, and -20 N of fore-aft while the robot walks forward. Not a sign
/// convention (vertical is positive), not the mass (3.300 kg, read back), not
/// the normal-force cap (200 N, six times m*g). An over-large vertical
/// paired with a large backward fore-aft, at 84% of what the friction cone
/// allows at that vertical, is a solution riding the cone edge.
///
/// The defaults are `q_diag: [1.0; 24]` and `r_diag: [1e-3; 24]`. The input
/// vector is twelve forces in newtons followed by twelve joint velocities in
/// rad/s, and `r_diag`'s own doc says those "two distinct scales coexist ...
/// unlike the 12-state version we cannot share a scalar" -- and then the
/// default shares one. Forces of tens of newtons cost the same per unit as
/// velocities of tens of rad/s, so the optimiser buys force cheaply. The
/// 12-state SRBD avoids this by having twelve inputs that are all forces.
///
/// Sweeping the GRF half alone. If this is the cause, the vertical should
/// come down toward 34 and the fore-aft toward zero.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_fullcentroidal_grf_cost() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    for w in [1e-3, 1e-2, 1e-1, 1.0] {
        for (gtag, kp) in [("kp100", 100.0), ("kp5", 5.0)] {
            let params = WbcParams {
                gait_mode: GaitMode::FullCentroidal,
                fcm_grf_cost: Some(w),
                actuation: Actuation::LeggedControl { kp, kd: 3.0 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Trot r={w:e} {gtag}"), &samples, cmd, 1.0);
        }
    }
}

/// THE 24-STATE MPC ALSO WEIGHS FIVE UNIT SYSTEMS EQUALLY.
///
/// Fixing the input side worked: `r_diag[0..12]` from 1e-3 to 1 takes the
/// planned vertical from 50.5 N to 32.6 against m*g of 32.4, and the fore-aft
/// from -20.9 to -1.2. The state side has the same shape of problem --
/// `q_diag: [1.0; 24]` over `[v_com (m/s), omega (rad/s), base_pos (m), euler
/// (rad), joint_q (rad)]`.
///
/// The base-position block is the one that stands out. A quadruped walking
/// forward accumulates position without bound, so a cost on absolute position
/// is a cost on an error that only grows -- and the 12-state SRBD was already
/// measured not to need absolute position at all: a 1 km offset and an 8 m
/// drift left every figure bit-identical.
///
/// Swept as blocks rather than 24 scalars, with the GRF cost held at the
/// value the input sweep found.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_fullcentroidal_state_cost() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    //                       v_com omega  pos  euler joint_q
    let cases: &[(&str, [f64; 5])] = &[
        ("default", [1.0, 1.0, 1.0, 1.0, 1.0]),
        ("no-pos", [1.0, 1.0, 0.0, 1.0, 1.0]),
        ("no-pos no-jq", [1.0, 1.0, 0.0, 1.0, 0.0]),
        // Velocity tracking is the actual objective, so weight it.
        ("vel-led", [10.0, 5.0, 0.0, 5.0, 0.1]),
        ("vel-led+jq", [10.0, 5.0, 0.0, 5.0, 1.0]),
    ];
    for &(tag, q) in cases {
        for (gtag, kp) in [("kp100", 100.0), ("kp5", 5.0)] {
            let params = WbcParams {
                gait_mode: GaitMode::FullCentroidal,
                fcm_grf_cost: Some(1.0),
                fcm_state_cost: Some(q),
                actuation: Actuation::LeggedControl { kp, kd: 3.0 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 12.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            report_walk(&format!("Trot {tag} {gtag}"), &samples, cmd, 1.0);
        }
    }
}

/// FOUR CONFOUNDS REMOVED AT ONCE.
///
/// A line-by-line audit against `legged_control/legged_wbc` found the eight
/// task formulations faithful -- same A/b/D/f blocks, same friction pyramid
/// rows, same selection matrix, same hierarchical projection. The divergences
/// are elsewhere, and four of them are cheap to remove together:
///
/// 1. `a_lin_body = R^T a_world` drops `- omega_b x v_b`. `v[0..6]` is built
///    as a body-frame spatial velocity, so the matching acceleration needs
///    that term. It is purely lateral and scales as `vx * wz` -- 0.15 m/s^2
///    at 0.3 m/s and 0.5 rad/s, against a reference of order 0.5-1.
/// 2. Per-task weights (eom 1000, no-contact 1000, base-accel 200,
///    contact-force 5) that `legged_control` does not have; it concatenates
///    tasks within a level unweighted. Row-scaling an exactly-solvable
///    least-squares system does not move its solution, and levels 0 and 1
///    are exactly solvable here, so the weights are inert except where an
///    inequality binds -- which would make the tuning table built on them
///    a record of closed-loop noise.
/// 3. `tau_gravity`, a task with no counterpart in the original. It anchors
///    tau toward `RNEA(q, 0, 0)`, the torque to hold the legs up with the
///    base externally supported and the feet unloaded -- not the stance
///    torque, which is dominated by `J^T f`.
/// 4. The joint servo: kp=100 here against legged_control's 5, with `pos_des`
///    from analytical IK rather than the MPC's optimised state.
///
/// Measured one at a time and then together, so a null result on the whole
/// does not hide an effect in a part.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_legged_control_confounds() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    //             tag              coriolis  flat-w  drop-tg  kp
    let cases: &[(&str, bool, bool, bool, f64)] = &[
        ("baseline", false, false, false, 100.0),
        ("+coriolis", true, false, false, 100.0),
        ("+flat weights", false, true, false, 100.0),
        ("+drop tau_grav", false, false, true, 100.0),
        ("all three", true, true, true, 100.0),
        ("all three, kp5", true, true, true, 5.0),
        ("kp5 alone", false, false, false, 5.0),
    ];
    for &(tag, cor, flat, drop, kp) in cases {
        let params = WbcParams {
            base_accel_coriolis: cor,
            flat_wbc_weights: flat,
            drop_tau_gravity: drop,
            actuation: Actuation::LeggedControl { kp, kd: 3.0 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 12.0,
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// THE MISSING CORIOLIS TERM, UNDER THE COMMAND THAT ACTUALLY EXCITES IT.
///
/// `a_lin_body = R^T a_world` omits `- omega_b x v_b`. `v[0..6]` is built as a
/// body-frame spatial velocity (`wbc_pipeline.rs:373-376` says so), so the
/// matching acceleration needs that term.
///
/// `namiashi_legged_control_confounds` found adding it changed nothing, but
/// that ran a straight forward walk where `wz` is 0.5 deg/s -- the term is
/// `omega x v`, so it is identically zero there and the null result says
/// nothing. It is purely lateral and scales as `vx * wz`, so it only appears
/// when the robot is walking and turning at once.
///
/// Both signs of `wz`, because the claim is that the error carries the sign of
/// `vx * wz`: if so, the lateral drift should be asymmetric in `wz` without
/// the term and symmetric with it.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_base_accel_coriolis() {
    const I: usize = 0; // Trot
    let (_, .., cmd) = NAMIASHI_TUNED[I];
    for wz in [0.6, -0.6] {
        for coriolis in [false, true] {
            let params = WbcParams {
                base_accel_coriolis: coriolis,
                cmd_vx: 0.5 * cmd,
                cmd_wz: wz,
                total_time_s: 16.0,
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let tag = if coriolis { "with" } else { "without" };
            report_walk_cmd(
                &format!("Trot wz={wz:+.1} {tag}"),
                &samples,
                0.5 * cmd,
                0.0,
                wz,
                1.0,
            );
        }
    }
}

/// HOW MUCH TORQUE DOES SUPPORTING THIS ROBOT ACTUALLY COST?
///
/// The case against adopting legged_control's architecture rests on one
/// physical claim: that at 3.3 kg with 2.5 N*m peak and 0.306 m legs there is
/// no headroom for an MPC to author accelerations, because supporting body
/// weight already consumes the budget. The figure quoted was "one stance foot
/// carrying m*g/2 at a 0.15 m moment arm is 2.4 N*m", i.e. peak torque at
/// static stance.
///
/// That does not match the leg's actual Jacobian at this stance. A vertical
/// foot force produces no thigh torque at all -- the thigh axis responds to
/// fore-aft force -- and vertical load is carried by the knee and hip-roll.
/// So the claim needs computing rather than estimating, and it is the only
/// physical argument in the case, so it is worth getting right.
///
/// Measured directly: hold still, then walk each gait, and report the peak and
/// mean torque per joint role as a fraction of what the joint can deliver.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_support_torque_budget() {
    eprintln!("---- static stance: what holding still costs ----");
    let params = WbcParams {
        total_time_s: 4.0,
        burn_in_s: 1.0,
        cmd_vx: 0.0,
        ..namiashi_tuned_params(0)
    };
    if let Some(samples) = run_wbc_sim(params) {
        report_walk("static", &samples, 0.0, 1.5);
    }

    eprintln!("---- each tuned gait ----");
    for i in 0..NAMIASHI_TUNED.len() {
        let (gait, .., cmd_vx) = NAMIASHI_TUNED[i];
        let params = WbcParams {
            total_time_s: 16.0,
            ..namiashi_tuned_params(i)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("{gait:?}"), &samples, cmd_vx, 1.0);
    }
}

/// FIXING THE WIRING, ONE LAYER AT A TIME, ON A PATH THAT CARRIES THE TORQUE.
///
/// The case for rejecting this architecture collapsed on inspection, so the
/// measurements have to be redone. Three separate defects invalidated most of
/// them:
///
/// - `WbcPipeline` hardcodes `mass_kg: 9.0` and a 9 kg inertia diagonal, and
///   only five of the fifty-nine tests in this file override them. At a static
///   stand that makes the largest soft weight in the QP command the trunk
///   downward at 0.63 g, continuously. Every low-gain experiment -- the ones
///   that hand authority to the WBC -- ran with the WBC most wrong about the
///   robot.
/// - `namiashi_ocs2_footstep_planner` runs `Actuation::VelocityIdeal`, where
///   `deliver_tau` is false and the WBC's torque is discarded. Both the
///   "FullCentroidal is bit-identical to Mpc" and the "predicted footstep gives
///   1/8" results came from a configuration where the MPC could not reach the
///   plant at all. The same error was found and corrected for a different test
///   and never propagated here.
/// - The SRBD horizon is 10 steps at 0.030 s = 0.300 s, shorter than Trot's
///   0.320 s cycle. A receding-horizon controller that cannot see one contact
///   sequence has nothing to predict. `legged_control` runs 1.0 s.
///
/// And two settings the reference has that this does not: `q_diag[p_x]` is 0
/// against `task.info:159`'s 1000, so nothing in the cost grows when the body
/// falls behind -- which is the whole explanation for the MPC planning +0.22 N
/// -- and `initializeInputCostWeight`'s Jacobian-mapped joint-velocity weight
/// is implemented and never called.
///
/// Adjudicated on delivered force, attitude and how much of the plan gets
/// through. Tracking percentage is a gate, not a score: 107% of command from
/// an open-loop Raibert plan on flat ground is already near the geometric
/// ceiling `max_step/(T*duty)`, and there is nothing there for a predictive
/// Does the MPC's own predicted trajectory add anything over a naive attitude
/// regulator, on the one channel (`base_accel`, priority 1) the null-space
/// measurement showed actually has room to matter?
///
/// `namiashi_contact_force_authority` found `contact_force` (priority 2) inert
/// at Trot's two-foot stance -- one scalar direction out of 44 -- and the real
/// authority sits at priority 1, split between `base_accel` (the MPC-derived
/// Newton's-law feedforward) and `swing_leg`. That measurement did not ask
/// whether the *specific plan* riding on `base_accel` matters, only that the
/// channel has room. This does: it replaces the MPC's GRF-derived feedforward
/// with a quasi-static gravity split (no momentum plan, no footstep-driven
/// moment) plus an explicit roll/pitch PD, and compares that bare regulator
/// against the corrected FullCentroidal plan under the same push battery.
///
///   A  fcm fixed        the real plan: sparse QP, 0.900 s, cost ratio fixed
///   B  pd kp=400 kd=40   critically-damped attitude PD, gravity-split GRF,
///                        no plan at all
///   C  pd kp=900 kd=60   a stiffer, faster PD -- so a loss for A is not
///                        credited to B being under-tuned
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_base_accel_ablation() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 11.0,
        wbc_real_inertia: true,
        ..namiashi_tuned_params(I)
    };
    let fcm_fixed = || WbcParams {
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        fcm_horizon_steps: Some(30),
        fcm_sparse_qp: true,
        fcm_jointv_cost: Some(1e-3),
        ..base()
    };
    let arms: Vec<(&str, Box<dyn Fn() -> WbcParams>)> = vec![
        ("A fcm fixed", Box::new(fcm_fixed)),
        (
            "B pd kp=400",
            Box::new(move || WbcParams { attitude_pd_ablation: Some((400.0, 40.0)), ..base() }),
        ),
        (
            "C pd kp=900",
            Box::new(move || WbcParams { attitude_pd_ablation: Some((900.0, 60.0)), ..base() }),
        ),
    ];
    // Same two worst-case phases as the crossover sweep, at the baseline
    // impulse (12 N) plus the largest one that sweep tests (52 N), so this
    // reads directly against those results.
    let phase_steps = [1_usize, 7];
    let magnitudes_n = [12.0, 52.0];
    for (tag, mk) in arms {
        for &step in &phase_steps {
            let t_push = 6.0 + period * step as f64 / 8.0;
            for &f_n in &magnitudes_n {
                let mut p = mk();
                p.push = Some((t_push, [0.0, f_n, 0.0], 0.12));
                let Some(samples) = run_wbc_sim(p) else { return };
                report_push(
                    &format!("Trot {tag} p{step} F={f_n:.0}N"),
                    &samples,
                    t_push,
                    cmd,
                );
            }
        }
    }
}

/// Push magnitude crossover: does a bigger disturbance find a regime where a
/// corrected MPC plan survives and the open-loop baseline does not?
///
/// The 8-phase push battery (12 N x 0.12 s, 1.44 N*s) never found daylight
/// between arms: 0/8 falls everywhere, and on flat ground at steady state the
/// open-loop Raibert plan is already near the geometric ceiling
/// `max_step/(T*duty)`, so a predictive layer has nothing to add. That impulse
/// was chosen to be survivable, not to be adversarial. This raises the
/// magnitude at the two phases that were worst in that battery (p1, p7 --
/// `namiashi_mpc_plan_under_push` had A at 11.8/13.0 deg and C at 10.5/12.1 deg
/// there) to look for the crossover -- the point where geometric footstep
/// timing alone can no longer recover but a plan that reasons about the whole
/// body's momentum still can. If no crossover appears before both arms fail
/// together, disturbance rejection is not where this MPC earns its keep either,
/// and the honest place left to look is uneven terrain, not bigger flat-ground
/// shoves.
///
///   A  production `GaitMode::Mpc` (SRBD, the tuned gaits' baseline)
///   C  FullCentroidal, sparse QP at 0.900 s, `r_diag[12..24] = 1e-3` (the
///      cost-ratio fix from `namiashi_fcm_jointv_cost_confirm`)
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_push_magnitude_crossover() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 11.0,
        wbc_real_inertia: true,
        ..namiashi_tuned_params(I)
    };
    let arms: Vec<(&str, Box<dyn Fn() -> WbcParams>)> = vec![
        ("A srbd", Box::new(base)),
        (
            "C fcm fixed",
            Box::new(move || WbcParams {
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_horizon_steps: Some(30),
                fcm_sparse_qp: true,
                fcm_jointv_cost: Some(1e-3),
                ..base()
            }),
        ),
    ];
    // Steps 1 and 7 of the 8-phase cycle, the two that were worst for both
    // arms in the earlier battery.
    let phase_steps = [1_usize, 7];
    let magnitudes_n = [12.0, 14.0, 16.0, 18.0, 20.0, 28.0, 36.0, 44.0, 52.0];
    for (tag, mk) in arms {
        for &step in &phase_steps {
            let t_push = 6.0 + period * step as f64 / 8.0;
            for &f_n in &magnitudes_n {
                let mut p = mk();
                p.push = Some((t_push, [0.0, f_n, 0.0], 0.12));
                let Some(samples) = run_wbc_sim(p) else { return };
                report_push(
                    &format!("Trot {tag} p{step} F={f_n:.0}N"),
                    &samples,
                    t_push,
                    cmd,
                );
            }
        }
    }
}

/// Is the contact-force task's authority a weight problem or a structural one?
///
/// With the FullCentroidal plan corrected (`r_diag[12..24] = 1e-3`, sparse QP at
/// 0.900 s) the MPC now asks for something physically sensible: vertical force
/// on mg, fore-aft -4.5 N, off the friction cone. The WBC still does not deliver
/// it -- its solved fore-aft force is -1.15 N and the feet produce +0.27 N, the
/// opposite sign.
///
/// Two explanations, and they call for different work:
///
///   - Weights. `contact_force` is 5.0 against `base_accel`'s 200.0. If the
///     task is simply outbid, raising it must move the delivered force.
///   - Structure. `contact_force` sits at priority 2, underneath BaseAccel and
///     SwingLeg at priority 1, and can only move inside the null space of
///     whatever they have already decided. If priority 1 leaves no freedom, the
///     contact force is fully determined before the task is ever consulted and
///     its weight cannot matter at all.
///
/// The sweep separates them. Four decades of `contact_force`, including zero --
/// if the solved force is the same at 0 and at 5000, the task is inert and the
/// answer is structural. The last two arms drop `base_accel` by 100x, which is
/// the only handle on how much room priority 1 leaves behind.
#[test]
#[ignore = "6 sims -- run with --ignored"]
fn namiashi_contact_force_authority() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        wbc_real_inertia: true,
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        fcm_horizon_steps: Some(30),
        fcm_sparse_qp: true,
        fcm_jointv_cost: Some(1e-3),
        ..namiashi_tuned_params(I)
    };
    let arms: Vec<(&str, WbcParams)> = vec![
        ("cf=5 default", base()),
        ("cf=0", WbcParams { contact_force_weight: Some(0.0), ..base() }),
        ("cf=100", WbcParams { contact_force_weight: Some(100.0), ..base() }),
        ("cf=5000", WbcParams { contact_force_weight: Some(5000.0), ..base() }),
        (
            "ba=2 cf=5",
            WbcParams { base_accel_weight: Some(2.0), ..base() },
        ),
        (
            "ba=2 cf=5000",
            WbcParams {
                base_accel_weight: Some(2.0),
                contact_force_weight: Some(5000.0),
                ..base()
            },
        ),
    ];
    for (tag, params) in arms {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// Does a correct MPC plan finally earn its keep under a disturbance?
///
/// This is the question the whole MPC thread has been circling. On flat ground
/// an open-loop Raibert plan is already near the geometric ceiling
/// `max_step/(T*duty)`, so there is nothing there for a predictive layer to
/// win, and every tracking number in this file has said so. A push is the one
/// place a plan that looks ahead should beat one that does not.
///
/// Three arms, 8 push phases each, because
/// `namiashi_push_phase_dependence` established that which foot pair is loaded
/// when the impulse lands decides the outcome -- a single push time would be
/// reporting an accident of timing as a property of the controller.
///
///   A  production: `GaitMode::Mpc`, the SRBD path the tuned gaits ship with.
///   B  FullCentroidal, sparse QP at 0.900 s, default `r_diag`. The QP solves
///      every tick but the plan is pinned to the friction cone: |fx|/fz = 0.486
///      against mu = 0.50, vertical force 39.6 N against a 32.4 N weight.
///   C  same, with `r_diag[12..24] = 1e-3` so the joint_v cost meets the GRF
///      cost. Plan off the cone, vertical force on mg, fore-aft -4.5 N.
///
/// If C does not beat B and A here, then a correct plan is not what the
/// disturbance response was missing, and the remaining suspect is what happens
/// downstream of it -- the WBC's contact-force task, which measurably prefers
/// being told to ask for nothing.
#[test]
#[ignore = "24 sims -- run with --ignored"]
fn namiashi_mpc_plan_under_push() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 11.0,
        wbc_real_inertia: true,
        ..namiashi_tuned_params(I)
    };
    let fcm = || WbcParams {
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        fcm_horizon_steps: Some(30),
        fcm_sparse_qp: true,
        ..base()
    };
    // Closures rather than values: `WbcParams` is not `Clone`, and each push
    // phase needs its own copy with a different `push` time.
    let arms: Vec<(&str, Box<dyn Fn() -> WbcParams>)> = vec![
        ("A srbd prod", Box::new(base)),
        ("B fcm cone", Box::new(fcm)),
        (
            "C fcm fixed",
            Box::new(move || WbcParams { fcm_jointv_cost: Some(1e-3), ..fcm() }),
        ),
    ];
    for (tag, mk) in arms {
        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let mut p = mk();
            p.push = Some((t_push, [0.0, 12.0, 0.0], 0.12));
            let Some(samples) = run_wbc_sim(p) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

/// Confirms that the GRF-vs-joint_v cost imbalance is what pins the plan to
/// the friction cone.
///
/// `namiashi_fcm_grf_cost_ratio` showed that raising `r_diag[0..12]` takes the
/// plan off the cone while scaling `q_diag` down by the same factor does not.
/// The reason is that `fcm_grf_cost` moves only the GRF half of `r_diag`: the
/// joint_v half stays at its 1.0 default. So "r = 1" is not "GRF cost equals
/// state cost", it is "GRF cost equals joint_v cost".
///
/// If that is the mechanism, lowering joint_v to meet GRF must do the same
/// thing as raising GRF to meet joint_v, and raising joint_v further must make
/// it worse.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_fcm_jointv_cost_confirm() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        wbc_real_inertia: true,
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        fcm_horizon_steps: Some(30),
        fcm_sparse_qp: true,
        ..namiashi_tuned_params(I)
    };
    let arms: Vec<(&str, WbcParams)> = vec![
        ("jv=1e-3 meet grf", WbcParams { fcm_jointv_cost: Some(1e-3), ..base() }),
        ("jv=100 worse?", WbcParams { fcm_jointv_cost: Some(100.0), ..base() }),
    ];
    for (tag, params) in arms {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// Is the plan's friction-cone saturation a cost-ratio problem?
///
/// With the sparse QP the numerics are out of the way, and what is left is a
/// plan pinned to the cone at every horizon: |fx|/fz = 0.486..0.489 against
/// mu = 0.50, with the mean planned vertical force 39.6 N against a 32.4 N
/// weight while the trunk holds 0.233 m.
///
/// The production weights are `r_diag[0..12] = 1e-3` on GRF against
/// `q_diag[0..12] = 1.0` on the body states -- a thousand to one. At that ratio
/// force is nearly free, so the QP buys any state-error reduction with as much
/// of it as the cone allows. That would also explain why zeroing individual Q
/// blocks did nothing in `namiashi_fcm_fore_aft_attribution`: whatever block
/// was left still dominated 1e-3.
///
/// If the ratio is the cause, then raising `r_diag` and lowering `q_diag` must
/// have the same effect, which is what the last arm checks. If only one of them
/// moves the plan, the ratio is not the story.
///
/// All arms run the sparse QP at 0.900 s, where the plan is a converged
/// solution rather than a failure fallback.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_fcm_grf_cost_ratio() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        wbc_real_inertia: true,
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        fcm_horizon_steps: Some(30),
        fcm_sparse_qp: true,
        ..namiashi_tuned_params(I)
    };
    let mut arms: Vec<(String, WbcParams)> =
        vec![("r=1e-3 (default)".to_string(), base())];
    for w in [1e-2, 1e-1, 1.0, 10.0] {
        arms.push((
            format!("r={w}"),
            WbcParams { fcm_grf_cost: Some(w), ..base() },
        ));
    }
    // The other side of the same ratio: leave GRF at 1e-3 and drop the body
    // state weights by 1000x instead. `fcm_state_cost` is
    // `[v_com, omega, base_pos, euler, joint_q]`; joint_q defaults to 0.1, so
    // scale it by the same factor rather than setting it equal to the others.
    // The decisive one. `fcm_grf_cost` moves only `r_diag[0..12]`; the joint_v
    // half stays at 1.0. So `r=1` is not "GRF cost equals state cost", it is
    // "GRF cost equals joint_v cost" -- and if that is what matters, then
    // lowering joint_v to meet GRF must do the same thing as raising GRF to
    // meet joint_v, while raising joint_v further must make it worse.
    arms.push((
        "jv=1e-3 (meet grf)".to_string(),
        WbcParams { fcm_jointv_cost: Some(1e-3), ..base() },
    ));
    arms.push((
        "jv=100 (worse?)".to_string(),
        WbcParams { fcm_jointv_cost: Some(100.0), ..base() },
    ));
    arms.push((
        "q/1000 instead".to_string(),
        WbcParams {
            fcm_state_cost: Some([1e-3, 1e-3, 1e-3, 1e-3, 1e-4]),
            ..base()
        },
    ));
    for (tag, params) in arms {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// Condensed against sparse QP, at horizons the condensed form cannot reach.
///
/// `namiashi_fcm_fore_aft_attribution` established that the FullCentroidal
/// plan has two failure modes and neither answers to a cost weight: at a
/// 0.300 s horizon the QP solves but pins itself to the friction cone
/// (|fx|/fz = 0.489 against mu = 0.50, with fz 23% over mg to buy the
/// headroom), and at 0.900 s it returns `NumericalError` on 2208 of 2220
/// solves. The second is a property of the condensed formulation: it builds
/// `A_x[k] = A_k...A_0` and `B_u[k][j] = A_k...A_{j+1} B_j` explicitly, and at
/// dt 0.030 with an angular block near 1/0.027 those powers overflow the
/// conditioning of `B_u' Q B_u`.
///
/// The sparse form keeps the states as decision variables, so no power of
/// `A_d` is ever formed. `sparse_and_condensed_agree_at_short_horizon` checks
/// the two describe the same problem where both are sound; this measures what
/// the difference buys on the robot.
///
/// The last row is legged_control's own horizon (66 x 0.015 = 1.0 s), which is
/// the setting this port has never been able to run.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_fcm_sparse_vs_condensed() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        wbc_real_inertia: true,
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        ..namiashi_tuned_params(I)
    };
    let arms: Vec<(&str, WbcParams)> = vec![
        ("cond 0.30s", WbcParams { fcm_horizon_steps: Some(10), ..base() }),
        (
            "sparse 0.30s",
            WbcParams { fcm_horizon_steps: Some(10), fcm_sparse_qp: true, ..base() },
        ),
        ("cond 0.90s", WbcParams { fcm_horizon_steps: Some(30), ..base() }),
        (
            "sparse 0.90s",
            WbcParams { fcm_horizon_steps: Some(30), fcm_sparse_qp: true, ..base() },
        ),
        (
            "sparse 1.0s lc dt",
            WbcParams {
                fcm_horizon_steps: Some(66),
                fcm_dt_per_step: Some(0.015),
                fcm_sparse_qp: true,
                ..base()
            },
        ),
    ];
    for (tag, params) in arms {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// Where the FullCentroidal MPC's -19 N of fore-aft plan comes from.
///
/// ANSWER (measured): the friction cone, not any cost weight. Every arm below
/// lands at |fx|/fz = 0.489..0.495 against mu = 0.50 -- the solution is pinned
/// to the cone boundary, so the state weights cannot move it, and the vertical
/// force runs 23% above mg because inflating fz is how the QP buys cone
/// headroom for the horizontal force it wants.
///
/// The long-horizon arms (F, G) do not answer to the reference either. With
/// `QG_MPC_SOLVE_LOG=1`: 5550/5550 Solved at 0.300 s, and 2208/2220
/// NumericalError + 12/2220 InsufficientProgress at 0.900 s -- not one success.
/// `failed_solution` hands the WBC a zeroed `first_input`, which is why the
/// planned force reads +0.03 N. So the collapse is a numerical failure in the
/// condensed QP, not a consequence of what the reference contains: arm F
/// (gamma on) collapses exactly as hard as arm G (gamma off).
///
/// `namiashi_wiring_repair_ladder` measured the repaired FullCentroidal path
/// planning -19.45 N of fore-aft force and +39.79 N of vertical force on a
/// 3.3 kg robot (weight 32.4 N) whose feet deliver +0.255 N and whose trunk
/// holds 0.236 m. The plan is not dimensionally credible, and the mode walks
/// well anyway, so something in the cost is being paid for with GRF that has
/// nothing to do with the ground.
///
/// Two candidates, both in `q_diag`, which defaults to `[1.0; 24]` across five
/// different physical units:
///
///   - `joint_q` (12 of the 24 entries, half the state cost). The reference
///     holds joint_q at its current value for the whole horizon unless
///     `dynamic_joint_q_reference` is on, so every swing leg is a growing
///     "error" the MPC is charged for. Its only actuator is GRF, and the
///     `d(alpha)/d(joint_q)` coupling gives it a path -- so it can pour force
///     into stopping a leg from swinging.
///   - `base_pos`. Not the accumulating-reference story: the reference is
///     rebuilt from the current state every cycle
///     (`full_centroidal_controller.rs:1434`), exactly as legged_control does,
///     so the position error starts at zero each solve and cannot integrate.
///     Still worth zeroing, to separate it from the joint_q term.
///
/// Runs at the default 10 x 0.030 horizon, because the question is which cost
/// term the force answers to, not how far ahead it looks.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_fcm_fore_aft_attribution() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    let _ = period;
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        wbc_real_inertia: true,
        gait_mode: GaitMode::FullCentroidal,
        legged_control_parity: true,
        ..namiashi_tuned_params(I)
    };
    let arms: Vec<(&str, WbcParams)> = vec![
        ("A all q=1", base()),
        (
            "B joint_q=0",
            WbcParams { fcm_state_cost: Some([1.0, 1.0, 1.0, 1.0, 0.0]), ..base() },
        ),
        (
            "C dyn joint_q ref",
            WbcParams { dynamic_joint_q_reference: true, ..base() },
        ),
        (
            "D base_pos=0",
            WbcParams { fcm_state_cost: Some([1.0, 1.0, 0.0, 1.0, 1.0]), ..base() },
        ),
        (
            "E both=0",
            WbcParams { fcm_state_cost: Some([1.0, 1.0, 0.0, 1.0, 0.0]), ..base() },
        ),
        // The combination the ladder implies should work. Extending the horizon
        // alone drove the planned vertical force to +1.58 N against a 32.4 N
        // weight -- past about a third of a gait cycle the frozen-joint_q
        // reference is not a motion the robot can perform, so linearizing about
        // it produces a model in which asking for no force is optimal. gamma
        // makes each horizon step an IK solution of the actual footstep plan,
        // which is the only thing that makes a long horizon meaningful here.
        (
            "F dyn ref + 0.9s",
            WbcParams {
                dynamic_joint_q_reference: true,
                fcm_horizon_steps: Some(30),
                ..base()
            },
        ),
        (
            "G horizon 0.9s only",
            WbcParams { fcm_horizon_steps: Some(30), ..base() },
        ),
    ];
    for (tag, params) in arms {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }
}

/// layer to win.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_wiring_repair_ladder() {
    const I: usize = 0; // Trot
    let (_, period, .., cmd) = NAMIASHI_TUNED[I];
    // One rung per fix, each keeping the ones below it. `Actuation::Torque`
    // throughout, because it is the path where the WBC's torque is the
    // command rather than a discarded argument.
    let base = || WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 12.0,
        ..namiashi_tuned_params(I)
    };
    let rungs: Vec<(&str, WbcParams)> = vec![
        ("0 baseline", base()),
        ("1 wbc real m,I", WbcParams { wbc_real_inertia: true, ..base() }),
        (
            "2 + mpc composite I",
            WbcParams {
                wbc_real_inertia: true,
                mpc_composite_inertia: true,
                ..base()
            },
        ),
        (
            "3 + horizon 0.9s",
            WbcParams {
                wbc_real_inertia: true,
                mpc_composite_inertia: true,
                mpc_horizon_steps: Some(30),
                ..base()
            },
        ),
        (
            "4 + q_diag[p_x]",
            WbcParams {
                wbc_real_inertia: true,
                mpc_composite_inertia: true,
                mpc_horizon_steps: Some(30),
                mpc_px_cost: Some(1000.0),
                ..base()
            },
        ),
        // FullCentroidal is its own ladder, not a continuation of the one
        // above. `set_srbd_mpc_config` is a silent no-op outside
        // `GaitMode::Mpc` (generator.rs:334), so rungs 2-4 cannot reach this
        // mode at all -- carrying their flags down here would label the rows
        // with repairs that were never applied. It does not need rungs 1-2
        // either: FullCentroidal already auto-detects m=3.300 kg and the
        // composite inertia (0.02722, 0.07575, 0.06584), which is most of why
        // it starts out ahead of the SRBD path.
        //
        // What it does still differ from legged_control on is the horizon --
        // 10 x 0.030 = 0.300 s against 1.0 s -- and that is what rungs 6-7
        // measure. Rung 9 separates the warm start from the iteration count,
        // because `fcm_warm_start` changes both at once.
        (
            "5 fcm (parity)",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                ..base()
            },
        ),
        (
            "6 + horizon 0.9s",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_horizon_steps: Some(30),
                ..base()
            },
        ),
        (
            "7 + lc dt 0.015",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_horizon_steps: Some(66),
                fcm_dt_per_step: Some(0.015),
                ..base()
            },
        ),
        // Against rung 5, not rung 7. Stacked on 66 x 0.015 this arm stalls --
        // 100% CPU with no simulated progress for 45 s -- and the plan it would
        // be weighting has already collapsed to +0.10 N, so there is nothing
        // there left to measure.
        (
            "8 jv weight",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_taskspace_jv_weight: Some([1.0, 1.0, 1.0]),
                ..base()
            },
        ),
        (
            "9 sqp=1 only",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_sqp_iterations: Some(1),
                ..base()
            },
        ),
        (
            "10 + warm start",
            WbcParams {
                wbc_real_inertia: true,
                gait_mode: GaitMode::FullCentroidal,
                legged_control_parity: true,
                fcm_warm_start: true,
                ..base()
            },
        ),
    ];
    for (tag, params) in rungs {
        let Some(samples) = run_wbc_sim(params) else { return };
        report_walk(&format!("Trot {tag}"), &samples, cmd, 1.0);
    }

    // The top rung under a push, since disturbance is where a predictive
    // layer is supposed to earn its place.
    eprintln!("---- push, 8 phases: baseline vs repaired ----");
    // Three arms, not two: the SRBD rung with the best flat-ground drift and
    // the FullCentroidal rung with the best roll disagree about which repair
    // helped, and a push is the one place the difference could show up.
    for (tag, repaired) in [("baseline", 0), ("srbd", 3), ("fcm", 5)] {
        for step in 0..8 {
            let t_push = 6.0 + period * step as f64 / 8.0;
            let mut p = base();
            p.push = Some((t_push, [0.0, 12.0, 0.0], 0.12));
            p.total_time_s = 11.0;
            if repaired > 0 {
                p.wbc_real_inertia = true;
                p.mpc_composite_inertia = true;
                p.mpc_horizon_steps = Some(30);
                p.mpc_px_cost = Some(1000.0);
            }
            if repaired == 5 {
                p.gait_mode = GaitMode::FullCentroidal;
                p.legged_control_parity = true;
            }
            let Some(samples) = run_wbc_sim(p) else { return };
            report_push(&format!("Trot {tag} p{step}"), &samples, t_push, cmd);
        }
    }
}

fn assert_forward_command_advances_body(samples: &[WbcSample]) {
    // Skip burn-in for tilt + displacement metrics.
    let dt: f64 = 0.002;
    let burn_in_steps = (0.5 / dt).round() as usize;
    let walk = &samples[burn_in_steps..];

    // No fall during the entire run (including burn-in).
    let min_z = samples.iter().map(|s| s.body_z).fold(f64::INFINITY, f64::min);
    assert!(
        min_z > TRUNK_Z_FALL_THRESHOLD_M,
        "forward walk: trunk fell, min_z = {min_z:.3} m",
    );

    // Forward displacement during the walking window.
    let x_start = walk.first().map(|s| s.body_x).unwrap_or(0.0);
    let x_end = walk.last().map(|s| s.body_x).unwrap_or(0.0);
    let dx = x_end - x_start;
    eprintln!(
        "[wbc] forward_command: Δx = {dx:.3} m over {:.1} s \
         (threshold ≥ {:.2})",
        walk.last().map(|s| s.t).unwrap_or(0.0)
            - walk.first().map(|s| s.t).unwrap_or(0.0),
        MIN_DISPLACEMENT_M,
    );
    assert!(
        dx >= MIN_DISPLACEMENT_M,
        "forward walk: Δx = {dx:.3} m < {} m — WBC produced near-zero \
         net forward motion (足踏み regression?)",
        MIN_DISPLACEMENT_M,
    );

    // Bounded body tilt during walking. WBC + Position-PD with
    // gravity-comp should keep the trunk roughly level on flat ground.
    let peak_roll = walk
        .iter()
        .map(|s| s.roll.abs())
        .fold(0.0_f64, f64::max);
    let peak_pitch = walk
        .iter()
        .map(|s| s.pitch.abs())
        .fold(0.0_f64, f64::max);
    eprintln!(
        "[wbc] forward_command: peak |roll| = {peak_roll:.2} rad, \
         peak |pitch| = {peak_pitch:.2} rad",
    );
    // 0.5 rad ≈ 30° — generous because WBC q[3..7] = identity means
    // tilt feedback is approximate.
    assert!(
        peak_roll < 0.5,
        "forward walk: |roll| peak {peak_roll:.2} rad too large",
    );
    assert!(
        peak_pitch < 0.5,
        "forward walk: |pitch| peak {peak_pitch:.2} rad too large",
    );
}

/// Sanity check for the staircase environment at three rise heights, loads,
/// stands, doesn't fall while still on the approach floor, and writes a
/// replay per rise so the geometry can be looked at rather than trusted.
///
/// 0.10 m is a large step for this robot: ~43% of namiashi's tuned stance
/// height (0.235 m) and 2-3x the swing clearance the gait planner budgets on
/// flat ground (0.035-0.045 m, terrain-blind everywhere in this codebase).
/// 0.05 m and 0.02 m are added to see where that clearance mismatch stops
/// being the dominant effect -- 0.02 m sits below the swing clearance
/// entirely. `run_m` and `n_steps` are held fixed across the three so rise is
/// the only thing that changes. Whether any of them can be climbed is an open
/// question this test does not answer -- it only confirms the terrain loads
/// correctly and the robot survives the approach up to the first riser.
#[test]
#[ignore = "writes a replay per rise for visual inspection -- run with --ignored"]
fn namiashi_staircase_environment_smoke() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    for (tag, rise_m) in [("rise_10", 0.10), ("rise_05", 0.05), ("rise_02", 0.02)] {
        let stairs = StaircaseCfg {
            rise_m,
            run_m: 0.20,
            n_steps: 10,
            approach_m: 1.5,
            top_platform_m: 1.5,
            half_width_m: 1.0,
        };
        eprintln!(
            "[stairs {tag}] rise={:.2}m run={:.2}m steps={}  total_rise={:.2}m  \
             first riser at x={:.2}m  top platform x=[{:.2}, {:.2}]m",
            stairs.rise_m,
            stairs.run_m,
            stairs.n_steps,
            stairs.total_rise_m(),
            stairs.approach_m,
            stairs.top_platform_start_x(),
            stairs.top_platform_start_x() + stairs.top_platform_m,
        );
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 6.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            replay_dir: Some(format!("/tmp/nami_stairs/{tag}")),
            ..namiashi_tuned_params(I)
        };
        let approach_end_x = stairs.approach_m;
        let Some(samples) = run_wbc_sim(params) else { return };
        let min_z_on_approach = samples
            .iter()
            .filter(|s| s.body_x < approach_end_x - 0.05)
            .map(|s| s.body_z)
            .fold(f64::INFINITY, f64::min);
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        eprintln!(
            "[stairs {tag}] min_z on approach = {min_z_on_approach:.3} m   reached x = {max_x:.3} m"
        );
        assert!(
            min_z_on_approach > TRUNK_Z_FALL_THRESHOLD_M,
            "[{tag}] fell while still on the approach floor: min_z = {min_z_on_approach:.3} m",
        );
    }
}

/// Does the 2 cm-rise staircase actually get climbed?
///
/// `namiashi_staircase_environment_smoke`'s 2 cm run reached x=2.815 m in 6 s
/// -- more progress than 5 cm or 10 cm -- but its own trace showed a lateral
/// drift accelerating once it started climbing, and it walked off that test's
/// 2 m-wide platform (half_width_m=1.0) and free-fell before reaching the top.
/// A first widen (half_width 3.0 m, top platform 2.0 m, 14 s) climbed cleanly
/// -- z rose smoothly to the expected plateau (base stance 0.235 m + 0.20 m
/// total rise = 0.435 m) by x=3.7 m, right at the top platform's start -- but
/// then walked off the *far* end of that platform too and free-fell again,
/// this time purely because the platform was too short for how far it kept
/// walking with a persistent heading drift.
///
/// This is the version sized from that measurement (half_width 6.0 m, top
/// platform 8.0 m, 16 s), and it holds: final position (x=11.27, y=-3.80,
/// z=0.436) is stable on the platform, not mid-fall. The lateral drift itself
/// turns out not to be a runaway -- it decelerates once past the last riser
/// (roughly -0.38 m/s while climbing, down to -0.07 m/s by the second half of
/// the flat-top walk) and looks like the same kind of disturbance-response
/// settling the push tests measure, just stretched over ten small
/// perturbations (one per riser) instead of one impulse.
///
/// So: climbable, cleanly, with a real but self-correcting lateral
/// disturbance along the way. 5 cm and 10 cm did not even engage the stairs
/// within 6 s in the three-rise smoke test (they bounced off the first riser
/// in place) -- whether they climb given more time, or cannot climb at all,
/// is still open and is not what this test measures.
#[test]
#[ignore = "writes a replay for visual inspection -- run with --ignored"]
fn namiashi_staircase_rise02_wide_platform() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.02,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    eprintln!(
        "[stairs wide] rise={:.2}m run={:.2}m steps={}  total_rise={:.2}m  \
         first riser at x={:.2}m  top platform x=[{:.2}, {:.2}]m  half_width={:.1}m",
        stairs.rise_m,
        stairs.run_m,
        stairs.n_steps,
        stairs.total_rise_m(),
        stairs.approach_m,
        stairs.top_platform_start_x(),
        stairs.top_platform_start_x() + stairs.top_platform_m,
        stairs.half_width_m,
    );
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 16.0,
        wbc_real_inertia: true,
        staircase: Some(stairs),
        replay_dir: Some("/tmp/nami_stairs/rise_02_wide".to_string()),
        ..namiashi_tuned_params(I)
    };
    let approach_end_x = stairs.approach_m;
    let top_start_x = stairs.top_platform_start_x();
    let Some(samples) = run_wbc_sim(params) else { return };
    let min_z_on_approach = samples
        .iter()
        .filter(|s| s.body_x < approach_end_x - 0.05)
        .map(|s| s.body_z)
        .fold(f64::INFINITY, f64::min);
    let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
    let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
    let final_s = samples.last().unwrap();
    let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
    eprintln!(
        "[stairs wide] min_z on approach = {min_z_on_approach:.3} m   \
         reached x = {max_x:.3} m   max z = {max_z:.3} m   \
         final (x,y,z) = ({:.3}, {:.3}, {:.3}) m   reached top = {reached_top}",
        final_s.body_x, final_s.body_y, final_s.body_z,
    );
    assert!(
        min_z_on_approach > TRUNK_Z_FALL_THRESHOLD_M,
        "fell while still on the approach floor: min_z = {min_z_on_approach:.3} m",
    );
    assert!(
        reached_top,
        "did not end the run standing on the top platform: final z = {:.3} m at x = {:.3} m",
        final_s.body_z, final_s.body_x,
    );
}

/// Is the 5 cm-rise staircase's failure a swing-clearance problem, or does it
/// need foothold placement (perception, planning) to get anywhere at all?
///
/// The fixed swing-height schedule this codebase uses everywhere is
/// terrain-blind: 0.040 m on Trot, chosen for flat ground, with no idea what
/// is in front of the foot. A 5 cm riser is taller than that clearance, so a
/// foot on the open-loop trajectory can catch the riser face before finishing
/// its swing regardless of how good the horizontal touchdown target is.
///
/// This sweeps `swing_height_m` -- still blind, still one scalar, still the
/// same schedule for every step including the flat approach -- to see whether
/// clearance alone is the blocker. If some value gets the robot onto the
/// stairs and progressing, the horizontal (open-loop Raibert) placement was
/// already good enough and the missing piece was vertical clearance, not
/// perception. If nothing does, the failure is not (only) clearance and a
/// foothold planner would need to do more than just clear the riser.
///
/// Uses the same wide-platform methodology `namiashi_staircase_rise02_wide_
/// platform` needed to avoid conflating "fell off the test track" with
/// "failed to climb."
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_staircase_5cm_swing_clearance_sweep() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    for swing_h in [0.040_f64, 0.060, 0.080, 0.100, 0.120] {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            swing_height_m: Some(swing_h),
            replay_dir: Some(format!("/tmp/nami_stairs/rise05_swing_{swing_h:.3}")),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        eprintln!(
            "[stairs 5cm swing={swing_h:.3}] reached x={max_x:.3}m  max z={max_z:.3}m  \
             final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
            final_s.body_x, final_s.body_y, final_s.body_z,
        );
    }
}

/// Does slowing the gait + widening the swing arc (both taken directly from
/// a namiashi RL policy that DOES clear the 5 cm staircase, measured via
/// sim-to-sim replay in MuJoCo -- see go2_rl/namiashi_rl -- not guessed)
/// get further than `namiashi_staircase_5cm_swing_clearance_sweep`'s own
/// swing-height-alone sweep did?
///
/// The RL policy's own recorded joint trace, compared against its
/// steady-state flat-ground reference and against this WBC's own 5 cm
/// climb attempt, showed three things the WBC has never tried in
/// combination:
///   1. gait period 0.320s (Trot) -> 0.41-0.44s, self-selected, on the
///      approach AND while climbing -- never swept together with a wider
///      swing arc, only alone (`namiashi_prop_retune_trot_crawl`, which
///      only measured flat-ground speed tracking, not climbing).
///   2. calf (knee) joint range roughly DOUBLES relative to the flat
///      steady-state Trot reference (0.42 rad -> 0.95-1.09 rad) --
///      `swing_height_m` is a foot-space clearance knob, not a joint-space
///      one, but it is this test's only available lever toward that same
///      effect, so it is swept wider here (up to 0.20 m, above the
///      earlier sweep's 0.12 m ceiling) alongside the slower period.
///   3. yaw drift stays under 11 deg over the whole climb, vs. this WBC's
///      own 23+ deg in the first 6 s alone at the standard 0.320s/0.04m
///      settings -- tested separately in
///      `namiashi_staircase_5cm_rl_inspired_hip_bias` (the L-R hip-angle
///      asymmetry finding), not folded in here, to keep this sweep's
///      result attributable to period+swing alone.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_staircase_5cm_rl_inspired_period_swing() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    for period in [0.320_f64, 0.420, 0.450] {
        for swing_h in [0.040_f64, 0.080, 0.120, 0.160, 0.200] {
            let params = WbcParams {
                actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 20.0,
                wbc_real_inertia: true,
                staircase: Some(stairs),
                cycle_period_s: Some(period),
                swing_height_m: Some(swing_h),
                replay_dir: Some(format!("/tmp/nami_stairs/rise05_rl_period{period:.3}_swing{swing_h:.3}")),
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
            let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
            let final_s = samples.last().unwrap();
            let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
            eprintln!(
                "[stairs 5cm period={period:.3} swing={swing_h:.3}] reached x={max_x:.3}m  max z={max_z:.3}m  \
                 final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
                final_s.body_x, final_s.body_y, final_s.body_z,
            );
        }
    }
}

/// Does a constant left-right hip-roll asymmetry -- taken directly from an
/// RL policy's own recorded joint trace on a successful 5cm climb (see
/// `hip_lr_bias_rad`'s doc comment) -- buy any yaw/roll stability on the
/// standard 5cm staircase? Swept both signs (the IK/RL sign convention
/// correspondence was not independently verified) and several magnitudes
/// around the measured +0.07..+0.16 rad range, at the standard 0.320s/
/// 0.04m gait (isolated from `namiashi_staircase_5cm_rl_inspired_period_
/// swing`'s period/swing changes, so any effect here is attributable to
/// the hip bias alone).
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_staircase_5cm_rl_inspired_hip_bias() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    for bias in [-0.16_f64, -0.10, -0.05, 0.05, 0.10, 0.16] {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            hip_lr_bias_rad: Some(bias),
            replay_dir: Some(format!("/tmp/nami_stairs/rise05_rl_hipbias_{bias:+.3}")),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        eprintln!(
            "[stairs 5cm hip_bias={bias:+.3}] reached x={max_x:.3}m  max z={max_z:.3}m  \
             final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
            final_s.body_x, final_s.body_y, final_s.body_z,
        );
    }
}

/// Tiny xorshift64* PRNG -- avoids adding the `rand` crate as a dependency
/// for what is otherwise a handful of `next_f64`/`next_gauss` calls.
struct Xorshift64(u64);
impl Xorshift64 {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// Uniform in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Standard normal via Box-Muller.
    fn next_gauss(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// A constant hip-lr bias did not help
/// (`namiashi_staircase_5cm_rl_inspired_hip_bias`). This gates the SAME
/// correction (`hip_bias_gate`) on a detected riser disturbance instead of
/// applying it constantly, and rather than guessing the gate's three
/// parameters (`trigger_m`, `bias_mag`, `duration_s`), searches for them
/// with a small gradient-free (1+K) hill-climber: each generation samples
/// K perturbations of the current best around a shrinking Gaussian radius,
/// evaluates each via `run_wbc_sim`'s own `max_x` (how far up the 5cm
/// staircase the run got before falling/timing out), and keeps whichever
/// scored highest -- reached_top acts as an early-stop, not the fitness
/// itself, since max_x still orders "got further but didn't quite finish"
/// runs usefully while `reached_top` alone would not.
///
/// This is deliberately NOT a neural-network policy: the three numbers
/// this converges to ARE the model -- no separate distillation step from
/// a learned policy into an interpretable rule is needed, by construction.
#[test]
#[ignore = "optimization run -- slow, run with --ignored"]
fn namiashi_staircase_5cm_hip_gate_search() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();

    let eval = |trigger_m: f64, bias_mag: f64, duration_s: f64| -> (f64, bool) {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            hip_bias_gate: Some(HipBiasGateCfg {
                trigger_m,
                bias_mag,
                bias_gain: 0.0,
                max_bias_rad: bias_mag,
                duration_s,
            }),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return (f64::NEG_INFINITY, false) };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        (max_x, reached_top)
    };

    // Center on ContactReflexCfg's own established trigger (0.065 m) and
    // the earlier constant-bias sweep's tested range for bias_mag.
    let mut center = (0.065_f64, 0.10_f64, 0.30_f64);
    let bounds = [(0.02, 0.15), (0.0, 0.30), (0.05, 1.0)];
    let mut best = eval(center.0, center.1, center.2);
    eprintln!("[hip_gate_search] gen=0 (seed) params={center:?} max_x={:.3} reached_top={}", best.0, best.1);
    if best.1 {
        return;
    }

    let mut rng = Xorshift64(0xC0FFEE_u64.wrapping_mul(2654435761).wrapping_add(1));
    const GENERATIONS: usize = 6;
    const CANDIDATES_PER_GEN: usize = 6;
    for g in 1..=GENERATIONS {
        // Shrinking search radius: starts at ~40% of each param's own
        // range, halves every 2 generations.
        let shrink = 0.5_f64.powf((g - 1) as f64 / 2.0);
        let sigmas = [
            (bounds[0].1 - bounds[0].0) * 0.4 * shrink,
            (bounds[1].1 - bounds[1].0) * 0.4 * shrink,
            (bounds[2].1 - bounds[2].0) * 0.4 * shrink,
        ];
        let mut gen_best: Option<(f64, bool, (f64, f64, f64))> = None;
        for _ in 0..CANDIDATES_PER_GEN {
            let cand = (
                (center.0 + sigmas[0] * rng.next_gauss()).clamp(bounds[0].0, bounds[0].1),
                (center.1 + sigmas[1] * rng.next_gauss()).clamp(bounds[1].0, bounds[1].1),
                (center.2 + sigmas[2] * rng.next_gauss()).clamp(bounds[2].0, bounds[2].1),
            );
            let (max_x, reached_top) = eval(cand.0, cand.1, cand.2);
            eprintln!(
                "[hip_gate_search] gen={g} params=(trigger={:.4} bias={:.4} dur={:.3}) max_x={max_x:.3} reached_top={reached_top}",
                cand.0, cand.1, cand.2,
            );
            if gen_best.is_none_or(|(bx, _, _)| max_x > bx) {
                gen_best = Some((max_x, reached_top, cand));
            }
        }
        let (gb_x, gb_top, gb_params) = gen_best.unwrap();
        if gb_x > best.0 {
            best = (gb_x, gb_top);
            center = gb_params;
        }
        eprintln!(
            "[hip_gate_search] gen={g} best-so-far params={center:?} max_x={:.3} reached_top={}",
            best.0, best.1,
        );
        if best.1 {
            eprintln!("[hip_gate_search] reached_top -- stopping early");
            return;
        }
    }
    eprintln!(
        "[hip_gate_search] DONE. best params={center:?}  max_x={:.3}  reached_top={}",
        best.0, best.1,
    );
}

/// Extends `namiashi_staircase_5cm_hip_gate_search`'s converged rule
/// (trigger=0.0297 m, bias=0.273 rad, dur=0.052 s -> max_x=2.857 m,
/// reached_top=false, ~68% up the stairs -- the best model-based number
/// this whole investigation produced, but still short of the platform)
/// with a 4th parameter, `bias_gain`: scale the correction by how far past
/// the trigger the FK error actually was, rather than applying the same
/// fixed nudge to a small brush and a hard collision alike. Re-searches
/// all 4 parameters together from a seed centered on v1's optimum
/// (`bias_gain=0` recovers it exactly), per the user's explicit request to
/// extend the gate rule and re-search rather than stop at v1 or bring in
/// unrelated RL-derived static parameters.
#[test]
#[ignore = "optimization run -- slow, run with --ignored"]
fn namiashi_staircase_5cm_hip_gate_search_v2() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    const MAX_BIAS_RAD: f64 = 0.35; // fixed safety clamp, not searched

    let eval = |trigger_m: f64, bias_mag: f64, bias_gain: f64, duration_s: f64| -> (f64, bool) {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            hip_bias_gate: Some(HipBiasGateCfg {
                trigger_m,
                bias_mag,
                bias_gain,
                max_bias_rad: MAX_BIAS_RAD,
                duration_s,
            }),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return (f64::NEG_INFINITY, false) };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        (max_x, reached_top)
    };

    // Seed at v1's converged optimum, bias_gain=0 (exactly recovers v1).
    let mut center = (0.0297_f64, 0.2726_f64, 0.0_f64, 0.052_f64);
    let bounds = [(0.02, 0.15), (0.0, 0.30), (0.0, 4.0), (0.05, 1.0)];
    let mut best = eval(center.0, center.1, center.2, center.3);
    eprintln!("[hip_gate_search_v2] gen=0 (seed) params={center:?} max_x={:.3} reached_top={}", best.0, best.1);
    if best.1 {
        return;
    }

    let mut rng = Xorshift64(0xFACADE_u64.wrapping_mul(2654435761).wrapping_add(1));
    const GENERATIONS: usize = 6;
    const CANDIDATES_PER_GEN: usize = 8;
    for g in 1..=GENERATIONS {
        let shrink = 0.5_f64.powf((g - 1) as f64 / 2.0);
        let sigmas = [
            (bounds[0].1 - bounds[0].0) * 0.4 * shrink,
            (bounds[1].1 - bounds[1].0) * 0.4 * shrink,
            (bounds[2].1 - bounds[2].0) * 0.4 * shrink,
            (bounds[3].1 - bounds[3].0) * 0.4 * shrink,
        ];
        let mut gen_best: Option<(f64, bool, (f64, f64, f64, f64))> = None;
        for _ in 0..CANDIDATES_PER_GEN {
            let cand = (
                (center.0 + sigmas[0] * rng.next_gauss()).clamp(bounds[0].0, bounds[0].1),
                (center.1 + sigmas[1] * rng.next_gauss()).clamp(bounds[1].0, bounds[1].1),
                (center.2 + sigmas[2] * rng.next_gauss()).clamp(bounds[2].0, bounds[2].1),
                (center.3 + sigmas[3] * rng.next_gauss()).clamp(bounds[3].0, bounds[3].1),
            );
            let (max_x, reached_top) = eval(cand.0, cand.1, cand.2, cand.3);
            eprintln!(
                "[hip_gate_search_v2] gen={g} params=(trigger={:.4} bias={:.4} gain={:.3} dur={:.3}) \
                 max_x={max_x:.3} reached_top={reached_top}",
                cand.0, cand.1, cand.2, cand.3,
            );
            if gen_best.is_none_or(|(bx, _, _)| max_x > bx) {
                gen_best = Some((max_x, reached_top, cand));
            }
        }
        let (gb_x, gb_top, gb_params) = gen_best.unwrap();
        if gb_x > best.0 {
            best = (gb_x, gb_top);
            center = gb_params;
        }
        eprintln!(
            "[hip_gate_search_v2] gen={g} best-so-far params={center:?} max_x={:.3} reached_top={}",
            best.0, best.1,
        );
        if best.1 {
            eprintln!("[hip_gate_search_v2] reached_top -- stopping early");
            return;
        }
    }
    eprintln!(
        "[hip_gate_search_v2] DONE. best params={center:?}  max_x={:.3}  reached_top={}",
        best.0, best.1,
    );
}

/// Both `namiashi_staircase_5cm_hip_gate_search` and `..._v2`'s "converged"
/// optima turned out to be suspiciously sharp: seeding v2 from v1's winner
/// rounded to 4 decimal places (0.0297 vs the actual 0.029736654...) alone
/// dropped max_x from 2.857 m to 1.643 m -- the same ~1.5-1.7 m plateau
/// nearly every OTHER sampled point in both searches landed on. That is a
/// red flag for a rule meant to generalize (sim-to-sim, real hardware,
/// sensor noise), not just fit one exact deterministic trace, so per the
/// user's explicit request this checks whether v2's winner
/// (trigger=0.11347570313929631, bias=0.27363657175214146,
/// gain=2.012430903480257, dur=0.11419665821391292, max_x=3.175) sits on a
/// genuine basin or another isolated spike, before trusting it as anything
/// more than an artifact of this one search run.
///
/// Three probes, all against the exact same winning point as baseline:
///   1. One-parameter-at-a-time +/-5% and +/-10% perturbations (8 runs) --
///      isolates which single parameter (if any) the result is sensitive to.
///   2. Six independent joint perturbations, all 4 params simultaneously
///      shifted by a uniform +/-10% each (a fresh, unrelated RNG stream) --
///      closer to what "the same rule on a slightly different day" would
///      see than the one-at-a-time probes are.
///   3. Two discretization changes (dt scaled 0.8x and 1.25x, holding
///      host_rate_hz fixed) -- checks whether the result is an artifact of
///      this exact timestep rather than the physical rule itself.
#[test]
#[ignore = "optimization run -- slow, run with --ignored"]
fn namiashi_staircase_5cm_hip_gate_robustness() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    const MAX_BIAS_RAD: f64 = 0.35;
    const BASE: (f64, f64, f64, f64) = (
        0.11347570313929631,
        0.27363657175214146,
        2.012430903480257,
        0.11419665821391292,
    );

    let eval_dt = |trigger_m: f64, bias_mag: f64, bias_gain: f64, duration_s: f64, dt: f64| -> (f64, bool) {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            hip_bias_gate: Some(HipBiasGateCfg {
                trigger_m,
                bias_mag,
                bias_gain,
                max_bias_rad: MAX_BIAS_RAD,
                duration_s,
            }),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return (f64::NEG_INFINITY, false) };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        (max_x, reached_top)
    };
    let eval = |trigger_m: f64, bias_mag: f64, bias_gain: f64, duration_s: f64| {
        eval_dt(trigger_m, bias_mag, bias_gain, duration_s, 0.0005)
    };

    let (base_x, base_top) = eval(BASE.0, BASE.1, BASE.2, BASE.3);
    eprintln!("[hip_gate_robustness] baseline params={BASE:?} max_x={base_x:.3} reached_top={base_top}");

    eprintln!("[hip_gate_robustness] -- probe 1: one-at-a-time +/-5%/+/-10% --");
    let names = ["trigger", "bias", "gain", "dur"];
    let mut probe1_max = f64::NEG_INFINITY;
    let mut probe1_min = f64::INFINITY;
    for (i, name) in names.iter().enumerate() {
        for frac in [-0.10, -0.05, 0.05, 0.10] {
            let mut p = [BASE.0, BASE.1, BASE.2, BASE.3];
            p[i] *= 1.0 + frac;
            let (x, top) = eval(p[0], p[1], p[2], p[3]);
            probe1_max = probe1_max.max(x);
            probe1_min = probe1_min.min(x);
            let pct = frac * 100.0;
            eprintln!(
                "[hip_gate_robustness] {name}{pct:+.0}% params=({:.4},{:.4},{:.3},{:.3}) max_x={x:.3} reached_top={top}",
                p[0], p[1], p[2], p[3],
            );
        }
    }

    eprintln!("[hip_gate_robustness] -- probe 2: 6x joint +/-10% perturbations --");
    let mut rng = Xorshift64(0xB0B0CAFE_u64.wrapping_mul(2654435761).wrapping_add(1));
    let mut probe2_max = f64::NEG_INFINITY;
    let mut probe2_min = f64::INFINITY;
    for _ in 0..6 {
        let mut jitter = |v: f64| v * (1.0 + 0.10 * (2.0 * rng.next_f64() - 1.0));
        let p = [jitter(BASE.0), jitter(BASE.1), jitter(BASE.2), jitter(BASE.3)];
        let (x, top) = eval(p[0], p[1], p[2], p[3]);
        probe2_max = probe2_max.max(x);
        probe2_min = probe2_min.min(x);
        eprintln!(
            "[hip_gate_robustness] joint params=({:.4},{:.4},{:.3},{:.3}) max_x={x:.3} reached_top={top}",
            p[0], p[1], p[2], p[3],
        );
    }

    eprintln!("[hip_gate_robustness] -- probe 3: dt scaled 0.8x / 1.25x --");
    for scale in [0.8, 1.25] {
        let (x, top) = eval_dt(BASE.0, BASE.1, BASE.2, BASE.3, 0.0005 * scale);
        eprintln!("[hip_gate_robustness] dt*{scale:.2} max_x={x:.3} reached_top={top}");
    }

    eprintln!(
        "[hip_gate_robustness] SUMMARY baseline={base_x:.3}  \
         probe1(one-at-a-time) range=[{probe1_min:.3},{probe1_max:.3}]  \
         probe2(joint +/-10%) range=[{probe2_min:.3},{probe2_max:.3}]",
    );
}

// Interactive keyboard teleop of the WBC/MPC pipeline used to live here as
// a #[test] (namiashi_staircase_5cm_teleop), but mujoco-rs's viewer opens a
// winit event loop, and winit requires the real process main thread --
// cargo test's harness always runs test bodies on a worker thread
// (regardless of --test-threads), which the viewer cannot tolerate. Moved
// to examples/namiashi_wbc_teleop.rs (a real `fn main()`, run via
// `cargo run --release --no-default-features --features
// "mujoco,mujoco-viewer" --example namiashi_wbc_teleop`), which calls
// run_wbc_sim (now pub, namiashi_sim::wbc_harness) with the same
// live_cmd/live_viewer wiring.

/// Is the yaw drift simply unregulated, and does regulating it unblock the
/// terrain footplan?
///
/// `WbcPipeline::yaw_pd_gain` has existed since the drift was identified but
/// has never been measured on its own -- it defaults to `(0.0, 0.0)`, an
/// exact no-op, so every staircase number in this file so far was taken with
/// NOTHING holding heading. `yaw_ref` stays at its default 0.0, which is the
/// correct target here: the run starts at yaw 0 and `cmd_wz` is 0 throughout,
/// so "hold straight" is literally yaw_ref = 0.
///
/// Motivated by re-measuring the terrain footplan (an idealized, exact
/// per-leg touchdown height from `StaircaseCfg::height_at`) and finding it
/// reaches x=1.609 m against a bare baseline's 1.579 m -- i.e. perfect
/// terrain knowledge bought essentially nothing -- while ending 0.715 m
/// sideways of the track. A footplan cannot show its value while the run is
/// being ended by an unrelated failure, so this sweeps the yaw gain both
/// without and with the footplan: the first column asks whether the drift is
/// regulable at all, the second whether removing it lets terrain knowledge
/// finally pay.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_staircase_5cm_yaw_regulation() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();

    // (0,0) first so the table carries its own baseline rather than asking
    // the reader to trust a number from another test's output.
    let gains = [(0.0, 0.0), (20.0, 2.0), (50.0, 5.0), (100.0, 10.0), (200.0, 20.0)];
    for footplan in [None, Some(TerrainFootplanCfg { clearance_m: 0.02, horizontal_margin_m: 0.05 })] {
        let tag = if footplan.is_some() { "footplan" } else { "bare    " };
        for (kp, kd) in gains {
            let params = WbcParams {
                actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 20.0,
                wbc_real_inertia: true,
                staircase: Some(stairs),
                terrain_footplan: footplan,
                yaw_pd_gain: if (kp, kd) == (0.0, 0.0) { None } else { Some((kp, kd)) },
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
            // Drift stats over the UPRIGHT window only. `run_wbc_sim` keeps
            // sampling after a topple, and a tumbling body sweeps every yaw
            // there is -- measuring across that reports the fall, not the
            // heading error the gain was supposed to prevent, and it does so
            // in the direction that makes a working gain look broken.
            let upright = samples
                .iter()
                .position(|s| s.body_z < TRUNK_Z_FALL_THRESHOLD_M)
                .unwrap_or(samples.len());
            let fell = upright < samples.len();
            let walking = &samples[..upright.max(1)];
            let max_yaw_deg = walking
                .iter()
                .map(|s| s.yaw.abs().to_degrees())
                .fold(0.0_f64, f64::max);
            let max_abs_y = walking.iter().map(|s| s.body_y.abs()).fold(0.0_f64, f64::max);
            let x_at_fall = walking.last().map(|s| s.body_x).unwrap_or(0.0);
            let final_s = samples.last().unwrap();
            let reached_top =
                final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
            eprintln!(
                "[yaw_reg {tag} kp={kp:5.1} kd={kd:4.1}] max_x={max_x:.3}m  \
                 upright: max|yaw|={max_yaw_deg:5.1}deg max|y|={max_abs_y:.3}m \
                 until x={x_at_fall:.3}m  fell={fell}  reached_top={reached_top}",
            );
        }
    }
}

/// Does a per-leg NOMINAL STANCE height unblock the 5 cm staircase?
///
/// The one thing about the leg geometry that no experiment in this file had
/// ever varied: `auto_detect_kinematics_config` hands all four legs a single
/// shared `nominal_foot_body`, so the whole gait is built on a flat-ground
/// assumption even in the runs that fed it exact terrain knowledge. On a
/// staircase the front and rear legs are on different steps for essentially
/// the entire climb.
///
/// Everything else measured so far plateaus at 1.40-1.61 m: blind swing
/// clearance, three reflex designs, the oracle touchdown footplan (1.609 m
/// against a 1.579 m bare baseline), and yaw regulation both alone and
/// combined with the footplan. The riser is at 1.5 m, so that plateau is
/// "cannot commit to step 1". This asks whether the support pattern being
/// flat is why.
///
/// Swept against the footplan rather than instead of it: the two act on
/// opposite halves of the same stride (stance geometry vs swing touchdown)
/// and the interesting cell is whether they only pay off together.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_staircase_5cm_terrain_stance() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();

    for footplan in [None, Some(TerrainFootplanCfg { clearance_m: 0.02, horizontal_margin_m: 0.05 })] {
        let tag = if footplan.is_some() { "+footplan" } else { "bare     " };
        // gain 0.0 first: an exact no-op, so each column carries its own
        // baseline instead of borrowing one from another test's run.
        for gain in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0] {
            let params = WbcParams {
                actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
                host_rate_hz: Some(400.0),
                dt: 0.0005,
                cmd_vx: cmd,
                total_time_s: 20.0,
                wbc_real_inertia: true,
                staircase: Some(stairs),
                terrain_footplan: footplan,
                terrain_stance: if gain == 0.0 {
                    None
                } else {
                    Some(TerrainStanceCfg { gain, max_offset_m: 0.10 })
                },
                ..namiashi_tuned_params(I)
            };
            let Some(samples) = run_wbc_sim(params) else { return };
            let upright = collapse_index(&samples, &stairs).unwrap_or(samples.len());
            let fell = upright < samples.len();
            // Progress over the UPRIGHT window. A whole-run max_x reports a
            // numerical blow-up as progress -- the first pass of this sweep
            // printed 16.190 m for one cell, well past the 11.5 m end of the
            // terrain, from the robot being flung rather than climbing.
            let walking = &samples[..upright.max(1)];
            let max_x = walking.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
            let max_z = walking.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
            let raw_max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
            let final_s = samples.last().unwrap();
            let reached_top =
                final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
            // ~3.8 steps of rise per 0.19 m of trunk lift; printed so the
            // headline number is "how far up", not just "how far along".
            let steps = ((max_z - samples[0].body_z) / 0.05).max(0.0);
            // Past the far end of the built terrain there is nothing to walk
            // on, so any x beyond it is the solver launching the robot, not
            // progress -- and it can do that WITHOUT tripping the fall
            // threshold (one cell here holds trunk z high all the way to
            // x=12 m). Called out explicitly rather than left for the reader
            // to notice the implausible number.
            let terrain_end_x = top_start_x + stairs.top_platform_m;
            let diverged = raw_max_x > terrain_end_x;
            // Where it ENDS, not just how far it got: gain 1.0 never
            // collapses yet finishes below the first riser, having climbed
            // and then walked itself back down. max_x alone scores that as
            // the best non-falling run in the sweep.
            eprintln!(
                "[stance {tag} gain={gain:.1}] upright: max_x={max_x:.3}m max_z={max_z:.3}m \
                 (~{steps:.1} steps)  final_x={:.3}m  fell={fell}  \
                 raw_max_x={raw_max_x:.3}m  reached_top={reached_top}{}",
                final_s.body_x,
                if diverged { "  <- DIVERGED (past terrain end)" } else { "" },
            );
        }
    }
}

/// First sample where the trunk has collapsed RELATIVE TO THE STEP IT IS
/// STANDING ON, or `None` if it never does.
///
/// `TRUNK_Z_FALL_THRESHOLD_M` is a world-frame absolute, which is correct on
/// flat ground and quietly wrong on a staircase: a robot belly-down on step 3
/// sits at z=0.251 m, above the 0.18 m threshold, and reads as upright. The
/// stance-height sweep reported `fell=false` for exactly that state until a
/// replay showed the robot flat on the stairs and motionless. Same class of
/// bug as `base_height_floor`'s on the RL side, which needed the identical
/// "ground is not at world zero" correction.
fn collapse_index(samples: &[WbcSample], stairs: &StaircaseCfg) -> Option<usize> {
    samples
        .iter()
        .position(|s| s.body_z - stairs.height_at(s.body_x) < TRUNK_Z_FALL_THRESHOLD_M)
}

/// What actually ends the two stance-height runs.
///
/// Neither holds what it climbs: gain 1.0 never collapses but finishes at
/// x=0.737 m, below the first riser, and gain 1.45 reaches ~3.6 steps and
/// then goes flat on the stairs around t=15 s. `max_x` cannot distinguish
/// those, and neither can a still frame -- so this dumps the signals that
/// separate the candidate causes, on a 0.5 s grid across the whole run:
///
///   - yaw: the documented failure mode is a slow uncommanded turn-around,
///     after which "forward" in the body frame is backward in world x. If
///     the retreat is that, yaw crosses 90 deg before x starts dropping.
///   - max tau_frac: 1.0 means a joint is clamped at its effort limit, so
///     the modelled controller is not the controller running.
///   - max qd_frac: `mujoco_sim` brakes overspeed with a torque applied
///     BEFORE the effort clamp, so a too-fast joint presents as a joint out
///     of torque -- a different problem with a different fix.
///   - stance count and total contact fz: a collapse with feet still loaded
///     is a strength/posture problem; one with fz falling away first is a
///     loss of contact.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_staircase_5cm_stance_failure_diag() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    for gain in [1.0, 1.45] {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            terrain_stance: Some(TerrainStanceCfg { gain, max_offset_m: 0.10 }),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let collapse = collapse_index(&samples, &stairs);
        eprintln!(
            "\n=== stance gain={gain:.2}  collapse at {} ===",
            collapse
                .map(|i| format!("t={:.2}s x={:.3}m", samples[i].t, samples[i].body_x))
                .unwrap_or_else(|| "never".into()),
        );
        eprintln!(
            "    t     x      z    z-terr   yaw    pitch  stance  fz_tot  \
             tau_max qd_max  sat"
        );
        let mut next = 0.0_f64;
        for s in &samples {
            if s.t + 1e-9 < next {
                continue;
            }
            next += 0.5;
            let tau_max = s.tau_frac.iter().cloned().fold(0.0_f64, f64::max);
            let qd_max = s.qd_frac.iter().cloned().fold(0.0_f64, f64::max);
            let n_sat = s.tau_frac.iter().filter(|&&f| f > 0.99).count();
            let n_stance = s.stance_mask.iter().filter(|&&b| b).count();
            eprintln!(
                "{:6.2} {:6.3} {:6.3} {:6.3} {:7.1} {:7.1} {:5}/4 {:7.1} \
                 {:6.2} {:6.2} {:3}",
                s.t,
                s.body_x,
                s.body_z,
                s.body_z - stairs.height_at(s.body_x),
                s.yaw.to_degrees(),
                s.pitch.to_degrees(),
                n_stance,
                s.total_fz_world,
                tau_max,
                qd_max,
                n_sat,
            );
        }
    }
}

/// Replay source for the stance-height comparison clip: the same staircase
/// with the per-leg correction off and at gain=1.45, so the difference is
/// visible rather than only tabulated.
#[test]
#[ignore = "writes replays for video -- run with --ignored"]
fn namiashi_staircase_5cm_terrain_stance_video() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    for (gain, dir) in [(0.0, "stance_off"), (1.0, "stance_100"), (1.45, "stance_145")] {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            terrain_stance: if gain == 0.0 {
                None
            } else {
                Some(TerrainStanceCfg { gain, max_offset_m: 0.10 })
            },
            replay_dir: Some(format!("/tmp/nami_stairs/{dir}")),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let upright = collapse_index(&samples, &stairs).unwrap_or(samples.len());
        let walking = &samples[..upright.max(1)];
        let max_x = walking.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        eprintln!("[stance_video gain={gain:.2} -> {dir}] max_x={max_x:.3}m  fell={}", upright < samples.len());
    }
}

/// Does a respawn land the robot STANDING STILL, or mid-stride?
///
/// The reference pose is "standing still with no gait playing", which is
/// two things: the body back at its spawn, and the controller back at the
/// cycle origin with a zero command. Restoring only the body leaves the
/// phase where it was, so the legs resume a step the robot is no longer
/// positioned for -- which looks like the reset did not take.
///
/// Walks first, so the gait is genuinely mid-stride when the respawn lands,
/// then asks whether the feet are all down and the body is where it started.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_respawn_lands_at_standstill() {
    use namiashi_sim::teleop::LiveTeleop;
    use std::sync::{Arc, Mutex};

    let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
    // Walk for 3 s, respawn, then stand for 1.5 s with no command.
    {
        let mut st = live.lock().unwrap();
        st.cmd = [0.4, 0.0, 0.0];
    }
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: 0.0,
        total_time_s: 3.0,
        wbc_real_inertia: true,
        live_teleop: Some(live.clone()),
        live_viewer: false,
        ..namiashi_tuned_params(0)
    };
    let Some(walk) = run_wbc_sim(params) else { return };
    let end = walk.last().unwrap();
    let stance_at_end = end.stance_mask.iter().filter(|&&b| b).count();
    eprintln!(
        "[standstill] after 3 s walking: x={:.3} m, {stance_at_end}/4 feet down, \
         phase mid-stride",
        end.body_x,
    );

    // Same run, but the respawn fires partway and the command is released.
    {
        let mut st = live.lock().unwrap();
        st.cmd = [0.0; 3];
        st.respawn_requested = true;
    }
    let params2 = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: 0.0,
        total_time_s: 1.5,
        wbc_real_inertia: true,
        live_teleop: Some(live.clone()),
        live_viewer: false,
        ..namiashi_tuned_params(0)
    };
    let Some(after) = run_wbc_sim(params2) else { return };
    let s = after.last().unwrap();
    let down = s.stance_mask.iter().filter(|&&b| b).count();
    eprintln!(
        "[standstill] after respawn + 1.5 s idle: x={:.3} m  z={:.3} m  \
         {down}/4 feet down  |vx meas| over last 0.5 s = {:.4} m/s",
        s.body_x,
        s.body_z,
        {
            let tail: Vec<_> = after.iter().rev().take(1000).collect();
            let dx = tail.first().unwrap().body_x - tail.last().unwrap().body_x;
            (dx / 0.5).abs()
        },
    );
}

/// Does a `mj_resetData`-style reset get caught, and does respawn undo it?
///
/// The viewer's Reset button reaches this simulation -- `sync_data` is a
/// three-way merge, so whatever the viewer changed in its own copy is
/// written back into ours -- but it raises no event this side can
/// subscribe to. The only trace is the clock: `mj_resetData` sets time to
/// zero. This reproduces the reset directly (same call the viewer makes)
/// and checks both halves: that the clock going backwards is detectable,
/// and that respawn puts the robot somewhere it can stand.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_viewer_reset_is_caught() {
    use articara::mjcf::MjcfExportOptions;
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");
    // Seed the BENT stance exactly as run_wbc_sim does. This is the whole
    // mechanism: the MJCF carries no joint `ref`, so qpos0 keeps every hinge
    // at zero -- legs straight -- while the base's auto-lift is computed for
    // the seeded bent pose. Without this line the spawn pose IS qpos0 and a
    // reset changes nothing, which is why an earlier version of this test
    // reproduced no sinking at all and looked like the report was wrong.
    let kin = auto_detect_kinematics_config(&robot, &DEFAULT_FOOT_LINKS)
        .expect("auto-detect kinematics");
    seed_joint_positions_from_kinematics(&mut robot, &kin);
    robot.rebuild_misarta_model();
    let opts = MjcfExportOptions {
        base_xy: Some((
            -(ring.ring_m / 2.0 + pd / 2.0),
            -(ring.ring_m / 2.0 - pw / 2.0),
        )),
        extra_asset_xml: Some(ring.asset_xml("kawasaki")),
        extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
        add_actuators: true,
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
    let dt = sim.timestep();
    for _ in 0..(1.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let before = sim.sim_time();
    let z_ok = sim.body_world_position(&robot.root_link).unwrap()[2];

    // Exactly what UiEvent::ResetSimulation does to the data it holds.
    sim.mj_data_mut().reset();
    sim.mj_data_mut().forward();
    let after_reset = sim.sim_time();
    let z_reset = sim.body_world_position(&robot.root_link).unwrap()[2];
    // Feet on straight legs reach further than the bent stance the spawn
    // height was computed for, so this is the sinking, in one number: how
    // far the lowest foot ends up BELOW the surface it should rest on.
    let deepest = (0..4)
        .filter_map(|i| {
            let link = DEFAULT_FOOT_LINKS[i].1;
            sim.body_world_position(link).map(|p| p[2])
        })
        .fold(f64::INFINITY, f64::min);
    eprintln!(
        "[reset] t {before:.3} -> {after_reset:.3} (detected: {})  trunk z {z_ok:.3} -> {z_reset:.3}  \
         lowest foot z={deepest:+.4} m (ring surface is 0)",
        after_reset < before - 1e-9,
    );

    // The reset POSE does not penetrate (see the +0.021 m above). If the
    // report is real the sinking has to come from what happens next: the
    // per-joint PD still holds targets for the bent stance, so it yanks a
    // straight-legged robot back down through whatever it is standing on.
    // Step it and watch the lowest foot.
    let mut worst = f64::INFINITY;
    for _ in 0..(1.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
        for i in 0..4 {
            if let Some(p) = sim.body_world_position(DEFAULT_FOOT_LINKS[i].1) {
                worst = worst.min(p[2]);
            }
        }
    }
    eprintln!(
        "[reset] 1 s AFTER the reset, with the PD still holding stance targets: \
         lowest foot reached {worst:+.4} m  trunk z={:.3}",
        sim.body_world_position(&robot.root_link).unwrap()[2],
    );

    sim.respawn(&mut robot, 0.10);
    let z_respawn = sim.body_world_position(&robot.root_link).unwrap()[2];
    for _ in 0..(1.5 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let z_settled = sim.body_world_position(&robot.root_link).unwrap()[2];
    eprintln!(
        "[reset] respawned z={z_respawn:.3} -> settled z={z_settled:.3} \
         (standing height was {z_ok:.3})",
    );
}

/// Does the arm key actually move the arm, and stop at the joint's limits?
///
/// The arm is not part of the gait -- the .misa drives `arm_pitch_joint` as
/// its own Position-mode actuator (kp=5) and run_wbc_sim's Torque switch
/// only touches the twelve leg joints -- so it is commanded by position
/// target, and its travel is the model's own -2.3 .. +0.85 rad rather than
/// a range picked here. Holds the key one way, then the other, and reads
/// the joint back out of the sim.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_arm_command() {
    use namiashi_sim::teleop::LiveTeleop;
    use std::sync::{Arc, Mutex};
    for (label, rate, secs) in [("up (Y)", 1.0, 2.0), ("down (G)", -1.0, 5.0)] {
        let live = Arc::new(Mutex::new(LiveTeleop {
            arm_rate: rate,
            ..LiveTeleop::new(GaitType::Trot)
        }));
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0,
            total_time_s: secs,
            wbc_real_inertia: true,
            live_teleop: Some(live.clone()),
            live_viewer: false,
            ..namiashi_tuned_params(0)
        };
        if run_wbc_sim(params).is_none() {
            return;
        }
        let st = *live.lock().unwrap();
        eprintln!(
            "[arm {label:<9} {secs:.0}s] commanded {:+.3} rad ({:+.0} deg)  \
             limits -2.300 .. +0.850",
            st.arm_angle_rad,
            st.arm_angle_rad.to_degrees(),
        );
    }

    // The above reads back this harness's own integrator, which proves the
    // key is wired and clamped and NOTHING about whether the arm moved.
    // kp=5 against the arm's own weight is a real question, so drive the
    // position target directly and read the joint out of the sim.
    {
        use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
        use articara::mujoco_sim::MujocoSim;
        use articara::robot::RobotModel;
        let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
        let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");
        let ji = robot.joint_map["arm_pitch_joint"];
        let opts = MjcfExportOptions {
            ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        let dt = sim.timestep();
        for target in [0.85, -2.30, 0.0] {
            sim.set_position_target(ji, target);
            for _ in 0..(3.0 / dt) as u32 {
                sim.step(&mut robot, dt, true);
            }
            let (q, _) = sim.joint_q_qd("arm_pitch_joint").expect("arm state");
            eprintln!(
                "[arm measured] target {target:+.3} rad -> actual {q:+.3} rad  \
                 (error {:+.3})",
                q - target,
            );
        }
    }
}

/// Does respawn put the robot back at its START pose, rather than at
/// MuJoCo's `qpos0`?
///
/// The viewer's own Reset button calls `mj_resetData`, which sets every
/// hinge to its `ref` -- legs straight. namiashi's spawn height was
/// computed for a BENT stance, so straight legs put the feet through the
/// surface and the body inside it, which is the sinking the Reset button
/// produces. `MujocoSim::respawn` restores the pose the sim started in
/// instead. This walks the robot away first, so "went back" is a real
/// claim rather than "never moved".
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_respawn_restores_start_pose() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::mjcf::MjcfExportOptions;
    use articara::robot::RobotModel;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");
    let opts = MjcfExportOptions {
        base_xy: Some((
            -(ring.ring_m / 2.0 + pd / 2.0),
            -(ring.ring_m / 2.0 - pw / 2.0),
        )),
        extra_asset_xml: Some(ring.asset_xml("kawasaki")),
        extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
        add_actuators: true,
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
    let dt = sim.timestep();
    let start = sim.body_world_position(&robot.root_link).unwrap();

    // Shove it off the platform. Simply "not commanding anything" does not
    // move this robot -- the .misa puts the joints in Position mode, so the
    // per-joint PD holds the spawn pose and the first version of this test
    // watched it sit still for two seconds and called that displacement.
    sim.apply_external_force(&robot.root_link, [22.0, 14.0, 0.0], [0.0; 3], 0.6);
    for _ in 0..(2.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let fallen = sim.body_world_position(&robot.root_link).unwrap();
    eprintln!(
        "[respawn] pushed to ({:+.3},{:+.3}) -- {:.3} m from spawn",
        fallen[0],
        fallen[1],
        ((fallen[0] - start[0]).powi(2) + (fallen[1] - start[1]).powi(2)).sqrt(),
    );

    sim.respawn(&mut robot, 0.10);
    let after = sim.body_world_position(&robot.root_link).unwrap();
    // Let it settle out of the 10 cm drop.
    for _ in 0..(1.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let settled = sim.body_world_position(&robot.root_link).unwrap();
    eprintln!(
        "[respawn] start z={:.3}  after 2 s uncontrolled z={:.3}  \
         respawned z={:.3} (start+0.10={:.3})  settled z={:.3}",
        start[2], fallen[2], after[2], start[2] + 0.10, settled[2],
    );
    eprintln!(
        "[respawn] xy back to spawn: dx={:+.4} dy={:+.4}",
        after[0] - start[0],
        after[1] - start[1],
    );
}

/// Do these moves survive a change of robot?
///
/// Both trajectories and the attack were searched and measured on
/// `namiashi_3p3_prop`. The fixtures carry two other distinct models:
/// `namiashi_3p3_hip` has the same mass, joints and actuators with the leg
/// mass moved into the hip (thigh 0.021 kg against 0.057), and `namiashi` is
/// 2.40 kg with weaker joints (hip 1.5 N.m against 2.5).
///
/// Two things this pins down, from
/// `examples/namiashi_cross_model_check`'s fuller sweep:
///
/// The unhurried recovery turns the body over just as often on every model
/// -- peak trunk +z above 0.9 in 14 of 18 conditions on all of them -- and
/// converts that into standing in 7, 2 and 0. So what a change of robot
/// breaks is the catch, not the turn.
///
/// And the arm attack's mechanism does not carry over at all: on
/// `namiashi_3p3_hip` the full move leaves the opponent up at +0.463 while
/// the no-arm control puts it down at -0.918. The arm makes it WORSE there.
/// Asserted so that a later change which quietly makes the attack
/// model-independent shows up as a failing test rather than going unnoticed.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_moves_across_models() {
    use namiashi_sim::attack::AttackParams;
    use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::self_righting::{evaluate_rolled, Drive, RECOVERY_GENTLE};
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/namiashi");
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };
    let ring = KawasakiRingCfg::default();

    // How often the unhurried recovery gets the body all the way over, as
    // opposed to all the way over AND standing.
    // Six conditions each (two sites x three rolls, at one friction), so
    // the counts here are a subset of the example's eighteen and not
    // directly comparable to the 14/18 quoted above.
    for (label, file) in [
        ("3p3_prop", "namiashi_3p3_prop.misa"),
        ("3p3_hip", "namiashi_3p3_hip.misa"),
        ("2.4 kg", "namiashi.misa"),
    ] {
        let misa = dir.join(file);
        let (mut ok, mut peaked, mut n) = (0, 0, 0);
        for xy in [(1.60, 1.60), (0.35, 0.35)] {
            for deg in [180.0_f64, 90.0, -90.0] {
                let o = evaluate_rolled(
                    &misa, &ring, xy, 0.70, &RECOVERY_GENTLE, 20.0, teleop, None,
                    deg.to_radians(),
                );
                n += 1;
                ok += o.righted() as u32;
                peaked += (o.peak_up > 0.9) as u32;
            }
        }
        eprintln!("[models] {label:<10} unhurried: righted {ok}/{n}, reached upright {peaked}/{n}");
        assert!(
            peaked >= 3,
            "{label}: the recovery no longer even turns the body over ({peaked}/{n})",
        );
    }

    // The attack, each model against a locked copy of itself.
    let attack = |file: &str, plan: AttackParams| -> f64 {
        let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
        live.lock().unwrap().attack_at_sim_s = Some(2.0);
        let ring = KawasakiRingCfg::default();
        let samples = run_wbc_sim(WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0,
            total_time_s: 12.0,
            wbc_real_inertia: true,
            live_teleop: Some(Arc::clone(&live)),
            live_viewer: false,
            spawn_xy: Some((-0.38, 0.0)),
            opponent: Some((LockedPose::default(), [0.0, 0.0, ring.z_top_m() + 0.30])),
            kawasaki_ring: Some(ring),
            attack_params: plan,
            misa_file: Box::leak(
                dir.join(file).to_string_lossy().into_owned().into_boxed_str(),
            ),
            ..namiashi_tuned_params(0)
        })
        .expect("run_wbc_sim");
        samples.last().unwrap().foe.expect("no opponent")[3]
    };
    let no_arm = AttackParams { arm_down: -2.3, arm_up: -2.3, ..AttackParams::DEFAULT };

    let prop = (attack("namiashi_3p3_prop.misa", AttackParams::DEFAULT),
                attack("namiashi_3p3_prop.misa", no_arm));
    let hip = (attack("namiashi_3p3_hip.misa", AttackParams::DEFAULT),
               attack("namiashi_3p3_hip.misa", no_arm));
    eprintln!("[models] attack 3p3_prop: full {:+.3}, no-arm {:+.3}", prop.0, prop.1);
    eprintln!("[models] attack 3p3_hip:  full {:+.3}, no-arm {:+.3}", hip.0, hip.1);

    assert!(prop.0 < 0.3 && prop.1 > 0.3, "the attack stopped working on the model it was built on");
    assert!(
        hip.0 > 0.3,
        "the attack now works on 3p3_hip ({:+.3}) -- good, but every doc and \
         comment saying the mechanism does not carry across models is now wrong",
        hip.0,
    );
}

/// Can it get up from lying on its SIDE, not just from flat on its back?
///
/// Every self-righting measurement so far started at a 180-degree roll,
/// which is not how a robot that has been knocked over usually ends up. On
/// its side the body is halfway there, but halfway in a direction that can
/// go either way: gravity finishes the job or undoes it depending on which
/// way it is leaning, so it is not safe to assume the easier-looking start
/// is easier.
///
/// It is not, for one of the two trajectories.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_self_righting_from_its_side() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::self_righting::{evaluate_rolled, Drive, RECOVERY_FAST, RECOVERY_GENTLE};

    let ring = KawasakiRingCfg::default();
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };
    let sites: [(&str, (f64, f64)); 2] =
        [("flat floor", (1.60, 1.60)), ("ring surface", (0.35, 0.35))];

    let (mut fast_ok, mut gentle_ok, mut total) = (0, 0, 0);
    for (name, xy) in sites {
        // Both ways over. Which side it lies on is not a symmetry: the hip
        // roll limits are mirrored (-0.785..1.05 against -1.05..0.785) and
        // the arm is off-centre in y.
        for deg in [90.0_f64, -90.0] {
            let roll = deg.to_radians();
            let g = evaluate_rolled(
                &misa, &ring, xy, 0.70, &RECOVERY_GENTLE, 20.0, teleop, None, roll,
            );
            let f = evaluate_rolled(
                &misa, &ring, xy, 0.70, &RECOVERY_FAST, 20.0, teleop, None, roll,
            );
            total += 1;
            fast_ok += f.righted() as u32;
            gentle_ok += g.righted() as u32;
            eprintln!(
                "[side] {name:<13} roll {deg:>+5.0} deg  gentle {:+.3}  fast {:+.3} \
                 (t {:.2} s)",
                g.final_up, f.final_up, f.t_right_s,
            );
        }
    }
    eprintln!("[side] fast {fast_ok}/{total}, gentle {gentle_ok}/{total}");
    // The unhurried trajectory used to score 0/4 here: it had been searched
    // from a 180-degree start only and settled at 0.47 from every side-lying
    // start tried. `GENTLE_V3` was searched with +/-90 in the training set
    // and with the rock able to mirror itself, and gets some of them. It is
    // still well behind `Shift`+`V`, which is the point of asserting both:
    // `V` is the default key and someone knocked onto their side will press
    // it first.
    assert!(fast_ok >= 3, "Shift+V no longer gets up from its side: {fast_ok}/{total}");
    assert!(
        gentle_ok >= 1,
        "the unhurried recovery is back to never getting up from its side: \
         {gentle_ok}/{total}",
    );
}

/// Does the `X` attack actually turn the opponent over?
///
/// The move is: crouch the front and drop the arm, creep forward so the arm
/// goes under, then extend the front legs while the arm swings up. Every
/// part of that is plausible on paper, and the only thing that settles it is
/// where the opponent ends up.
///
/// Scored on the OPPONENT's trunk +z in world coordinates -- +1 as placed,
/// -1 flat on its back -- and on how far it moved. Both matter: shoving it
/// across the ring without turning it over is a different move, and reads as
/// success to anything watching only the attitude at the end.
///
/// The opponent is joint-locked, so nothing it does can help or hinder; what
/// is being measured is entirely the attacker's move against the real
/// masses and collision shapes.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_arm_attack_tips_the_opponent() {
    use namiashi_sim::attack::AttackParams;
    use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    // (final opponent trunk +z, how far it was pushed)
    let run = |start_x: f64, plan: AttackParams| -> (f64, f64) {
        let ring = KawasakiRingCfg::default();
        let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
        // Scheduled in the loop, not posted from a thread: see
        // `namiashi_teleop_v_key_rights_the_robot` for what that cost. 2 s
        // in, so the burn-in and the initial settle are done.
        live.lock().unwrap().attack_at_sim_s = Some(2.0);
        let samples = run_wbc_sim(WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0,
            total_time_s: 12.0,
            wbc_real_inertia: true,
            live_teleop: Some(Arc::clone(&live)),
            live_viewer: false,
            spawn_xy: Some((start_x, 0.0)),
            opponent: Some((LockedPose::default(), [0.0, 0.0, ring.z_top_m() + 0.30])),
            kawasaki_ring: Some(ring),
            attack_params: plan,
            ..namiashi_tuned_params(0)
        })
        .expect("run_wbc_sim");
        let first = samples[0].foe.expect("no opponent in the scene");
        let last = samples.last().unwrap().foe.unwrap();
        (
            last[3],
            ((last[0] - first[0]).powi(2) + (last[1] - first[1]).powi(2)).sqrt(),
        )
    };

    // 0.38 m out, which `examples/namiashi_attack_sweep` measured as the
    // range where the move works: at 0.45 m the arm never arrives, and by
    // 0.32 m the robot tips the opponent whatever the arm does, shoving it
    // 0.71 m along the ring in the process. That last one matters -- being
    // too close turns this from a lever into a bulldoze, and a test run
    // there would pass without the arm existing.
    const RANGE_M: f64 = -0.38;

    let (up, moved) = run(RANGE_M, AttackParams::DEFAULT);
    eprintln!("[attack] full move:          opponent up {up:+.3}, moved {moved:.3} m");
    assert!(up < 0.3, "opponent still upright after the attack: trunk +z {up:+.3}");

    // The arm has to be what does it. Same move with the arm left up: if the
    // opponent goes over anyway, this test is measuring a robot walking into
    // something and would keep passing however the arm were wired.
    let (up_noarm, moved_noarm) = run(
        RANGE_M,
        AttackParams { arm_down: -2.3, arm_up: -2.3, ..AttackParams::DEFAULT },
    );
    eprintln!(
        "[attack] arm never lowered:  opponent up {up_noarm:+.3}, moved {moved_noarm:.3} m"
    );
    assert!(
        up_noarm > 0.3,
        "the opponent goes over without the arm ({up_noarm:+.3}) -- this is a shove, \
         not the move, and the attack is not being tested",
    );

    // And the front legs have to extend. Crouching and staying there leaves
    // the arm doing the whole lift from below and it does not get there.
    let (up_nolift, _) = run(RANGE_M, AttackParams { front_rise_m: 0.055, ..AttackParams::DEFAULT });
    eprintln!("[attack] front never lifts:  opponent up {up_nolift:+.3}");
    assert!(
        up_nolift > 0.3,
        "the front-leg extension turns out not to matter: {up_nolift:+.3}",
    );
}

/// Does the `V` key path in `run_wbc_sim` actually right the robot?
///
/// Everything else about self-righting has been measured through
/// `self_righting::evaluate_with`, which reproduces the teleop's drive law
/// but is not the teleop. This drives the real thing: `Shift`+`N` to land on
/// its back, then `V`, both through the same `LiveTeleop` the viewer writes
/// to, with the gait controller, the WBC and the whole harness loop in
/// place. Those are exactly the parts `evaluate_with` does not have.
///
/// The request is fired from a thread on a wall-clock delay because nothing
/// in `LiveTeleop` reports sim time unless the viewer is running. A second
/// is far inside a 40 s WBC run, and the assertion is on the attitude at the
/// end rather than on timing, so the delay only has to land somewhere in the
/// first half.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_teleop_v_key_rights_the_robot() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
    // On its back from the first tick.
    {
        let mut st = live.lock().unwrap();
        st.respawn_requested = true;
        st.respawn_inverted = true;
    }
    // ...and `V` half a second of SIM time later, matching the settle the
    // evaluator gives it. Scheduled inside the physics loop rather than
    // posted from a thread: the loop runs thousands of ticks per wall-clock
    // millisecond, so a request sent "half a second in" landed at a
    // different sim time on every run and this test disagreed with itself
    // about whether the robot got up.
    //
    // Half a second also has to pass at all. Firing on the same tick as the
    // respawn would be cancelled -- the respawn handler clears a pending
    // recovery, deliberately, so that landing somewhere new cannot leave a
    // half-run trajectory driving the legs.
    live.lock().unwrap().recover_at_sim_s = Some(0.5);

    let ring = KawasakiRingCfg::default();
    let samples = run_wbc_sim(WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: 0.0,
        total_time_s: 20.0,
        wbc_real_inertia: true,
        live_teleop: Some(Arc::clone(&live)),
        live_viewer: false,
        spawn_xy: Some((0.35, 0.35)),
        kawasaki_ring: Some(ring),
        ..namiashi_tuned_params(0)
    })
    .expect("run_wbc_sim");

    // Trunk +z from the reported roll and pitch: +1 upright, -1 on its back.
    let up = |s: &namiashi_sim::wbc_harness::WbcSample| s.roll.cos() * s.pitch.cos();
    let worst = samples.iter().map(up).fold(f64::INFINITY, f64::min);
    let best = samples.iter().map(up).fold(f64::NEG_INFINITY, f64::max);
    let end = up(samples.last().expect("samples"));
    eprintln!(
        "[V key] {} samples: lowest trunk +z {worst:+.3}, highest {best:+.3}, final {end:+.3}",
        samples.len(),
    );
    let step = (samples.len() / 24).max(1);
    for s in samples.iter().step_by(step) {
        eprintln!(
            "[V key] t {:6.2}  trunk +z {:+.3}  z {:.3} m  contacts {}",
            s.t,
            up(s),
            s.body_z,
            s.foot_fz.iter().filter(|f| **f > 1.0).count(),
        );
    }
    assert!(
        worst < -0.7,
        "never actually landed on its back -- lowest trunk +z was {worst:+.3}, \
         so this test proved nothing about righting",
    );

    // Upright, and still upright two seconds later. NOT the attitude at the
    // end of the run: the recovery travels about 0.6 m, which from a spawn
    // 0.35 m off centre puts the robot within a few centimetres of the edge
    // of a 1.9 m ring, and the first version of this test spent its last 25
    // seconds watching it drift off and land on the venue floor 0.25 m
    // below. That is a fair thing to know and a separate question from
    // whether the key works.
    // The last two seconds, not the first moment the attitude crosses 0.9:
    // the roll passes through upright on its way over and briefly touches
    // it, so an "upright, and still upright two seconds later" test measured
    // that transient and failed on a run that ended standing.
    let t_last = samples.last().expect("samples").t;
    let held = samples
        .iter()
        .filter(|s| s.t > t_last - 2.0)
        .map(up)
        .fold(f64::INFINITY, f64::min);
    eprintln!(
        "[V key] lowest trunk +z over the final 2 s: {held:+.3}  (final {end:+.3}, peak {best:+.3})"
    );
    assert!(
        held > 0.9,
        "not standing at the end: lowest trunk +z over the final two seconds \
         was {held:+.3}",
    );
}

/// Does the searched recovery trajectory get the robot back on its feet
/// through the control path the teleop actually uses?
///
/// Driven as `run_wbc_sim` drives it -- leg joints in torque mode with a
/// host-side PD and gravity compensation, attitude from the Madgwick IMU
/// filter rather than the simulator's pose -- because the trajectory is
/// sensitive to that choice. `RECOVERY` rights the robot in 14 of these 15
/// conditions on the teleop drive and 7 of 15 on the .misa's Position
/// actuators, which is why the search was eventually pointed at the former.
///
/// Scores on the simulator's true attitude while the CONTROLLER sees only
/// the filter, so this measures the recovery and not the estimator agreeing
/// with itself.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_self_righting_teleop_drive() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::self_righting::{
        evaluate_with, Drive, GENTLE_JOINT_RAD_S, GENTLE_OMEGA_RAD_S, RECOVERY_FAST,
        RECOVERY_GENTLE,
    };

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let sites: [(&str, (f64, f64)); 5] = [
        ("flat venue floor", (1.60, 1.60)),
        ("open ring surface", (0.35, 0.35)),
        ("ring centre (bowl)", (0.0, 0.0)),
        ("on a round plate", ring.round_plate_centres[0]),
        (
            "red platform (edge)",
            (-(ring.ring_m / 2.0 + pd / 2.0), -(ring.ring_m / 2.0 - pw / 2.0)),
        ),
    ];
    // The teleop's own gains and filter constant.
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };
    let mut counts = [0_u32; 2];
    let mut rough = [0.0_f64; 2];
    for (which, (label, plan, secs)) in
        [("gentle", RECOVERY_GENTLE, 20.0), ("fast", RECOVERY_FAST, 20.0)]
            .into_iter()
            .enumerate()
    {
        for (name, xy) in sites {
            for mu in [0.30_f64, 0.70, 1.00] {
                let o = evaluate_with(&misa, &ring, xy, mu, &plan, secs, teleop);
                counts[which] += o.righted() as u32;
                rough[which] = rough[which].max(o.rms_omega_rad_s);
                eprintln!(
                    "[{label:<6}] {name:<20} mu {mu:.2}  final up {:+.3}  t {:>5.2} s  \
                     travel {:.3} m  RMS w {:.2}  qd {:.2}  {}",
                    o.final_up,
                    o.t_right_s,
                    o.travel_m,
                    o.rms_omega_rad_s,
                    o.rms_joint_rad_s,
                    if o.righted() { "RIGHTED" } else { "no" },
                );
            }
        }
        eprintln!("[{label:<6}] {}/15 righted, worst RMS w = {:.2} rad/s", counts[which], rough[which]);
    }
    // Measured: gentle 2/15 at an RMS trunk angular speed under 1.5 rad/s,
    // fast 12/15 at up to 4.2.
    //
    // The gentle number was 5/15 for `GENTLE_V2`. `GENTLE_V3` replaced it to
    // get side falls working -- 9 of 18 side-lying starts against 2 -- and
    // gave back three of these on-its-back conditions doing it. This grid
    // only ever starts the robot flat on its back, so it sees the cost and
    // none of the gain; `namiashi_self_righting_from_its_side` is the other
    // half of the picture and the two have to be read together. The trade is real and the point of keeping
    // both. A standing robot measures 0.310 rad/s, so the gentle one is
    // within about 5x of doing nothing and the fast one is 10 to 15x.
    assert!(counts[0] >= 2, "gentle self-righting regressed: {}/15", counts[0]);
    assert!(counts[1] >= 11, "fast self-righting regressed: {}/15", counts[1]);
    assert!(
        rough[0] < 2.0 * GENTLE_OMEGA_RAD_S,
        "gentle recovery is no longer gentle: worst RMS w = {:.2} rad/s",
        rough[0],
    );
    let _ = GENTLE_JOINT_RAD_S;
}

/// Does `Shift`+`N` actually put the robot on its back, and does it STAY
/// there?
///
/// Two separate claims, and the first does not imply the second. Writing a
/// 180-degree quaternion into `qpos` guarantees the pose at t=0 only; a
/// 3.3 kg body dropped 25 cm onto a heightfield with its legs sticking up
/// is free to bounce back onto its feet, or land on an edge and roll off
/// the ring, either of which would make this a useless starting state for
/// testing self-righting. So: check the orientation at spawn, then settle
/// it under gravity and check again.
///
/// Measures the trunk's own up-axis in world coordinates (row 2 of the
/// rotation matrix applied to body +z). Upright is +1, on its back is -1.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_inverted_respawn_stays_inverted() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mjcf::MjcfExportOptions;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");
    let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");
    let opts = MjcfExportOptions {
        base_xy: Some((
            -(ring.ring_m / 2.0 + pd / 2.0),
            -(ring.ring_m / 2.0 - pw / 2.0),
        )),
        extra_asset_xml: Some(ring.asset_xml("kawasaki")),
        extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
        add_actuators: true,
        ..MjcfExportOptions::default()
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
    let dt = sim.timestep();

    let root = robot.root_link.clone();
    let up = |sim: &MujocoSim| -> f64 {
        (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z
    };
    let yaw0 = sim.body_world_yaw(&robot.root_link).unwrap();
    eprintln!("[inverted] upright: trunk +z_world = {:+.3}", up(&sim));

    sim.respawn_inverted(&mut robot, 0.03);
    let at_spawn = up(&sim);
    let z_spawn = sim.body_world_position(&robot.root_link).unwrap()[2];
    let yaw1 = sim.body_world_yaw(&robot.root_link).unwrap();

    // Two seconds is well past the drop and any bounce; the joint PDs hold
    // the stance pose throughout, which is what the legs would do if the
    // robot flipped with its controller still running.
    for _ in 0..(2.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let settled = up(&sim);
    let p = sim.body_world_position(&robot.root_link).unwrap();
    eprintln!(
        "[inverted] at spawn: trunk +z_world = {at_spawn:+.3}  z = {z_spawn:.3} m  \
         (yaw {:+.1} deg -> {:+.1} deg)",
        yaw0.to_degrees(),
        yaw1.to_degrees(),
    );
    eprintln!(
        "[inverted] after 2 s settling: trunk +z_world = {settled:+.3}  \
         at ({:+.3},{:+.3},{:.3})",
        p[0], p[1], p[2],
    );
    assert!(
        at_spawn < -0.99,
        "spawn pose is not inverted: trunk +z_world = {at_spawn:+.3}",
    );
    assert!(
        settled < -0.7,
        "did not stay inverted -- settled with trunk +z_world = {settled:+.3}",
    );
}

/// Does the live trunk-height control actually move the trunk, and by the
/// amount asked for?
///
/// `body_lift_m` reaches the robot through three sign flips -- a key that
/// says "up", a `nominal_foot_body.z` that increases to CROUCH, and an
/// accumulator it now shares with two levelling corrections -- so "the
/// number changed" is not evidence the body did.
///
/// Drives the LIVE path specifically (`live_teleop` set, `live_viewer` off),
/// because the static `trunk_drop_m` reaches the same nominal by a different
/// route and would pass whether or not the keys work. Flat ground, standing,
/// so the answer should simply be the commanded offset.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_body_height_command() {
    use namiashi_sim::teleop::LiveTeleop;
    use std::sync::{Arc, Mutex};
    const I: usize = 0; // Trot
    let mut base_z = None;
    for lift in [0.0, -0.03, 0.03] {
        let live = Arc::new(Mutex::new(LiveTeleop {
            body_lift_m: lift,
            ..LiveTeleop::new(GaitType::Trot)
        }));
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0,
            total_time_s: 3.0,
            wbc_real_inertia: true,
            live_teleop: Some(live),
            live_viewer: false, // the shared state is read regardless
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let tail: Vec<_> = samples.iter().rev().take(400).collect();
        let z = tail.iter().map(|s| s.body_z).sum::<f64>() / tail.len() as f64;
        let delta = base_z.map(|b: f64| z - b);
        base_z.get_or_insert(z);
        eprintln!(
            "[body_height lift={lift:+.3}] trunk z={z:.4} m{}",
            delta.map(|d| format!("  (delta {d:+.4} m, asked {lift:+.3})")).unwrap_or_default(),
        );
    }
}

/// Does IMU + encoders alone level the trunk when the feet straddle two
/// heights?
///
/// The case asked for: standing on the かわさきロボット競技大会 ring with
/// some feet up on a raised obstacle and some down on the flat plate, and
/// wanting the trunk level anyway. `TerrainStanceCfg` already does this,
/// but by querying the terrain -- an oracle standing in for a vision system
/// namiashi has no room to carry. `ProprioStanceCfg` derives the same
/// per-leg heights from forward kinematics of the feet that are in contact,
/// projected into a gravity-aligned frame by a Madgwick filter on the trunk
/// IMU. No exteroception anywhere in that path.
///
/// Spawned straddling a round-hole plate's edge, so the front feet are on
/// 15 mm of plate and the rear on the ring: exactly the two-level stance,
/// with no walking needed to reach it. Trunk roll/pitch after settling is
/// the measurement -- the correction off is the control.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
#[cfg(feature = "mujoco-viewer")]
fn namiashi_ring_proprio_levelling() {
    use namiashi_sim::ring::KawasakiRingCfg;
    const I: usize = 0; // Trot
    let ring = KawasakiRingCfg::default();
    // The right-hand round plate spans x 0.50..0.80 at y 0. Straddle its
    // near edge so front and rear legs land on different levels.
    let spawn = (0.50, 0.0);
    // The last row crouches WHILE levelling: the two write the same nominal
    // through one accumulator, and an accumulator that drops a contributor
    // looks exactly like a controller that ignores a key.
    // The `enabled` column is the `B` toggle. It must gate the CORRECTION,
    // not just a flag: configured-but-toggled-off has to land on the same
    // tilt as never configured at all, or the key is decorative.
    for (gain, lift, enabled) in [
        (0.0, 0.0, true),   // not configured
        (1.0, 0.0, false),  // configured, toggled off
        (1.0, 0.0, true),   // configured, on
        (1.0, -0.03, true), // on, while crouching
    ] {
        let live = std::sync::Arc::new(std::sync::Mutex::new(namiashi_sim::teleop::LiveTeleop {
            body_lift_m: lift,
            level_enabled: enabled,
            ..namiashi_sim::teleop::LiveTeleop::new(GaitType::Trot)
        }));
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0, // stand: this is a posture question, not a gait one
            total_time_s: 4.0,
            wbc_real_inertia: true,
            kawasaki_ring: Some(ring.clone()),
            spawn_xy: Some(spawn),
            live_teleop: Some(live),
            proprio_stance: if gain == 0.0 {
                None
            } else {
                Some(ProprioStanceCfg { gain, ..Default::default() })
            },
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        // Last second only: the first few hundred ms are the drop onto the
        // plate, which says nothing about the steady posture.
        let tail: Vec<_> = samples.iter().filter(|s| s.t > samples.last().unwrap().t - 1.0).collect();
        let n = tail.len().max(1) as f64;
        let mean_roll = tail.iter().map(|s| s.roll).sum::<f64>() / n;
        let mean_pitch = tail.iter().map(|s| s.pitch).sum::<f64>() / n;
        let max_tilt = tail
            .iter()
            .map(|s| s.roll.abs().max(s.pitch.abs()))
            .fold(0.0_f64, f64::max);
        eprintln!(
            "[proprio_level gain={gain:.1} lift={lift:+.3} on={enabled:<5}] \
             roll={:+.2}deg pitch={:+.2}deg  \
             max|tilt|={:.2}deg  final z={:.3}m",
            mean_roll.to_degrees(),
            mean_pitch.to_degrees(),
            max_tilt.to_degrees(),
            samples.last().unwrap().body_z,
        );
    }
}

/// Is `TerrainStanceCfg`'s gain=1.5 optimum a basin or a spike?
///
/// `namiashi_staircase_5cm_terrain_stance` found 1.5 reaching 1.915 m and
/// ~3.8 steps upright, against 1.470 m for the same run with the correction
/// off -- the first thing in this file to move that plateau without either
/// falling or diverging. But both neighbours in that sweep are worse (1.0
/// climbs 2.8 steps, 2.0 falls), and this investigation has already been
/// fooled once: `namiashi_staircase_5cm_hip_gate_search` converged on a
/// point that collapsed under a 4th-decimal-digit rounding of its own
/// parameters. A peak that narrow is a property of one deterministic
/// trajectory, not of the robot.
///
/// So: fine gain steps either side, then the same timestep change that
/// exposed the hip-gate result as an artifact.
#[test]
#[ignore = "sweep -- run with --ignored"]
fn namiashi_staircase_5cm_terrain_stance_robustness() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();

    let run = |gain: f64, dt: f64, label: &str| {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            terrain_stance: Some(TerrainStanceCfg { gain, max_offset_m: 0.10 }),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let upright = collapse_index(&samples, &stairs).unwrap_or(samples.len());
        let fell = upright < samples.len();
        let walking = &samples[..upright.max(1)];
        let max_x = walking.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let max_z = walking.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let steps = ((max_z - samples[0].body_z) / 0.05).max(0.0);
        let raw_max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let diverged = raw_max_x > top_start_x + stairs.top_platform_m;
        eprintln!(
            "[stance_rob {label:>12} gain={gain:.3} dt={dt:.5}] max_x={max_x:.3}m \
             (~{steps:.1} steps)  fell={fell}{}",
            if diverged { "  <- DIVERGED" } else { "" },
        );
    };

    eprintln!("[stance_rob] -- gain sweep either side of 1.5 --");
    for gain in [1.30, 1.40, 1.45, 1.50, 1.55, 1.60, 1.70] {
        run(gain, 0.0005, "gain");
    }
    eprintln!("[stance_rob] -- timestep change at the optimum --");
    for scale in [0.8, 1.25] {
        run(1.50, 0.0005 * scale, "dt-change");
    }
}

/// Does the swing-collision reflex get the 5 cm staircase past where blind
/// clearance alone could not?
///
/// `namiashi_staircase_5cm_swing_clearance_sweep` showed swing height was
/// never the blocker -- every value from 0.040 to 0.120 m tips over the same
/// way, ~1 s after first contact with the riser. `ContactReflexCfg` acts on
/// the signal that actually discriminates the collision (swing FK error
/// spiking to ~0.09 m against a flat-ground ceiling of ~0.05 m): trigger at
/// 0.065 m -- above every flat-ground sample seen, below the measured
/// collision spike -- lift 0.06 m straight up from wherever the foot
/// actually is, resume normal tracking once error falls back under 0.025 m.
///
/// Same wide-platform sizing `namiashi_staircase_rise02_wide_platform`
/// needed to avoid conflating "fell off the test track" with "failed to
/// climb" -- 0.05 m risers taking longer per step than 0.02 m ones, if this
/// works at all, so there is no reason to assume less room is enough.
#[test]
#[ignore = "writes a replay for visual inspection -- run with --ignored"]
fn namiashi_staircase_5cm_contact_reflex() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 20.0,
        wbc_real_inertia: true,
        staircase: Some(stairs),
        contact_reflex: Some(ContactReflexCfg {
            trigger_m: 0.065,
            resume_m: 0.025,
            lift_m: 0.06,
            freeze_phase_during_reflex: true,
        }),
        replay_dir: Some("/tmp/nami_stairs/rise05_reflex".to_string()),
        ..namiashi_tuned_params(I)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
    let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
    let final_s = samples.last().unwrap();
    let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
    eprintln!(
        "[stairs 5cm reflex] reached x={max_x:.3}m  max z={max_z:.3}m  \
         final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
        final_s.body_x, final_s.body_y, final_s.body_z,
    );
}

/// Does perfect, idealized height knowledge alone get the 5 cm staircase
/// climbed -- before spending effort on how a real sensor would acquire it?
///
/// `TerrainFootplanCfg` queries `StaircaseCfg::height_at` (exact, no sensor
/// noise, no mapping latency -- the best case for perception) every tick and
/// raises each swinging foot's z target to `terrain height + clearance`,
/// continuously, not just reactively after a collision the way
/// `ContactReflexCfg` did. Horizontal touchdown placement is untouched --
/// isolating whether vertical clearance informed by real height knowledge is
/// the missing piece, same discipline as the earlier blind clearance sweep.
#[test]
#[ignore = "writes a replay for visual inspection -- run with --ignored"]
fn namiashi_staircase_5cm_terrain_footplan() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 20.0,
        wbc_real_inertia: true,
        staircase: Some(stairs),
        terrain_footplan: Some(TerrainFootplanCfg { clearance_m: 0.02, horizontal_margin_m: 0.05 }),
        replay_dir: Some("/tmp/nami_stairs/rise05_footplan".to_string()),
        ..namiashi_tuned_params(I)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
    let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
    let final_s = samples.last().unwrap();
    let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
    eprintln!(
        "[stairs 5cm footplan] reached x={max_x:.3}m  max z={max_z:.3}m  \
         final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
        final_s.body_x, final_s.body_y, final_s.body_z,
    );
}

/// Same terrain footplan (vertical clearance + horizontal snap), on Walk
/// instead of Trot -- is the flat-ground-tuned gait's own timing (0.320 s
/// period, 0.50 duty for Trot vs 0.500 s, 0.75 duty for Walk) part of why
/// every footplan variant collapses shortly after engaging the riser,
/// independent of how the touchdown target itself is computed? Walk keeps
/// more feet down for more of the cycle and moves an order of magnitude
/// slower, which should give the whole-body balance far more margin to
/// absorb whatever a stairs-specific disruption costs -- a much cheaper
/// hypothesis to test than any further per-leg placement tuning.
#[test]
#[ignore = "writes a replay for visual inspection -- run with --ignored"]
fn namiashi_staircase_5cm_terrain_footplan_walk() {
    const I: usize = 1; // Walk
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 25.0,
        wbc_real_inertia: true,
        staircase: Some(stairs),
        terrain_footplan: Some(TerrainFootplanCfg { clearance_m: 0.02, horizontal_margin_m: 0.05 }),
        replay_dir: Some("/tmp/nami_stairs/rise05_footplan_walk".to_string()),
        ..namiashi_tuned_params(I)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
    let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
    let final_s = samples.last().unwrap();
    let reached_top = final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
    eprintln!(
        "[stairs 5cm footplan walk] reached x={max_x:.3}m  max z={max_z:.3}m  \
         final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
        final_s.body_x, final_s.body_y, final_s.body_z,
    );
}

/// Is the 5 cm staircase's collapse a WBC priority-structure limit, the same
/// one `namiashi_contact_force_authority` measured on flat ground -- or does
/// it not transfer here?
///
/// That earlier test found `base_accel` (default weight 200.0, against
/// `swing_leg`'s 1.0) crowding out essentially all of the 13 dimensions
/// priority 0 leaves behind, and dropping it to 2.0 took flat-ground push
/// recovery from -0.497 m of lateral drift to -0.017 m -- the best result
/// anywhere in that investigation. Every terrain-footplan variant tried on
/// this staircase reaches tread-1 height and then tips over within ~2 s
/// regardless of gait speed or how the touchdown target is computed, which
/// stopped looking like a placement problem and started looking like this
/// one. This sweeps `base_accel_weight` on top of the best footplan config
/// (Trot, vertical clearance + horizontal snap) to check directly rather
/// than assume the connection.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_staircase_5cm_base_accel_weight_sweep() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let top_start_x = stairs.top_platform_start_x();
    for base_accel_weight in [200.0_f64, 20.0, 5.0, 2.0, 0.5] {
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            terrain_footplan: Some(TerrainFootplanCfg {
                clearance_m: 0.02,
                horizontal_margin_m: 0.05,
            }),
            base_accel_weight: Some(base_accel_weight),
            replay_dir: Some(format!("/tmp/nami_stairs/rise05_ba_{base_accel_weight:.1}")),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top =
            final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        eprintln!(
            "[stairs 5cm ba={base_accel_weight:.1}] reached x={max_x:.3}m  max z={max_z:.3}m  \
             final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
            final_s.body_x, final_s.body_y, final_s.body_z,
        );
    }
}

/// Sanity check before trusting the staircase null-space measurement: does
/// this same low-support pattern appear on *flat* ground under the exact
/// same params the staircase tests use (`Actuation::Torque`, `wbc_real_
/// inertia: true`, 400 Hz host), with `staircase`/`terrain_footplan` simply
/// removed? If so, the anomaly is in this param combination, not in the
/// terrain.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_flat_ground_same_params_as_staircase_nullspace_check() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 8.0,
        wbc_real_inertia: true,
        ..namiashi_tuned_params(I)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk("flat same-params-as-staircase", &samples, cmd, 1.0);
}

/// Narrows the anomaly `namiashi_flat_ground_same_params_as_staircase_
/// nullspace_check` ruled out (it is not the actuation/inertia params):
/// is it the box-built approach floor itself (`StaircaseCfg`, even on its
/// flat section), or `TerrainFootplanCfg` (which should be a no-op on flat
/// ground per its own `terrain_z <= 1e-6` guard, but "should be" is exactly
/// what needs checking rather than assuming)? Pushes the first riser far
/// enough away (approach_m=100) that the whole run stays on the box floor,
/// with terrain_footplan left off.
#[test]
#[ignore = "diagnostic -- run with --ignored"]
fn namiashi_staircase_box_floor_only_nullspace_check() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 100.0,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };
    let params = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        cmd_vx: cmd,
        total_time_s: 8.0,
        wbc_real_inertia: true,
        staircase: Some(stairs),
        ..namiashi_tuned_params(I)
    };
    let Some(samples) = run_wbc_sim(params) else { return };
    report_walk("box-floor-only (no stairs reached)", &samples, cmd, 1.0);
}

/// Where between 2 cm (climbs cleanly) and 5 cm (never climbs) does this
/// stop working? Namiashi's flat-ground swing clearance is a fixed
/// 0.035-0.045 m -- 3 cm sits inside that budget, 4 cm sits right at its
/// edge, 5 cm is already past it. Reuses the best config found so far
/// (Trot, terrain footplan with vertical clearance + horizontal snap, wide
/// platform sizing) at each rise, changing only `rise_m`.
#[test]
#[ignore = "large sweep -- run with --ignored"]
fn namiashi_staircase_rise_sweep_3_4cm() {
    const I: usize = 0; // Trot
    let (_, _period, .., cmd) = NAMIASHI_TUNED[I];
    for rise_m in [0.03_f64, 0.04, 0.05] {
        let stairs = StaircaseCfg {
            rise_m,
            run_m: 0.20,
            n_steps: 10,
            approach_m: 1.5,
            top_platform_m: 8.0,
            half_width_m: 6.0,
        };
        let top_start_x = stairs.top_platform_start_x();
        let params = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: cmd,
            total_time_s: 20.0,
            wbc_real_inertia: true,
            staircase: Some(stairs),
            terrain_footplan: Some(TerrainFootplanCfg {
                clearance_m: 0.02,
                horizontal_margin_m: 0.05,
            }),
            yaw_pd_gain: Some((200.0, 20.0)),
            replay_dir: Some(format!("/tmp/nami_stairs/rise{:.0}cm_footplan_yaw", rise_m * 100.0)),
            ..namiashi_tuned_params(I)
        };
        let Some(samples) = run_wbc_sim(params) else { return };
        let max_x = samples.iter().map(|s| s.body_x).fold(f64::NEG_INFINITY, f64::max);
        let max_z = samples.iter().map(|s| s.body_z).fold(f64::NEG_INFINITY, f64::max);
        let final_s = samples.last().unwrap();
        let reached_top =
            final_s.body_x >= top_start_x && final_s.body_z > TRUNK_Z_FALL_THRESHOLD_M;
        eprintln!(
            "[stairs rise={:.0}cm] reached x={max_x:.3}m  max z={max_z:.3}m  \
             final (x,y,z)=({:.3},{:.3},{:.3})m  reached_top={reached_top}",
            rise_m * 100.0,
            final_s.body_x, final_s.body_y, final_s.body_z,
        );
    }
}
