//! Interactive, real-time keyboard teleop of the WBC/MPC pipeline on the
//! same 5 cm / 10-step staircase every WBC/MPC measurement in
//! `tests/wbc_walk.rs` is built on -- the model-based counterpart to
//! `namiashi_rl_teleop.rs`, sharing the same key bindings so the same
//! muscle memory drives either controller.
//!
//! This is a proper binary, not a `#[test]`, because `mujoco-rs`'s
//! viewer creates a `winit` event loop, and `winit` requires that to
//! happen on the real process main thread -- `cargo test`'s harness
//! always runs each test body on a worker thread (by design, so tests
//! can run in parallel), which the viewer cannot tolerate regardless of
//! `--test-threads`. A `cargo run` binary's `fn main()` IS the process
//! main thread, so this works where a `#[test]`-based version could not.
//!
//! Calls `namiashi_sim::wbc_harness::run_wbc_sim` directly -- the exact same,
//! already-validated WBC/MPC pipeline (misa-wbc QP, GaitController,
//! contact reflex/footplan machinery, all of it) that every other
//! measurement in `tests/wbc_walk.rs` uses, not a simplified stand-in.
//!
//! Run: `cargo run --release --no-default-features --features
//! "mujoco,mujoco-viewer" --example namiashi_wbc_teleop`
//!
//! Keys: press `K` in the viewer, or see `namiashi_sim::teleop`'s module docs.
//! Holding a key moves, releasing it stops. Two worth knowing about here:
//! `B` toggles the proprioceptive trunk levelling, and `P`/`.` change what
//! the controller BELIEVES the friction is, separately from `O`/`L` which
//! change the friction itself -- the two were found to disagree (0.5
//! assumed against 0.7 simulated).
//!
//! Each gait keeps its own tuned speed envelope (Crawl 0.17, Walk 0.33,
//! Trot 0.80 m/s), since each is bounded by its own
//! `max_step_length_m / (cycle_period_s * duty_factor)` -- commanding
//! Trot's speed in Crawl would just saturate. Switching gait is cleanest
//! from a standstill: the phase generator holds its cycle phase across a
//! swap, so a mid-stride switch snaps the legs to their new offsets.

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use std::sync::{Arc, Mutex};

    use articara::mjcf::StaircaseCfg;
use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{
        namiashi_tuned_params, run_wbc_sim, Actuation, ProprioStanceCfg, WbcParams,
    };
    use quadruped_gait::GaitType;

    // `--field ring` (default) or `--field stairs`.
    let args: Vec<String> = std::env::args().collect();
    let field = args
        .iter()
        .position(|a| a == "--field")
        .and_then(|i| args.get(i + 1).cloned())
        .unwrap_or_else(|| "ring".into());
    // Heightfield grid pitch, mm. Physics does not care (5 mm and 40 mm
    // benchmark identically -- see examples/kawasaki_ring_bench), but the
    // grid is drawn as 2 triangles per cell, so 5 mm is ~289k triangles and
    // a host on software OpenGL will feel that. Exposed so the trade
    // against hole fidelity can be made without a code edit.
    let cell_mm: f64 = args
        .iter()
        .position(|a| a == "--cell-mm")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(5.0);
    // Frames per second to draw. Lower it when the DISPLAY is the
    // bottleneck (ssh -X, software GL): see WbcParams::render_hz.
    // A joint-locked opponent at the ring centre, on by default with
    // `--no-opponent` to turn it off. Real masses and collision shapes, no
    // actuators and no hinges, but a free root joint -- it cannot act, and
    // it can be turned over.
    let opponent = !args.iter().any(|a| a == "--no-opponent");
    let render_hz: f64 = args
        .iter()
        .position(|a| a == "--render-hz")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(60.0);

    // Start stopped, in Trot -- NAMIASHI_TUNED[0], the known-good preset.
    // Deliberately NOT the hip_bias_gate experiment: that was shown
    // non-robust under trivial parameter perturbation
    // (namiashi_staircase_5cm_hip_gate_robustness) and has no place in a
    // hands-on demo.
    let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));

    let base = WbcParams {
        actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
        host_rate_hz: Some(400.0),
        dt: 0.0005,
        // Stand still until a key is pressed; live_teleop overrides this
        // from the first post-burn-in tick anyway.
        cmd_vx: 0.0,
        total_time_s: 1800.0, // ends via the viewer window closing, not a timeout
        wbc_real_inertia: true,
        live_teleop: Some(live),
        live_viewer: true,
        render_hz: Some(render_hz),
        // Configured always; `B` gates it live. Proprioceptive only -- IMU
        // and encoders, no terrain oracle -- so it is honest to leave on.
        proprio_stance: Some(ProprioStanceCfg::default()),
        ..namiashi_tuned_params(0)
    };
    let params = match field.as_str() {
        "stairs" => WbcParams {
            staircase: Some(StaircaseCfg {
                rise_m: 0.05,
                run_m: 0.20,
                n_steps: 10,
                approach_m: 1.5,
                top_platform_m: 8.0,
                half_width_m: 6.0,
            }),
            ..base
        },
        "ring" => {
            let ring = KawasakiRingCfg { cell_m: cell_mm / 1000.0, ..Default::default() };
            // On the red start platform, facing the ring. Spawning at the
            // origin would drop the robot onto the centre bowl.
            let (w, d) = ring.red_platform_m;
            let foe = opponent.then(|| {
                (LockedPose::default(), [0.0, 0.0, ring.z_top_m() + 0.30])
            });
            WbcParams {
                spawn_xy: Some((-(ring.ring_m / 2.0 + d / 2.0), -(ring.ring_m / 2.0 - w / 2.0))),
                opponent: foe,
                kawasaki_ring: Some(ring),
                ..base
            }
        }
        other => {
            eprintln!("unknown --field {other:?}: expected \"ring\" or \"stairs\"");
            std::process::exit(2);
        }
    };
    eprintln!("[teleop] press K in the viewer for the controls list");
    if opponent && field == "ring" {
        // The range is the whole trick: measured in
        // `examples/namiashi_attack_sweep`, the arm does not reach from
        // 0.45 m and by 0.32 m the robot tips the opponent by walking into
        // it whatever the arm does.
        eprintln!(
            "[teleop] opponent at the ring centre -- drive to about 0.38 m out, \
             facing it, then press X"
        );
    }
    eprintln!("[teleop] field = {field}  (--field ring | stairs, --cell-mm {cell_mm}, --render-hz {render_hz})");
    run_wbc_sim(params);
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer")))]
fn main() {
    eprintln!(
        "this example needs: cargo run --release --no-default-features \
         --features mujoco,mujoco-viewer --example namiashi_wbc_teleop"
    );
    std::process::exit(2);
}
