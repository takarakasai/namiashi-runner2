//! Do the self-righting trajectories and the arm attack survive a change of
//! robot?
//!
//! Everything so far was searched and measured on one model,
//! `namiashi_3p3_prop`. The fixtures carry three others that differ in ways
//! this particular set of moves should care about:
//!
//!   * `namiashi_3p3_hip` -- same 3.30 kg, same joints, same actuators, but
//!     the leg mass moved into the hip (thigh 0.021 kg against 0.057, calf
//!     0.021 against 0.059). A recovery that works by throwing legs around
//!     has just lost most of the mass it was throwing.
//!   * `namiashi` and `namiashi2` -- 2.40 kg, and WEAKER: hip and thigh
//!     effort 1.5 N.m against 2.5, calf 2.205 against 3.889. Lighter helps,
//!     less torque does not, and which wins is not obvious.
//!
//! Four fixtures, three models. `namiashi2` differs from `namiashi` only in
//! its leg actuator gains (kp 20 / kv 0.5 against 100 / 1.2), and both this
//! and the teleop put the leg joints in torque mode with a host-side PD, so
//! those gains are never read. The two report identical numbers to three
//! decimals in every cell below, which is the check working rather than a
//! coincidence.
//!
//! Reported as counts over starting attitudes, because a trajectory fitted
//! to one model can fail on another either by being unable to turn the body
//! over at all or by turning it over and not catching it, and the peak
//! column separates those.
//!
//! Run: `cargo run --release --no-default-features --features
//!       "mujoco,mujoco-viewer" --example namiashi_cross_model_check`

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use namiashi_sim::attack::AttackParams;
    use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::self_righting::{
        evaluate_rolled, Drive, RECOVERY_FAST, RECOVERY_GENTLE,
    };
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/namiashi");
    let models: [(&str, &str); 4] = [
        ("3p3_prop (tuned on)", "namiashi_3p3_prop.misa"),
        ("3p3_hip", "namiashi_3p3_hip.misa"),
        ("namiashi (2.4 kg)", "namiashi.misa"),
        ("namiashi2 (2.4 kg)", "namiashi2.misa"),
    ];
    let ring = KawasakiRingCfg::default();
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };
    let sites: [(&str, (f64, f64)); 2] =
        [("flat floor", (1.60, 1.60)), ("ring surface", (0.35, 0.35))];
    let rolls = [180.0_f64, 90.0, -90.0];

    // ---- self-righting -------------------------------------------------
    println!("self-righting, 2 sites x 3 rolls x 3 friction = 18 per cell\n");
    println!(
        "{:>20} | {:>12} {:>12} | {:>12} {:>12}",
        "model", "V righted", "V reached up", "Shift+V", "reached up"
    );
    for (label, file) in models {
        let misa = dir.join(file);
        let (mut g_ok, mut g_peak, mut f_ok, mut f_peak, mut n) = (0, 0, 0, 0, 0);
        for (_, xy) in sites {
            for deg in rolls {
                for mu in [0.30_f64, 0.70, 1.00] {
                    let r = deg.to_radians();
                    let g = evaluate_rolled(
                        &misa, &ring, xy, mu, &RECOVERY_GENTLE, 20.0, teleop, None, r,
                    );
                    let f = evaluate_rolled(
                        &misa, &ring, xy, mu, &RECOVERY_FAST, 20.0, teleop, None, r,
                    );
                    n += 1;
                    g_ok += g.righted() as u32;
                    f_ok += f.righted() as u32;
                    // "Reached upright at some point" separates a trajectory
                    // that cannot turn the body over on this model from one
                    // that turns it over and drops it.
                    g_peak += (g.peak_up > 0.9) as u32;
                    f_peak += (f.peak_up > 0.9) as u32;
                }
            }
        }
        println!(
            "{label:>20} | {g_ok:>9}/{n} {g_peak:>9}/{n} | {f_ok:>9}/{n} {f_peak:>9}/{n}"
        );
    }

    // ---- the arm attack ------------------------------------------------
    //
    // Attacker and opponent are the same model, so this is each robot
    // against itself -- a heavier opponent is harder to tip but a heavier
    // attacker pushes harder, and holding them equal keeps the comparison
    // about the move rather than about a mismatch.
    println!("\narm attack at 0.38 m, attacker and opponent the same model\n");
    println!(
        "{:>20} | {:>9} {:>9} | {:>9} {:>9}",
        "model", "foe up", "moved m", "no-arm up", "moved m"
    );
    for (label, file) in models {
        let misa = dir.join(file);
        let run = |plan: AttackParams| -> (f64, f64) {
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
                    misa.to_string_lossy().into_owned().into_boxed_str(),
                ),
                ..namiashi_tuned_params(0)
            });
            let Some(samples) = samples else { return (f64::NAN, f64::NAN) };
            let first = samples[0].foe.expect("no opponent");
            let last = samples.last().unwrap().foe.unwrap();
            (
                last[3],
                ((last[0] - first[0]).powi(2) + (last[1] - first[1]).powi(2)).sqrt(),
            )
        };
        let (up, moved) = run(AttackParams::DEFAULT);
        let (up_n, moved_n) = run(AttackParams {
            arm_down: -2.3,
            arm_up: -2.3,
            ..AttackParams::DEFAULT
        });
        println!(
            "{label:>20} | {up:>9.3} {moved:>9.3} | {up_n:>9.3} {moved_n:>9.3}  {}",
            if up < 0.3 && up_n > 0.3 {
                "the arm does it"
            } else if up < 0.3 {
                "TIPPED, but so does the no-arm control"
            } else {
                "no tip"
            }
        );
    }
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer")))]
fn main() {
    eprintln!("needs --features mujoco,mujoco-viewer");
    std::process::exit(2);
}
