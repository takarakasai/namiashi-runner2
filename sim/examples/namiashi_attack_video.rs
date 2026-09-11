//! Record the arm attack for rendering.
//!
//! Runs the real `X` path -- `run_wbc_sim` with the gait, the WBC and the
//! joint-locked opponent -- and writes the harness's own replay trace, so
//! the clip cannot show a move different from the one the regression test
//! measures.
//!
//! Records two runs at the same range: the move, and the move with the arm
//! left up. That second one is the point of the video as much as the first,
//! because the honest question about a robot walking at something and
//! knocking it over is whether the arm did anything, and 0.38 m is close
//! enough that it is not obvious by eye.
//!
//! The ring's elevation travels as a separate PGM: the `<hfield>` is
//! declared with nrow/ncol and no `file=`, so anything loading the XML on
//! its own draws a flat ring and looks entirely plausible doing it.
//!
//! Run: `cargo run --release --no-default-features --features
//!       "mujoco,mujoco-viewer" --example namiashi_attack_video --
//!       --out /tmp/nami_attack`

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use namiashi_sim::attack::AttackParams;
    use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    let args: Vec<String> = std::env::args().collect();
    let out = args
        .iter()
        .position(|a| a == "--out")
        .and_then(|i| args.get(i + 1).cloned())
        .unwrap_or_else(|| "/tmp/nami_attack".into());

    // The range the sweep found: from 0.45 m the arm never arrives, and by
    // 0.32 m the opponent goes over whatever the arm does.
    const RANGE_M: f64 = -0.38;

    for (sub, plan) in [
        ("attack", AttackParams::DEFAULT),
        (
            "no_arm",
            AttackParams { arm_down: -2.3, arm_up: -2.3, ..AttackParams::DEFAULT },
        ),
    ] {
        let dir = format!("{out}/{sub}");
        let ring = KawasakiRingCfg::default();
        let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
        live.lock().unwrap().attack_at_sim_s = Some(2.0);

        let samples = run_wbc_sim(WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: 0.0005,
            cmd_vx: 0.0,
            total_time_s: 10.0,
            wbc_real_inertia: true,
            live_teleop: Some(Arc::clone(&live)),
            live_viewer: false,
            spawn_xy: Some((RANGE_M, 0.0)),
            opponent: Some((LockedPose::default(), [0.0, 0.0, ring.z_top_m() + 0.30])),
            kawasaki_ring: Some(ring.clone()),
            attack_params: plan,
            replay_dir: Some(dir.clone()),
            ..namiashi_tuned_params(0)
        })
        .expect("run_wbc_sim");

        let (nrow, ncol) = ring.grid();
        let heights = ring.heights();
        let z_top = ring.z_top_m();
        let mut pgm = format!("P2\n{ncol} {nrow}\n255\n");
        // Top row is +y so the file reads like a plan view; the renderer
        // flips it back for MuJoCo, whose grid starts at -y.
        for r in (0..nrow).rev() {
            for c in 0..ncol {
                let v = (heights[r * ncol + c] as f64 / z_top.max(1e-9) * 255.0)
                    .round()
                    .clamp(0.0, 255.0) as u32;
                pgm.push_str(&format!("{v} "));
            }
            pgm.push('\n');
        }
        std::fs::write(format!("{dir}/ring_elevation.pgm"), pgm).expect("write pgm");

        let first = samples[0].foe.expect("no opponent");
        let last = samples.last().unwrap().foe.unwrap();
        let moved = ((last[0] - first[0]).powi(2) + (last[1] - first[1]).powi(2)).sqrt();
        std::fs::write(
            format!("{dir}/outcome.txt"),
            format!(
                "foe_up_start {:.3}\nfoe_up_end {:.3}\nfoe_moved {moved:.3}\n\
                 attack_at_s 2.00\nset_s {:.2}\ncreep_s {:.2}\nlift_s {:.2}\n",
                first[3], last[3], plan.set_s, plan.creep_s, plan.lift_s,
            ),
        )
        .expect("write outcome");
        println!(
            "{sub}: opponent up {:+.3} -> {:+.3}, moved {moved:.3} m  -> {dir}",
            first[3], last[3],
        );
    }
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer")))]
fn main() {
    eprintln!("needs --features mujoco,mujoco-viewer");
    std::process::exit(2);
}
