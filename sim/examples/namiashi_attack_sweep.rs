//! What does the attack actually need to connect?
//!
//! The first run of `namiashi_arm_attack_tips_the_opponent` had every phase
//! working -- the trunk dropped 26 mm and rose 21 mm, the arm swung -- and
//! the robot still ended 0.19 m short of the opponent, because the creep
//! moved it BACKWARDS by 16 mm over two seconds. So this sweeps the two
//! things that decide whether the arm ever arrives: how fast the creep is
//! commanded and how far out the attack starts.
//!
//! Reports the opponent's final trunk +z (+1 as placed, -1 flat on its
//! back), how far the opponent was pushed, and how far the attacker
//! actually travelled during the creep -- the last of which is the one that
//! explains the others.
//!
//! Run: `cargo run --release --no-default-features --features
//!       "mujoco,mujoco-viewer" --example namiashi_attack_sweep`

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer"))]
fn main() {
    use namiashi_sim::attack::AttackParams;
    use namiashi_sim::ring::{KawasakiRingCfg, LockedPose};
    use namiashi_sim::teleop::LiveTeleop;
    use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
    use quadruped_gait::GaitType;
    use std::sync::{Arc, Mutex};

    // Which parts are load-bearing. The move flips the opponent from 0.38 m
    // out, but it also shoves it up to 0.7 m along the ring, which is what
    // walking into something looks like as much as what levering it looks
    // like. So each phase gets switched off in turn: if it still flips with
    // the arm never lowered, the arm was not what did it.
    let variants: [(&str, AttackParams); 4] = [
        ("full move", AttackParams::DEFAULT),
        (
            "arm never lowered",
            AttackParams { arm_down: -2.3, arm_up: -2.3, ..AttackParams::DEFAULT },
        ),
        (
            "front never lifts",
            AttackParams { front_rise_m: 0.055, ..AttackParams::DEFAULT },
        ),
        (
            "no crouch, no lift",
            AttackParams { front_drop_m: 0.0, front_rise_m: 0.0, ..AttackParams::DEFAULT },
        ),
    ];
    println!(
        "{:>7} {:>19} | {:>8} {:>9} {:>9} {:>8}",
        "start x", "variant", "foe up", "foe moved", "my creep", "gap left"
    );
    for start_x in [-0.45_f64, -0.38, -0.32] {
        for (label, plan_v) in variants {
            let creep_s = plan_v.creep_s;
            let ring = KawasakiRingCfg::default();
            let live = Arc::new(Mutex::new(LiveTeleop::new(GaitType::Trot)));
            live.lock().unwrap().attack_at_sim_s = Some(2.0);
            let plan = plan_v;
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

            let at = |t: f64| {
                samples
                    .iter()
                    .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))
                    .unwrap()
            };
            // The creep runs from `2 + set_s` to `2 + set_s + creep_s`.
            let (t0, t1) = (2.0 + plan.set_s, 2.0 + plan.set_s + creep_s);
            let travelled = at(t1).body_x - at(t0).body_x;
            let first = samples[0].foe.unwrap();
            let last = samples.last().unwrap().foe.unwrap();
            let moved =
                ((last[0] - first[0]).powi(2) + (last[1] - first[1]).powi(2)).sqrt();
            // How far the arm tip was from the opponent's trunk at the end
            // of the creep: 0.387 m is the tip's reach ahead of trunk centre
            // with the arm down (measured).
            let gap = (last[0] - (at(t1).body_x + 0.387)).max(0.0);
            println!(
                "{start_x:>7.2} {label:>19} | {:>8.3} {moved:>9.3} {travelled:>9.3} \
                 {gap:>8.3}  {}",
                last[3],
                if last[3] < 0.3 { "TIPPED" } else { "" }
            );
        }
    }
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer")))]
fn main() {
    eprintln!("needs --features mujoco,mujoco-viewer");
    std::process::exit(2);
}
