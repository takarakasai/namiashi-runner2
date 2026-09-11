//! Does the searched recovery still work through the control path the teleop
//! actually uses?
//!
//! `examples/namiashi_self_righting_search` optimises against the .misa's
//! Position actuators and reads attitude straight out of the simulator.
//! `run_wbc_sim` does neither: its leg joints are in torque mode driven by a
//! host-side PD with gravity compensation, and attitude comes from a
//! Madgwick filter fed by the trunk IMU. Either difference could break a
//! trajectory that only ever ran under the other, so this re-runs the
//! winner over the full fifteen-condition grid under both.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_check`

#[cfg(feature = "mujoco")]
fn main() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::self_righting::{
        evaluate_with, Drive, GENTLE_JOINT_RAD_S, GENTLE_OMEGA_RAD_S, GENTLE_TRAVEL_M, RECOVERY_GENTLE,
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
    // The teleop's own gains and IMU filter constant, so a disagreement here
    // is a disagreement about the robot and not about the tuning.
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };

    println!(
        "{:<22} {:>5} | {:>7} | {:>7} {:>6} {:>7} {:>7} {:>7}",
        "site", "mu", "pos up", "up", "t", "d_xy m", "w rms", "qd rms"
    );
    let (mut ok_p, mut ok_t, mut total) = (0, 0, 0);
    for (name, xy) in sites {
        for mu in [0.30_f64, 0.70, 1.00] {
            // 20 s, matching the search's validation horizon. At 8 s a
            // recovery that stands up at 6 s is scored mid-transition.
            let p = evaluate_with(&misa, &ring, xy, mu, &RECOVERY_GENTLE, 20.0, Drive::Position);
            let t = evaluate_with(&misa, &ring, xy, mu, &RECOVERY_GENTLE, 20.0, teleop);
            total += 1;
            ok_p += p.righted() as u32;
            ok_t += t.righted() as u32;
            println!(
                "{name:<22} {mu:>5.2} | {:>7.3} | {:>7.3} {:>6.2} {:>7.3} {:>7.1} {:>7.1}  {}",
                p.final_up,
                t.final_up,
                t.t_right_s,
                t.travel_m,
                t.rms_omega_rad_s,
                t.rms_joint_rad_s,
                if t.righted() { "RIGHTED" } else { "no" }
            );
        }
    }
    println!("\nposition actuators: {ok_p}/{total}   teleop drive: {ok_t}/{total}");
    println!(
        "gentleness targets (RMS): w <= {GENTLE_OMEGA_RAD_S} rad/s, qd <= \
         {GENTLE_JOINT_RAD_S} rad/s, travel <= {GENTLE_TRAVEL_M} m"
    );
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
