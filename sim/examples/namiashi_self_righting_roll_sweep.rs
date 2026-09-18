//! From which attitudes can namiashi get back up?
//!
//! Everything measured so far started flat on its back. A robot that has
//! been knocked over usually does not: it ends on its side, and which side
//! it leans towards decides whether gravity is helping or fighting. So this
//! sweeps the starting roll from just past upright to fully inverted, in
//! both directions.
//!
//! Both trajectories are run at each angle. `cos(roll)` is the trunk's own
//! +z as PLACED -- 90 degrees is 0.000, on its side -- not where it settles;
//! a robot let go at 60 degrees mostly falls back onto its feet on its own
//! before the recovery does anything.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_roll_sweep`

#[cfg(feature = "mujoco")]
fn main() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::self_righting::{
        evaluate_rolled, Drive, RECOVERY_FAST, RECOVERY_GENTLE,
    };

    let ring = KawasakiRingCfg::default();
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");
    // The teleop's own drive law, so this measures what the `V` key does.
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };
    // Flat venue floor and the open ring surface, at the friction the search
    // never trained on -- two sites rather than one, because "can it get up
    // from its side" should not hinge on a single patch of ground.
    let sites: [(&str, (f64, f64)); 2] =
        [("flat floor", (1.60, 1.60)), ("ring surface", (0.35, 0.35))];

    println!(
        "{:>13} {:>7} {:>9} | {:>8} {:>8} | {:>8} {:>8}",
        "site", "roll", "cos(roll)", "gentle", "t", "fast", "t"
    );
    for (name, xy) in sites {
        for deg in [60.0_f64, 90.0, 120.0, 180.0, -60.0, -90.0, -120.0] {
            let roll = deg.to_radians();
            let g = evaluate_rolled(
                &misa, &ring, xy, 0.70, &RECOVERY_GENTLE, 20.0, teleop, None, roll,
            );
            let f = evaluate_rolled(
                &misa, &ring, xy, 0.70, &RECOVERY_FAST, 20.0, teleop, None, roll,
            );
            // Both start from the same drop, so either reports the same
            // settled attitude; take it from one.
            println!(
                "{name:>13} {deg:>6.0}\u{b0} {:>9.3} | {:>8.3} {:>8.2} | {:>8.3} {:>8.2}  {}",
                deg.to_radians().cos(),
                g.final_up,
                g.t_right_s,
                f.final_up,
                f.t_right_s,
                match (g.righted(), f.righted()) {
                    (true, true) => "both",
                    (true, false) => "gentle only",
                    (false, true) => "fast only",
                    (false, false) => "neither",
                }
            );
        }
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
