//! Record the self-righting recovery for rendering.
//!
//! Writes, per variant, a directory holding the exported MJCF, the ring's
//! elevation grid as a PGM, and a `trace.csv` replay. The elevation has to
//! travel separately: the `<hfield>` is declared with `nrow`/`ncol` and no
//! `file=`, so anything loading the XML on its own gets an all-zero grid and
//! renders a perfectly flat ring -- plausible-looking, and wrong.
//!
//! Runs through `evaluate_traced`, the same function the search and the
//! tests use, so the video cannot drift from what was measured.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_video -- --out /tmp/nami_right`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::robot::RobotModel;
    use namiashi_sim::self_righting::{
        evaluate_traced, Drive, RECOVERY_FAST, RECOVERY_GENTLE,
    };

    let args: Vec<String> = std::env::args().collect();
    let out = args
        .iter()
        .position(|a| a == "--out")
        .and_then(|i| args.get(i + 1).cloned())
        .unwrap_or_else(|| "/tmp/nami_right".into());

    let ring = KawasakiRingCfg::default();
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");
    // On the open ring surface at the friction the search never trained on,
    // which is a condition both variants have to earn rather than one picked
    // for looking good.
    let site = (0.35, 0.35);
    let mu = 0.70;
    let teleop = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };

    for (sub, plan, secs) in [
        ("gentle", RECOVERY_GENTLE, 20.0),
        ("fast", RECOVERY_FAST, 20.0),
    ] {
        let dir = format!("{out}/{sub}");
        std::fs::create_dir_all(&dir).expect("mkdir");

        let robot = RobotModel::from_misa(&misa).expect("load robot");
        let opts = MjcfExportOptions {
            base_xy: Some(site),
            extra_asset_xml: Some(ring.asset_xml("kawasaki")),
            extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        std::fs::write(
            format!("{dir}/model.xml"),
            articara::mjcf::export_mjcf_with_options(&robot, opts),
        )
        .expect("write xml");

        let (nrow, ncol) = ring.grid();
        let heights = ring.heights();
        let z_top = ring.z_top_m();
        let mut pgm = format!("P2\n{ncol} {nrow}\n255\n");
        // Top row is +y, so the file reads like a plan view; the renderer
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

        let mut rows = Vec::new();
        let o = evaluate_traced(
            &misa,
            &ring,
            site,
            mu,
            &plan,
            secs,
            teleop,
            Some((&mut rows, 1.0 / 60.0)),
        );

        let mut csv = String::from(
            "t,root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz,\
             FL_hip_joint,FL_thigh_joint,FL_calf_joint,\
             FR_hip_joint,FR_thigh_joint,FR_calf_joint,\
             RL_hip_joint,RL_thigh_joint,RL_calf_joint,\
             RR_hip_joint,RR_thigh_joint,RR_calf_joint,arm_pitch_joint,up_est,handed_back\n",
        );
        for (t, p, q, up_est, handed) in &rows {
            csv.push_str(&format!("{t:.5}"));
            for v in p.iter().chain(q.iter()) {
                csv.push_str(&format!(",{v:.6}"));
            }
            csv.push_str(&format!(",{up_est:.6},{}\n", u8::from(*handed)));
        }
        std::fs::write(format!("{dir}/trace.csv"), csv).expect("write trace");

        // Written next to the trace so the overlay quotes measurements
        // rather than restating them by hand.
        std::fs::write(
            format!("{dir}/outcome.txt"),
            format!(
                "final_up {:.3}\npeak_up {:.3}\nt_right_s {:.2}\ntravel_m {:.3}\n\
                 rms_omega {:.2}\nrms_joint {:.2}\nmax_rate {:.2}\n",
                o.final_up,
                o.peak_up,
                o.t_right_s,
                o.travel_m,
                o.rms_omega_rad_s,
                o.rms_joint_rad_s,
                plan.max_rate_rad_s,
            ),
        )
        .expect("write outcome");
        println!(
            "{sub}: {} rows, final up {:.3}, t_right {:.2} s, RMS w {:.2} qd {:.2}  -> {dir}",
            rows.len(),
            o.final_up,
            o.t_right_s,
            o.rms_omega_rad_s,
            o.rms_joint_rad_s,
        );
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
