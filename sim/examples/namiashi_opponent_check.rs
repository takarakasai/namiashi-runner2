//! Is the opponent robot actually in the scene, crouched, and resting on the
//! ring?
//!
//! `locked_robot_worldbody_xml` builds it by rewriting a re-export of the
//! same model -- deleting every hinge and baking the angle it held into the
//! child body's orientation. Three things can go wrong quietly there: the
//! XML can fail to splice and leave no opponent at all, the welds can land
//! at the wrong angles and leave it standing tall instead of crouched, and
//! it can spawn intersecting the ring and be shoved out. So: count the
//! bodies, measure the trunk height, and let it settle.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_opponent_check`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
use namiashi_sim::ring::{locked_robot_worldbody_xml, KawasakiRingCfg, LockedPose};
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");
    let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");

    for pose in [LockedPose::default(), LockedPose { leg: [0.0, 0.9, -1.8], arm: 0.85 }] {
        let opponent = locked_robot_worldbody_xml(
            &robot,
            "foe_",
            // Ring centre, high enough to drop rather than to start inside
            // the centre bowl's rim.
            [0.0, 0.0, ring.z_top_m() + 0.30],
            pose,
        );
        assert!(!opponent.is_empty(), "opponent XML came back empty");

        let opts = MjcfExportOptions {
            base_xy: Some((
                -(ring.ring_m / 2.0 + pd / 2.0),
                -(ring.ring_m / 2.0 - pw / 2.0),
            )),
            extra_asset_xml: Some(ring.asset_xml("kawasaki")),
            extra_worldbody_xml: Some(format!(
                "{}\n{opponent}",
                ring.worldbody_xml("kawasaki")
            )),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        // Dumped so the scene can be looked at, not just measured -- an
        // opponent at the wrong scale or facing the wrong way passes every
        // number above.
        if pose == LockedPose::default() {
            let dir = std::env::args()
                .position(|a| a == "--dump")
                .and_then(|i| std::env::args().nth(i + 1));
            if let Some(dir) = dir {
                std::fs::create_dir_all(&dir).expect("mkdir");
                std::fs::write(
                    format!("{dir}/model.xml"),
                    articara::mjcf::export_mjcf_with_options(&robot, opts.clone()),
                )
                .expect("write xml");
                let (nrow, ncol) = ring.grid();
                let h = ring.heights();
                let zt = ring.z_top_m();
                let mut pgm = format!("P2\n{ncol} {nrow}\n255\n");
                for r in (0..nrow).rev() {
                    for c in 0..ncol {
                        let v = (h[r * ncol + c] as f64 / zt.max(1e-9) * 255.0)
                            .round()
                            .clamp(0.0, 255.0) as u32;
                        pgm.push_str(&format!("{v} "));
                    }
                    pgm.push('\n');
                }
                std::fs::write(format!("{dir}/ring_elevation.pgm"), pgm).expect("write pgm");
                println!("dumped {dir}/model.xml");
            }
        }
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
        let dt = sim.timestep();

        let z0 = sim.body_world_position("foe_trunk").expect("no foe_trunk body");
        for _ in 0..(2.0 / dt) as u32 {
            sim.step(&mut robot, dt, true);
        }
        let z = sim.body_world_position("foe_trunk").unwrap();
        let feet: Vec<f64> = ["FL", "FR", "RL", "RR"]
            .iter()
            .filter_map(|p| sim.body_world_position(&format!("foe_{p}_foot")))
            .map(|p| p[2])
            .collect();
        let mine = sim.body_world_position(&robot.root_link).unwrap();
        println!(
            "pose leg {:?} arm {:.2}\n  spawned trunk z {:.3} -> settled {:.3} m at \
             ({:+.3},{:+.3})\n  ring surface here {:.3} m, feet z {:?}\n  \
             my own trunk {:.3} m (unaffected)",
            pose.leg,
            pose.arm,
            z0[2],
            z[2],
            z[0],
            z[1],
            ring.height_at(0.0, 0.0),
            feet.iter().map(|v| (v * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
            mine[2],
        );
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
