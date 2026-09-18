//! Why is the ring slow? Times a fixed amount of simulated time on each
//! field so the cost can be attributed rather than guessed at.
//!
//! The teleop paces itself to real time, so a real-time factor under 1
//! shows up directly as a low frame rate -- 2.5 fps against a 60 fps target
//! means the physics is running ~24x slower than real time. That is a
//! physics-cost problem, not a rendering one, and the ring is the only new
//! thing in the loop.
//!
//! Compares flat ground and the staircase (both boxes/planes, both already
//! known to run comfortably faster than real time) against the ring at
//! several heightfield resolutions. If cost tracks the grid pitch, the fix
//! is resolution; if the ring is slow even when coarse, the heightfield
//! itself is the wrong representation here and boxes would be better.
//!
//! Run: `cargo run --release --no-default-features --features mujoco \
//!   --example kawasaki_ring_bench`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions, StaircaseCfg};
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    const DT: f64 = 0.0005; // what both teleop demos run at
    const SIM_S: f64 = 1.0;

    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");

    let stairs = StaircaseCfg {
        rise_m: 0.05,
        run_m: 0.20,
        n_steps: 10,
        approach_m: 1.5,
        top_platform_m: 8.0,
        half_width_m: 6.0,
    };

    println!("{:>22}  {:>9}  {:>8}  {:>7}  {:>9}", "field", "wall/sim", "rt", "ncon", "ngeom");
    for (label, cell_m) in [
        ("flat plane", None),
        ("staircase (boxes)", None),
        ("ring hfield 5 mm", Some(0.005)),
        ("ring hfield 10 mm", Some(0.010)),
        ("ring hfield 20 mm", Some(0.020)),
        ("ring hfield 40 mm", Some(0.040)),
    ] {
        let mut robot = RobotModel::from_misa(&misa).expect("load namiashi");
        let ring = cell_m.map(|c| KawasakiRingCfg { cell_m: c, ..Default::default() });
        let opts = match (&ring, label) {
            (Some(r), _) => {
                let (pw, pd) = r.red_platform_m;
                MjcfExportOptions {
                    base_xy: Some((
                        -(r.ring_m / 2.0 + pd / 2.0),
                        -(r.ring_m / 2.0 - pw / 2.0),
                    )),
                    extra_asset_xml: Some(r.asset_xml("k")),
                    extra_worldbody_xml: Some(r.worldbody_xml("k")),
                    add_actuators: true,
                    ..MjcfExportOptions::default()
                }
            }
            (None, "staircase (boxes)") => MjcfExportOptions {
                extra_worldbody_xml: Some(stairs.worldbody_xml()),
                add_actuators: true,
                ..MjcfExportOptions::default()
            },
            (None, _) => MjcfExportOptions {
                ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 4.0, roll: 0.0, pitch: 0.0 }),
                add_actuators: true,
                ..MjcfExportOptions::default()
            },
        };
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        if let Some(r) = &ring {
            sim.set_hfield_data("k", &r.heights()).expect("fill hfield");
        }
        // Settle first, so the timed window is steady-state contact rather
        // than the initial drop.
        for _ in 0..(0.3 / DT) as u32 {
            sim.step(&mut robot, DT, true);
        }
        let ngeom = sim.mj_model().ffi().ngeom;
        let t0 = std::time::Instant::now();
        let mut ncon_sum = 0usize;
        let n = (SIM_S / DT) as u32;
        for _ in 0..n {
            sim.step(&mut robot, DT, true);
            ncon_sum += sim.contacts().len();
        }
        let wall = t0.elapsed().as_secs_f64();
        println!(
            "{label:>22}  {:>8.2}s  {:>8.2}  {:>7.1}  {ngeom:>9}",
            wall,
            SIM_S / wall,
            ncon_sum as f64 / n as f64,
        );
    }

    // Physics alone is only half the teleop loop; the other half is the
    // WBC/MPC pipeline (GaitController + misa-wbc QP + MPC) that run_wbc_sim
    // drives every host tick. Time that too, on both fields, since a cost
    // that is the same on each would clear the ring entirely.
    println!("\n{:>22}  {:>9}  {:>8}", "run_wbc_sim on", "wall/sim", "rt");
    for label in ["staircase", "ring"] {
        use namiashi_sim::wbc_harness::{namiashi_tuned_params, run_wbc_sim, Actuation, WbcParams};
        let ring = KawasakiRingCfg::default();
        let (pw, pd) = ring.red_platform_m;
        let base = WbcParams {
            actuation: Actuation::Torque { kp: 100.0, kd: 1.2 },
            host_rate_hz: Some(400.0),
            dt: DT,
            cmd_vx: 0.3,
            total_time_s: 2.0,
            wbc_real_inertia: true,
            ..namiashi_tuned_params(0)
        };
        let params = if label == "ring" {
            WbcParams {
                spawn_xy: Some((
                    -(ring.ring_m / 2.0 + pd / 2.0),
                    -(ring.ring_m / 2.0 - pw / 2.0),
                )),
                kawasaki_ring: Some(ring),
                ..base
            }
        } else {
            WbcParams { staircase: Some(stairs), ..base }
        };
        let t0 = std::time::Instant::now();
        let got = run_wbc_sim(params).is_some();
        let wall = t0.elapsed().as_secs_f64();
        println!("{label:>22}  {:>8.2}s  {:>8.2}{}", wall, 2.0 / wall,
                 if got { "" } else { "  (skipped)" });
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs: cargo run --features mujoco --example kawasaki_ring_bench");
    std::process::exit(2);
}
