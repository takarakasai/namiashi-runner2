//! Can namiashi get off its back, and with what?
//!
//! An open-loop probe, not a controller. The point is to find out which
//! actuators can even produce the motion before designing anything around
//! them -- the staircase work in `tests/wbc_walk.rs` spent a long time
//! adding corrections on top of a motion that turned out to be torque
//! saturated, and the cheap way to avoid repeating that is to check the
//! primitive first.
//!
//! Starts from `MujocoSim::respawn_inverted`, so every strategy sees the
//! same on-its-back pose rather than whatever a hand-flip in the viewer
//! happened to produce.
//!
//! Scored by the trunk's own +z axis in world coordinates: +1 upright, 0 on
//! its side, -1 on its back. Peak is reported next to final because the
//! interesting failures are the ones that reach vertical and fall back --
//! those say "nearly", where a flat -1 says "wrong actuator".
//!
//! Phase 1 runs on the flat venue floor on purpose. An earlier version
//! scored on the red platform and its two successes both slid 0.7 m, i.e.
//! straight off a 5 cm plinth: the fall was doing the flipping, not the
//! arm. A plinth edge is a genuine assist, but it has to be measured
//! separately from what the robot can do unaided.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_probe`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");

    // Flat, featureless, well clear of the ring: nothing to slide off, no
    // edge to help.
    const FLAT: (f64, f64) = (1.60, 1.60);
    // Spawn stance, as loaded from the .misa.
    const STANCE: [f64; 3] = [0.0, 0.9, -1.8];
    const ARM_LO: f64 = -2.3;
    const ARM_HI: f64 = 0.85;

    /// One open-loop plan: joint targets as a function of time. All of them
    /// go through the same position actuators the real robot has, including
    /// the arm's, whose kp is 5.0 against the legs' 100.0 -- a ramped target
    /// therefore commands very little arm torque, while a step saturates it.
    #[derive(Clone, Copy)]
    struct Plan {
        name: &'static str,
        /// Seconds from arm-high to arm-low. 0 is a step (max torque).
        sweep_s: f64,
        /// Seconds between sweeps.
        period_s: f64,
        /// Splay the hips as the arm sweeps, to catch the roll.
        legs_help: bool,
        /// 0 splays both sides, +1 pushes with the left legs only, -1 the
        /// right. A symmetric splay is two mirrored pushes and produces no
        /// net roll moment except through whatever asymmetry the dynamics
        /// happen to supply -- which is why the symmetric plans all stall
        /// just past vertical.
        asym: i32,
        /// Tuck the legs instead of holding the stance.
        legs_tuck: bool,
        /// Arm only, no sweep -- the do-nothing control.
        idle: bool,
        /// Add a third phase between "on its side" and "standing": push
        /// across the top with whichever legs are underneath. Without it
        /// every plan stalls at about 50 degrees, because a plain stance
        /// pose has nothing to push against from side-on.
        finish: bool,
    }

    impl Plan {
        /// `up` is the trunk's world +z, fed back for one binary decision
        /// only: whether the flip is done. Without it the legs stay splayed
        /// and the robot settles resting on a splayed leg at 45 degrees --
        /// which the first version of this probe scored as success, because
        /// cos(45) = 0.707 sits just over a 0.7 threshold.
        fn targets(&self, t: f64, up: f64, g_body_y: f64) -> (f64, [[f64; 3]; 4]) {
            // Hand back to a normal stance only once the roll is nearly
            // done. Handing back at 0.3 (72 degrees of tilt) was tried and
            // consistently dropped the robot back onto its back: at that
            // angle it still needs the push.
            if up > 0.85 {
                return (ARM_HI, [STANCE; 4]);
            }
            if self.finish && up > 0.0 {
                // Past vertical and still going over. `g_body_y` is gravity
                // in the trunk frame: its sign says which side of the body
                // is underneath. Extend that side into the ground to push
                // the trunk the rest of the way; fold the other side so it
                // does not catch and stop the roll short.
                let down_is_left = g_body_y > 0.0;
                let push = [if down_is_left { 1.05 } else { -1.05 }, 0.2, -0.5];
                let fold = [0.0, 2.5, -2.6];
                let (l, r) = if down_is_left { (push, fold) } else { (fold, push) };
                return (ARM_HI, [l, r, l, r]);
            }
            if self.idle {
                return (ARM_HI, [STANCE; 4]);
            }
            let phase = t % self.period_s;
            let rest = 0.25 * self.period_s / 2.0;
            let arm = if phase < rest {
                ARM_HI
            } else if self.sweep_s <= 0.0 {
                ARM_LO
            } else {
                let s = ((phase - rest) / self.sweep_s).clamp(0.0, 1.0);
                ARM_HI - (ARM_HI - ARM_LO) * s
            };
            let legs = if self.legs_tuck {
                [[0.0, 2.5, -2.6]; 4]
            } else if self.legs_help {
                // Splay in step with the arm, so the legs are already out to
                // the side when the body reaches vertical and can carry it
                // the rest of the way instead of dropping back.
                let s = ((phase - rest) / self.sweep_s.max(0.05)).clamp(0.0, 1.0);
                let thigh = 0.9 - 0.9 * s;
                let calf = -1.8 + 1.3 * s;
                // Pushing side reaches out; the other side folds in so it
                // does not prop the body up on the far side.
                let (push_l, push_r) = match self.asym {
                    1 => (1.05 * s, -1.05 * s),
                    -1 => (-0.785 * s, 0.785 * s),
                    _ => (1.05 * s, 0.785 * s),
                };
                let (tuck_l, tuck_r) = match self.asym {
                    1 => ([push_r, 2.5, -2.6], [push_r, 2.5, -2.6]),
                    -1 => ([push_l, 2.5, -2.6], [push_l, 2.5, -2.6]),
                    _ => ([push_r, thigh, calf], [push_r, thigh, calf]),
                };
                match self.asym {
                    1 => [
                        [push_l, thigh, calf],
                        tuck_r,
                        [push_l, thigh, calf],
                        tuck_r,
                    ],
                    -1 => [
                        tuck_l,
                        [push_r, thigh, calf],
                        tuck_l,
                        [push_r, thigh, calf],
                    ],
                    _ => [
                        [push_l, thigh, calf],
                        [push_r, thigh, calf],
                        [push_l, thigh, calf],
                        [push_r, thigh, calf],
                    ],
                }
            } else {
                [STANCE; 4]
            };
            (arm, legs)
        }
    }

    const BASE: Plan = Plan {
        name: "",
        sweep_s: 0.35,
        period_s: 2.0,
        legs_help: false,
        asym: 0,
        legs_tuck: false,
        idle: false,
        finish: false,
    };

    /// Run one plan from the inverted pose. Returns (peak up, final up,
    /// time first upright, xy travel).
    fn run(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        plan: Plan,
        hip_limit: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        let mut robot = RobotModel::from_misa(misa).expect("load namiashi");
        // Diagnostic only: widen the hip roll range to tell "this robot
        // cannot reach the pose" apart from "these scripts are bad".
        if let Some(lim) = hip_limit {
            for j in robot.joints.iter_mut() {
                if j.name.ends_with("_hip_joint") {
                    j.lower = -lim;
                    j.upper = lim;
                }
            }
        }
        let opts = MjcfExportOptions {
            base_xy: Some(xy),
            extra_asset_xml: Some(ring.asset_xml("kawasaki")),
            extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
        sim.set_slide_friction_all(mu);
        let dt = sim.timestep();
        let root = robot.root_link.clone();

        sim.respawn_inverted(&mut robot, 0.03);
        // Settle the drop first, so the plan is scored on its motion and not
        // on the landing.
        for _ in 0..(0.5 / dt) as u32 {
            sim.step(&mut robot, dt, true);
        }
        let start = sim.body_world_position(&root).unwrap();

        let arm_ji = robot.joint_map.get("arm_pitch_joint").copied();
        let leg_ji: Vec<[Option<usize>; 3]> = ["FL", "FR", "RL", "RR"]
            .iter()
            .map(|p| {
                [
                    robot.joint_map.get(&format!("{p}_hip_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_thigh_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_calf_joint")).copied(),
                ]
            })
            .collect();

        let (mut peak, mut t_right, mut t) = (-1.0_f64, f64::NAN, 0.0);
        while t < 8.0 {
            let up_now =
                (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
            let r_wb = sim.body_world_orientation(&root).unwrap();
            let g_body_y = (r_wb.inverse() * nalgebra::Vector3::new(0.0, 0.0, -1.0)).y;
            let (arm, legs) = plan.targets(t, up_now, g_body_y);
            if let Some(ji) = arm_ji {
                sim.set_position_target(ji, arm);
            }
            for (leg, targets) in leg_ji.iter().zip(legs.iter()) {
                for (ji, &q) in leg.iter().zip(targets.iter()) {
                    if let Some(ji) = ji {
                        sim.set_position_target(*ji, q);
                    }
                }
            }
            sim.step(&mut robot, dt, true);
            t += dt;
            let up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
            peak = peak.max(up);
            if up > 0.9 && t_right.is_nan() {
                t_right = t;
            }
        }
        let up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
        let end = sim.body_world_position(&root).unwrap();
        let d = ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2)).sqrt();
        (peak, up, t_right, d)
    }

    fn verdict(peak: f64, up: f64) -> &'static str {
        if up > 0.9 {
            "RIGHTED"
        } else if up > 0.5 {
            "on its side, not up"
        } else if peak > 0.9 {
            "righted, then fell back"
        } else if peak > 0.0 {
            "past vertical, fell back"
        } else if peak > -0.7 {
            "lifted, short of vertical"
        } else {
            "no effect"
        }
    }

    // ---- phase 1: which actuator, and how fast --------------------------
    let plans = [
        Plan { name: "nothing (control)", idle: true, ..BASE },
        Plan { name: "arm step (max torque)", sweep_s: 0.0, ..BASE },
        Plan { name: "arm sweep 0.10 s", sweep_s: 0.10, ..BASE },
        Plan { name: "arm sweep 0.35 s", sweep_s: 0.35, ..BASE },
        Plan { name: "arm sweep 0.70 s", sweep_s: 0.70, ..BASE },
        Plan { name: "arm step, legs tucked", sweep_s: 0.0, legs_tuck: true, ..BASE },
        Plan { name: "arm step + leg splay", sweep_s: 0.0, legs_help: true, ..BASE },
        Plan { name: "arm 0.10 s + leg splay", sweep_s: 0.10, legs_help: true, ..BASE },
        Plan { name: "leg splay only", sweep_s: 0.35, legs_help: true, ..BASE },
        Plan { name: "one-side push L", sweep_s: 0.10, legs_help: true, asym: 1, ..BASE },
        Plan { name: "one-side push R", sweep_s: 0.10, legs_help: true, asym: -1, ..BASE },
        Plan {
            name: "one-side L + arm",
            sweep_s: 0.0,
            legs_help: true,
            asym: 1,
            ..BASE
        },
        Plan {
            name: "rock, sym, 0.6 s",
            sweep_s: 0.10,
            period_s: 0.6,
            legs_help: true,
            ..BASE
        },
        Plan {
            name: "rock, one-side L, 0.6 s",
            sweep_s: 0.10,
            period_s: 0.6,
            legs_help: true,
            asym: 1,
            ..BASE
        },
        Plan {
            name: "rock, one-side L, 0.4 s",
            sweep_s: 0.08,
            period_s: 0.4,
            legs_help: true,
            asym: 1,
            ..BASE
        },
        Plan { name: "splay + finish push", sweep_s: 0.10, legs_help: true, finish: true, ..BASE },
        Plan {
            name: "arm + splay + finish",
            sweep_s: 0.0,
            legs_help: true,
            finish: true,
            ..BASE
        },
        Plan {
            name: "rock 0.6 s + finish",
            sweep_s: 0.10,
            period_s: 0.6,
            legs_help: true,
            finish: true,
            ..BASE
        },
    ];
    println!("phase 1 -- flat venue floor, mu = 0.70");
    println!("{:<24} {:>8} {:>8} {:>8}  {}", "plan", "peak up", "final", "d_xy m", "verdict");
    let mut best = plans[1];
    let mut best_score = f64::NEG_INFINITY;
    for p in plans {
        // "leg splay only" means the same splay with the arm parked.
        let plan = if p.name == "leg splay only" { Plan { legs_help: true, ..p } } else { p };
        let (peak, up, _, d) = run(&misa, &ring, FLAT, 0.70, plan, None);
        println!("{:<24} {peak:>8.3} {up:>8.3} {d:>8.3}  {}", p.name, verdict(peak, up));
        if !p.idle && up + 0.1 * peak > best_score {
            best_score = up + 0.1 * peak;
            best = plan;
        }
    }

    // ---- phase 2: does the winner survive being moved -------------------
    //
    // A single success on one spot of one surface is what the hip-bias gate
    // looked like before it turned out to be a spike rather than a basin
    // (namiashi_staircase_5cm_hip_gate_robustness). Nothing in this plan
    // adapts, so any site or friction that breaks it is a real hole.
    let sites: [(&str, (f64, f64)); 5] = [
        ("flat venue floor", FLAT),
        ("open ring surface", (0.35, 0.35)),
        ("ring centre (bowl)", (0.0, 0.0)),
        ("on a round plate", ring.round_plate_centres[0]),
        (
            "red platform (edge)",
            (-(ring.ring_m / 2.0 + pd / 2.0), -(ring.ring_m / 2.0 - pw / 2.0)),
        ),
    ];
    println!("\nphase 2 -- robustness of {:?}", best.name);
    println!(
        "{:<22} {:>5} {:>8} {:>8} {:>8} {:>7}  {}",
        "site", "mu", "peak up", "final", "d_xy m", "t_right", "verdict"
    );
    let (mut ok, mut total) = (0, 0);
    for (site, xy) in sites {
        for mu in [0.30_f64, 0.70, 1.00] {
            let (peak, up, t_right, d) = run(&misa, &ring, xy, mu, best, None);
            total += 1;
            if up > 0.9 {
                ok += 1;
            }
            println!(
                "{site:<22} {mu:>5.2} {peak:>8.3} {up:>8.3} {d:>8.3} {t_right:>7.2}  {}",
                verdict(peak, up)
            );
        }
    }
    println!("\n{ok}/{total} righted");

    // ---- phase 3: is the ceiling kinematic? -----------------------------
    //
    // Every plan above stalls on its side at about 45 degrees. Two very
    // different explanations: the hip roll range (-0.785..1.05 rad) stops
    // the underneath legs reaching far enough beneath the trunk to push it
    // over, or the scripts are simply not good enough. Re-running the same
    // scripts on a model whose only change is a wider hip separates them.
    println!("\nphase 3 -- same plans, flat floor, mu 0.70, hip roll range widened");
    println!("{:<24} {:>9} {:>8} {:>8}  {}", "plan", "hip lim", "peak up", "final", "verdict");
    for p in [
        Plan { name: "arm 0.10 s + leg splay", sweep_s: 0.10, legs_help: true, ..BASE },
        Plan { name: "splay + finish push", sweep_s: 0.10, legs_help: true, finish: true, ..BASE },
        Plan { name: "one-side push L", sweep_s: 0.10, legs_help: true, asym: 1, ..BASE },
    ] {
        for lim in [1.05_f64, 1.60, 2.40] {
            let (peak, up, _, _) = run(&misa, &ring, FLAT, 0.70, p, Some(lim));
            println!("{:<24} {lim:>9.2} {peak:>8.3} {up:>8.3}  {}", p.name, verdict(peak, up));
        }
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
