//! Interactive, real-time keyboard teleop of the namiashi RL (PPO) stair-
//! climb policy -- the Rust-side counterpart to
//! `namiashi_staircase_5cm_teleop` (`tests/wbc_walk.rs`), so the SAME
//! keyboard muscle memory drives either controller on the SAME staircase,
//! in the SAME language/viewer, rather than one being a Rust `MjViewer`
//! window and the other a separate Python `mujoco.viewer` process.
//!
//! This is a direct, close-to-line-for-line Rust port of
//! `go2_rl/sim2sim_namiashi_mujoco.py` -- SAME constants (Isaac joint
//! order, default stance, DCMotor saturation/effort/velocity limits,
//! action scale, PD gains), SAME `dc_motor_clip` formula (an exact port of
//! `isaaclab.actuators.actuator_pd.DCMotor._clip_effort`, ported to Python
//! first and now here), SAME 45-d observation layout (ang_vel · grav ·
//! cmd · joint_pos_rel · joint_vel · last_action). Deliberately uses `ort`
//! (pyke's binding to the actual Microsoft onnxruntime C++ engine) rather
//! than `policy-runtime`'s `tract-onnx` (a different, pure-Rust ONNX
//! engine chosen there specifically for the real robot's aarch64
//! no-C-deps hardware-deploy constraint, which does not apply to a
//! desktop sim2sim/teleop tool) -- using the same engine as the Python
//! validation script means the Rust and Python paths share not just the
//! same weights but the same numerics, closing off a whole class of
//! engine-level discrepancy (the DCMotor-torque sim2sim bug earlier this
//! investigation is exactly the kind of subtle mismatch worth guarding
//! against on purpose).
//!
//! Limitation: only supports `history_length=1` policies (the base
//! 45-d-observation checkpoint, e.g. model_9393) -- no per-term history
//! stacking, unlike `sim2sim_namiashi_mujoco.py --history-length`.
//!
//! Run (needs a `libonnxruntime.so`/`.dylib`/`.dll` at runtime --
//! point `ORT_DYLIB_PATH` at the one already installed by go2_rl's
//! Python venv, e.g. `.../site-packages/onnxruntime/capi/libonnxruntime.so.1.28.0`):
//!   ORT_DYLIB_PATH=/path/to/libonnxruntime.so \
//!     cargo run --release --no-default-features --features "mujoco,mujoco-viewer,onnx" \
//!     --example namiashi_rl_teleop -- --onnx policy.onnx [--vx 0.8] [--vy 0] [--wz 0]
//!
//! Keys: press `K` in the viewer, or see `namiashi_sim::teleop`'s module docs,
//! shared verbatim with `namiashi_wbc_teleop.rs`. Holding a key moves,
//! releasing it stops. O/L change the ground's friction, which is physics
//! and so applies here exactly as it does to the WBC demo.
//!
//! `--viz` (build with `--features ...,viz`) additionally streams the pose to
//! the articara GUI over Zenoh — planned (the policy's q_des, drawn as a
//! ghost) and measured (MuJoCo's state) on quadruped-gait's default keys, the
//! same contract go2-runner's `policy --sim` uses. Open the namiashi `.misa`
//! in articara (`cargo run --release --features viz`), then in the
//! "Live feed (Zenoh)" window pick topology=Connect with the publisher's
//! `--viz-endpoint` (e.g. `tcp/127.0.0.1:7447`) and Subscribe. The MjViewer
//! window stays the input surface — the keyboard drives THIS window; articara
//! is a second, remote-capable view (terrain/contacts render only here).
//!
//! The gait, swing-height, body-height, levelling and controller-mu keys
//! (1/2/3, R/F, =/-, B, P/.) do nothing here: a learned policy has no gait
//! schedule to switch, no swing-height or stance-height parameter to set,
//! no nominal to level, and no friction cone to inform -- it decides foot
//! clearance, posture and contact timing itself, per step, from the
//! observation, and never reasons about mu at all. That contrast is itself
//! worth feeling directly against the WBC/MPC demo.

#[cfg(all(feature = "mujoco", feature = "mujoco-viewer", feature = "onnx"))]
fn main() {
    use std::sync::{Arc, Mutex};

    use articara::mjcf::{MjcfExportOptions, StaircaseCfg};
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::rbd::model::ActuatorMode;
    use articara::robot::RobotModel;
    use nalgebra::Vector3;
    use namiashi_sim::wbc_harness::RING_HFIELD;
    use ort::session::Session;
    use ort::value::TensorRef;

    // ── CLI ──────────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let get = |key: &str| -> Option<String> {
        args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
    };
    let Some(onnx_path) = get("--onnx") else {
        eprintln!("usage: namiashi_rl_teleop --onnx P.onnx [--vx V] [--vy V] [--wz V]");
        std::process::exit(2);
    };
    let vx0: f64 = get("--vx").and_then(|v| v.parse().ok()).unwrap_or(0.8);
    let vy0: f64 = get("--vy").and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let wz0: f64 = get("--wz").and_then(|v| v.parse().ok()).unwrap_or(0.0);
    // Same `--field` choice as namiashi_wbc_teleop, so the two demos can be
    // put on the same ground without remembering two spellings.
    let field = get("--field").unwrap_or_else(|| "ring".into());
    // See namiashi_wbc_teleop: physics is indifferent to this, rendering is
    // not (2 triangles per cell).
    let cell_mm: f64 = get("--cell-mm").and_then(|v| v.parse().ok()).unwrap_or(5.0);
    // See WbcParams::render_hz -- lower this when the display, not the
    // physics, is what cannot keep up.
    let render_hz: f64 = get("--render-hz").and_then(|v| v.parse().ok()).unwrap_or(60.0);
    let viz_on = args.iter().any(|a| a == "--viz");
    let viz_endpoint = get("--viz-endpoint");
    let viz_rate_hz: f64 = get("--viz-rate").and_then(|v| v.parse().ok()).unwrap_or(100.0);
    #[cfg(not(feature = "viz"))]
    if viz_on {
        eprintln!(
            "--viz needs the `viz` feature: cargo run --release --no-default-features \
             --features mujoco,mujoco-viewer,onnx,viz --example namiashi_rl_teleop ..."
        );
        std::process::exit(2);
    }

    // ── Constants (ported verbatim from sim2sim_namiashi_mujoco.py) ────────
    const ISAAC_NAMES: [&str; 12] = [
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    ];
    const DEFAULT_ISAAC: [f64; 12] = [
        0.0, 0.0, 0.0, 0.0, 0.695, 0.695, 0.695, 0.695, -1.390, -1.390, -1.390, -1.390,
    ];
    // DCMotorCfg's THREE parameters, not a single flat torque clip -- see
    // sim2sim_namiashi_mujoco.py's own comment on SATURATION_EFFORT_ISAAC
    // for why the low-speed ceiling is EFFORT_LIMIT (rated), not
    // SATURATION_EFFORT (peak), and how big a sim2sim gap using the wrong
    // one opened up before that fix.
    const SATURATION_EFFORT: [f64; 12] = [2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 3.88889, 3.88889, 3.88889, 3.88889];
    const EFFORT_LIMIT: [f64; 12] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5556, 1.5556, 1.5556, 1.5556];
    const VELOCITY_LIMIT: [f64; 12] =
        [33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 21.5357, 21.5357, 21.5357, 21.5357];
    const ACTION_SCALE: f64 = 0.25;
    const KP: f64 = 25.0;
    const KD: f64 = 0.5;
    const ARM_DEFAULT: f64 = 0.0;
    const ARM_KP: f64 = 40.0;
    const ARM_KD: f64 = 2.0;
    const PHYSICS_DT: f64 = 0.005; // 200 Hz, matching the Python script's own recorded model.xml
    const INFER_HZ: f64 = 50.0;

    fn dc_motor_clip(effort: f64, joint_vel: f64, saturation_effort: f64, effort_limit: f64, velocity_limit: f64) -> f64 {
        let vel_at_effort_lim = velocity_limit * (1.0 + effort_limit / saturation_effort);
        let v = joint_vel.clamp(-vel_at_effort_lim, vel_at_effort_lim);
        let torque_speed_top = saturation_effort * (1.0 - v / velocity_limit);
        let torque_speed_bottom = saturation_effort * (-1.0 - v / velocity_limit);
        let max_effort = torque_speed_top.min(effort_limit);
        let min_effort = torque_speed_bottom.max(-effort_limit);
        effort.clamp(min_effort, max_effort)
    }

    // ── Robot + staircase (same fixture/geometry every WBC/MPC test in
    // tests/wbc_walk.rs uses) ───────────────────────────────────────────
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");
    let mut robot = RobotModel::from_misa(&misa).unwrap_or_else(|e| panic!(".misa load failed ({}): {e}", misa.display()));
    let stairs = StaircaseCfg { rise_m: 0.05, run_m: 0.20, n_steps: 10, approach_m: 1.5, top_platform_m: 8.0, half_width_m: 6.0 };
    let ring = KawasakiRingCfg { cell_m: cell_mm / 1000.0, ..Default::default() };

    for (k, name) in ISAAC_NAMES.iter().enumerate() {
        let Some(&ji) = robot.joint_map.get(*name) else { panic!("joint missing: {name}") };
        robot.joints[ji].actuator_mode = ActuatorMode::Torque;
        robot.joint_positions[ji] = DEFAULT_ISAAC[k];
    }
    let arm_ji = robot.joint_map.get("arm_pitch_joint").copied();
    if let Some(ji) = arm_ji {
        robot.joints[ji].actuator_mode = ActuatorMode::Torque;
        robot.joint_positions[ji] = ARM_DEFAULT;
    }
    robot.rebuild_misarta_model();

    let opts = match field.as_str() {
        "stairs" => MjcfExportOptions {
            extra_worldbody_xml: Some(stairs.worldbody_xml()),
            add_actuators: true,
            timestep: Some(PHYSICS_DT),
            ..MjcfExportOptions::default()
        },
        "ring" => {
            // On the red start platform: the origin is the centre bowl.
            let (w, d) = ring.red_platform_m;
            MjcfExportOptions {
                extra_worldbody_xml: Some(ring.worldbody_xml(RING_HFIELD)),
                extra_asset_xml: Some(
                    ring.asset_xml(RING_HFIELD) + articara::mjcf::scene_lighting_asset_xml().as_str(),
                ),
                extra_visual_xml: Some(articara::mjcf::scene_lighting_visual_xml()),
                base_xy: Some((
                    -(ring.ring_m / 2.0 + d / 2.0),
                    -(ring.ring_m / 2.0 - w / 2.0),
                )),
                add_actuators: true,
                timestep: Some(PHYSICS_DT),
                ..MjcfExportOptions::default()
            }
        }
        other => {
            eprintln!("unknown --field {other:?}: expected \"ring\" or \"stairs\"");
            std::process::exit(2);
        }
    };
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    if field == "ring" {
        // Declared with nrow/ncol and no file, so MuJoCo holds an
        // uninitialised grid until this lands -- unfilled reads as a
        // perfectly flat ring, i.e. the obstacles silently not existing.
        sim.set_hfield_data(RING_HFIELD, &ring.heights()).expect("fill ring hfield");
    }
    sim.set_gravity_compensation(false); // Torque mode carries gravity itself -- see run_wbc_sim's Actuation::Torque comment
    let dt = sim.timestep();
    let decim = ((1.0 / INFER_HZ) / dt).round().max(1.0) as u32;
    println!("[namiashi_rl_teleop] dt={dt}s decim={decim} (-> {:.1} Hz inference)", 1.0 / (dt * decim as f64));

    // ── ONNX Runtime session (real onnxruntime via `ort`, not tract) ────
    let mut session = Session::builder()
        .unwrap_or_else(|e| panic!("Session::builder: {e}"))
        .commit_from_file(&onnx_path)
        .unwrap_or_else(|e| panic!("load {onnx_path}: {e}"));
    println!("[namiashi_rl_teleop] loaded {onnx_path}");

    // ── Settle: 0.5s of PD-hold at the default stance before engaging the
    // policy (mirrors sim2sim_namiashi_mujoco.py's own settle ramp). ────
    let settle_ticks = (0.5 / dt).round() as u32;
    for _ in 0..settle_ticks {
        for (k, name) in ISAAC_NAMES.iter().enumerate() {
            let ji = robot.joint_map[*name];
            let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
            let tau = dc_motor_clip(KP * (DEFAULT_ISAAC[k] - q) - KD * dq, dq, SATURATION_EFFORT[k], EFFORT_LIMIT[k], VELOCITY_LIMIT[k]);
            sim.set_torque_target(ji, tau);
        }
        if let Some(ji) = arm_ji {
            let (q, dq) = sim.joint_q_qd("arm_pitch_joint").expect("arm joint state");
            sim.set_torque_target(ji, (ARM_KP * (ARM_DEFAULT - q) - ARM_KD * dq).clamp(-6.865, 6.865));
        }
        sim.step(&mut robot, dt, true);
    }

    // ── Live teleop command + viewer. Bindings come from namiashi_sim::teleop
    // so this and namiashi_wbc_teleop.rs cannot drift apart. ────────────
    // A learned policy has no gait to switch, so unlike the WBC demo the
    // envelope is fixed -- namiashi_rl's own training command range
    // (namiashi_rl/env_cfg.py), which is Trot-like.
    const ENV: namiashi_sim::teleop::SpeedEnvelope =
        namiashi_sim::teleop::SpeedEnvelope { vx: 0.8, vy: 0.3, wz: 1.0 };
    let live = Arc::new(Mutex::new(namiashi_sim::teleop::LiveTeleop {
        cmd: [vx0, vy0, wz0],
        // Ground friction is physics, so it is seeded and steerable here
        // exactly as in the WBC demo -- "does the learned policy cope with
        // a slippery floor" is one of the more interesting things this
        // pair of demos can be asked side by side.
        ground_mu: sim.slide_friction(),
        ..namiashi_sim::teleop::LiveTeleop::new(quadruped_gait::GaitType::Trot)
    }));
    let mut live_ground_mu = live.lock().unwrap().ground_mu;
    let mut viewer = mujoco::viewer::MjViewer::launch_passive(sim.mj_model().clone(), 0).expect("launch MjViewer");
    {
        use namiashi_sim::teleop::{draw_hud, poll_cmd, poll_friction_deltas, FRICTION_RANGE};
        let live = live.clone();
        viewer.add_ui_callback_detached(move |ctx| {
            let mut st = live.lock().unwrap();
            if namiashi_sim::teleop::poll_help_toggle(ctx) {
                st.show_help = !st.show_help;
            }
            if let Some(inverted) = namiashi_sim::teleop::poll_respawn(ctx) {
                st.respawn_requested = true;
                st.respawn_inverted = inverted;
            }
            st.cmd = poll_cmd(ctx, ENV);
            let (dg, _) = poll_friction_deltas(ctx);
            if dg != 0.0 {
                st.ground_mu = (st.ground_mu + dg).clamp(FRICTION_RANGE.0, FRICTION_RANGE.1);
            }
            // `gaited: false` -- no gait row, no swing-height row, no
            // controller-mu row: the policy has none of those knobs (it
            // has no friction cone at all -- it never reasons about mu,
            // it just reacts), and showing a dead control would
            // misrepresent what this controller actually exposes.
            draw_hud(ctx, &st, ENV, "RL policy (ONNX)", false);
        });
    }
    eprintln!("[teleop] press K in the viewer for the controls list");
    eprintln!("[teleop] field = {field}  (--field ring | stairs, --cell-mm {cell_mm}, --render-hz {render_hz})");

    // ── articara ライブ配信（--viz、feature "viz"）。quadruped-gait の
    // VizPublisher が planned/measured の対・別スレッド送信・満杯時ドロップ
    // を持つ。キーは既定（go2/gait/planned|measured — articara 側の既定と
    // 同じ）。`--viz-endpoint` を与えるとそこで待ち受け、articara は
    // topology=Connect で同じエンドポイントを差す。───────────────────────
    #[cfg(feature = "viz")]
    let mut viz_pub = if viz_on {
        use quadruped_gait::viz_net::VizEndpoints;
        use quadruped_gait::viz_pub::{VizPublisher, VizPublisherConfig};
        let endpoints = match viz_endpoint.as_deref() {
            Some(ep) => VizEndpoints::listen([ep]),
            None => VizEndpoints::auto(),
        };
        let cfg = VizPublisherConfig {
            rate_hz: viz_rate_hz,
            dt,
            endpoints,
            ..VizPublisherConfig::default()
        };
        let p = VizPublisher::new(cfg).unwrap_or_else(|e| panic!("viz publisher: {e}"));
        eprintln!(
            "[teleop] viz: planned/measured を配信します（articara の Live feed、endpoint {}）",
            viz_endpoint.as_deref().unwrap_or("マルチキャスト探索")
        );
        Some(p)
    } else {
        None
    };
    #[cfg(not(feature = "viz"))]
    let _ = (viz_endpoint, viz_rate_hz);

    // ── Main loop: ONNX inference every `decim` physics ticks, held
    // between (matches sim2sim_namiashi_mujoco.py's own decimation). ───
    let mut last_action = [0.0f32; 12];
    let mut q_des_isaac = DEFAULT_ISAAC;
    let wall_start = std::time::Instant::now();
    let mut fps_meter = namiashi_sim::teleop::FpsMeter::new();
    let mut pending_mu: Option<f64> = None;
    let mut k: u64 = 0;
    loop {
        if k % decim as u64 == 0 {
            let ang_vel_w = sim.body_world_angular_velocity(&robot.root_link).expect("root omega");
            let q_wb = sim.body_world_orientation(&robot.root_link).expect("root quat");
            let ang_vel_b = q_wb.inverse_transform_vector(&Vector3::new(ang_vel_w[0], ang_vel_w[1], ang_vel_w[2]));
            let grav_b = q_wb.inverse_transform_vector(&Vector3::new(0.0, 0.0, -1.0));
            let cmd = live.lock().unwrap().cmd;

            let mut obs = [0.0f32; 45];
            obs[0] = ang_vel_b.x as f32;
            obs[1] = ang_vel_b.y as f32;
            obs[2] = ang_vel_b.z as f32;
            obs[3] = grav_b.x as f32;
            obs[4] = grav_b.y as f32;
            obs[5] = grav_b.z as f32;
            obs[6] = cmd[0] as f32;
            obs[7] = cmd[1] as f32;
            obs[8] = cmd[2] as f32;
            for (i, name) in ISAAC_NAMES.iter().enumerate() {
                let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
                obs[9 + i] = (q - DEFAULT_ISAAC[i]) as f32;
                obs[21 + i] = dq as f32;
            }
            obs[33..45].copy_from_slice(&last_action);

            let input = ndarray::Array2::from_shape_vec((1, 45), obs.to_vec()).expect("obs shape");
            let outputs = session
                .run(ort::inputs![TensorRef::from_array_view(&input).expect("tensor view")])
                .unwrap_or_else(|e| panic!("inference: {e}"));
            let (_, action) = outputs[0].try_extract_tensor::<f32>().expect("extract action");
            last_action.copy_from_slice(action);
            for i in 0..12 {
                q_des_isaac[i] = DEFAULT_ISAAC[i] + ACTION_SCALE * action[i] as f64;
            }
        }

        for (i, name) in ISAAC_NAMES.iter().enumerate() {
            let ji = robot.joint_map[*name];
            let (q, dq) = sim.joint_q_qd(name).expect("leg joint state");
            let tau = dc_motor_clip(KP * (q_des_isaac[i] - q) - KD * dq, dq, SATURATION_EFFORT[i], EFFORT_LIMIT[i], VELOCITY_LIMIT[i]);
            sim.set_torque_target(ji, tau);
        }
        if let Some(ji) = arm_ji {
            let (q, dq) = sim.joint_q_qd("arm_pitch_joint").expect("arm joint state");
            sim.set_torque_target(ji, (ARM_KP * (ARM_DEFAULT - q) - ARM_KD * dq).clamp(-6.865, 6.865));
        }
        sim.step(&mut robot, dt, true);
        k += 1;

        // planned（方策の q_des）と measured（MuJoCo の実測）を対で配信。
        // フレームは毎周期組むが 12 関節の詰め替えだけで、送信の間引き・
        // 別スレッド化・seq の対付けは VizPublisher の側にある。
        #[cfg(feature = "viz")]
        if let Some(p) = viz_pub.as_mut() {
            use quadruped_gait::viz::{GaitVizFrame, VIZ_FORMAT_VERSION};
            let tr = robot.base_transform.translation;
            let (roll, pitch, yaw) = robot.base_transform.rotation.euler_angles();
            let feet = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
            let fz = sim.contact_force_per_foot(&feet);
            let stance = [fz[0] > 5.0, fz[1] > 5.0, fz[2] > 5.0, fz[3] > 5.0];
            let mut planned_j = [0.0f64; 12];
            let mut measured_j = [0.0f64; 12];
            for l in 0..4 {
                // slot 順 FL,FR,RL,RR × (hip, thigh, calf) ← Isaac 型順
                for (jj, iso) in [l, 4 + l, 8 + l].into_iter().enumerate() {
                    planned_j[3 * l + jj] = q_des_isaac[iso];
                    let (q, _) = sim.joint_q_qd(ISAAC_NAMES[iso]).expect("joint state");
                    measured_j[3 * l + jj] = q;
                }
            }
            let planned = GaitVizFrame {
                version: VIZ_FORMAT_VERSION,
                seq: 0, // publisher が対で振り直す
                t_s: k as f64 * dt,
                pose: [tr.x, tr.y, tr.z, yaw],
                pose_rp: [0.0, 0.0],
                joints: planned_j,
                stance,
            };
            let measured = GaitVizFrame {
                pose_rp: [roll, pitch],
                joints: measured_j,
                ..planned.clone()
            };
            p.publish(|| (planned, Some(measured)));
        }

        // ~60 Hz render/sync cadence, independent of the finer physics dt.
        let render_decim = ((1.0 / render_hz.max(1.0)) / dt).round().max(1.0) as u64;
        if k % render_decim == 0 {
            // HUD telemetry, body frame so `vx meas` is comparable to the
            // `vx cmd` shown beside it. Same cadence/contract as the WBC
            // demo's (see run_wbc_sim's live_viewer block).
            {
                let fps = fps_meter.tick();
                let p = robot.base_transform.translation;
                let v_w = sim
                    .body_world_linear_velocity(&robot.root_link)
                    .unwrap_or([0.0; 3]);
                let v_b = robot
                    .base_transform
                    .rotation
                    .inverse_transform_vector(&Vector3::new(v_w[0], v_w[1], v_w[2]));
                let mut st = live.lock().unwrap();
                st.body_x_m = p.x;
                st.body_z_m = p.z;
                st.measured_vx_mps = v_b.x;
                st.sim_time_s = k as f64 * dt;
                st.wall_time_s = wall_start.elapsed().as_secs_f64();
                st.fps = fps;
                if (st.ground_mu - live_ground_mu).abs() > 1e-9 {
                    live_ground_mu = st.ground_mu;
                    pending_mu = Some(live_ground_mu);
                }
            }
            if let Some(mu) = pending_mu.take() {
                // Outside the lock/borrow above: set_slide_friction_all
                // needs &mut sim, which the telemetry block is holding.
                sim.set_slide_friction_all(mu);
                eprintln!("[teleop] ground mu -> {mu:.2}");
            }
            // See run_wbc_sim's copy: sync_data merges the viewer's own
            // edits back into our data, so its Reset button really does
            // reset this sim -- to qpos0, legs straight, feet through the
            // floor. A clock that went backwards is the only trace it
            // leaves.
            let t_before = sim.sim_time();
            viewer.sync_data(sim.mj_data_mut());
            let reset_seen = sim.sim_time() < t_before - 1e-9;
            let (asked, inverted) = {
                let mut st = live.lock().unwrap();
                (std::mem::replace(&mut st.respawn_requested, false), st.respawn_inverted)
            };
            if reset_seen || asked {
                if inverted && asked {
                    sim.respawn_inverted(&mut robot, 0.03);
                } else {
                    sim.respawn(&mut robot, 0.10);
                }
                eprintln!("[teleop] respawned{}", if inverted && asked { " upside down" } else { "" });
            }
            let _ = viewer.render();

            // Pace once per rendered frame, not per physics tick -- see
            // run_wbc_sim's live_viewer block for why the per-tick version
            // collapses the frame rate on a coarse-timer host.
            let target = std::time::Duration::from_secs_f64(k as f64 * dt);
            let elapsed = wall_start.elapsed();
            if elapsed < target {
                std::thread::sleep(target - elapsed);
            }
            if !viewer.running() {
                break;
            }
        }
    }
}

#[cfg(not(all(feature = "mujoco", feature = "mujoco-viewer", feature = "onnx")))]
fn main() {
    eprintln!(
        "this example needs: cargo run --release --no-default-features \
         --features mujoco,mujoco-viewer,onnx --example namiashi_rl_teleop -- --onnx P.onnx"
    );
    std::process::exit(2);
}
