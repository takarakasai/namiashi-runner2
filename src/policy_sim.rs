//! `policy --sim` — namiashi 方策を MuJoCo（misa-plant-mujoco）で閉ループに回す。
//!
//! 実機と**同じ** NamiashiRefController・同じ観測組み立て・同じデコードを
//! 通すので、ここが歩けば「実機に送る位置目標と同じ計算」が歩いている。
//! 違うのは Plant だけ（シリアル ↔ MuJoCo）。
//!
//! Python の `sim2sim_namiashi_ref_mujoco.py` と同じ構造（物理 5 ms × 4 =
//! 50 Hz 推論、ZOH、位置 PD kp25/kd0.5）だが、物理は articara の MJCF 出力
//! （namiashi_meas.misa、実測質量）で、あちらの model.xml とは別実装。
//!
//! ```text
//! export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
//! cargo run --release --features sim -- policy --sim --model policy.onnx \
//!     --keys --viz --viz-endpoint tcp/127.0.0.1:7447
//! ```
//!
//! articara（`cargo run --release --features viz`、モデルに namiashi_meas.misa
//! を開く）の Live gait feed で同じエンドポイントを差すと、指令（planned、
//! ゴースト）と MuJoCo の実測（measured）が重なって描かれる。

use std::io::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use misa_plant_mujoco::{MujocoPlant, SimOptions};
use misa_policy_runner::namiashi::{misa_to_isaac, DEFAULT_ISAAC};
use misa_runner::jointvec::JointVec;
use misa_runner::misa_core::{AxisCommand, AxisId, Command, ControlMode, Observation, Plant};
use misa_runner::viz;
use misa_runner::AppConfig;

use crate::policy::{
    cmd_clamp, load_controller, obs_input, spawn_keyboard, write_leg_targets, Args, Recorder, CONTROL_DT,
};

/// 物理の刻み [s]（学習側 sim.dt と同じ）。
const PHYSICS_DT: f64 = 0.005;
/// 胴体の初期高さ [m]（namiashi_dump_flat_mjcf と同じ）。
const BASE_HEIGHT_M: f64 = 0.235;
/// 転倒判定。
const FALL_HEIGHT_M: f64 = 0.10;
const FALL_TILT_RAD: f64 = 60.0_f64.to_radians();
/// 腕（学習側と同じ固定保持）。
const ARM_KP: f64 = 40.0;
const ARM_KD: f64 = 2.0;

fn socket_addr_of(endpoint: &str) -> Option<String> {
    endpoint.split_once('/').map(|(_, rest)| rest.to_string())
}

fn check_viz_port(endpoint: &str) -> Result<(), String> {
    let Some(addr) = socket_addr_of(endpoint) else { return Ok(()) };
    match std::net::TcpListener::bind(&addr) {
        Ok(l) => {
            drop(l);
            Ok(())
        }
        Err(e) => Err(format!(
            "viz エンドポイント {endpoint} を待ち受けられません（{e}）。別の go2-run / namiashi-run / zenohd が使っていないか確認（--viz-endpoint で番号を変えられる）"
        )),
    }
}

pub(crate) fn run(a: &Args) -> Result<(), String> {
    let mut ctl = load_controller(a)?;
    let cfg = AppConfig::load(&a.robot)?;
    let layout = misa_runner::snapshot::axis_layout(&cfg)?;
    let axes = layout.table.clone();
    let n = axes.len();
    let misa_path = a
        .misa
        .clone()
        .unwrap_or_else(|| "models/namiashi/namiashi_meas.misa".into());

    let mut home: Vec<(String, f64)> = Vec::with_capacity(n);
    for (i, ax) in axes.axes().iter().enumerate() {
        let q = if i < 12 { DEFAULT_ISAAC[misa_to_isaac(i)] } else { 0.0 };
        home.push((ax.name.clone(), q));
    }
    let opts = SimOptions {
        misa_path: misa_path.clone(),
        control_period_s: CONTROL_DT,
        timestep_s: Some(PHYSICS_DT),
        actuator_kp: a.kp,
        actuator_kv: a.kv,
        torque_scale: 1.0,
        velocity_kv: 20.0,
        base_height_m: BASE_HEIGHT_M,
        home,
        // Python の参照シーン（namiashi_dump_flat_mjcf: impratio 10・elliptic）に
        // 合わせる。摩擦は --friction で振れる（ゴム足 0.4〜1.0 の範囲）。
        friction: Some(a.friction.map_or([0.8, 0.005, 0.0001], |mu| [mu, 0.005, 0.0001])),
        impratio: Some(10.0),
        cone: Some("elliptic".into()),
        contact_threshold_n: 5.0,
        feet: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
            .iter()
            .map(|s| s.to_string())
            .collect(),
        root_link: "trunk".into(),
        // 関節種別ごとのゲイン（hayaashi 向け）など、後から増えた項目は既定のまま。
        ..SimOptions::default()
    };
    let mut plant = MujocoPlant::new(axes, &opts)?;
    let mut obs = Observation::empty(n, 4);
    let mut cmd = Command::idle(n);
    eprintln!(
        "policy-sim: {misa_path}  kp={:.1} kv={:.2} 摩擦={:.2} 遅延={} サブステップ",
        a.kp,
        a.kv,
        a.friction.unwrap_or(0.8),
        a.delay_substeps
    );

    if a.viz {
        if let Some(ep) = a.viz_endpoint.as_deref() {
            check_viz_port(ep)?;
        }
    }
    let mut publisher = if a.viz {
        let vcfg = viz::VizConfig {
            enabled: true,
            endpoint: a.viz_endpoint.clone(),
            rate_hz: a.viz_rate_hz,
            ..viz::VizConfig::default()
        };
        let p = viz::Publisher::new(&vcfg)?;
        eprintln!(
            "policy-sim: viz を {} / {} へ配信します（{} Hz）",
            viz::VizConfig::default().key_planned,
            viz::VizConfig::default().key_measured,
            a.viz_rate_hz
        );
        Some(p)
    } else {
        None
    };

    // 腕は固定保持（学習側と同じ）。
    let hold_arm = |cmd: &mut Command| {
        for i in 12..n {
            if let Some(ax) = cmd.get_mut(AxisId::new(i as u16)) {
                *ax = AxisCommand {
                    mode: ControlMode::Position,
                    position_rad: 0.0,
                    velocity_rad_s: 0.0,
                    torque_ff_nm: 0.0,
                    kp_nm_per_rad: ARM_KP,
                    kd_nm_s_per_rad: ARM_KD,
                };
            }
        }
    };

    // 立位で 0.5 s 落ち着かせる。
    write_leg_targets(&mut cmd, &DEFAULT_ISAAC, a.kp, a.kv, 0.0);
    hold_arm(&mut cmd);
    for _ in 0..(0.5 / CONTROL_DT) as u64 {
        plant.exchange(&cmd, &mut obs)?;
    }

    let clamp = cmd_clamp(a);
    let cmd_shared = Arc::new(Mutex::new(clamp(a.cmd0)));
    let quit = Arc::new(AtomicBool::new(false));
    let kb = if a.keyboard {
        Some(spawn_keyboard(cmd_shared.clone(), quit.clone(), clamp.clone())?)
    } else {
        None
    };

    // 指令遅延の模擬（物理サブステップ単位）。misa-plant-mujoco は 1 exchange で
    // 4 サブステップ進めるので、tick 単位の遅延に丸める（--delay 4 = 1 tick）。
    let delay_ticks = (a.delay_substeps as f64 / (CONTROL_DT / PHYSICS_DT)).round() as usize;
    let mut ring: std::collections::VecDeque<[f64; 12]> =
        std::iter::repeat(DEFAULT_ISAAC).take(delay_ticks + 1).collect();

    ctl.reset();
    let mut held = ctl.hold();
    let start_xy = plant.base_position().map(|p| [p[0], p[1]]).unwrap_or([0.0; 2]);
    // ヨーは毎周期積算する（アンラップ）。最終 rpy の差を ±π に折ると、
    // 半回転を超えた旋回試行の符号が反転して見える（Python 側 sim2sim で
    // 実際に起きた計測バグ。2026-09-19 修正）。
    let mut yaw_prev = obs.imu.map(|i| i.rpy_rad[2]).unwrap_or(0.0);
    let mut yaw_acc = 0.0f64;
    let mut fell: Option<String> = None;
    let mut tilt_max = 0.0f64;
    let mut z_min = f64::INFINITY;
    let mut status = Instant::now();
    let mut k: u64 = 0;
    let mut rec = match a.record.as_deref() {
        Some(p) => Some(Recorder::open(p)?),
        None => None,
    };
    eprintln!("policy-sim: RUNNING\r");
    while !quit.load(Ordering::Relaxed) {
        let t = k as f64 * CONTROL_DT;
        if let Some(d) = a.duration {
            if t >= d {
                break;
            }
        }
        let cmd_now = *cmd_shared.lock().unwrap();
        let (inp, tilt) = obs_input(&obs)?;
        tilt_max = tilt_max.max(tilt);
        let base = plant.base_position().unwrap_or([0.0, 0.0, BASE_HEIGHT_M]);
        z_min = z_min.min(base[2]);
        if base[2] < FALL_HEIGHT_M || tilt > FALL_TILT_RAD {
            fell = Some(format!("t={t:.1}s 高さ {:.2} m / 傾き {:.0}°", base[2], tilt.to_degrees()));
            break;
        }
        if !a.hold {
            match ctl.tick(&inp, cmd_now) {
                Ok(q) => held = q,
                Err(e) => {
                    eprintln!("policy-sim: 推論失敗（保持）: {e}\r");
                    held = ctl.hold();
                }
            }
        }
        if let Some(r) = rec.as_mut() {
            let rpy = obs.imu.map(|i| i.rpy_rad).unwrap_or([0.0; 3]);
            let q_des = if a.hold { DEFAULT_ISAAC } else { held };
            r.row(t, cmd_now, &q_des, &inp, rpy);
        }
        ring.push_back(if a.hold { DEFAULT_ISAAC } else { held });
        let delayed = ring.pop_front().unwrap_or(DEFAULT_ISAAC);
        write_leg_targets(&mut cmd, &delayed, a.kp, a.kv, 0.0);
        hold_arm(&mut cmd);
        plant.exchange(&cmd, &mut obs)?;
        k += 1;
        if let Some(imu) = obs.imu {
            let d = imu.rpy_rad[2] - yaw_prev;
            yaw_acc += d.sin().atan2(d.cos());
            yaw_prev = imu.rpy_rad[2];
        }

        if let Some(p) = publisher.as_mut() {
            let imu = obs.imu.unwrap_or_default();
            let mut planned = JointVec::zeros();
            let mut measured = JointVec::zeros();
            for i in 0..12 {
                let (leg, j) = (i / 3, i % 3);
                planned.legs[leg][j] = held[misa_to_isaac(i)];
                measured.legs[leg][j] = obs.axes()[i].position_rad;
            }
            let stance: [bool; 4] =
                core::array::from_fn(|l| obs.contacts.get(l).copied().flatten().unwrap_or(false));
            let view = viz::BodyView {
                xy: [base[0], base[1]],
                yaw: imu.rpy_rad[2],
                z: base[2],
                rp: [0.0, 0.0],
                stance,
            };
            let mview = viz::BodyView { rp: [imu.rpy_rad[0], imu.rpy_rad[1]], ..view };
            p.maybe_publish(|seq| {
                viz::Frames::both(viz::frame(seq, t, &planned, &view), viz::frame(seq, t, &measured, &mview))
            });
        }

        if status.elapsed().as_secs_f64() > 0.5 {
            let imu = obs.imu.unwrap_or_default();
            eprint!(
                "\rpolicy-sim: t={t:6.1}s cmd=({:+.2},{:+.2},{:+.2}) xy=({:+.2},{:+.2}) z={:.3} yaw={:+.2}   ",
                cmd_now[0], cmd_now[1], cmd_now[2], base[0], base[1], base[2], imu.rpy_rad[2]
            );
            let _ = std::io::stderr().flush();
            status = Instant::now();
        }
        // 実時間で回す（ヘッドレスなら速度は物理任せでもよいが、viz とキーの都合）。
        if a.keyboard || a.viz {
            let wall = Instant::now();
            let _ = wall;
            std::thread::sleep(std::time::Duration::from_secs_f64(CONTROL_DT * 0.8));
        }
    }
    quit.store(true, Ordering::Relaxed);
    if let Some(r) = rec.take() {
        eprintln!("\r\npolicy-sim: --record {} 行を書きました\r", r.finish());
    }
    if let Some(h) = kb {
        let _ = h.join();
    }
    let base = plant.base_position().unwrap_or([0.0; 3]);
    let yaw = yaw_acc;
    let el = (k as f64 * CONTROL_DT).max(1e-9);
    let d = [base[0] - start_xy[0], base[1] - start_xy[1]];
    eprintln!(
        "\r\n[policy-sim namiashi] cmd=({:+.2},{:+.2},{:+.2})  dx={:+.2}m ({:+.3} m/s)  dy={:+.2}m  yaw={:+.2}rad ({:+.3} rad/s)  z_min={:.3}  tilt_max={:.1}deg  FELL={}",
        a.cmd0[0], a.cmd0[1], a.cmd0[2], d[0], d[0] / el, d[1], yaw, yaw / el, z_min, tilt_max.to_degrees(),
        fell.is_some()
    );
    if let Some(f) = fell {
        eprintln!("policy-sim: 転倒 {f}");
    }
    Ok(())
}
