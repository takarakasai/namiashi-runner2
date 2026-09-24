//! `policy` — namiashi WalkFlat 契約（misa-policy-runner `namiashi`）を実機で回す。
//!
//! ```text
//! namiashi-run policy --model policy.onnx [--robot robots/namiashi.toml]
//!                     [--keys] [--vx 0.1] [--vy 0] [--wz 0] [--wz-max 0.2]
//!                     [--duration 20] [--hold]
//! namiashi-run policy --sim ...   MuJoCo（--features sim、policy_sim.rs）
//! ```
//!
//! **位置目標だけ**を 50 Hz で送る。MG4005E の内蔵位置ループが学習側の
//! kp25/kd0.5 の実体で、ゲインは触らない（学習は ×0.6–1.6 の DR 済み）。
//!
//! 立ち上げは misa-runner の `run` と同じ手順（`runner::start_serial`:
//! IMU・マルチターン原点・12 軸の初回読み出し・電源投入後 1 回の原点張り直し）
//! を通してから、実測姿勢 → 立位へ 2 s でランプし、方策に渡す。
//!
//! 安全: 傾き 45° で即脱力、観測の欠損・推論失敗が連続したら中断、
//! 終了時は立位へ 1 s で戻してから脱力。**初回はベンチで脚を浮かせて**、
//! 内蔵ループの応答を確認してから床に下ろす（doc/ の立ち上げ手順）。

use std::io::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use misa_policy_runner::namiashi::{
    clamp_cmd, gravity_from_rpy, misa_to_isaac, NamiashiObsInput, NamiashiRefController,
    CMD_VX_RANGE, CMD_VY_RANGE,
};
use misa_policy_runner::OnnxPolicy;
use misa_runner::misa_core::{AxisCommand, AxisId, Command, ControlMode, Observation, Plant};
use misa_runner::misa_hal::ch348::PortMap;
use misa_runner::runner::{start_serial, RunOptions};
use misa_runner::AppConfig;

/// 推論と指令の周期（学習時の decimation 4 × 5 ms）。
pub(crate) const CONTROL_DT: f64 = 0.02;
/// 実測姿勢 → 立位のランプ [s]。
const RAMP_SECS: f64 = 2.0;
/// 傾きの即時脱力 [rad]。
const TILT_ABORT_RAD: f64 = 45.0_f64.to_radians();
/// 観測欠損・推論失敗がこれだけ連続したら中断。
const MAX_CONSECUTIVE_FAULTS: u32 = 10;
/// 運用の既定 |wz| 上限（契約世代ごと）。
///
/// v12〜v14 は MuJoCo で純旋回 0.2 でも転倒したので 0.1（回らないが転ばない）。
/// v15（種較正）は純旋回 ±0.4 が両エンジンで 99–115%・転倒なしなので学習域
/// いっぱいの 0.4（go2_rl doc/namiashi_policy_architecture.md §6）。
const DEFAULT_WZ_MAX_V12: f64 = 0.1;
const DEFAULT_WZ_MAX_V15: f64 = 0.4;

pub(crate) struct Args {
    pub model: String,
    /// 参照歩容の契約世代（--contract v12|v15|v16|v24。**既定 v24 = デプロイ標準
    /// = v24_long_s103/policy_5500**、2026-09-25 に v16 から切り替え）。ONNX はどの世代も 47 入力で自動判別
    /// できない — チェックポイントに合わせること（間違えると歩く。悪く。
    /// エラーは出ない）。
    pub contract: Contract,
    pub robot: String,
    pub sim: bool,
    pub keyboard: bool,
    pub cmd0: [f64; 3],
    pub wz_max: f64,
    pub vx_max: Option<f64>,
    pub duration: Option<f64>,
    pub hold: bool,
    pub viz: bool,
    pub viz_endpoint: Option<String>,
    pub viz_rate_hz: f64,
    /// `--sim` だけ: モデル・接触・ゲイン・遅延。
    pub misa: Option<String>,
    pub kp: f64,
    pub kv: f64,
    pub friction: Option<f64>,
    pub delay_substeps: usize,
    /// 毎 tick の指令・実測を CSV に落とす（実機のステップ応答・現物合わせ用）。
    pub record: Option<String>,
    /// PI 方位保持（misa-policy-runner `heading::HeadingServo`）。方策の前進時
    /// ヨー偏り（v24 系で −0.04〜−0.08 rad/s、報酬では消えなかった）を IMU
    /// ヨーで閉じる。`--heading-hold KP KI`（Go2 実績 2 0.5）。
    pub heading_hold: Option<(f64, f64)>,
    /// 方位サーボを旋回指令中も効かせる（`--heading-hold-turn`）。既定は直進
    /// （|wz| < 0.05）だけ。旋回の過不足と複合指令のヨー偏りを直せるが、
    /// 姿勢の代償があり得る。
    pub heading_hold_turn: bool,
}

fn usage() -> String {
    "usage: namiashi-run policy --model policy.onnx [--robot robots/namiashi.toml] [--sim]\n\
     \x20  [--keys] [--vx V] [--vy V] [--wz V] [--wz-max 0.2] [--vx-max V] [--duration S] [--hold]\n\
     \x20  [--viz] [--viz-endpoint tcp/127.0.0.1:7447] [--viz-rate 100] [--record out.csv] [--heading-hold 2 0.5] [--heading-hold-turn]\n\
     \x20  sim だけ: [--misa PATH] [--kp 25] [--kv 0.5] [--friction 0.8] [--delay 0..4]"
        .into()
}

pub(crate) fn parse(args: &[String]) -> Result<Args, String> {
    let mut out = Args {
        model: String::new(),
        contract: Contract::V24,
        robot: "robots/namiashi.toml".into(),
        sim: false,
        keyboard: false,
        cmd0: [0.0; 3],
        wz_max: f64::NAN, // 契約確定後に埋める
        vx_max: None,
        duration: None,
        hold: false,
        viz: false,
        viz_endpoint: None,
        viz_rate_hz: 100.0,
        misa: None,
        kp: 25.0,
        kv: 0.5,
        friction: None,
        delay_substeps: 0,
        record: None,
        heading_hold: None,
        heading_hold_turn: false,
    };
    let mut it = args.iter();
    fn val<'a>(it: &mut std::slice::Iter<'a, String>, key: &str) -> Result<&'a str, String> {
        it.next().map(|s| s.as_str()).ok_or_else(|| format!("{key} に値がありません"))
    }
    fn num(it: &mut std::slice::Iter<'_, String>, key: &str) -> Result<f64, String> {
        val(it, key)?.parse().map_err(|e| format!("{key}: {e}"))
    }
    while let Some(a) = it.next() {
        match a.as_str() {
            "--model" => out.model = val(&mut it, "--model")?.to_string(),
            "--robot" => out.robot = val(&mut it, "--robot")?.to_string(),
            "--sim" => out.sim = true,
            "--keys" | "--keyboard" => out.keyboard = true,
            "--vx" => out.cmd0[0] = num(&mut it, "--vx")?,
            "--vy" => out.cmd0[1] = num(&mut it, "--vy")?,
            "--wz" => out.cmd0[2] = num(&mut it, "--wz")?,
            "--wz-max" => out.wz_max = num(&mut it, "--wz-max")?,
            "--contract" => {
                out.contract = match val(&mut it, "--contract")? {
                    "v12" => Contract::V12,
                    "v15" => Contract::V15,
                    "v16" => Contract::V16,
                    "v24" => Contract::V24,
                    other => return Err(format!("--contract は v12 / v15 / v16 / v24 です（{other:?}）")),
                }
            }
            "--vx-max" => out.vx_max = Some(num(&mut it, "--vx-max")?),
            "--duration" => out.duration = Some(num(&mut it, "--duration")?),
            "--hold" => out.hold = true,
            "--viz" => out.viz = true,
            "--viz-endpoint" => out.viz_endpoint = Some(val(&mut it, "--viz-endpoint")?.to_string()),
            "--viz-rate" => out.viz_rate_hz = num(&mut it, "--viz-rate")?,
            "--misa" => out.misa = Some(val(&mut it, "--misa")?.to_string()),
            "--kp" => out.kp = num(&mut it, "--kp")?,
            "--kv" => out.kv = num(&mut it, "--kv")?,
            "--friction" => out.friction = Some(num(&mut it, "--friction")?),
            "--delay" => out.delay_substeps = num(&mut it, "--delay")? as usize,
            "--record" => out.record = Some(val(&mut it, "--record")?.to_string()),
            "--heading-hold" => {
                let kp = num(&mut it, "--heading-hold KP")?;
                let ki = num(&mut it, "--heading-hold KI")?;
                if !(kp > 0.0 && ki >= 0.0) {
                    return Err(format!("--heading-hold は KP > 0, KI ≥ 0（{kp} {ki}）"));
                }
                out.heading_hold = Some((kp, ki));
            }
            "--heading-hold-turn" => out.heading_hold_turn = true,
            "-h" | "--help" => return Err(usage()),
            other => return Err(format!("知らないオプション: {other}\n{}", usage())),
        }
    }
    if out.model.is_empty() {
        return Err(format!("--model が要ります\n{}", usage()));
    }
    if out.wz_max.is_nan() {
        out.wz_max = match out.contract {
            Contract::V12 => DEFAULT_WZ_MAX_V12,
            Contract::V15 | Contract::V16 | Contract::V24 => DEFAULT_WZ_MAX_V15,
        };
    }
    Ok(out)
}

/// `--record`: 1 tick 1 行の CSV。列は t, cmd(3), q_des(12, Isaac 型順),
/// q(12), dq(12), rpy(3), gyro(3)。実機の内蔵位置ループの応答（指令 → 実測の
/// 遅れ・追従幅）を後から当てはめるための生データ。書けなければ止めずに警告。
pub(crate) struct Recorder {
    w: std::io::BufWriter<std::fs::File>,
    n: u64,
}

impl Recorder {
    pub(crate) fn open(path: &str) -> Result<Self, String> {
        let f = std::fs::File::create(path).map_err(|e| format!("--record {path}: {e}"))?;
        let mut w = std::io::BufWriter::new(f);
        let mut head = String::from("t,cmd_vx,cmd_vy,cmd_wz");
        for (pre, n) in [("qdes", 12), ("q", 12), ("dq", 12)] {
            for i in 0..n {
                head.push_str(&format!(",{pre}{i}"));
            }
        }
        head.push_str(",roll,pitch,yaw,gx,gy,gz\n");
        w.write_all(head.as_bytes()).map_err(|e| e.to_string())?;
        Ok(Self { w, n: 0 })
    }

    pub(crate) fn row(&mut self, t: f64, cmd: [f64; 3], q_des: &[f64; 12], inp: &NamiashiObsInput, rpy: [f64; 3]) {
        let mut line = format!("{t:.3},{:.4},{:.4},{:.4}", cmd[0], cmd[1], cmd[2]);
        for v in q_des.iter().chain(inp.joint_q_isaac.iter()).chain(inp.joint_dq_isaac.iter()) {
            line.push_str(&format!(",{v:.5}"));
        }
        for v in rpy.iter().chain(inp.gyro_rad_s.iter()) {
            line.push_str(&format!(",{v:.5}"));
        }
        line.push('\n');
        if self.w.write_all(line.as_bytes()).is_ok() {
            self.n += 1;
        }
    }

    pub(crate) fn finish(mut self) -> u64 {
        let _ = self.w.flush();
        self.n
    }
}

/// 指令のクランプ: 学習包絡 ∩ 運用上限。
pub(crate) type CmdClamp = Arc<dyn Fn([f64; 3]) -> [f64; 3] + Send + Sync>;

pub(crate) fn cmd_clamp(a: &Args) -> CmdClamp {
    let wz_max = a.wz_max.abs();
    let vx_max = a.vx_max;
    Arc::new(move |c: [f64; 3]| {
        let mut c = clamp_cmd(c);
        if let Some(m) = vx_max {
            c[0] = c[0].clamp(-m.abs(), m.abs());
        }
        c[2] = c[2].clamp(-wz_max, wz_max);
        c
    })
}

const STEP_V: f64 = 0.05;
const STEP_WZ: f64 = 0.05;

/// WASD テレオペ（別スレッド、crossterm の raw モード）。
pub(crate) fn spawn_keyboard(
    cmd: Arc<Mutex<[f64; 3]>>,
    quit: Arc<AtomicBool>,
    clamp: CmdClamp,
) -> Result<std::thread::JoinHandle<()>, String> {
    use crossterm::event::{self, Event, KeyCode};
    use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
    enable_raw_mode().map_err(|e| format!("raw mode: {e}"))?;
    let vx_hi = clamp([100.0, 0.0, 0.0])[0];
    let vx_lo = clamp([-100.0, 0.0, 0.0])[0];
    let vy_hi = clamp([0.0, 100.0, 0.0])[1];
    let wz_hi = clamp([0.0, 0.0, 100.0])[2];
    eprintln!(
        "policy: keys — W/S = vx±{STEP_V:.2}, A/D = wz±{STEP_WZ:.2}, R/F = vy±{STEP_V:.2}, \
         0 か Space = 全部 0, Esc / q = 終了（立位に戻して脱力）\r\n\
         \x20      指令域: vx ∈ [{vx_lo:.2}, {vx_hi:.2}], |vy| ≤ {vy_hi:.2}, |wz| ≤ {wz_hi:.2}\r"
    );
    Ok(std::thread::spawn(move || {
        loop {
            if quit.load(Ordering::Relaxed) {
                break;
            }
            if let Ok(true) = event::poll(Duration::from_millis(100)) {
                if let Ok(Event::Key(k)) = event::read() {
                    let mut c = cmd.lock().unwrap();
                    match k.code {
                        KeyCode::Char('w') | KeyCode::Up => c[0] += STEP_V,
                        KeyCode::Char('s') | KeyCode::Down => c[0] -= STEP_V,
                        KeyCode::Char('a') | KeyCode::Left => c[2] += STEP_WZ,
                        KeyCode::Char('d') | KeyCode::Right => c[2] -= STEP_WZ,
                        KeyCode::Char('r') => c[1] += STEP_V,
                        KeyCode::Char('f') => c[1] -= STEP_V,
                        KeyCode::Char('0') | KeyCode::Char(' ') => *c = [0.0; 3],
                        KeyCode::Esc | KeyCode::Char('q') => {
                            quit.store(true, Ordering::Relaxed);
                            break;
                        }
                        KeyCode::Char('c') if k.modifiers.contains(event::KeyModifiers::CONTROL) => {
                            quit.store(true, Ordering::Relaxed);
                            break;
                        }
                        _ => {}
                    }
                    *c = clamp(*c);
                    eprint!("\rpolicy: cmd=({:+.2},{:+.2},{:+.2})      ", c[0], c[1], c[2]);
                    let _ = std::io::stderr().flush();
                }
            }
        }
        let _ = disable_raw_mode();
    }))
}

/// 観測（misa 脚順 12 軸 + IMU）→ 方策入力（Isaac 型順）。
pub(crate) fn obs_input(obs: &Observation) -> Result<(NamiashiObsInput, f64), String> {
    let imu = obs.imu.ok_or("IMU の観測がありません")?;
    let mut inp = NamiashiObsInput {
        gyro_rad_s: imu.gyro_rad_s,
        gravity_b: gravity_from_rpy(imu.rpy_rad),
        joint_q_isaac: [0.0; 12],
        joint_dq_isaac: [0.0; 12],
    };
    for i in 0..12 {
        let st = obs.get(AxisId::new(i as u16)).ok_or("脚 12 軸の観測が足りません")?;
        inp.joint_q_isaac[misa_to_isaac(i)] = st.position_rad;
        inp.joint_dq_isaac[misa_to_isaac(i)] = st.velocity_rad_s;
    }
    let tilt = (-inp.gravity_b[2]).clamp(-1.0, 1.0).acos();
    Ok((inp, tilt))
}

/// q_des（Isaac 型順）を脚 12 軸の位置指令に書く。他の軸は触らない。
pub(crate) fn write_leg_targets(cmd: &mut Command, q_isaac: &[f64; 12], kp: f64, kd: f64, max_speed: f64) {
    for i in 0..12 {
        if let Some(a) = cmd.get_mut(AxisId::new(i as u16)) {
            *a = AxisCommand {
                mode: ControlMode::Position,
                position_rad: q_isaac[misa_to_isaac(i)],
                velocity_rad_s: max_speed,
                torque_ff_nm: 0.0,
                kp_nm_per_rad: kp,
                kd_nm_s_per_rad: kd,
            };
        }
    }
}

/// 参照歩容の契約世代。
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Contract {
    V12,
    V15,
    V16,
    /// v24（後退の 3 段設計 + sg 1.25/0.78）。実機比較の第 2 候補 =
    /// v24_long_s103/exported/policy_5500.onnx（go2_rl doc §15）。
    V24,
}

impl Contract {
    fn label(self) -> &'static str {
        match self {
            Contract::V12 => "v12",
            Contract::V15 => "v15",
            Contract::V16 => "v16",
            Contract::V24 => "v24",
        }
    }
}

pub(crate) fn load_controller(a: &Args) -> Result<NamiashiRefController, String> {
    use misa_policy_runner::namiashi::RefGaitCfg;
    let policy = OnnxPolicy::load(&a.model, misa_policy_runner::namiashi::N_OBS)?;
    let gait = match a.contract {
        Contract::V12 => RefGaitCfg::v12(),
        Contract::V15 => RefGaitCfg::v15(),
        Contract::V16 => RefGaitCfg::v16(),
        Contract::V24 => RefGaitCfg::v24(),
    };
    let ctl = NamiashiRefController::new(policy, gait)?;
    eprintln!(
        "policy: {} を読み込みました — namiashi {} 契約（47 入力、位置目標のみ、学習域 vx {:.2}..{:.2} / |vy| ≤ {:.2} / |wz| ≤ {:.2}）",
        a.model,
        a.contract.label(),
        CMD_VX_RANGE.0, CMD_VX_RANGE.1, CMD_VY_RANGE.1, a.wz_max
    );
    Ok(ctl)
}

pub(crate) fn run(args: &[String]) -> Result<(), String> {
    let a = parse(args)?;
    if a.sim {
        #[cfg(feature = "sim")]
        return crate::policy_sim::run(&a);
        #[cfg(not(feature = "sim"))]
        return Err("このビルドには sim が入っていません（--features sim で有効化。MUJOCO_DYNAMIC_LINK_DIR も要る）".into());
    }

    let mut ctl = load_controller(&a)?;
    let cfg = AppConfig::load(&a.robot)?;
    log::info!("ロボット {} のプロファイル {} を読みました", cfg.name, a.robot);

    // ── 接続と立ち上げ（misa-runner の run と同じ手順）──
    let map = PortMap::discover().map_err(|e| e.to_string())?;
    let plant = misa_runner::plant::SerialPlant::connect_with(&cfg, &map)?;
    let pilot = misa_runner::pilot::SbusPilot::connect_with(&cfg, &map, true)?;
    let opts = RunOptions {
        allow_no_sbus: true,
        ..RunOptions::default()
    };
    let mut plant = start_serial(&cfg, &opts, plant, &pilot)?;
    let n = plant.axes().len();
    let max_speed = cfg.hardware.default_max_speed_rad_s();
    let mut obs = Observation::empty(n, 4);
    let mut cmd = Command::idle(n);

    plant.arm()?;
    // 脱力のまま数周期回して 12 軸と IMU を読む。
    for _ in 0..5 {
        plant.exchange(&cmd, &mut obs)?;
        std::thread::sleep(Duration::from_secs_f64(CONTROL_DT));
    }
    if obs.any_unread() {
        plant.disarm()?;
        return Err("まだ読めていない軸があります（配線・電源を確認）".into());
    }
    let (inp0, tilt0) = obs_input(&obs)?;
    if tilt0 > TILT_ABORT_RAD {
        plant.disarm()?;
        return Err(format!("起動時の傾きが {:.0}° — 機体を水平にしてから", tilt0.to_degrees()));
    }
    let start_isaac = inp0.joint_q_isaac;
    let default_isaac = ctl.default_pose_isaac();

    let loop_start = Instant::now();
    let mut tick: u64 = 0;
    let sleep_until_next = |tick: &mut u64| {
        *tick += 1;
        let next = loop_start + Duration::from_secs_f64(CONTROL_DT * *tick as f64);
        if let Some(d) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
    };

    // ── A: 実測姿勢 → 立位へランプ ──
    eprintln!("policy: 立位へ {RAMP_SECS} s でランプします");
    let ramp_n = (RAMP_SECS / CONTROL_DT) as u64;
    for i in 0..=ramp_n {
        let p = i as f64 / ramp_n as f64;
        let q: [f64; 12] = core::array::from_fn(|j| (1.0 - p) * start_isaac[j] + p * default_isaac[j]);
        write_leg_targets(&mut cmd, &q, 0.0, 0.0, max_speed);
        plant.exchange(&cmd, &mut obs)?;
        sleep_until_next(&mut tick);
    }
    for _ in 0..(0.5 / CONTROL_DT) as u64 {
        plant.exchange(&cmd, &mut obs)?;
        sleep_until_next(&mut tick);
    }

    // ── B: 方策ループ ──
    let clamp = cmd_clamp(&a);
    let cmd_shared = Arc::new(Mutex::new(clamp(a.cmd0)));
    let quit = Arc::new(AtomicBool::new(false));
    let kb = if a.keyboard {
        Some(spawn_keyboard(cmd_shared.clone(), quit.clone(), clamp.clone())?)
    } else {
        eprintln!(
            "policy: キーボード無効。vx={:.2} vy={:.2} wz={:.2} を保持",
            a.cmd0[0], a.cmd0[1], a.cmd0[2]
        );
        None
    };
    ctl.reset();
    let mut held = ctl.hold();
    let mut faults: u32 = 0;
    let mut status = Instant::now();
    let run_start = Instant::now();
    let mut abort: Option<String> = None;
    let mut rec = match a.record.as_deref() {
        Some(p) => Some(Recorder::open(p)?),
        None => None,
    };
    let mut servo = a.heading_hold.map(|(kp, ki)| {
        let mut sv = misa_policy_runner::heading::HeadingServo::new(kp, ki);
        if a.heading_hold_turn {
            sv.straight_wz = 10.0; // 実質「常に補正」
        }
        sv
    });
    if let Some(sv) = &servo {
        eprintln!("policy: 方位保持 ON（KP {} KI {}、補正 ±{} rad/s、{}、静止中は補正しない）\r",
            sv.kp, sv.ki, sv.clip, if a.heading_hold_turn { "旋回中も補正" } else { "直進のみ" });
    }
    eprintln!(
        "policy: {}\r",
        if a.hold { "--hold — 方策は走らせず立位を保持して観測だけ表示します" } else { "RUNNING" }
    );

    while !quit.load(Ordering::Relaxed) {
        if let Some(d) = a.duration {
            if run_start.elapsed().as_secs_f64() >= d {
                break;
            }
        }
        plant.exchange(&cmd, &mut obs)?;
        let cmd_user = *cmd_shared.lock().unwrap();
        let cmd_now = match servo.as_mut() {
            Some(sv) => sv.apply(obs.imu.map(|i| i.rpy_rad[2]).unwrap_or(0.0), cmd_user, CONTROL_DT),
            None => cmd_user,
        };
        let mut invalid = obs.any_unread();
        for i in 0..12 {
            if let Some(st) = obs.get(AxisId::new(i as u16)) {
                invalid |= !st.health.valid;
            }
        }
        match obs_input(&obs) {
            Ok((inp, tilt)) => {
                if tilt > TILT_ABORT_RAD {
                    abort = Some(format!("傾き {:.0}° — 脱力して中断しました", tilt.to_degrees()));
                    break;
                }
                if invalid {
                    faults += 1;
                } else if !a.hold {
                    match ctl.tick(&inp, cmd_now) {
                        Ok(q) => {
                            held = q;
                            faults = 0;
                        }
                        Err(e) => {
                            faults += 1;
                            eprintln!("policy: 推論失敗（保持）: {e}\r");
                            held = ctl.hold();
                        }
                    }
                }
                if let Some(r) = rec.as_mut() {
                    let rpy = obs.imu.map(|i| i.rpy_rad).unwrap_or([0.0; 3]);
                    let q_des = if a.hold { default_isaac } else { held };
                    r.row(run_start.elapsed().as_secs_f64(), cmd_now, &q_des, &inp, rpy);
                }
            }
            Err(e) => {
                faults += 1;
                eprintln!("policy: 観測異常（保持）: {e}\r");
            }
        }
        if faults >= MAX_CONSECUTIVE_FAULTS {
            abort = Some("観測異常・推論失敗が連続したため中断しました".into());
            break;
        }
        let target = if a.hold { default_isaac } else { held };
        write_leg_targets(&mut cmd, &target, 0.0, 0.0, max_speed);
        sleep_until_next(&mut tick);

        if status.elapsed().as_secs_f64() > 0.5 {
            let imu = obs.imu.unwrap_or_default();
            eprint!(
                "\rpolicy: t={:6.1}s cmd=({:+.2},{:+.2},{:+.2}) rpy=({:+.2},{:+.2}) gyro_z={:+.2} {}   ",
                ctl.gait_time_s(),
                cmd_now[0],
                cmd_now[1],
                cmd_now[2],
                imu.rpy_rad[0],
                imu.rpy_rad[1],
                imu.gyro_rad_s[2],
                plant.status_line()
            );
            let _ = std::io::stderr().flush();
            status = Instant::now();
        }
    }
    quit.store(true, Ordering::Relaxed);
    if let Some(r) = rec.take() {
        let n = r.finish();
        eprintln!("\r\npolicy: --record {} 行を書きました\r", n);
    }

    // ── C: 終了。中断は即脱力、通常終了は立位へ戻してから脱力 ──
    if let Some(msg) = abort {
        plant.disarm()?;
        if let Some(h) = kb {
            let _ = h.join();
        }
        return Err(msg);
    }
    eprintln!("\r\npolicy: 立位へ戻して脱力します\r");
    let (inp_end, _) = obs_input(&obs)?;
    let from = inp_end.joint_q_isaac;
    let down_n = (1.0 / CONTROL_DT) as u64;
    for i in 0..=down_n {
        let p = i as f64 / down_n as f64;
        let q: [f64; 12] = core::array::from_fn(|j| (1.0 - p) * from[j] + p * default_isaac[j]);
        write_leg_targets(&mut cmd, &q, 0.0, 0.0, max_speed);
        plant.exchange(&cmd, &mut obs)?;
        sleep_until_next(&mut tick);
    }
    plant.disarm()?;
    if let Some(h) = kb {
        let _ = h.join();
    }
    Ok(())
}
