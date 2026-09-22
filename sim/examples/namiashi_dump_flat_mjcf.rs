//! 実測質量モデル（namiashi_meas.misa）の**平地**シーン MJCF を書き出す。
//!
//! go2_rl の `sim2sim_namiashi_mujoco.py` が食べられる形（トルク
//! アクチュエータ `motor_<joint>`、root 胴体 `trunk`、timestep 5 ms、
//! 地面つき）。あちらの既定シーンは WBC ハーネスの replay dump
//! （階段入り、/tmp 置きで揮発）だったので、平地・実測質量の恒久版を
//! これで作る。
//!
//!   cargo run --release --example namiashi_dump_flat_mjcf -- \
//!       [--misa PATH] [--out PATH] [--pose "NAME=RAD,..."] [--base-z M]
//!
//! 既定: ../models/namiashi/namiashi_meas.misa → /tmp/namiashi_flat/model.xml
//!
//! **機体非依存**: トルクアクチュエータは可動関節すべてに付ける。立位姿勢は
//! `--pose` で与え、無指定のときは namiashi の既定姿勢を（その関節がある
//! ときだけ）使う。プレイブックの段 2 を別機体で回すのに要る（ANYmal C は
//! 関節名が `LF_HAA` 形式で、namiashi の名前が 1 つも無い）。

use articara::mjcf::{GroundPlaneCfg, MjcfExportOptions};
use articara::rbd::model::ActuatorMode;
use articara::robot::RobotModel;

const DEFAULT_ISAAC: [(&str, f64); 13] = [
    ("FL_hip_joint", 0.0),
    ("FR_hip_joint", 0.0),
    ("RL_hip_joint", 0.0),
    ("RR_hip_joint", 0.0),
    ("FL_thigh_joint", 0.695),
    ("FR_thigh_joint", 0.695),
    ("RL_thigh_joint", 0.695),
    ("RR_thigh_joint", 0.695),
    ("FL_calf_joint", -1.390),
    ("FR_calf_joint", -1.390),
    ("RL_calf_joint", -1.390),
    ("RR_calf_joint", -1.390),
    ("arm_pitch_joint", 0.0),
];

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |key: &str| -> Option<String> {
        args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
    };
    let misa = get("--misa").unwrap_or_else(|| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../models/namiashi/namiashi_meas.misa")
            .to_string_lossy()
            .into_owned()
    });
    let out = get("--out").unwrap_or_else(|| "/tmp/namiashi_flat/model.xml".into());
    let base_z: f64 = get("--base-z").map(|v| v.parse().expect("--base-z")).unwrap_or(0.235);

    let mut robot = RobotModel::from_misa(std::path::Path::new(&misa))
        .unwrap_or_else(|e| panic!(".misa load failed ({misa}): {e}"));

    // 可動関節はすべてトルク指令にする（名前を知らなくてよい）。
    let mut actuated = 0usize;
    for j in robot.joints.iter_mut() {
        // 固定関節にはアクチュエータを付けない（joint_type で判別する）。
        if j.joint_type != "fixed" {
            j.actuator_mode = ActuatorMode::Torque;
            actuated += 1;
        }
    }

    // 立位姿勢。--pose が無ければ namiashi の既定を、**在る関節にだけ**当てる。
    let mut posed = 0usize;
    match get("--pose") {
        Some(spec) => {
            for item in spec.split(',').filter(|s| !s.trim().is_empty()) {
                let (name, value) = item
                    .split_once('=')
                    .unwrap_or_else(|| panic!("--pose の項は NAME=RAD 形式: {item:?}"));
                let name = name.trim();
                let q: f64 = value.trim().parse().unwrap_or_else(|e| panic!("--pose {item:?}: {e}"));
                let Some(&ji) = robot.joint_map.get(name) else {
                    panic!("joint missing: {name}（--pose）")
                };
                robot.joint_positions[ji] = q;
                posed += 1;
            }
        }
        None => {
            for (name, q) in DEFAULT_ISAAC {
                if let Some(&ji) = robot.joint_map.get(name) {
                    robot.joint_positions[ji] = q;
                    posed += 1;
                }
            }
            assert!(posed > 0, "既定姿勢の関節が 1 つも無い — --pose で与えること");
        }
    }
    robot.rebuild_misarta_model();

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, base_z]),
        ground_plane: Some(GroundPlaneCfg { z: 0.0, half_size: 20.0, roll: 0.0, pitch: 0.0 }),
        add_actuators: true,
        timestep: Some(0.005),
        // MuJoCo の既定（impratio 1・pyramidal）は接地足が荷重の下で這い、
        // 歩容の並進が消える（misa-plant-mujoco の注記と同じ現象を本件でも
        // 実測: 指令 0.2 に対し実効 0.04 m/s）。MuJoCo の推奨に合わせる。
        impratio: Some(10.0),
        cone: Some("elliptic".into()),
        ..MjcfExportOptions::default()
    };
    let xml = articara::mjcf::export_mjcf_with_options(&robot, opts);
    assert!(!xml.is_empty(), "MJCF export failed");
    if let Some(dir) = std::path::Path::new(&out).parent() {
        std::fs::create_dir_all(dir).expect("create out dir");
    }
    std::fs::write(&out, &xml).expect("write model.xml");
    eprintln!(
        "[dump_flat_mjcf] {misa} -> {out} ({} bytes, 可動 {actuated} 関節, 姿勢 {posed} 関節, base_z {base_z})",
        xml.len()
    );
}
