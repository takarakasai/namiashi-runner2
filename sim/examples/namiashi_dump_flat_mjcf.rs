//! 実測質量モデル（namiashi_meas.misa）の**平地**シーン MJCF を書き出す。
//!
//! go2_rl の `sim2sim_namiashi_mujoco.py` が食べられる形（トルク
//! アクチュエータ `motor_<joint>`、root 胴体 `trunk`、timestep 5 ms、
//! 地面つき）。あちらの既定シーンは WBC ハーネスの replay dump
//! （階段入り、/tmp 置きで揮発）だったので、平地・実測質量の恒久版を
//! これで作る。
//!
//!   cargo run --release --example namiashi_dump_flat_mjcf -- \
//!       [--misa PATH] [--out PATH]
//!
//! 既定: ../models/namiashi/namiashi_meas.misa → /tmp/namiashi_flat/model.xml

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

    let mut robot = RobotModel::from_misa(std::path::Path::new(&misa))
        .unwrap_or_else(|e| panic!(".misa load failed ({misa}): {e}"));
    for (name, q) in DEFAULT_ISAAC {
        let Some(&ji) = robot.joint_map.get(name) else { panic!("joint missing: {name}") };
        robot.joints[ji].actuator_mode = ActuatorMode::Torque;
        robot.joint_positions[ji] = q;
    }
    robot.rebuild_misarta_model();

    let opts = MjcfExportOptions {
        base_pos: Some([0.0, 0.0, 0.235]),
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
    eprintln!("[dump_flat_mjcf] {misa} -> {out} ({} bytes)", xml.len());
}
