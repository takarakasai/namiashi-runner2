//! namiashi/かわロボ 特化のシミュレーション資産。articara の namiashi-wbc
//! ブランチで育った「ロボットと競技環境にしか意味のない側」がここに住む。
//! 汎用の下回り（mjcf の escape hatch、`respawn_*`、シーン照明、
//! `StaircaseCfg`、WBC/MPC 本体）は articara / quadruped-gait 側。
//!
//! フィーチャ名は articara と同名（`mujoco` / `mujoco-viewer`）にしてあり、
//! 移設したコードの `#[cfg]` はそのまま生きている。

/// 競技リング（かわさきロボット競技大会）と、関節ロックした対戦相手。
pub mod ring;

/// 腕攻撃: 相手を転がすアームの一手と、その評価。
pub mod attack;

/// 起き上がり: 探索で得た回復軌道（V / Shift+V）とその評価器。
pub mod self_righting;

/// WBC 試験ハーネス。`tests/wbc_walk.rs` と各 example の共有部。
#[cfg(feature = "mujoco")]
pub mod wbc_harness;

/// テレオペのキーバインドと HUD（`examples/namiashi_wbc_teleop.rs`,
/// `examples/namiashi_rl_teleop.rs`）。
#[cfg(feature = "mujoco-viewer")]
pub mod teleop;
