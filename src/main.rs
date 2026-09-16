//! `namiashi-run` — namiashi を動かす実行ファイル。
//!
//! **中身は misa-runner。** namiashi はシリアル直結（LKMTech の脚 12 軸 +
//! WIT の IMU + S.BUS）で、その Plant / Pilot は misa-runner が組み込みで
//! 持っている（`kind = "serial"`）。差すべき機体固有の Backend が無いので、
//! ここは misa-runner の `main_with` を空の Backend 列で呼ぶだけ。
//! サブコマンドも制御則も向こうのものがそのまま出る:
//!
//! ```text
//! namiashi-run check  --robot robots/namiashi.toml   設定とモデルを検証（実機に触れない）
//! namiashi-run dump   --robot robots/namiashi.toml   歩容を実機なしで再生
//! namiashi-run sim    --robot robots/namiashi.toml   MuJoCo（--features sim）
//! namiashi-run run    --robot robots/namiashi.toml   制御ループ（プロポ操縦）
//! namiashi-run policy --model policy.onnx [--sim]    RL 方策（misa-policy-runner、
//!                                                    50 Hz 位置目標のみ）
//! ```
//!
//! `policy` だけはここで受ける（go2-runner と同じ形）— misa-runner の run
//! ループには RL の「50 Hz 推論 + 位置目標」の差し込み口がまだ無い。
//!
//! **機体固有のものはリポジトリの側にある**: `robots/namiashi.toml`（配線・
//! 校正値・トルク上限）、`models/namiashi`、`doc/`（立ち上げ・SBC の運用）。
//! 制御の変更は misa-runner へ。この機体に Backend が要るようになったら
//! （別のバスに替える、など）hayaashi-runner の `KsmBridge` を手本に足す。

mod policy;
#[cfg(feature = "sim")]
mod policy_sim;

fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(|s| s.as_str()) == Some("policy") {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
            .format_timestamp_millis()
            .init();
        return match policy::run(&args[1..]) {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("エラー: {e}");
                std::process::ExitCode::FAILURE
            }
        };
    }
    misa_runner::main_with(&[])
}
