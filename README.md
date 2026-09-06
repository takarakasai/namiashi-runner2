# namiashi-runner2

四脚ロボット **namiashi**（LKMTech MG4005 ×12、腕 RC サーボ ×1、シリアル直結）の
**機体固有部分**。制御そのもの（歩容・WBC・MPC・推定器・安全ゲート・MuJoCo）は
[misa-runner](https://github.com/takarakasai/misa-runner) にあり、ここは
それを呼ぶ実行ファイルと、この機体にしか無いものを持つ。

| ここにあるもの | |
|---|---|
| `robots/namiashi.toml` | プロファイル。配線・モータ id・ゼロ点・可動域・**WBC のトルク上限**（下）|
| `models/namiashi/` | モデル（`.misa` とメッシュ）。同梱の質量は 2.4 kg（→「モデルの質量」）|
| `doc/` | 実機の立ち上げ（`bringup_checklist.md`）、モータ対応表（`motor_map.md`）、SBC の運用（`boot_config.md` / `runtime_tuning.md` / systemd ユニット / 検証スクリプト）|
| `scripts/dev-siblings.sh` | 隣の misa-runner を直しながら試すときの `[patch]` |
| `src/main.rs` | `misa_runner::main_with(&[])`。Backend は差していない（シリアル直結の Plant は misa-runner の組み込み）|

**旧 [namiashi-runner](https://github.com/takarakasai/namiashi-runner) は過去の
大会の環境再現のために残してある。** 新しい変更はこちらと misa-runner へ。

## ビルドと実行

```sh
git clone --recurse-submodules ssh://git@github.com/takarakasai/namiashi-runner2.git
cd namiashi-runner2                         # models/namiashi は namiashi_description の submodule
cargo build --release                       # misa-runner は GitHub から取る
./target/release/namiashi-run check --robot robots/namiashi.toml
./target/release/namiashi-run dump  --robot robots/namiashi.toml --gait trot
./target/release/namiashi-run run   --robot robots/namiashi.toml
```

MuJoCo で回すときは `--features sim`（`MUJOCO_DYNAMIC_LINK_DIR` が要る。
misa-runner の README「sim」）。サブコマンド・フラグ・環境変数はすべて
misa-runner のもの。

## この機体の事実（制御の前提）

### モータ: LKMTech MG4005E-i10

- 連続定格 1.5 N·m（減速機出力。モデルの `effort` はこれ）。calf は外側に
  さらに 1.556 の減速（`effort = 2.205`）。
- **24 V での瞬時最大トルクは 2.5 N·m** → `[wbc] torque_scale = 1.6667`。
  バッテリは **19.8 V と 25.6 V の 2 種**で、25.6 V ならこの値、**19.8 V は
  電圧比で 1.375**（まずはこれで。飽和 — `MISA_WBC_DEBUG` の「飽和」行 — を
  見ながら実測で詰める）。
- 電流: ベンチ電源の 5 A 制限はテスト用。電源は 30 A、本番バッテリは 25 A。
  5 A で試すときだけ `[wbc] max_torque_nm` で頭打ちに。
- `torque_constant_nm_per_a` は未設定。**書くまでトルク制御は使えない**
  （N·m が電流 A として線に乗る。`check` が知らせ、`run` は止まる）。
- 多回転アブソリュートを持たない。起動時の張り直しは `doc/motor_map.md`。

### モデルの質量

同梱の `namiashi.misa` は CAD 由来の **2.4 kg（脚 36 %）**。実機は **3.3 kg
で脚が 73 %**（1 脚 600 g）。質量を直したモデルが articara の
`tests/fixtures/namiashi/namiashi_3p3_{prop,hip}.misa`（`rescale_mass.py`）に
ある。**3.3 kg は連続定格では trot 0.80 m/s が歩けない**（位置出力でも −2.4 m・
ヨー 227°）が、瞬時 2.5 N·m（1.667）で歩く。19.8 V 相当の 1.375 でも歩く。
MuJoCo での結果と採用可否の表は misa-runner の README「どこまで動くか
（複数モデル）」。

### 歩容の詰め値（articara の検証から）

`step_length_m = 0.145`、周期 trot 0.320 / walk 0.500 / crawl 0.800 s、
`swing_height_m = 0.04`、`mpc_capture_point_gain_s = 0`。歩容の速度上限は
`歩幅 / (周期 × 接地比)` なので、**ライブラリ既定の歩幅では crawl が
0.042 m/s しか出ない**。プロファイルには入れていない（実機の可動域と一緒に
決める）。

### 配線

UART と脚の対応は**基板の as-built**で `UART0=FL / UART1=RL / UART2=FR /
UART3=RR`（`doc/motor_map.md`。設計書と違う）。IMU は UART5、S.BUS は UART6、
腕は UART4（受信機直結、アプリからは駆動しない）。
