# namiashi-runner2

四脚ロボット **namiashi**（LKMTech MG4005 ×12、腕 RC サーボ ×1、シリアル直結）の
**機体固有部分**。制御そのもの（歩容・WBC・MPC・推定器・安全ゲート・MuJoCo）は
[misa-runner](https://github.com/takarakasai/misa-runner) にあり、ここは
それを呼ぶ実行ファイルと、この機体にしか無いものを持つ。

| ここにあるもの | |
|---|---|
| `robots/namiashi.toml` | プロファイル。配線・モータ id・ゼロ点・可動域・**WBC のトルク上限**（下）|
| `models/namiashi/` | モデル（`.misa` とメッシュ）。同梱の質量は 2.4 kg（→「モデルの質量」）|
| `doc/` | 実機の立ち上げ（`bringup_checklist.md`）、モータ対応表（`motor_map.md`）、SBC の運用（`boot_config.md` / `runtime_tuning.md` / systemd ユニット / 検証スクリプト）、実機の可視化（`viz_live.md`）|
| `tests/model_consistency.rs` | `models/namiashi` の `.misa` と URDF サイドカーの整合（角度・順序）。submodule が要る |
| `scripts/dev-siblings.sh` | 隣の misa-runner を直しながら試すときの `[patch]` |
| `src/main.rs` | `misa_runner::main_with(&[])`。Backend は差していない（シリアル直結の Plant は misa-runner の組み込み）|

**旧 [namiashi-runner](https://github.com/takarakasai/namiashi-runner) は過去の
大会の環境再現のために残してある。** 新しい変更はこちらと misa-runner へ。

## ビルドと実行

```sh
git clone --recurse-submodules https://github.com/takarakasai/namiashi-runner2.git
cd namiashi-runner2                         # models/namiashi は namiashi_description の submodule
cargo build --release                       # misa-runner は GitHub から取る
./target/release/namiashi-run check --robot robots/namiashi.toml
./target/release/namiashi-run dump  --robot robots/namiashi.toml --gait trot
./target/release/namiashi-run run   --robot robots/namiashi.toml
```

MuJoCo で回すときは `--features sim`（`MUJOCO_DYNAMIC_LINK_DIR` が要る。
misa-runner の README「sim」）。サブコマンド・フラグ・環境変数はすべて
misa-runner のもの。

## MuJoCo でリアルタイムに操縦する（MPC + WBC）

実機に触れずに、MPC 歩容 + 全身制御の構成をキーボードで動かして articara で
見る。**`--pilot keys` か `--viz` を付けると sim は実時間で流れる**（付けないと
全力で回って 10 秒ぶんが 1 秒で終わる）。

```sh
# 1) ビルド（MuJoCo の共有ライブラリが要る）
export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
export LD_LIBRARY_PATH=$MUJOCO_DYNAMIC_LINK_DIR
cargo build --release --features sim

# 2) MuJoCo（キーボード操縦 + articara へ配信。Ctrl-C まで）
./target/release/namiashi-run sim --robot robots/namiashi_mpc_wbc.toml \
    --pilot keys --secs 0 --viz --viz-endpoint tcp/127.0.0.1:7447

# 3) 別端末で articara を起動し、同じモデルを開く
cd ../articara && cargo run --release --features viz -- \
    --model ../namiashi-runner2/models/namiashi/namiashi.misa
#    「Live gait feed」パネルでエンドポイント tcp/127.0.0.1:7447 を入れて Start
```

### RL 方策（ONNX）で同じことをする

学習済みの RL 方策（go2_rl の namiashi_rl、45 次元観測の PPO）を MuJoCo で
回してキーボード操縦し、articara でも見る。物理・キー・地面は上の WBC デモと
同じで、コントローラだけが方策に替わる（`sim/` は単独 crate なので `cd sim`）:

```sh
cd sim
export MUJOCO_DYNAMIC_LINK_DIR=$HOME/.mujoco/mujoco-3.8.0/lib
# ort は onnxruntime の共有ライブラリを実行時に読む（go2_rl の venv のを指す）
export ORT_DYLIB_PATH=$HOME/work/install/isaac_5_1_0/env_isaaclab/lib/python3.11/site-packages/onnxruntime/capi/libonnxruntime.so.1.27.0
cargo run --release --no-default-features \
    --features mujoco,mujoco-viewer,onnx,viz --example namiashi_rl_teleop -- \
    --onnx /path/to/namiashi_policy.onnx --viz --viz-endpoint tcp/127.0.0.1:7447
# articara 側は上の 3) と同じ（Live feed を Connect / 同じエンドポイントで購読）
```

キー入力は **MjViewer のウィンドウ**が受ける（articara は第 2 の視点。
地形・接触が描けるのは MjViewer 側だけ）。配信は planned（方策の q_des、
ゴースト）と measured（実測）の対で、go2-runner の `policy --sim` と同じ
ワイヤ契約（quadruped-gait の `VizPublisher`）。`--viz` を使うには feature
`viz` が要る（付けずにビルドした場合は `--viz` 指定時にその旨を出して止まる）。

`robots/namiashi_mpc_wbc.toml` は `robots/namiashi.toml`（配線・校正値は同じ）に
articara が詰めた歩容の値（歩幅 0.145、周期 trot 0.320 / walk 0.500 / crawl
0.800、遊脚 0.04）、`controller = "mpc"`、`[wbc] enabled = true`、
`output = "position"`、`torque_scale = 1.6667` を足したもの。トルク出力を試す
なら `--wbc-output torque`。

キー（押すたびに 1 段。**離しても止まらない**ので止めるのはスペース）:

| キー | |
|---|---|
| `0` / `1` / `2` | 脱力 / 初期姿勢（立つ） / 歩行 |
| `z` / `x` / `c` | 歩容 Crawl / Walk / Trot |
| `w` `s` | 前進 / 後退を 1 段ずつ |
| `a` `d` | 左右真横 |
| `q` `e` | 左右旋回 |
| スペース | 速度 0 |
| `r` `f` | 胴体を高く / 低く |
| `i` `k` `j` `l` / `v` | 胴体の傾き（ピッチ・ロール） / 水平に戻す |
| `t` `g` / `y` `b` / `u` `n` | 歩容の周期 / 遊脚高さ / 歩幅を歩きながら ± |
| `o` | 全身制御の出力を巡回 OFF → 位置 → トルク → OFF（**歩きながら替えられる**。misa-runner 99b4165 より後） |
| `p` | 歩容コントローラ MPC ↔ CHAMP（立って止まっているときだけ効く） |

順序は `1`（立つ）→ `c`（Trot）→ `2`（歩行）→ `w` を数回。状態行の末尾に
`[MPC (SRBD) / WBC 位置]` のように今の構成が出る。articara が無くても
`--viz` を外せば動く（端末の状態行だけ）。動画にするなら `--features render`
と `--video DIR`（misa-runner README「sim」）。

**端末を raw モードにする**ので、落ちてエコーが戻らなかったら `reset`。

### 動画にする（オフスクリーン描画）

GUI 無しで MuJoCo を EGL で描いて PNG に落とし、ffmpeg でまとめる
（`--features render`。misa-runner README「sim」）。胴体高さの変更を検証した
ときの撮り方:

```sh
cargo build --release --features sim,render
./target/release/namiashi-run sim --robot robots/namiashi_mpc_wbc.toml \
    --gait trot --vx 0.3 --secs 15 --height-script "3:-0.04,6:0.03,9:0" \
    --video /tmp/nm --cam-az 100 --cam-el -12 --cam-dist 1.1
ffmpeg -framerate 30 -i /tmp/nm/frame_%05d.png \
    -vf "eq=brightness=0.28:contrast=1.5" -c:v libx264 -pix_fmt yuv420p videos/height.mp4
```

`--height-script` は「歩容が始まってからの秒 : 立ち高さからの差 [m]」
（misa-runner 6b5c667 以降）。`videos/` は追跡しない。

**歩いているところを撮るなら `--cam-fixed`**（床が無地なので、追従カメラだと
足だけ動いて機体は止まって見える）。`--cam-az 90` で右が前、`--cam-x` は
カメラが見る点の前後位置で、進む距離の半分に置く。

### 制御モードごとの歩行動画（2026-09-10）

同じ機体・同じ指令（walk 0.20 m/s、歩容 11 s、misa-runner fcd6b6c）で制御モード
だけ変えたもの。`videos/modes_walk020_v*.mp4`、4 本を並べたのが
`videos/modes_walk020_grid.mp4`（追跡しない）。

| # | モード | プロファイル / フラグ | 前後 [m] | 左右 [m] | ヨー |
|---|---|---|---|---|---|
| 1 | CHAMP、WBC 無し（位置サーボ） | `robots/namiashi.toml` | +2.03 | +0.31 | +10.9° |
| 2 | MPC、WBC 無し | 1 と 1 ビットも違わない（接地力を誰も使わない） | +2.03 | +0.31 | +10.9° |
| 3 | CHAMP + WBC 位置出力 | `namiashi_mpc_wbc.toml` + `--gait-controller champ` | +2.00 | +0.19 | +3.8° |
| 4 | **MPC + WBC 位置出力** | `robots/namiashi_mpc_wbc.toml` | +2.14 | +0.17 | +2.6° |
| 5 | MPC + WBC トルク出力（kd 0.3） | `namiashi_mpc_wbc.toml` + `--wbc-output torque` | +2.10 | +0.04 | +0.8° |

walk 0.20 では前進はどれも指令の 9 割前後で差が出ず、**差が出るのは横ずれとヨー**
（WBC を入れると 1/2〜1/4、トルク出力でほぼゼロ）。指令 0.20 × 11 s = 2.2 m。

```sh
./target/release/namiashi-run sim --robot robots/namiashi_mpc_wbc.toml --gait walk --vx 0.20 \
    --secs 14 --video /tmp/nm4 --cam-fixed --cam-az 90 --cam-el -12 --cam-dist 2.4 --cam-x 1.05 --cam-z 0.15
ffmpeg -framerate 30 -i /tmp/nm4/frame_%05d.png \
    -vf "eq=brightness=0.28:contrast=1.5,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf:expansion=none:text='4. MPC + WBC position output':fontsize=20:fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=6:x=10:y=10" \
    -c:v libx264 -pix_fmt yuv420p videos/modes_walk020_v4.mp4
ffmpeg -i v1.mp4 -i v3.mp4 -i v4.mp4 -i v5.mp4 \
    -filter_complex "[0:v][1:v]hstack[t];[2:v][3:v]hstack[b];[t][b]vstack[v]" -map "[v]" \
    -c:v libx264 -pix_fmt yuv420p videos/modes_walk020_grid.mp4
```

drawtext の文字列に `:` や `%` は入れない（フィルタ構文と衝突する）。

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

`nm_board/ch348` rev2 基板（CH348L, USB-C → 8ch UART）。UART の割り当ては
`spec_rev2_0_0_asbuilt.md` §4 のとおりで、**脚の対応は設計書と違う**
（`doc/motor_map.md`）:

| UART | 役割 | I/F |
|---|---|---|
| 0 / 2 / 1 / 3 | LEG = **FL / FR / RL / RR**（各 3 モータ、RS485 id 1–3）| RS485 |
| 4 | ARMA（腕サーボ。受信機直結、アプリからは駆動しない）| RS485 / TTL 切替 |
| 5 | IMU（WitMotion IWT603）| TTL |
| 6 | S.BUS（受信専用）| 反転 TTL |
| 7 | ARMB（予備）| RS485 / TTL 切替 |

### 未確定

- **初期姿勢（250×350×700 mm の直方体に収める姿勢）は未確定。**
  `control.start_pose` が指す `.misa` のポーズ名で決まる。暫定でモデルの
  `constrain`（thigh 1.0 / calf −2.0）を指している。
- **腕は受信機直結で、アプリからは駆動しない**（`[hardware.arm].protocol =
  "receiver_direct"`）。チキンヘッドと挨拶の腕動作は無効。`teleop.arm` の
  チャンネル（CH7）から角度を観測してログ・可視化に入れている。
- **プロファイルの歩幅はライブラリ既定のまま。** `max_vx_m_s = 0.15` を宣言
  しているが crawl は 0.042 m/s しか出ない。歩幅を上げれば直るが、遊脚の
  擦りと可動域は実機で確かめてから決める。

## WBC / MPC の評価（MuJoCo）

### どこまで動くか（複数モデル）

以下は misa-runner の WBC / MPC を namiashi のモデルで測った結果（2026-09-06、
misa-runner 5228dec）。測り方と機体に依らない結論は misa-runner の README
「評価の仕方と、機体に依らずに言えること」。

**歩容の詰め方でまるで変わる。進む量だけを見て WBC の良し悪しを判断しない
こと。** 歩容が出せる速度は `歩幅 / (周期 × 接地比)` で頭打ちになり、
ライブラリの既定では crawl が 0.042 m/s しか出ない（misa-runner の README「歩幅が速度の上限を決める」）。

articara が詰めた歩容の値（`step_length_m = 0.145`、周期 trot 0.320 /
walk 0.500 / crawl 0.800、`swing_height_m = 0.04`、`mpc_capture_point_gain_s
= 0`）、MPC 歩容、5 走行（trot 0.78 / 0.80 / 0.82、walk 0.33、crawl 0.17、
各 16 s）を **3 つのモデル**で回した。転倒はどれも無し。

| モデル | 質量 | 脚の質量 | `torque_scale` | 位置出力 | **トルク出力** |
|---|---|---|---|---|---|
| 同梱 `namiashi.misa` | 2.4 kg | 36 % | 1.0（連続定格） | 1.03 / 0.99 m / 2.8° | **0.99** / 1.76 m / 2.0° |
| `namiashi_3p3_prop`（実機の質量、脚に比例配分） | 3.3 kg | 73 % | **1.667**（MG4005 の瞬時） | 1.03 / 0.77 m / 4.2° | **1.02** / 0.31 m / 1.0° |
| `namiashi_3p3_hip`（実機の質量、hip に集中） | 3.3 kg | 73 % | **1.667** | 1.02 / 0.12 m / 1.2° | **1.06** / 0.32 m / 1.5° |
| `namiashi_3p3_prop` | 3.3 kg | 73 % | **1.375**（19.8 V バッテリ） | 1.03 / 0.81 m / 4.2° | **1.01** / 0.58 m / 2.5° |
| `namiashi_3p3_prop` | 3.3 kg | 73 % | 2.0（参考） | 0.98 / 0.98 m / 4.3° | 1.02 / 0.32 m / 1.2° |
| `namiashi_3p3_hip` | 3.3 kg | 73 % | 2.0（参考） | 1.02 / 0.06 m / 1.2° | 1.06 / 0.19 m / 0.2° |

セルは「平均追従率（進んだ距離 / (12.5 s × 指令)）/ trot 3 本の横ずれの平均
/ trot のヨーの平均」。3.3 kg のモデル（`articara/tests/fixtures/namiashi/`）
は **連続定格のままでは trot 0.80 が歩けない**（位置出力で −2.4 m・ヨー
227°、トルク出力は 0.40 m/s でも 603° 回る。周期の 78〜97 % でトルクが
上限に当たる）。**MG4005 の瞬時 2.5 N·m（= 1.667 倍）で歩ける**ので、実機の
モータの余裕は trot 0.80 に足りている。19.8 V バッテリの電圧比 1.375 でも
歩く（トルク 1.01、横ずれは 0.31 → 0.58 m と増える）。walk 0.33 は 3.3 kg でも定格内で歩く
（+4.78 m、飽和 10 %）。摩擦を硬くした床（`--impratio 10 --cone elliptic`）
でも 3.3 kg × 1.667 は trot 0.80 を位置出力 99 %・トルク出力（#3 + E 込み）
90 % で歩く — 2.4 kg × 連続定格が歩けなかった条件。

### 取り込んだ改善と採用可否（複数モデル）

legged_control と文献（Sleiman 2021、Grandia 2022、Bellicoso 2016）から
1 つずつ入れ、上と同じ 3 モデル × 5 走行で測った。基準は `joint_kd = 0.3`・
MPC・トルク出力（上の表の「トルク出力」の列）。セルは「平均追従率 /
|追従率 − 1| / trot 横ずれ m / trot ヨー °」。

| # | 項目 | 設定 | 2.4 kg | 3.3 kg prop | 3.3 kg hip | 採用 |
|---|---|---|---|---|---|---|
| — | 基準（トルク、kd 0.3） | | 0.99 / 0.03 / 1.76 / 2.0 | 1.02 / 0.03 / 0.32 / 1.2 | 1.06 / 0.06 / 0.19 / 0.2 | |
| — | 位置出力（参考） | `output = "position"` | 1.03 / 0.03 / 0.99 / 2.8 | 0.98 / 0.03 / 0.98 / 4.3 | 1.02 / 0.02 / 0.06 / 1.2 | |
| 0 | hybrid PD の kd | `[wbc] joint_kd = 0`（比較） | 1.10 / 0.10 / 0.90 / 2.8 | 1.09 / 0.09 / 0.33 / 1.0 | 1.08 / 0.08 / 0.57 / 1.6 | **既定 0.3**。0 は 3 モデルとも 8〜10 % 速すぎ、1.0 は崩れる（trot 0.80 で +2.3 m・ヨー −33°）|
| 1 | MPC に高さ・姿勢の観測 | `[gait] mpc_observe_pose = false`（比較） | 0.97 / 0.06 / 2.37 / 9.7 | 1.00 / 0.02 / 0.37 / 1.5 | 1.03 / 0.03 / 0.38 / 1.8 | **既定 true**。2.4 kg では切ると横ずれ・ヨーが悪化。3.3 kg は同等 |
| 2 | 18 状態 LKF | `[gait] estimator = "kalman"` | 1.04 / 0.04 / 1.21 / 10.8 | 1.04 / 0.04 / **0.08** / 1.5 | 1.08 / 0.08 / 0.12 / 0.4 | **選択可**（既定 leg_odometry）。3.3 kg では横ずれ最小、2.4 kg ではヨーが増える。MuJoCo の加速度計は胴体速度の差分 |
| 3 | 実測の接地（5 N）を WBC の立脚に | `[wbc] use_measured_contact = true` | 1.03 / 0.03 / **0.59** / **1.2** | 1.03 / 0.03 / 0.28 / 0.7 | 1.04 / 0.04 / 0.35 / 1.8 | **選択可・推奨**（既定 false）。3 モデルとも横ずれが減る唯一の項目。実機は接地を測れないので効かない（`None` → 計画） |
| A | 遊脚の加速度誤差積分（Grandia） | `[wbc] swing_accel_integral_k = 0.3`, `_sat_nm = 0.15` | 0.98 / **0.02** / 0.74 / **0.7** | 1.00 / **0.01** / 0.22 / **0.4** | 1.00 / **0.01** / 0.65 / 9.2 | **選択可**（既定 0）。速度の追従は 3 モデルとも最良、hip 集中モデルではヨーが出る |
| V | MPC の速度を開ループに | `[gait] mpc_observe_velocity = false` | 0.95 / 0.05 / 1.26 / 1.5 | 0.93 / 0.07 / 1.07 / 10.4 | 0.97 / 0.03 / 0.86 / 7.4 | **不採用**（選択は残す）。速度の閉ループを切ると横ずれ・ヨーが悪化する — MPC の v_y / ω_z のフィードバックがまっすぐ歩かせている |
| E | 推定器の立脚に実測の接地 | `[gait] estimator_use_measured_contact = true` | 推定速度が真値の 74 % → 86 %（位置出力、trot 0.80） | | | **選択可・推奨**。浮いている足を「止まっている」と信じる誤差を消す。残りは足の前滑り |
| 7 | 着地時の足の鉛直速度 | — | | | | 見送り。quadruped-gait では `centroidal` の parity 経路だけが見る |
| C | 接地力の変調（Bellicoso） | — | | | | 見送り。接地力タスクは 4 脚接地で解を動かせず、2 脚接地では力が一意 |
| D | 重み付き単一 QP（Grandia） | — | | | | 見送り。ライブラリに無い。HoQP で足りている |

### 指令通りに動くために分かったこと

`sim` が出す **「歩容中の前後距離 真値 / 推定器の積分」** と **「計画と接地の
ずれ」**、**「接地中の足の滑り（進行方向の成分）」** がこの節の根拠。

1. **速度の追従は閉ループで決まっている。** MPC は推定した速度と指令の差を
   埋めるので、推定が偽の不足を出せばそのぶん速く走る（sim の時間軸が
   歪んでいたとき +18 %）。速度の閉ループを切る（V）と横ずれ・ヨーが崩れる
   ので、**閉じたまま推定の質を上げる**のが筋。
2. **脚オドメトリの誤差は 2 つ。** (a) 計画の立脚なのに浮いている足（着地・
   離地で各足 3〜12 % の周期）— E で消える。(b) **接地中の足が進行方向へ
   流れる**（trot 0.80 で前足 +0.2 m/s、後足 +0.1 m/s）— 運動学からは見えず、
   IMU でも定常の速度差は観測できない。摩擦を硬くすると（`--impratio 10
   --cone elliptic`）流れは消えるが、代わりに着地の衝撃をそのまま受けて
   トルクが飽和し、trot 0.80 は 2.4 kg でも歩けなくなる（0.19 m/s。0.50 なら
   95 % で歩く。`torque_scale = 2` で 0.66 m/s）。**足の滑りは「摩擦の柔らかさ
   に頼って着地の衝撃を逃がしている」ことの裏返し**で、実機の床が硬いなら
   こちらの世界に近い。
3. **トルクの余裕が支配的。** 3.3 kg は連続定格では歩けず、MG4005 の瞬時
   2.5 N·m（1.667 倍）で歩く。2.4 kg × 連続定格でも trot 0.80 で周期の
   35〜40 % は上限に当たっている。実機のトルク上限は `torque_scale = 1.6667`
   として `robots/namiashi.toml` に入れた（19.8 V は 1.375）。
4. **接地のずれを縫う（#3, E）のは 3 モデルで一貫して効く。** 実機で使うには
   足裏の接地を測る手段が要る — センサを付けるか、**実測トルクから
   `f = −J⁻ᵀ τ` で接地力を推定する**（legged_control は推定接地力 > 40 N で
   接地と見なす。この機体は 5 N 程度）。トルク（電流）は今のドライバから
   読めるので、次に足すべきはこれ。

歩容を**既定値のまま**（歩幅 0.06/0.08/0.10 m）0.05 m/s で走らせると:

| 歩容 | コントローラ | WBC 無効 | 位置 | 速度 | トルク |
|---|---|---|---|---|---|
| crawl | champ（既定） | +0.388 m | +0.436 m | +0.029 m | +0.221 m |
| crawl | **mpc** | +0.403 m | **+0.476 m** | −0.874 m | 9.5 s で転倒 |
| trot | champ（既定） | +0.496 m | +0.590 m | +0.108 m | 4.5 s で転倒 |
| trot | **mpc** | +0.154 m | **+0.659 m** | +0.603 m | 4.0 s で転倒 |

- **位置出力は素の歩容より良い。** 詰めた設定で +0.2〜9 %、既定で +12〜19 %。
- **MPC は位置出力をさらに少し良くする。** 詰めた設定では差が小さいが、
  既定の crawl ではヨーのずれが 19.4° → 8.9° と目に見えて減る。
- **MPC 単体（WBC 無効）は trot を悪くする**（既定設定で +0.496 → +0.154 m）。
  接地点の捕捉点フィードバックが、追従の悪い相手に対して正帰還になる。
  `gait.mpc_capture_point_gain_s = 0` で消える（articara も 0 に落として
  いる）。
- **この表のトルク列は `joint_kd = 3` のもので無効**（上の「`joint_kd` に
  legged_control の 3 をそのまま使ってはいけない」）。詰めた設定での再測は
  上の表。
- **ゲインはすべて 2.4 kg のモデルで詰めたもの。** 実機は 3.3 kg で脚に
  73 %（補正モデルは `articara/tests/fixtures/namiashi/`）。実機へ持って
  いく前に採り直すこと。
- **速度出力は詰め切れていない。** シムのアクチュエータのゲイン
  （`--kv-velocity`、既定 20）に強く依る。位置を先に見ること。

### 歩幅が速度の上限を決める


