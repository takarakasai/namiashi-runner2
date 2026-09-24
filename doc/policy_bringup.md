# RL 方策（`policy`）の立ち上げ手順 — 仮設定での実機投入

対象: go2_rl の namiashi WalkFlat 方策（**デプロイ標準 = v24 s103@5500**。
`doc/namiashi_policy_architecture.md` §18。§5 に要約）を
`namiashi-run policy` で実機に載せる。**アクチュエータの現物合わせ（内蔵
位置ループのステップ応答）は未了**なので、学習側の kp25/kd0.5 は近似で、
DR（ゲイン ×0.6–1.6、遅延 0–20 ms、摩擦 0.4–1.1、質量 −0.2..+0.6 kg）に
頼っている。以下はその前提で安全側に組んだ順番。

## 0. 何を送るか

50 Hz で脚 12 軸の**位置目標だけ**（MG4005E の位置制御 0xA4）。ゲイン・
トルク前置は送らない。速度上限はプロファイルの `default_max_speed_rad_s`
（8.0）。腕は触らない（Idle）。

観測: WIT IMU（gyro、rpy → 重力射影）とエンコーダ（q, dq）。足裏センサ不要。

## 1. MuJoCo で同じバイナリを回す（実機に触れる前）

```sh
cd namiashi-runner2
./scripts/policy_sim.sh                      # 既定モデル、WASD、articara 配信
./scripts/policy_sim.sh MODEL.onnx --kp 15 --kv 0.3 --duration 10 --vx 0.2
./scripts/policy_sim.sh MODEL.onnx --delay 4 --friction 0.5
```

既定（v24 + サーボ、15 s）の実測は §5 の表。`--wz-max` の既定は契約連動
（v15/v16/v24 = 0.4、v12 = 0.1）。**ONNX は全世代 47 入力で自動判別できない**
ので、v16 以前のチェックポイントを使うときは `--contract` を明示すること
（間違えても歩く。悪く。エラーは出ない）。

**判定は必ず体座標の `fwd=` / `lat=` を読む。** 併記の世界座標 dx/dy は旋回中に
円弧を弦で測るので過小評価になり、複合指令の前進が「12%」のように見える
（実際は 104%。go2_rl doc §15/§18 の計測アーティファクト）。

## 2. ベンチ（脚を浮かせる）

1. ベンチ電源、電流制限あり。機体を吊るか台に載せて 4 脚を浮かせる。
2. 電源投入時は**伏せ姿勢**（`zero_multiturn_on_boot` が初回にそのときの
   姿勢を原点にする — `run` と同じ）。
3. まず `--hold`（方策を走らせず立位を保持、観測を表示）:
   ```sh
   cargo run --release -- policy --model MODEL.onnx --robot robots/namiashi.toml --hold
   ```
   確認: rpy が水平で gyro_z ≈ 0、12 軸が立位（hip 0 / thigh 0.695 / calf −1.39）
   に収まる、脚バスのレートが落ちない、モータが熱くならない。
4. 次に方策を回す（浮かせたまま）:
   ```sh
   cargo run --release -- policy --model MODEL.onnx --keys --wz-max 0.0 --duration 20
   ```
   `W` で vx を 0.05 刻みに上げる。脚が trot の形で動き、片側だけ遅れる・
   振動する軸がないか見る。**ここで内蔵ループの応答を記録**しておくと
   後の現物合わせ（kp/kd の当てはめ）に使える: `--record bench.csv` で
   毎 tick の q_des・q・dq・IMU が CSV に残る。

## 3. 床（低速・短時間）

1. `--wz-max 0.0 --vx-max 0.15 --duration 15` から。W を 2 回（0.10 m/s）。
2. 傾き 45° で自動脱力、`q` で立位に戻して脱力。
3. 前進 0.1 → 0.2、後退 0.1、横 0.1 の順。旋回は最後に `--wz-max 0.1` で。

## 4. 落ちどころ

- 起動直後に「まだ読めていない軸」: 脚バスの配線か電源。`namiashi-run legs` で確認。
- 立位で脚がガクつく: 内蔵ループが学習の kp25 より硬い/柔らかい。
  シムで `--kp 15` / `--kp 40` が歩けているので、まず速度上限
  （`default_max_speed_rad_s`）を疑う。
- 前進指令で横に流れる: 学習側と IMU の取り付け向きの不一致。`namiashi-run imu`
  で機体を前に傾けたとき pitch が + になるか。

## 5. デプロイ標準は v24（2026-09-25 に v16 から切り替え）

**既定**: `--contract v24`（フラグ省略時）、モデル
`.../2026-09-23_20-12-21_v24_long_s103/exported/policy_5500.onnx`、
**方位サーボ `--heading-hold 2 0.5 --heading-hold-turn` を付けて運用**。

体座標メトリクス + サーボ ON の Rust sim 25 ケースで v16 を全軸で上回る
（平均 |追従誤差| 5.1pt 対 11.2pt、最大傾き 4.5° 対 6.0°、転倒 0）:
前進 92–107% / 後退 105–111%（v16 は 71–72%）/ 横 99–104% / 旋回 100% /
複合 前進 101–104%・ヨー 100%。kp 15/40・遅延 4・摩擦 0.5 でも転倒なし。
経緯は go2_rl doc/namiashi_policy_architecture.md §18。

v16（`--contract v16`、`2026-09-19_01-11-02_v16_sgfwd/exported/policy_4598.onnx`）は
実機で比較するときの対照として残す。

## 6. 方位保持 `--heading-hold 2 0.5 --heading-hold-turn`（**推奨・常用**）

方策は世界系の方位参照を持たないので、前進指令でも小さなヨー偏り
（v24 系で −0.04〜−0.08 rad/s）がそのまま曲率になる。報酬（v31）では
消えなかったので、IMU ヨーで PI 補正する（misa-policy-runner
`heading::HeadingServo`、Go2 と同じ部品）。Rust sim 15 s で前進 0.2 の横流れ
−0.81 → −0.01 m、後退 +0.36 → +0.03 m（v16）、速度・傾き不変、補正 0.01–0.04
rad/s。静止中は補正しない。

**`--heading-hold-turn` を付けると旋回中も補正する**（参照は ∫wz_cmd を積むので
「指令どおりの角速度で回る」向き）。v24 の旋回 116/121% → **100/100%**、
複合のヨー 79/127% → **100/100%**、傾き・速度は不変、補正 0.02–0.08 rad/s。
**これがデプロイ標準の運用**（§5）。

**実機では IMU ヨーのドリフトが 1:1 で曲率になる**: 事前に静止 5 分の yaw を
ログし（`--hold --record yaw.csv`）、0.2 m/s で |横| ≤ 0.3 m/15 s を狙うなら
ドリフト ≲ 6°/min が目安。
