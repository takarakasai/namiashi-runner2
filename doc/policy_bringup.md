# RL 方策（`policy`）の立ち上げ手順 — 仮設定での実機投入

対象: go2_rl の namiashi WalkFlat 方策（v13 Adapt MLP。
`doc/namiashi_policy_architecture.md` §5 のデプロイ候補）を
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

実測（MLP 1500、10 s）: 前進 0.1/0.2/0.3 → 0.139/0.244/0.324 m/s、後退 0.12 →
0.092、横 0.12 → 1.10 m、停止 0、kp 15/40・遅延 20 ms・摩擦 0.5 でも転倒なし。
**純旋回 0.2 は転倒**、0.1 は回らないが転ばない、前進 0.15 + wz 0.1 は
yaw 94%（横流れあり）— これが v12 契約の `--wz-max` 既定 0.1 の根拠。

**v15（種較正、2026-09-19。既定契約）では旋回は解決済み**: policy_1899 +
`--contract v15`（既定）の Rust sim 実測（12 s）で純旋回 ±0.4 → 105/112%・
0.2 → 98%・併進ドリフト ≤0.01 m/s・転倒なし、前進 0.2/0.3 → 87/82%。
`--wz-max` の既定は契約連動（v15 = 0.4、v12 = 0.1）。v12〜v14 の ONNX を
使うときだけ `--contract v12` を明示する（入力幅が同じ 47 で自動判別
できない。間違えると歩く。悪く。エラーは出ない）。経緯は go2_rl
doc/namiashi_policy_architecture.md §6（ヨー計測の ±π 巻き付き）。

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
   後の現物合わせ（kp/kd の当てはめ）に使える（`--record` は未実装 —
   端末ログと動画で代用）。

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

## 5. 実機比較の第 2 候補: v24（後退が要る運用向け）

`--contract v24 --model .../2026-09-23_20-12-21_v24_long_s103/exported/policy_5500.onnx`。
MuJoCo で前進 96–104% / **後退 109/103%**（v16 は 73%）/ 傾き 3–4°、kp 15/40・
遅延 3 で転倒なし。複合（前進+旋回）のヨーは v16 より弱い（87% 対 102%）。
標準は v16 のまま — 実機で後退が要るときにこちらを試す
（go2_rl doc/namiashi_policy_architecture.md §15）。
