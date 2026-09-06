# articara で実機を可視化する

SBC（`radxa-cubie-a7z`）が姿勢を Zenoh へ流し、PC の articara が描く。
**SBC にディスプレイは要らない。**

作成日: 2026-08-21（実機組み上がり直後、段階 2-4 で実際に使った構成）

---

## 2 本のストリーム

配信は**キーを 2 本に分ける**。受け側は measured が来ればそれでモデルを駆動し、
planned を半透明のゴーストで重ねる。

| キー | 中身 |
|---|---|
| `go2/gait/planned` | コントローラが出した**目標角** |
| `go2/gait/measured` | エンコーダから読み戻した**実測角** |

**1 本のキーに両方流してはいけない。** チャネルが latest-wins なので上書きし
合い、受け側は指令と実測の間でガタつく（`--viz-key` と `--viz-key-measured` を
同じ値にすると起動時にエラーで止まる）。

`--viz` は 3 つのコマンドに付くが、**出すストリームが違う**。

| コマンド | planned | measured | モータ | 用途 |
|---|---|---|---|---|
| `legs --viz` | — | ✅ 実測角 + IMU 姿勢 | 🟢 **触れない** | 実機が今どうなっているかを見る |
| `run --viz` | ✅ 目標角 | ✅ 実測角 | 🔴 動く | **指令と実機のズレ**をゴーストで見る |
| `dump --viz` | ✅ 歩容の計算結果 | — | 🟢 実機不要 | 実機なしで歩容を確認 |

片方だけでも成立する。planned 単独ならゴーストは描かれず、measured 単独なら
それでモデルが動く。

**実測側で本当に測っているのは 12 関節と IMU の姿勢 3 軸（roll / pitch / yaw）。**
胴体の位置 `x, y` と高さはオドメトリが無いので歩容の値（`legs` では 0 と設定の
起立高さ）をそのまま入れている。つまり**画面上の胴体位置は実測ではない**。
脚の姿勢と胴体の向きだけが意味を持つ。

`legs --viz` は関節バスに加えて **IMU も開く**（読むだけ。指令は出さない）ので、
機体を傾ければ画面のモデルも傾き、水平に回せば向きも回る。IMU が開けなくても
関節角の確認は続行し、姿勢が `[0,0,0]` のままになる旨を警告する。

> **yaw だけは信頼度が一段落ちる。** roll / pitch は重力という絶対基準がある
> ので方位に依らず合うが、yaw にはそれが無い。WIT の IMU は地磁気で方位を
> 出しているので、**モータやフレームの鉄・電流の影響で狂う**。ゆっくり回って
> 見えるならドリフト、機体を動かすと飛ぶなら磁気外乱。**画面の向きが変でも
> roll / pitch と関節角は別の話**なので、そちらの確認には影響しない。
> 取付基準のずれだけなら `[hardware.imu] mount_offset_rad` の 3 番目で引ける。

立ち上げで実機を確かめたいなら `legs --viz`。起動時に

```
ライブ可視化: **実測角のみ**を配信します（指令は出しません）
```

と出るので、そこで確認できる。

### 送信は制御ループの外

`put` はブロッキングのネットワーク呼び出しで、JSON 直列化も安くない。どちらも
制御ループでやると周期を壊すので、**深さ 8 の有界チャネルで送信スレッドへ渡し、
満杯なら捨てている**（可視化は lossy でよい）。捨てた数は終了時に出る。

```
ライブ可視化: 4213 tick を配信しました（取りこぼし無し、put 8426 件）
```

取りこぼしが出るのは送信が制御周期に追いつかないとき。**周期側は無傷**なので
制御の問題ではないが、画面はコマ落ちする。ネットワークか `--viz-rate` を疑う。

---

## 準備

### PC 側

モデルは [`namiashi_description`](https://github.com/takarakasai/namiashi_description)
にある。**SBC から scp する必要は無い**（SBC 側の `models/namiashi/` も同じものの submodule）。

```sh
git clone https://github.com/takarakasai/namiashi_description.git
```

articara は `viz` フィーチャつきでビルドする。

```sh
cd articara && cargo build --release --features viz
```

### SBC 側

`viz` は既定で有効なので、通常のビルドでよい。
`--no-default-features` でビルドした場合は `--viz` が無言で効かなくなる。

---

## 手順

### 1. SBC で配信を始める

```sh
cd ~/work/misa-runner
./target/release/misa-run legs --secs 0 --viz --robot robots/namiashi.toml
```

`--secs 0`（または `--forever`）で Ctrl-C まで回り続ける。

期待するログ:

```
脚バスを開きました（指令は送りません。観測: Ctrl-C まで）
  FL → /dev/ttyCH9344USB0
  ...
Zenoh can be reached at: tcp/192.168.0.21:33969      ← 到達アドレス
Listening scout messages on 224.0.0.224:7446         ← マルチキャスト探索
ライブ可視化: 指令 'go2/gait/planned' / 実測 'go2/gait/measured' へ 100 Hz で配信します
  ライブ可視化: **実測角のみ**を配信します（指令は出しません）
  IMU → /dev/ttyCH9344USB5（胴体の roll/pitch を measured に載せます）
```

500 ms ごとに 4 本のバスと IMU が並ぶので、**articara を開かなくてもここだけで
関節角と姿勢は確認できる**。

```
FL  435.9Hz 最悪 3.17ms err=0    q=[+0.012 +0.003 -0.001] T=[31 32 31]°C ok=true
...
IMU rpy=(  +0.31,  -1.204,  +12.05)°  gyro=(  +0.02,  -0.01,  +0.00)°/s  198Hz err=0
--
```

### 2. PC で articara を起動する

```sh
cd articara
cargo run --release --features viz -- --model ../namiashi_description/namiashi.misa
```

### 3. articara で購読を始める

**Live gait feed** パネルにキーを入れて Start。

| 欄 | 値 |
|---|---|
| target のキー | `go2/gait/planned`（`--viz-key` で変えていなければ） |
| measured のキー | `go2/gait/measured`（`--viz-key-measured` で変えていなければ） |
| エンドポイント | マルチキャストが通るなら**空でよい**。通らなければ下記 |

`● target — frame #N` と `● measured — frame #N` の増え方で経路を確認する。
`legs --viz` なら measured だけ、`dump --viz` なら target だけが増える。
anchor を `full` にすると関節差が、`world` にすると胴体差が見える。

---

## ネットワーク

同一 LAN でマルチキャストが通れば**設定は要らない**。SBC は
`224.0.0.224:7446` で scout し、`tcp/<SBC の IP>:<動的ポート>` で待つ。

### マルチキャストが通っているかを確かめる

**SBC 側だけで分かること**（`legs --viz` を動かした状態で）:

```sh
ss -ulnp | grep 7446          # 224.0.0.224:7446 に bind しているか
ip maddr show wlan0 | grep 01:00:5e:00:00:e0   # group に join しているか
ip link show wlan0            # MULTICAST フラグが立っているか
```

`01:00:5e:00:00:e0` は `224.0.0.224` に対応するリンク層アドレス
（IPv4 マルチキャストの MAC は `01:00:5e` + group の下位 23 ビット）。
これが出ていれば join できている。

**端から端まで通っているか**は [`scout-check.py`](scout-check.py) で見る。
zenoh が使う group を**受動的に**覗くだけなので、`legs --viz` を動かしたまま
実行してよい。

```sh
# SBC と PC の両方で（相手側で articara を起動した状態で）
python3 doc/scout-check.py
```

```
224.0.0.224:7446 を受信中（15 秒）。自分の IP: ['127.0.0.1', '192.168.0.21']

  192.168.0.21    自分          (3 bytes)
  192.168.0.30    **他ホスト**  (52 bytes)     ← これが出れば通っている
```

**自分の IP しか出なければ届いていない。** 相手側で zenoh アプリ（articara）が
動いているかをまず確認し、動いているのに出ないなら経路で落ちている。

> **WiFi 越しは通らないことがよくある。** AP のクライアント分離や IGMP snooping
> で無線クライアント間のマルチキャストが落とされるため。SBC は wlan0 なので、
> PC も無線だと特に踏みやすい。**その場合はマルチキャストを諦めて下記 A に
> 切り替えるのが早い。**

通らない場合は 2 通り。

### A. 固定ポートで待ち受けて、articara から繋ぐ

```sh
# SBC
./target/release/misa-run legs --secs 0 --viz --viz-endpoint tcp/0.0.0.0:7447 \
    --robot robots/namiashi.toml
# articara のエンドポイント欄: tcp/192.168.0.21:7447
```

> **`--viz-endpoint` は「待ち受け側」の設定である。**
> 内部では `listen/endpoints` を設定し、**同時にマルチキャスト探索を切る**。
> 「PC の articara に繋ぎに行く指定」だと誤解すると、いくら待っても繋がらない。

### B. SSH トンネル

直接繋げない場合（別セグメント、ファイアウォール）。

```sh
# PC 側で
ssh -L 7447:127.0.0.1:7447 takara@192.168.0.21
# SBC 側:      --viz-endpoint tcp/127.0.0.1:7447
# articara 側: tcp/127.0.0.1:7447
```

---

## 確認できること / できないこと

| | |
|---|---|
| ✅ `(バス, id)` → 関節の対応 | FL の thigh を動かして、画面で FL の thigh が動くか |
| ✅ 符号の向き | 曲げた向きと画面の向きが一致するか |
| ❌ **角度の絶対値** | **ゼロ点未校正のうちは合わない**（段階 6 まで） |
| ❌ 腕 | `arm_pitch_joint` は**映らない**（下記） |
| ❌ 胴体の位置・高さ | オドメトリが無い。measured にも歩容の値が入る |
| ⚠ yaw の絶対値 | 地磁気基準。モータの鉄・電流で狂う |

**絶対角が合っていなくても異常ではない。** 段階 6 のゼロ出しを通すまで、表示は
「エンコーダの生値 − 未設定のゼロ点」であって機構的な角度ではない。
そこまでの間に見るのは**動きの対応と符号**だけ。

**腕は映らない。** `GaitVizFrame` が脚 12 関節ぶんしか運ばない器（Go2 向け）の
ため。`arm_pitch_joint` は articara 側で動かないままになる。異常ではない。

---

## 繋がらないときの切り分け

**上から順に潰す。** どこで切れているかで原因が変わる。

### 1. SBC 側で publisher が開いているか

```
ライブ可視化: 指令 'go2/gait/planned' / 実測 'go2/gait/measured' へ 100 Hz で配信します
```

出ていなければ `--viz` が効いていない。`--no-default-features` でビルドした
バイナリではないか確認する（その場合 `--viz` は**無言で**何もしない）。

### 2. Zenoh が到達可能アドレスを出しているか

```
Zenoh can be reached at: tcp/192.168.0.21:33969
```

`127.0.0.1` しか出ていなければ、そのアドレスへは PC から繋げない。

### 3. PC から SBC に届くか

```sh
ping 192.168.0.21
nc -vz 192.168.0.21 7447      # 固定ポートにした場合
```

### 4. articara がフレームを受けているか

キーが一致しているか。SBC 側で `--viz-key` / `--viz-key-measured` を変えたなら
articara 側も同じ値に。**片方だけ増える**なら、増えていない方のキーの不一致か、
そのコマンドがそもそもそのストリームを出していない（上表）。

`run --viz` で measured が出ないときは、まだ**一度も 12 軸そろって読み戻せて
いない**可能性がある。ゼロ姿勢のフレームを「崩れ落ちたロボット」として描かない
ため、最初の読み戻しが済むまで measured は送らない。

### 5. 姿勢が出るが動かない

`legs` のテキスト出力側で `q=[...]` が動いているか確認する。動いていなければ
可視化ではなく**バスの問題**。段階 2-2 / 2-3 に戻る。

---

## 関連

- 立ち上げ手順のなかでの位置づけ: [`bringup_checklist.md`](bringup_checklist.md) §2-4
- SBC + PC で分ける構成の背景: `handover.md` §5.6
- モデルの置き場と submodule: `README.md`
