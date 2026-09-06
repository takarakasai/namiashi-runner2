# 実行時チューニング（ロボット制御向け）

対象ホスト: `radxa-cubie-a7z`
Debian GNU/Linux 11 (bullseye) / Linux 5.15.147-21-a733 / aarch64
起動時間の最適化は [`boot_config.md`](boot_config.md) を参照。

調査日: 2026-08-20

## 前提

四脚ロボット **namiashi** の制御機として組み込む。制御アプリは
[`misa-runner`](../)（Rust）。

| | |
|---|---|
| 制御ループ | **500 Hz**（周期 2 ms）。`misa-actuator` のパイプライン処理対応に伴い 200 Hz から変更予定 |
| 実機 I/F | `nm_board/ch348` rev2 基板（CH348L, USB-C → 8ch UART） |
| UART0–3 | LEG1–4 = FL / FR / RL / RR（各 3 モータ, RS485） |
| UART4 | ARMA（腕サーボ, RS485/TTL 切替） |
| UART5 | IMU（WitMotion IWT603, TTL） |
| UART6 | **S.BUS（受信専用, 反転 TTL）— 本番の操縦入力（プロポ）** |
| UART7 | ARMB（予備） |
| ドライバ | `ch9344ser_linux`（out-of-tree, `/dev/ttyCH9344USB*`） |

**本番の制御経路は CH348 経由の S.BUS。SSH は開発時のみ。**
したがって最適化の指標は「電源投入から SSH まで」ではなく
**「電源投入から制御ループが S.BUS フレームを処理し始めるまで」**、
および**ループ周期の決定性（ジッタ）**。

## CPU 構成

| policy | CPU | 周波数 | 備考 |
|---|---|---|---|
| `policy0` | 0–5（6コア） | 416 MHz 〜 1794 MHz（9段） | little クラスタ |
| `policy6` | 6–7（2コア） | 416 MHz 〜 2002 MHz（12段） | big クラスタ |

`scaling_driver = cpufreq-dt`

## 調査1: アイドル時の周波数クランプ

### 観測

アイドル時、little クラスタ（6コア）が最低周波数に張り付いていた。

```
policy0: scaling_min_freq=416000  scaling_max_freq=416000  cpuinfo_max_freq=1794000
cooling_device0 (cpufreq-cpu0): cur_state=8 / max_state=8      ← 上限
温度: cpul_thermal_zone = 39 °C（トリップポイントは 60 °C）
```

### これは恒常的な throttle ではない

負荷をかけると即座に解放される（6プロセスのビジーループ 3 秒）。

```
負荷時: policy0 cur=1794000 max=1794000 cdev0_state=0
        policy6 cur=2002000 max=2002000 cdev1_state=0
```

### 原因: thermal governor が `power_allocator` (IPA)

```
$ cat /sys/class/thermal/thermal_zone*/policy
power_allocator     ← 全8ゾーン

$ cat /sys/class/thermal/thermal_zone0/sustainable_power   # cpul
3000
$ cat /sys/class/thermal/thermal_zone1/sustainable_power   # cpub
4000
```

IPA (Intelligent Power Allocation) は温度閾値ではなく**電力バジェット**で周波数上限を制御する。
各 cooling device の「要求電力」に応じてバジェットを配分するため、**アイドル時は要求が小さい
→ 配分が小さい → cooling state が高い → `scaling_max_freq` が下がる**。
39 °C（トリップ 60 °C 未満）でもクランプされるのはこのため。

### 500 Hz 制御での問題

**恒常 throttle ではないが、500 Hz では別の理由で問題になる。**

周期 2 ms のループで実処理が数百 µs のバーストだと、`ondemand` からは「低負荷」に見え続け、
IPA も要求電力を低く見積もる。結果:

- 周波数が上がりきらない
- 上がる場合もランプアップ遅延（`ondemand` のサンプリング周期 + IPA の PID 収束）が
  **そのままループのジッタになる**

ベンチマークでは全開になるのに実運用では上がらない、という典型パターン。

### 対処【適用済み】

cpufreq governor を `performance` に固定する。

```sh
echo performance | sudo tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor
```

永続化は systemd サービスで行った（`cpufrequtils` 等の追加パッケージ不要）:

```ini
# /etc/systemd/system/cpu-performance.service
[Unit]
Description=Pin cpufreq governor to performance (deterministic 500Hz control loop)
Documentation=file:///home/takara/work/misa-runner/doc/runtime_tuning.md

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do echo performance > "$g"; done'

[Install]
WantedBy=multi-user.target
```

```sh
sudo systemctl enable --now cpu-performance.service
```

ロールバック:

```sh
sudo systemctl disable --now cpu-performance.service
sudo rm /etc/systemd/system/cpu-performance.service && sudo systemctl daemon-reload
echo ondemand | sudo tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor
```

### 判定結果: IPA は governor を上書きする【確認済み】

`performance` governor は「**許可された上限まで上げる**」だけで、上限そのものは IPA が
cooling device 経由で下げる。実測の結果、**上書きされることが確認された**。

`performance` 適用後、アイドル 15 秒間の観測:

```
policy0: cur=1794000 max=1794000 cdev0=0       ← 解放
policy6: cur=416000  max=416000  cdev1=11/11   ← クランプされたまま (37 °C)
```

適用前は逆の状態だった（`cdev0=8`, `cdev1=0`）。**IPA は2つのクラスタ間で電力バジェットを
要求量に比例して配分するため、要求の無い側が最大 cooling state まで落とされる。**
これは電力バジェットの絶対量の問題ではなく IPA の配分方式そのものに由来するので、
`sustainable_power` の引き上げでは解決しない見込み。

### ランプアップ遅延の実測

`taskset -c 6` でビジーループを投入し、50 ms 間隔で `policy6/scaling_cur_freq` を追跡:

```
t=0.00  cur=416000   cdev1=11     ← 負荷投入
t=0.06  cur=416000   cdev1=11
t=0.13  cur=2002000  cdev1=0      ← 全開
（以後 2.5 秒間 2002000 を維持）

負荷終了 3 秒後: cur=416000 cdev1=11   ← 再クランプ
```

**416 MHz → 2002 MHz に 60〜130 ms。** 500 Hz（周期 2 ms）換算で **30〜65 制御サイクル分**、
クロック 1/5 の状態が続く。再クランプは負荷終了から 3 秒以内。

### 影響範囲の評価

実測から言えるのは以下:

- **クランプが起きるのは約 3 秒以上のアイドルの後**。負荷が継続している間は解放されたままだった
- したがって定常運転中の 500 Hz ループでは、IPA が解放状態を維持する可能性が高い
- 確実に影響するのは **(a) 起動・アーミング直後の最初の 130 ms 程度**、
  および **(b) 要求が中途半端で IPA の配分が振動する場合**

**(b) が起きるかどうかは実アプリでしか判定できない。** 合成ベンチのビジーループは
デューティ 100% なので、実際の「2 ms 周期で数百 µs だけ働く」パターンとは負荷の見え方が違う。

### 対処案: thermal policy の `step_wise` 化【未適用・要検討】

IPA の配分方式そのものを避けるなら `step_wise` に切り替える。トリップポイント（60 °C）を
超えるまで throttle しない方式で、決定性は最も高い。

```sh
echo step_wise | sudo tee /sys/class/thermal/thermal_zone0/policy   # cpul
echo step_wise | sudo tee /sys/class/thermal/thermal_zone1/policy   # cpub
```

`available_policies: power_allocator user_space step_wise fair_share`
（`policy` は書き込み可能: `-rw-r--r--`）

#### ただしファンの挙動が変わる問題がある

```
pwm-fan: cur_state=4/4, pwm1=255              ← 現在フル回転
cpul zone trip_point_0 = 60000 (active)   <- pwm-fan がここに拘束
cpul zone trip_point_1 = 60000 (passive)  <- cpufreq がここに拘束
温度: cpul 38.5 °C / cpub 37.9 °C
```

現在は IPA がファンをフル回転させており 38 °C を保っている。`step_wise` に切り替えると
**ファンは 60 °C まで回らなくなる**（trip_point_0 が 60 °C のため）。温度が上昇して 60 °C に
達すると、同じ 60 °C にある passive トリップで cpufreq の throttle も始まる。
つまり**持続負荷時の熱性能はかえって悪化しうる。**

**トリップポイントは読み取り専用**（`-r--r--r--`）なので、sysfs からファンの起動温度を
下げることはできない。変更するには Device Tree の修正が必要。

### 次にやるべきこと: 実アプリでの計測

システム側の合成負荷では (b) の判定ができないため、**さらにシステム設定を変更する前に
`misa-run` を実際に動かして周波数を観測する**のが順序として正しい。

```sh
# 別ターミナルで制御ループを回しながら
while :; do
  printf "%s p0=%s p6=%s cdev0=%s cdev1=%s %sC
"     "$(cut -d' ' -f1 /proc/uptime)"     "$(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq)"     "$(cat /sys/devices/system/cpu/cpufreq/policy6/scaling_cur_freq)"     "$(cat /sys/class/thermal/cooling_device0/cur_state)"     "$(cat /sys/class/thermal/cooling_device1/cur_state)"     "$(( $(cat /sys/class/thermal/thermal_zone0/temp) / 1000 ))"
  sleep 0.1
done
```

判定基準:

- 定常運転中 `cdev` が 0 のまま → IPA は問題にならない。`performance` 固定だけで十分
- `cdev` が振動する → `step_wise` 化を検討（ファンの問題とセットで評価）

最終的な判断材料は **`misa-run` 側で実測したループ周期**。
システム側の周波数観測は原因の切り分け用。

**計測器は既に存在する。** `handover.md` §5.5 によれば `misa-run run` の状態行に
「遅延最大」が出る。合成ベンチではなくこの数字で判断できる。

なお同§は対策として `chrt -f 50` と performance ガバナを挙げており、
本ドキュメントの方針と一致する。ただし同§の記述は「制御ループは 200 Hz 目標」のままで、
**500 Hz への変更は未反映**。

## 調査2: その他の決定性・信頼性に関わる設定

| 項目 | 現状 | 評価 |
|---|---|---|
| **`ulimit -r`（rtprio）** | **0** → **95**（2026-08-21 時点で適用済み） | 調査時は RT 優先度を取得できなかった。`limits.d` + `realtime` グループで解消（後述） |
| **USB autosuspend** | **2 秒** | CH348 は USB 接続。500 Hz の経路で autosuspend が働くのは危険 |
| `sched_rt_runtime_us` | 950000 / 1000000 | RT タスクは 95% 上限。ビジーループさせると throttle される |
| カーネル preemption | `CONFIG_PREEMPT=y` | **PREEMPT_RT ではない**。2 ms 周期は SCHED_FIFO + hrtimer なら実用範囲だがジッタは残る |
| `CONFIG_HZ` | 250 | tick 4 ms。hrtimer を使う限り周期精度の制約にはならない |
| `dkms` | **未インストール** | `handover.md` が「DKMS 推奨」とする `ch9344ser_linux` が入れられない |
| thermal zone | 8ゾーン全て kernel の thermal framework | `lm-sensors` 不要。詳細は `boot_config.md` |
| ファン制御 | `cooling_device3 type=pwm-fan` | カーネル側で自動制御。ユーザースペース不要 |

### RT 優先度の付与【開発時は適用済み】

```
# /etc/security/limits.d/99-realtime.conf
@realtime  -  rtprio  95
@realtime  -  memlock unlimited
```

ただし本番ではログインを伴わないため、**systemd サービス化して指定する方が確実**:

```ini
[Service]
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=80
```

`ulimit` に依存せず、ログインセッションも不要。

### USB autosuspend の無効化（未適用）

```sh
sudo nano /etc/kernel/cmdline    # usbcore.autosuspend=-1 を追加
sudo u-boot-update
```

現在のカーネルコマンドラインは `boot_config.md` を参照。

## 調査3: 常駐デーモンの棚卸し【2026-08-21】

`boot_config.md` 案F の続き。案F は**起動時間**の観点で不要サービスを整理したが、
ここでは**運転中に CPU を奪う常駐プロセス**の観点で棚卸しした。
到達目標は「`misa-runner` + SSH（デバッグ用）だけが動いている状態」。

調査時点の常駐サービスは 17 個。案F の効果で大物は残っていなかったが、
**案F で消したはずのものが復活している**ケースが 1 件見つかった。

### 残すもの

| ユニット | 理由 |
|---|---|
| `ssh` | デバッグ経路。要件 |
| `NetworkManager` / `wpa_supplicant` | SSH の経路が wlan0 なので込みで必須 |
| `dbus` / `polkit` | 土台。`polkit` は NetworkManager が依存 |
| `systemd-journald` / `logind` / `udevd` | 土台 |
| `cpu-performance` | 自作。governor 固定（調査1） |
| `getty@tty1` / `serial-getty@ttyAS0` | 最後の砦。**`ttyAS0` はデバッグ用 UART で、S.BUS の UART6（CH348 経由）とは別系統。競合しない** |
| `systemd-resolved` / `timesyncd` / `zramswap` | 軽量。`resolved` は LLMNR だけ切る余地あり（`0.0.0.0:5355` で listen 中） |
| `sddm` | enabled のままだが `multi-user.target` なので inactive。案D の判断どおりで問題なし |

### 発見1: `plymouthd` が起動から居座り続けている

```
$ ps -eo pid,etime,args | grep plymouth
217   44:28  @usr/sbin/plymouthd --mode=boot --attach-to-session --pid-file=/run/plymouth/pid
```

案D で `plymouth-start` / `quit` / `quit-wait` / `read-write` を 4 つとも mask したにも
かかわらず、プロセスは生きている。

- 起動元は **initramfs 側の hook**（`lsinitramfs` で plymouth 関連 98 ファイル）。
  systemd ユニットを mask しても initramfs は関係なく起動する
- そして**終了役の `plymouth-quit` を mask したせいで、誰も殺さない**

つまり mask は「起動を止める」どころか「**終了だけを止めて常駐させる**」方向に働いていた。

**`boot_config.md` 優先度3 の「`quiet splash` を外せば止まる」は誤り。**
現在の `/proc/cmdline` には `quiet` も `splash` も無いが、それでも起動している。

対処は initramfs から外すこと（`plymouth` パッケージの purge → `update-initramfs -u`）。
`apt-get -s purge plymouth` の simulate では `plymouth` / `plymouth-label` /
`plymouth-themes` の 3 つだけが対象で、`packagekit` などは巻き込まれない。

### 発見2: `rtkit-daemon` が案F の disable 後に復活している

```
$ systemctl is-enabled rtkit-daemon → disabled
$ systemctl is-active  rtkit-daemon → active      ← 復活
$ systemctl show rtkit-daemon -p ActiveEnterTimestamp
ActiveEnterTimestamp=Fri 2026-08-21 01:01:48 UTC   ← 起動直後
```

原因は **D-Bus activation**。ユーザセッションの `pulseaudio.service` が起動時に rtkit を
呼び出すため、system 側を `disable` しても意味がない。

さらに `systemctl --user disable pulseaudio` も効かない。vendor 側
（`/usr/lib/systemd/user/`）の enable が残るため、`--user mask` が要る。

**教訓: `disable` は「target から引かれるのを止める」だけで、D-Bus / socket 活性化は
止まらない。この種のサービスは `mask` でないと落ちない。**

### 発見3: `packagekit` が D-Bus activation で上がっている

`static` だが起動 20 分後（01:21:47）に立ち上がって 18 MB 常駐していた。
`mask` で止める。apt は PackageKit を経由しないので影響を受けない。

### 発見4: タイマー 4 本が運転中に発火しうる

| タイマー | 中身 | 判断 |
|---|---|---|
| `fwupd-refresh.timer` | `fwupdmgr refresh` = **ネットワークからメタデータ DL** | 無効化 |
| `man-db.timer` | 毎日 `mandb` 全走査 | 無効化 |
| `e2scrub_all.timer` | **LVM 上の ext4 専用** | この機体は LVM 無し（`sda3` 直マウント）→ **完全に無意味**。無効化 |
| `e2scrub_reap.service` | `e2scrub_all -A -r` = LVM スナップショットの後始末 | 同上で無意味。**タイマーではなく `WantedBy=default.target` の常時起動ユニットなので、最初の棚卸しで取りこぼした**（再起動後の blame で 558 ms / 5位に現れて発覚）。無効化 |
| `fstrim.timer` | 週次 TRIM（`sda` 238 GB） | **残す**。I/O ストール要因ではあるが、長期的にはフラッシュの性能維持に要る。運転時刻を避ける運用で対処 |

`apt-daily*` は案F で無効化済み。`cron` / `unattended-upgrades` は未インストール。

### 発見5: `haveged` は供給する余地が無い

```
$ cat /proc/sys/kernel/random/poolsize     → 256
$ cat /proc/sys/kernel/random/entropy_avail → 256   （＝満杯）
```

この kernel の CRNG では `entropy_avail` は `poolsize` = 256 が上限で、常に満杯として
報告される。`/dev/hwrng` も無い。haveged が入り込む余地は無く、CPU を舐めるだけ。

### 要判断: `irqbalance` は RT 制御ではむしろ有害寄り

割り込みが CPU6（big クラスタ）に集中している。

```
490: xhci   CPU0=10,475   CPU6=1,945,474    ← USB3 = CH348 の経路
437: ehci   CPU0=1,591    CPU6=104,973
455: (wifi) CPU0=1,212    CPU6=163,423
```

irqbalance は定期的に再配置しうるので、**CH348 の割り込み先が運転中に動く = ジッタ源**。
制御ループを big クラスタに置くなら、irqbalance を止めて `/proc/irq/490/smp_affinity` を
明示的に固定する方が決定的になる。

ただしこれは「未検証・今後の課題」の **CPU 親和性**と一体で決めるべき項目で、
実ループの周期ヒストグラムを取ってから判断する。**今回は保留。**

### 開発ツールの常駐に注意

調査中、VSCode Remote と claude が **合計 1.4 GiB / CPU 4〜12%** を消費していた。

```
667  node (extensionHost)  600 MB   4.3%
1141 claude                455 MB   5.3%
7439 claude                396 MB  11.8%
```

`--enable-remote-auto-shutdown` 付きなので SSH を切れば落ちるが、
**接続したまま運転すると 500 Hz ループの邪魔になる。** 運転時は VSCode Remote を切ること。

### 適用

スクリプト: `cleanup-daemons.sh`（SSH / NetworkManager / wpa_supplicant / getty 系には
一切触れないので、デバッグ経路は維持される）

| 項目 | 状態 |
|---|---|
| user `pulseaudio` の mask | **適用済み** |
| `rtkit-daemon` / `alsa-restore` / `alsa-state` の mask | **適用済み** |
| `packagekit` の mask | **適用済み** |
| `fwupd-refresh` / `man-db` / `e2scrub_all` タイマー無効化 | **適用済み** |
| `haveged` 無効化 | **適用済み** |
| `plymouth` purge + `update-initramfs` | **適用済み** |
| `e2scrub_reap` 無効化 | **適用済み**（再起動後の blame で発覚。後述） |
| `irqbalance` | **保留**（CPU 親和性の検討と一体） |

### 結果

常駐サービス **17 → 13**。`plymouthd` / `pulseaudio` / `rtkit-daemon` / `packagekitd` /
`haveged` は全てプロセスごと消滅。タイマーは 5 → 2（`fstrim` と
`systemd-tmpfiles-clean` のみ）。

13 という数は予想の 14 より 1 少ないが、これは `polkit.service` が `static`
= D-Bus 活性化で、認証が必要になった時だけ起動するため。異常ではない。

`ssh` / `NetworkManager` / `wpa_supplicant` / `getty@tty1` /
`serial-getty@ttyAS0` は全て running のままで、デバッグ経路は無傷。

なお `alsa-restore` は `masked / active (exited)` と表示されるが、これは起動時に
1 回走った痕跡（プロセスは無い）。mask は次回起動から効く。

### plymouth purge に伴う initramfs の変化【検証済み】

purge により `update-initramfs` が走り、initrd が再生成された。

| | 案B 適用時 | plymouth purge 後 |
|---|---|---|
| サイズ | 13,399,270 B | **7,015,212 B** |
| 総エントリ数 | 427 | **208** |
| モジュール数 | 3 | **2** |

**減ったモジュールは `updates/dkms/pvrsrvkm.ko.xz`（PowerVR GPU, 424 KB）1 個。**
plymouth の initramfs hook がスプラッシュ描画のために DRM ドライバを引き込んでいたため
で、plymouth が消えれば道連れに消える。ディスク上の
`/lib/modules/*/updates/dkms/pvrsrvkm.ko.xz` は残っており、GPU は root
（`sda3` の ext4、LVM 無し）の起動に一切関与しないので影響しない。

必須コンポーネントは再確認して全て残存:

```
init  conf/initramfs.conf  scripts/local  scripts/init-top/udev
usr/bin/sh  usr/sbin/blkid  usr/sbin/modprobe  usr/sbin/e2fsck
usr/bin/nuke  usr/bin/resume  usr/lib/systemd/systemd-udevd
usr/lib/modules/.../drivers/md/dm-mod.ko.xz
usr/lib/modules/.../fs/fuse/fuse.ko.xz
```

`update-initramfs` が出す
`find: '/var/tmp/mkinitramfs_XXXXXX/lib/modules/.../kernel': No such file or directory`
は案B で無害と検証済み（モジュールコピー前にフックが走るため）。

### 再起動後の検証【2026-08-21・全項目 OK】

検証スクリプト: `verify-after-reboot.sh` / 比較用スナップショット: `daemon-baseline.txt`

| 項目 | 結果 |
|---|---|
| 起動 | **成功**（initrd を作り直したため最大の懸念だった） |
| `plymouthd` の復活 | **無し** — initramfs から本当に消えたことが確定 |
| `pulseaudio` / `rtkit-daemon` / `packagekitd` / `haveged` | 全て起動せず |
| `alsa-restore` | `active (exited)` → **`inactive`**（mask が次回起動から効くことを確認） |
| `ssh` / `NetworkManager` / `wpa_supplicant` / `getty@tty1` / `serial-getty@ttyAS0` | 全て active。wlan0 も 192.168.0.21 で上がる |
| failed ユニット | 0 |
| タイマー | `fstrim` と `systemd-tmpfiles-clean` のみ |

**起動時間が 3.789 s → 3.489 s に短縮された**（案F 時点との比較）。

| | 案F 後 | 今回 |
|---|---|---|
| カーネル | 1.744 s | **1.494 s** |
| ユーザースペース | 2.045 s | 1.994 s |
| 合計 | 3.789 s | **3.489 s** |

カーネル側の -250 ms は **initrd が 13.4 MB → 7.0 MB になった分の展開時間**と見てよい。
「起動時間短縮の打ち止め判断」（`boot_config.md`）で打ち止めにしたはずが、
**常駐プロセスを削る作業の副産物として結果的に縮んだ。**

#### 常駐サービスは 13 個（14 の予想より 1 少ない）

`polkit.service` が一覧から消えた。これは `static` = D-Bus 活性化のユニットで、
**認証が必要になった時だけ起動する**ため。異常ではない。

#### 取りこぼし: `e2scrub_reap.service`

再起動後の blame で **558 ms / 5 位**に現れて発覚した。

```
Description=Remove Stale Online ext4 Metadata Check Snapshots
ExecStart=/sbin/e2scrub_all -A -r
WantedBy=default.target      ← タイマーではなく毎回起動する
```

`e2scrub_all.timer` は無効化したが、**後始末側は別ユニットで、しかも
`WantedBy=default.target` で毎回起動していた。** LVM が無い以上こちらも無意味。

```sh
sudo systemctl disable --now e2scrub_reap.service
```

**教訓: タイマーを無効化しても、同じ機能の `*_reap` / `*-cleanup` 系が
`WantedBy=default.target` で別に居ることがある。`list-timers` だけでは見つからない。**

### 副作用: apt の trigger が PackageKit の mask でエラーを出す

`packagekit` を mask したため、パッケージ操作時に
`radxa-system-config-common` の trigger が次を出すようになった。

```
Error: GDBus.Error:org.freedesktop.systemd1.UnitMasked: Unit packagekit.service is masked.
```

**表示だけで、apt の処理そのものは正常に完了している**（plymouth の purge も
initramfs の再生成も成功している）。mask を意図的に選んだ結果なので、無視してよい。

### 残った未整備

**`misa-run.service` は未インストール。これは意図的な判断。**

2026-08-21 時点で**機体を組み立て中**であり、サービス化は時期尚早。
まず**センサ確認等を手動で実行**して挙動を確かめる段階にある。

即座に投入できる状態にはある（下記は全て確認済み）。組み立てが済み、手動確認を
通過してからインストールすること。

| | 状態 |
|---|---|
| ユニットファイル | `doc/misa-run.service`（`systemd-analyze verify` 警告なし） |
| バイナリ | `misa-runner/target/release/misa-run`（2026-08-20 19:12 ビルド） |
| CH348 | 8 ポート認識（`/dev/ttyCH9344USB0-7`）、`ch9344` ロード済み |
| 権限 | `takara` は `dialout` + `realtime` に所属、`ulimit -r`=95 |
| 設定検証 | `misa-run check` → 設定 OK / 関節 18・nq=13 / 配線 FL-RR + IMU(UART5) + S.BUS(UART6) 全て解決、exit=0 |

手動確認に使えるサブコマンド（いずれも脚に指令を送らない）:

```sh
cd ~/work/misa-runner
./target/release/misa-run ports              # CH348 のポートを物理 UART 番号つきで一覧
./target/release/misa-run check              # 設定とモデルの検証（実機に触れない）
./target/release/misa-run imu   --secs 5     # IMU の値
./target/release/misa-run sbus  --secs 5     # プロポ入力と解釈結果
./target/release/misa-run legs  --secs 5     # 脚バスの状態と実効周期（指令は送らない）
./target/release/misa-run dump               # 歩容を実機なしで再生し可動域を検証
```

`sbus` は S.BUS2 のテレメトリ（`Rx-Batt` / `Ext-Volt`）とリンク断の理由
（`FAILSAFE` / `FRAME_LOST` / `TIMEOUT`）も出す。チェック項目は
[`bringup_checklist.md`](bringup_checklist.md) §3-2 を参照。

インストールは組み立て完了後:

```sh
sudo cp ~/work/misa-runner/doc/misa-run.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start misa-run     # まず手動起動で確認
sudo systemctl enable misa-run    # 挙動を確認してから自動起動を入れる
```

`Restart=` の判断（自動復帰させるか）は「判断待ち」のまま。既定は `Restart=no`。

パッケージとしては `cups` / `samba` / `bluez` / `avahi-daemon` がまだ入っている
（サービスは disabled で起動しない）。ディスク容量だけの話なので優先度は低い。

## CH348 / ch9344 ドライバの DKMS 登録【適用済み】

`handover.md` §5.4 が SBC 側の前提として挙げている項目のうち、
「ch9344 ドライバ（DKMS 推奨）」に対応した。

### 前提の確認

| 項目 | 状態 |
|---|---|
| カーネルヘッダ | `linux-headers-5.15.147-21-a733` インストール済み。`/lib/modules/$(uname -r)/build` → `/usr/src/linux-headers-...` 有効 |
| `dkms` | **インストール済み**（2.8.4-3, `/usr/sbin/dkms`） |
| ドライバソース | `/home/takara/work/board/ch9344ser_linux`（git HEAD `0450213`）。手動ビルド済みの `.ko` があったが DKMS 未登録 |
| 既存 DKMS モジュール | `aic8800-usb`（WiFi）, `img-bxm-dkms`（GPU）, `radxa-overlays` |

> **注意: `dkms` / `sysctl` / `modinfo` / `rfkill` が "command not found" になる。**
> ユーザーの `PATH` に `/usr/sbin` と `/sbin` が含まれていないため。
> `PATH=/usr/local/bin:/usr/bin:/bin:/usr/local/games:/usr/games`
> フルパスで呼ぶか PATH を通すこと。**「未インストール」と誤判定しやすい。**

### デバイスの認識

```
$ lsusb | grep 1a86
Bus 001 Device 004: ID 1a86:55d9 QinHeng Electronics USB2.0 To Multi Serial Ports
```

`ch9344.c` の ID テーブルに `USB_DEVICE(0x1a86, 0x55d9)` があり、
`ch9344.c:2982` に `idProduct == 0x55d9`（CH348 の 8 ポート構成）専用の分岐がある。

**`cdc_acm` による横取りは発生しなかった**（`/dev/ttyACM*` は生成されず）。
ブラックリストは不要。

### dkms.conf

```sh
PACKAGE_NAME="ch9344"
PACKAGE_VERSION="2.3-0450213"

# WCH の Makefile は KERNELRELEASE が非空だと obj-m を定義するだけの
# 「kbuild から include される側」に分岐する。DKMS は KERNELRELEASE を
# 渡すため、外側から `make` を呼ぶとターゲットが無く "No targets" になる。
# したがって kbuild (-C <kernel> M=<builddir>) を直接呼ぶ。
MAKE[0]="make -C ${kernel_source_dir} M=${dkms_tree}/${PACKAGE_NAME}/${PACKAGE_VERSION}/build modules"
CLEAN="make -C ${kernel_source_dir} M=${dkms_tree}/${PACKAGE_NAME}/${PACKAGE_VERSION}/build clean"

BUILT_MODULE_NAME[0]="ch9344"
DEST_MODULE_LOCATION[0]="/kernel/drivers/usb/serial"

AUTOINSTALL="yes"
```

#### ハマった点: `MAKE[0]` の形式

最初 `MAKE[0]="make KERNELDIR=${kernel_source_dir}"` としたところビルドが失敗した。

```
make: *** No targets.  Stop.
```

WCH の Makefile は以下の構造で、DKMS が渡す `KERNELRELEASE` によって `else` 側に落ちる。

```make
ifeq ($(KERNELRELEASE), )
    KERNELDIR := /lib/modules/$(shell uname -r)/build
    default: $(MAKE) -C $(KERNELDIR) M=$(PWD)
else
    obj-m := ch9344.o       ← ターゲットが1つも無い
endif
```

**kbuild を直接呼ぶ形式にすれば、ベンダー Makefile の `ifeq` 構造に依存しない。**
これが DKMS の標準的な呼び出し方。

### バージョン表記

`ch9344.c` 冒頭の変更履歴に `V1.0 - initial version` とあるが、これは履歴の最初の行であって
版数ではない。実際の版数は:

```
ch9344.c:83   #define VERSION_DESC "V2.3 On 2025.07"
```

カーネルログにも `ch9344: V2.3 On 2025.07` と出る。
DKMS のバージョンは `2.3-0450213`（upstream V2.3 + git short SHA）とした。

### 導入手順

```sh
VER=2.3-0450213
SRC=/home/takara/work/board/ch9344ser_linux/driver
sudo mkdir -p /usr/src/ch9344-$VER
sudo cp $SRC/ch9344.c $SRC/ch9344.h $SRC/Makefile /usr/src/ch9344-$VER/
sudo tee /usr/src/ch9344-$VER/dkms.conf   # 上記の内容
sudo dkms add    -m ch9344 -v $VER
sudo dkms build  -m ch9344 -v $VER
sudo dkms install -m ch9344 -v $VER
```

ソースは git 作業ツリーからコピーする。作業ツリーは変更されうるため、
DKMS が参照する実体は `/usr/src` 側に固定するのが正しい形。

ロールバック:

```sh
sudo dkms remove -m ch9344 -v 2.3-0450213 --all
sudo rm -rf /usr/src/ch9344-2.3-0450213
```

### 結果

```
$ dkms status | grep ch9344
ch9344, 2.3-0450213, 5.15.147-21-a733, aarch64: installed

$ ls /dev/ttyCH9344USB*
/dev/ttyCH9344USB0 ... /dev/ttyCH9344USB7      ← 8 ポート

$ dmesg | grep ch9344
usb_ch9344 1-1:1.0: ttyCH9344USB from 0 - 7: ch9344 device attached.
usbcore: registered new interface driver usb_ch9344
ch9344: USB serial driver for ch9344/ch348.
ch9344: V2.3 On 2025.07
```

配置先は `/lib/modules/5.15.147-21-a733/updates/dkms/ch9344.ko.xz`。
`AUTOINSTALL="yes"` なので、カーネル更新時に新カーネル向けへ自動再ビルドされる。

### シリアルポートの権限

```
crw-rw---- 1 root dialout 168, 0 /dev/ttyCH9344USB0
```

`handover.md` §5.4 が「実行ユーザを `dialout` に入れるか udev ルールを置く。
入っていないと全ポートが `Permission denied`」としている。

```sh
sudo usermod -aG dialout takara     # 反映には再ログインが必要
```

**本番のロボット運用では systemd サービス化するため、`SupplementaryGroups=dialout` を
サービス側に指定する方が確実**（ログインセッションに依存しない）。

## Rust ツールチェーン

`misa-runner` は edition 2024 を使うため **Rust 1.85 以上**が必要
（`handover.md` §5.4）。

| | |
|---|---|
| 現状 | **未インストール**（`~/.cargo`, `~/.rustup` なし） |
| Debian bullseye の `rustc` | 1.48.0 — **要求に全く届かない** |
| 対応 | `rustup` を使う |
| ディスク空き | 220 GB（問題なし） |

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
. "$HOME/.cargo/env"
```

ビルド（`handover.md` §5.2）:

```sh
cd ~/work/misa-runner
cargo build --release --no-default-features    # viz 不要なら軽い方
./target/release/misa-run check
./target/release/misa-run ports                # 8 UART の役割付けを確認
```

ビルド時間の見積もりは handover が「4 コアの SBC で 10〜20 分」。本機は 8 コアなので
もう少し短いはず。バイナリサイズは `--no-default-features` で 62 MB、`strip` で 20 MB。

## モータバスの 20 ms 問題（`lkmotor-driver`）【パッチ適用済み・コンパイル検証済み / 実機未検証】

`handover.md` §4-4 が挙げている既知の罠。500 Hz 化に直接影響するため
調査した。対象は `misa-actuator` の `crates/lkmotor-driver`（pin: `5db57cf`）。

### 症状

> `lkmotor_driver::Rs485Driver` の応答タイムアウトは実質 20 ms が下限。
> 実測で 3 台無応答のバスは 16 Hz まで落ちた。1 台落ちたときの縮退性能がこれで決まる。

500 Hz は周期 2 ms。**モータ 1 台が無応答になると 20 ms、つまり周期の 10 倍を消費する。**

### 機序（handover.md の記述は原因の指摘がやや不正確）

handover.md は「締切判定が read の後にあるため」としているが、実際には
**締切判定は read の前にある**（`driver.rs:142`）。真の原因は別。

```rust
// crates/lkmotor-driver/src/driver.rs
const READ_POLL_TIMEOUT: Duration = Duration::from_millis(20);   // 固定値

pub fn open(device: &str, baud: u32, response_timeout: Duration) -> Result<Self> {
    let port = serialport::new(&device, baud)
        .timeout(READ_POLL_TIMEOUT)      // ← ポートの read タイムアウトは常に 20 ms
        ...
}

pub fn recv_for(&mut self, motor_id: MotorId) -> Result<Response> {
    let deadline = Instant::now() + self.response_timeout;   // 例: now + 5 ms
    loop {
        try_decode(...)                       // → NeedMore
        if Instant::now() >= deadline { return Err(Timeout) }   // 判定は read の前
        self.port.read(&mut scratch)          // ← ここで最大 20 ms 眠る
    }
}
```

**ポートの read タイムアウトが `response_timeout` と無関係な固定 20 ms である**ことが原因。

時系列:

| t | 出来事 |
|---|---|
| 0 ms | `deadline = 5 ms` を設定。`0 < 5` なので判定を通過 |
| 0–20 ms | `read()` が `poll(2)` で 20 ms 眠る |
| 20 ms | `read()` が `TimedOut` を返す。ループ先頭へ |
| 20 ms | `20 >= 5` なので `Err(Timeout)` |

`response_timeout_ms = 5` は**設定しても効かない死に設定**。同じ構造が
`recv_broadcast_replies`（`driver.rs:280`）にもある。

### 修正案

`serialport` 4.9.0 の Unix 実装を確認したところ、read 直前にタイムアウトを
締切までの残り時間へ合わせる方法が使える。

```rust
// serialport-4.9.0/src/posix/tty.rs
fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
    self.timeout = timeout;      // フィールド代入のみ。syscall なし
    Ok(())
}

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        super::poll::wait_read_fd(self.fd, self.timeout)?;   // poll(2) の待ち時間
        nix::unistd::read(self.fd, buf)
    }
}
```

- `set_timeout` は**フィールド代入のみでコストゼロ**。read ごとに呼んで問題ない
- `read` は `poll(2)` の後に `read(2)`。データが来れば poll が即座に返るので、
  タイムアウトを短くしても取りこぼさない（上限が縮むだけ）

パッチは 2 箇所:

```diff
--- a/crates/lkmotor-driver/src/driver.rs
+++ b/crates/lkmotor-driver/src/driver.rs
@@ recv_for
-            if Instant::now() >= deadline {
+            let now = Instant::now();
+            if now >= deadline {
                 return Err(Error::Timeout {
                     motor_id: motor_id.get(),
                 });
             }
+            // poll(2) の待ち時間を「締切までの残り」に合わせる。
+            // 固定の READ_POLL_TIMEOUT のままだと response_timeout より長く眠る。
+            let _ = self.port.set_timeout(deadline - now);

             match self.port.read(&mut scratch) {

@@ recv_broadcast_replies
-                if Instant::now() >= deadline {
+                let now = Instant::now();
+                if now >= deadline {
                     break;
                 }
+                let _ = self.port.set_timeout(deadline - now);

                 match self.port.read(&mut scratch) {
```

`READ_POLL_TIMEOUT` は `open()` 時の初期値としてそのまま残してよい
（`response_timeout` が判明する前の既定値）。

### 500 Hz の成立性

> **【2026-08-21 実測により覆った】この節の結論は誤り。** 初通電で `misa-run legs` を
> 測ったところ **1 バスあたり 415〜440 Hz** で、500 Hz に届かなかった。
> 律速はワイヤ上の時間ではなく **USB の往復レイテンシ**。
> 次節「初通電での実測」を読むこと。以下は当時の机上計算として残す。

**電気的には成立する。** 調査で分かったこと:

| 項目 | 値 |
|---|---|
| `baud`（leg バス） | ~~**1,000,000**（1 Mbps, 8N1 → 10 µs/byte）~~ **実機は 2 Mbps**（`motor_map.md` の訂正を参照） |
| ブロードキャスト TX | `BROADCAST_FRAME_LEN = 11` bytes → **110 µs** |
| 応答フレーム | `HEADER_SIZE(5) + data + 1` ≈ 14 bytes → **約 140 µs** |
| 1 バスあたり（3 モータ） | 110 + 140×3 ≈ **530 µs** |
| 周期（500 Hz） | 2000 µs |

さらに **4 本のバスは並列**。`legs.rs:399` で脚ごとに専用スレッドを立てている。

```rust
let thread = std::thread::Builder::new()
    .name(format!("leg-{}", leg.prefix()))     // leg-FL / leg-FR / leg-RL / leg-RR
    .spawn(move || worker.run())?;
```

したがって 530 µs は 4 本の合計ではなく 1 本あたり。**2 ms の周期に対して余裕がある。**

さらに `robots/namiashi.toml` は既に:

```toml
[hardware.legs]
bus_rate_hz = 500.0          # ← バススレッドは既に 500 Hz
response_timeout_ms = 5
```

**バス側は既に 500 Hz で回っている。** 変更予定なのは `control.rate_hz`（現在 200.0）の方。
`misa-runner/src/config.rs:51` に `control.rate_hz > bus_rate_hz` を弾く検証がある。

### 初通電での実測【2026-08-21】— 律速は USB の往復レイテンシ

`handover.md` §2 が「いちばん大きい未知は通信レート」として実測を待っていた項目。
機体が組み上がり、初めて 12 軸に通電して `misa-run legs --secs 10` を採った。

```
FL  415.8Hz 最悪 6.00ms err=0    q=[+0.304 +0.356 +0.440] T=[32 32 32]°C ok=true
FR  442.7Hz 最悪 5.59ms err=0    q=[+0.544 +0.544 +0.079] T=[32 32 32]°C ok=true
RL  420.7Hz 最悪 5.61ms err=0    q=[+0.347 +0.480 +0.254] T=[33 34 32]°C ok=true
RR  417.6Hz 最悪 6.02ms err=0    q=[+0.287 +0.459 +0.210] T=[32 33 32]°C ok=true
```

12 軸すべて `ok=true` / `err=0`、温度 32〜34 °C。**通信は健全。**
ただし周期は目標に届いていない。

| | 1 周期 | 相当レート |
|---|---|---|
| 理論（2 Mbps のワイヤ時間のみ） | 265 µs | 3774 Hz |
| **実測** | **約 2380 µs** | **約 420 Hz** |
| 目標（`bus_rate_hz = 500`） | 2000 µs | 500 Hz |

**ワイヤ上の時間は全体の約 11% しかない。** 残り約 2100 µs を 3 モータで割ると、
**1 トランザクションあたり約 700 µs** が往復のオーバヘッド。

これは「未検証・今後の課題」に挙げていた **CH348 の USB レイテンシ**そのもの。
`handover.md` §2 の「RS485 の 1 トランザクションは USB の往復レイテンシに律速され、
それが何 µs なのかはモータを繋がないと分からない」という予想が当たった形。

**したがって上節の「電気的には成立する」という結論は、律速要因を取り違えていた。**
ボーレートを 1 → 2 Mbps に上げてもワイヤ時間が 530 → 265 µs になるだけで、
支配項の 2100 µs は動かない。

#### 効いてくる帰結

- **`control.rate_hz = 500` は現状では設定できない。** `config.rs:51` に
  `control.rate_hz > bus_rate_hz` を弾く検証があり、実効の `bus_rate_hz` が
  約 420 Hz である以上 500 は通らない
- **現在の `control.rate_hz = 200`（周期 5 ms）でも、最悪 6 ms は周期を超える。**
  バススレッドは制御ループと別スレッドなので即破綻はしないが、
  最悪ケースで 1 周期ぶん古い状態を読むことになる
- 500 Hz を目指すなら、攻める先はボーレートではなく **USB の往復回数か 1 往復の
  レイテンシ**。`ch9344` に低レイテンシ設定（FTDI の `latency_timer` 相当）が
  あるか、3 モータぶんを 1 往復にまとめられるかが論点になる

#### `response_timeout_ms = 20` の判断は正しかった

最悪応答が **5.59〜6.02 ms**。`bringup_checklist.md` §0-1 が
「組み上がったばかりの機体でモータの応答が 6〜15 ms かかると、パッチ前なら
通っていたものが初めてタイムアウトする」と警告していた帯にちょうど入っている。

**`response_timeout_ms = 5` のままなら、この初通電はタイムアウト多発になっていた。**
症状は「モータが応答しない」で、実際にこの日の直前に踏んだボーレート誤りと
区別がつかず、切り分けが二重に難しくなっていたはず。

### ただし `response_timeout_ms = 5` も 500 Hz には合わない

20 ms のバグを直しても、**設定値の 5 ms 自体が周期 2 ms の 2.5 倍**。
応答 1 本の実時間が約 140 µs であることを踏まえると、**1 ms 程度まで下げるのが妥当**。

修正の順序:

1. `lkmotor-driver` の read タイムアウトを締切連動にする（上記パッチ）
2. `response_timeout_ms` を 5 → 1 程度に下げる
3. `control.rate_hz` を 200 → 500 に上げる
4. `misa-run run` の「遅延最大」で確認

### 適用状況

**2026-08-20: パッチ適用済み。ただしコンパイル未検証（Rust 未導入のため）。**

| | |
|---|---|
| チェックアウト | `/home/takara/work/misa-actuator`（`dev-siblings.sh` の想定レイアウト） |
| ブランチ | `fix/rs485-read-timeout-tracks-deadline` |
| 分岐元 | `5db57cf` — **`main` の HEAD と `misa-runner` の pin が完全一致** していた |
| 変更 | `crates/lkmotor-driver/src/driver.rs` のみ、18 行追加 / 3 行削除 |
| コミット | ~~**未実施**~~ → **2026-08-21 に `a203ce0` で main へ。push 済み** |

> **2026-08-21: 取り込み完了。** それまで作業ツリーにしか無く、`dev-siblings.sh`
> がオフだと GitHub の `5db57cf`（未パッチ）を取っていたため、**ビルドしたバイナリ
> には入っていなかった**。`misa-runner` の `Cargo.lock` も `a203ce0` へ更新済み。
>
> なお「無応答時に 1 周 10 秒」はこのパッチとは**無関係**だった。
> `response_timeout_ms` を 20 → 200 に変えても総時間が 1.3 秒しか動かず、
> 受信側は未パッチでも設定どおり効いている。**効かないのは 20 ms 未満に
> 詰めたときだけ**で、500 Hz 化の段階で意味を持つ。

型の裏取り（`serialport` 4.9.0）:

- `SerialPort` トレイトに `fn set_timeout(&mut self, timeout: Duration) -> Result<()>` あり（`lib.rs:548`）
- `impl SerialPort for Box<dyn SerialPort>` が転送実装を持つ（`lib.rs:734`）ので
  `self.port: Box<dyn SerialPort>` に対して直接呼べる
- `driver.rs` にテストモジュールは無く、`from_port` を使う偽ポートのテストも無い

**検証済み（2026-08-20）:**

```
$ cargo tree --no-default-features -i lkmotor-driver
lkmotor-driver v0.1.0 (/home/takara/work/misa-actuator/crates/lkmotor-driver)
└── misa-hal v0.1.0 → misa-runner v0.1.0
```

`cargo build --release --no-default-features` 成功。`.cargo/config.toml` の `[patch]` により
**パッチ入りのローカルクレートがリンクされている**ことを確認済み。

**未検証事項:**

- 実機での効果測定（無応答モータを作った際の周期劣化が 20 ms → `response_timeout` 相当に
  改善するか）。**配線待ち**
- `cargo test` は未実行

### 影響範囲

`misa-actuator` は別リポジトリで、`misa-runner` は SHA で pin している
（`Cargo.lock`: `git+https://github.com/takarakasai/misa-actuator.git#5db57cf5...`）。
反映には misa-actuator 側へのコミット・push と、`misa-runner` の pin 更新が必要。

ローカルで試すだけなら `misa-runner/scripts/dev-siblings.sh` が
`.cargo/config.toml` に `[patch]` を書き出し、ビルドがローカルチェックアウトを見るようになる。

> `dev-siblings.sh` は既存チェックアウトに対して
> `git pull --ff-only origin <現在のブランチ>` を試すだけ。
> `fix/rs485-...` は origin に無いので pull は失敗して警告を出すのみで、
> **ローカルの変更は保持される**。安全に実行できる。

`recv_for` / `recv_broadcast_replies` は lkmotor 系の全利用者が通る経路なので、
`lkmotor-cli` や `misa-actuator-tui` にも影響する（いずれもタイムアウトが
設定値どおりに効くようになる方向で、劣化はしない見込み）。

## ビルド環境の構築【完了】

`misa-runner` を SBC 上でビルドできる状態にするまでに必要だったもの。

### 必要だったパッケージ

| パッケージ | 症状 | 備考 |
|---|---|---|
| **Rust 1.85+**（rustup） | — | Debian の `rustc` 1.48 では edition 2024 が通らない |
| **`build-essential`** | `/usr/bin/ld: cannot find Scrt1.o` / `crti.o` | `gcc` は入っていたが `libc6-dev` が無く、C 実行ファイルを一切リンクできなかった |
| **`pkg-config` `libudev-dev`** | `libudev-sys` の build script が panic | `serialport` の依存 |

> **`gcc` があること ≠ ユーザースペースをリンクできること。**
> DKMS のカーネルモジュールビルドは `-nostdlib` でリンクするため `Scrt1.o` を必要とせず、
> `libc6-dev` が無くても通ってしまう。DKMS が通ったことを根拠に
> 「ビルドツールは揃っている」と判断すると誤る。

`Cargo.lock` 上の `*-sys` クレートのうち Linux で実際にシステムライブラリを要求するのは
**`libudev-sys` のみ**。他（`core-foundation-sys` / `io-kit-sys` / `security-framework-sys` は
macOS、`windows-sys` は Windows、`jni-sys` は Android、`js-sys` は wasm、`dirs-sys` は純 Rust）は
このターゲットではビルドされない。

### ローカル兄弟チェックアウト

```sh
cd ~/work/misa-runner
./scripts/dev-siblings.sh          # .cargo/config.toml に [patch] を書き出す
./scripts/dev-siblings.sh --off    # git 依存へ戻す
```

`fix/rs485-read-timeout-tracks-deadline` を checkout した状態で実行しても、
`git pull --ff-only origin <branch>` が失敗して警告を出すだけで**ローカルの変更は保持される**
（実地で確認済み）。

### 疎通確認の結果（配線前・2026-08-20）

`misa-run check`: 設定 / モデル（関節 18, nq=13）/ ポーズ / シーケンス / プロポ割り当て /
配線表まで全て OK。

`misa-run ports`: CH348 の UART 番号解決に成功。

```
UART  デバイス                役割
   0  /dev/ttyCH9344USB0        LEG1 (FL) RS485
   1  /dev/ttyCH9344USB1        LEG2 (FR) RS485
   2  /dev/ttyCH9344USB2        LEG3 (RL) RS485
   3  /dev/ttyCH9344USB3        LEG4 (RR) RS485
   4  /dev/ttyCH9344USB4        ARMA (RS485/TTL)
   5  /dev/ttyCH9344USB5        IMU (TTL)
   6  /dev/ttyCH9344USB6        S.BUS (受信専用)
   7  /dev/ttyCH9344USB7        ARMB (RS485/TTL)
```

**`handover.md` §5.4 の SBC 側前提は全て満たされた。**

配線が未完のため、以下は未実施:

- `misa-run imu --secs 10`
- `misa-run sbus --secs 10`
- `misa-run legs --secs 10`
- `misa-run run` の「遅延最大」による周期実測 — **IPA の判定と 20 ms パッチの効果測定は
  どちらもこれ待ち**

## PREEMPT_RT の要否判定【調査完了・導入不要と結論】

500 Hz（周期 2000 µs）に対してスケジューリング遅延が足りるかを `cyclictest` で実測した。

### Debian の RT カーネルは使えない

apt には `linux-image-6.1.0-0.deb11.22-rt-arm64`（PREEMPT_RT）があるが、これは
**Debian の汎用 arm64 カーネル**で、この機体では起動しない。

```
SoC: sun60iw2 (Allwinner A733)
ビルトインの vendor BSP ドライバ: 71 個
  kernel/bsp/drivers/clk/sunxi-ng/ccu-sun60iw2.ko
  kernel/bsp/drivers/ufs/sunxi-ufs-platform.ko    ← ルートファイルシステム
```

**ルートは SoC 直付け UFS (`4520000.ufs`) で、そのドライバは Radxa の BSP にしか存在しない。**
mainline に sun60iw2 のサポートは実質無い。汎用カーネルではルートをマウントできない。

唯一の道は Radxa の BSP カーネル（5.15.147-21-a733）に PREEMPT_RT パッチを当てて再ビルド
することだが、大幅に改変された BSP ツリーへのパッチ適用と 71 個の BSP ドライバの RT 適合
（raw spinlock、atomic 文脈での sleep 等）が必要で、数週間規模かつ失敗のリスクも高い。

### 実測条件

```sh
sudo apt install rt-tests
cyclictest -m -S -p 80 -i 500 -D 3m -h 1000 -q
```

`-p 80`（SCHED_FIFO 80）、`-i 500`（500 µs 間隔 = 500 Hz より細かい粒度）、
`-m`（mlockall）、`-S`（コアごとに 1 スレッド、計 8）。
アイドル 3 分と、全コア CPU 負荷 + ディスク I/O をかけた状態 3 分の 2 通り。

governor は `performance`、RT throttle は 950000/1000000。

> **重要:** cyclictest は起動時に `/dev/cpu_dma_latency` を 0 に設定する
> （出力 1 行目 `# /dev/cpu_dma_latency set to 0us`）。両測定とも**深い C-state は
> 無効化された状態**で行われている。

### 結果

各 2,879,000 サンプル超。

| | アイドル | **負荷時** |
|---|---|---|
| p99 | 13 µs | 13 µs |
| p99.99 | 62 µs | 84 µs |
| p99.9999 | 906 µs | 195 µs |
| **Max** | 907 µs | **206 µs** |
| ≥ 200 µs のサンプル数 | 202 | **1**（290万分の1） |
| ≥ 300 µs のサンプル数 | 196 | **0** |

### 判定: PREEMPT_RT は不要

**負荷時の最悪値 206 µs は、周期 2000 µs の 10.3%。** ≥300 µs は 1 サンプルも無い。
PREEMPT_RT で得られるのは数十 µs 程度の改善で、**数週間のカーネル作業とリスクに見合わない**。

さらにこの測定は**保守的（悲観側）**である。負荷試験終了時の温度は 63.3 °C で、
passive トリップの 60 °C を超えていた。つまり**サーマルスロットリングが働いている状態で
なお 206 µs** だった。

### アイドル時のテールは IPA の周波数クランプで説明がつく

アイドル時だけ 500〜900 µs の帯に約 195 サンプルが集中している（3 分 = 180 秒に対し
**約 1.08 回/秒**）。C-state は無効化されているので、アイドル状態からの復帰遅延ではない。

**周波数比で説明できる:**

```
負荷時 Max 206 µs × (1794000 / 416000 = 4.31) ≈ 888 µs
アイドル時 Max                                  = 907 µs
```

アイドル時は IPA が要求電力を低く見積もってクラスタを 416 MHz にクランプするため
（前述「調査1」）、同じコードパスが約 4 倍の時間を要する。これが 500〜900 µs の帯の正体と
考えられる。

**実運用への含意:** 制御ループが動いている間はクロックが上がるため、負荷時の数値
（206 µs）が実態に近い。ただし**制御ループが 2 ms 周期で 1.5 ms 眠る**パターンが IPA から
どう見えるかは実アプリでしか判定できない。「調査1」の結論と同じく、`misa-run run` の
実測待ち。

### 結論

| 項目 | 判定 |
|---|---|
| PREEMPT_RT の導入 | **不要**。負荷時 206 µs / 周期 2000 µs で十分 |
| SCHED_FIFO の付与 | **有効**。cyclictest はこの条件で測っている |
| 残る不確定要素 | IPA のクランプ挙動（実アプリでの測定待ち） |
| 次に疑うべき箇所 | USB(CH348) のトランザクション遅延、`ch9344` ドライバのバッファリング |

**スケジューリング遅延はボトルネックではない。** ジッタが問題になるとすれば、
原因は CPU ではなく USB 側にある可能性が高い。

## RT 優先度の付与【基盤整備済み】

### 訂正: `--realtime` は RT スケジューリングと無関係

`misa-run` の `--realtime` フラグは **`dump` のリプレイを実時間ペースで流すためのもの**で
（`dump.rs` でのみ使用、`--viz` 用）、SCHED_FIFO とは関係しない。`run` のフラグでもない。

`handover.md` §5.5 のとおり **misa-run 自身は優先度制御を実装していない**。
`mlockall` も使っていない。RT 優先度は外部から付与する必要がある。

### 開発時: `limits.d`

```
# /etc/security/limits.d/99-realtime.conf
@realtime   -   rtprio      95
@realtime   -   memlock     unlimited
```

`realtime` グループを作成し `takara` を追加（反映には再ログインが必要）。
これで `chrt -f 50 ./target/release/misa-run run` が sudo なしで実行できる。

> **rtprio 95 は「上限」であって推奨値ではない。実際に使う値は 20〜50 程度に留めること。**
> 本機は非 PREEMPT_RT カーネルで、USB(CH348) の完了処理は softirq / `ksoftirqd` が担う。
> `ksoftirqd` は **SCHED_OTHER** なので、RT スレッドがコアを飽和させると
> **依存している USB 処理自体を餓死させる**。安全弁は
> `kernel.sched_rt_runtime_us = 950000`（RT は 95% まで）のみ。

### 本番: systemd ユニット（テンプレート、未インストール）

配線が未完で動作確認できないためインストールしていない。要点:

```ini
[Unit]
After=dev-ttyCH9344USB0.device
BindsTo=dev-ttyCH9344USB0.device     # USB を抜くとサービスも停止
After=basic.target                    # ネットワークには依存しない
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
User=takara
SupplementaryGroups=dialout           # ログインセッションに依存しない
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=50
LimitRTPRIO=95
LimitMEMLOCK=infinity
Restart=no                            # 下記の判断待ち
```

`CPUSchedulingPolicy` を systemd に任せることで、**アプリ側に `CAP_SYS_NICE` も
`rtprio` 制限も不要**になる。プロセスの全スレッド（制御ループ / `leg-FL`〜`RR` /
`imu` / `sbus`）が SCHED_FIFO になる。

#### 判断待ち: `WantedBy=` — 起動時間に 1.6 秒効く

テンプレートは `WantedBy=multi-user.target` にしてあるが、**これは
「電源投入 → S.BUS 応答」を 1.6 秒遅らせる。**

CH348 の tty は 3.398 s に生え、`multi-user.target` は 3.489 s。この状態だと
差は 91 ms しかないので一見どうでもよい。しかし
[`boot_config.md`](boot_config.md)「CH348 の tty 生成が 1.764 秒待たされている」
の①（`ch9344` の先読み）を入れると tty が **1.84 s** に前倒しされ、
**律速が `multi-user.target` 側に移る**。

```
現状        3.398 s (tty) → 3.489 s (multi-user) → misa-run
① だけ      1.84 s  (tty) → 3.489 s (multi-user) → misa-run    効果なし
① + ②       1.84 s  (tty) → misa-run                           -1.65 s
```

**①②は片方ずつ評価すると両方とも「効果が薄い」と見えてしまう。**
サービス投入時にセットで設計すること。misa-run はネットワークにも
ログインセッションにも依存しないので、`multi-user.target` を待つ理由は無い。

#### 判断待ち: `Restart=`

`on-failure` にすれば制御プロセスが落ちても自動復帰するが、
「異常終了 → 再起動 → **脚が再び動き出す**」という挙動になる。misa-run は受信断・
フェイルセーフで速度 0・その場起立に入る設計だが、落ちた原因によっては安全側に倒れるとは
限らない。**既定は `Restart=no`（手動復帰）**にしてある。

### Rust 側で実装する案について

`libc = "0.2"` は既に `misa-runner` の直接依存なので実装自体は容易
（`libc::sched_setscheduler` / `libc::pthread_setschedparam`）。ただし:

- **「制御スレッドだけ RT」は逆効果になりうる。** 制御ループはモータ応答を `leg-*`
  スレッド経由で待つ。制御スレッドだけ FIFO でバススレッドが SCHED_OTHER だと、
  制御スレッドがコアを占有した時にバススレッドが走れず応答が遅れる。
  結局ほぼ全スレッド RT = `chrt` / systemd と同じになる
- 昇格には権限が要るが**降格には要らない**。systemd が全スレッドを FIFO にした上で、
  アプリ側は RT にしたくないスレッド（`--viz` の zenoh など）だけ `SCHED_OTHER` に
  落とすのが、`unsafe` も権限も最小で済む形

**まず systemd の一律 FIFO で測り、差別化が必要と分かってから実装する**方針とした。

## CH348 の write がモータ電源 OFF で 5 秒詰まる【実測・回避済み】

**症状:** `misa-run legs` を Ctrl-C しても終了に 41 秒かかる。モータが無応答の
とき 1 周が 10 秒（0.1 Hz）まで落ちる。

**原因は受信タイムアウトではなく送信側だった。** pyserial で
`/dev/ttyCH9344USB0` へ 13 バイトを連続送信した実測:

```
write #1:       0.3 ms  ok
write #2:    5202.9 ms  ok      ← 詰まる
write #3:       0.2 ms  ok
write #4:    5119.9 ms  ok      ← 詰まる
```

**モータ電源が OFF だと、1 回おきに約 5.2 秒詰まる。** これが全ての数字を
説明する（1 周 ≈ 2 回の詰まり ≈ 10 秒、停止 = 4 バス直列で 41 秒）。

切り分けの経緯:

| 疑ったもの | 結果 |
|---|---|
| 受信タイムアウト | **無関係。** `response_timeout_ms` を 20 → 200 にしても総時間は 1.3 秒しか動かない |
| `lkmotor-driver` の 20 ms パッチ | **無関係**（上記と同じ理由）。ただし未コミットでビルドに入っていないのは別途事実 |
| `flush_rx` の無限ループ | `probe_motor`（`calib scan`）の経路で、`legs` のループは通らない |
| **CH348 の write** | **これ。** 上の実測 |

### 対処（`misa-runner` 側、適用済み）

ドライバ / ハードウェア側の挙動なのでアプリからは消せない。**無駄な write を
減らす**方向で 3 点入れた。

1. **`LegArray` の `Drop` で 4 本へ先に停止を知らせる。** それまでは
   `LegBus` が 1 本ずつ「フラグを立てて join」しており、停止が直列化して
   4 倍かかっていた
2. **軸の切れ目でも停止フラグを見る。** 周期の切れ目まで待つと、詰まり 1 回ぶん
   余計に待つ
3. **終了時の `disable` は応答が取れている軸だけに送る。** 届かないモータは
   こちらが励磁しているモータではないので、安全性は損なわない

結果: **41 秒 → 5.5〜15.6 秒。**

ばらつくのは、停止要求が来た瞬間に詰まりの最中かどうかで決まるため。
**進行中の write は OS レベルで中断できない**ので、ここが下限になる。

### 残っている改善余地

全軸が無応答のバスに毎周期 3 軸ぶん投げ続けている。バックオフ（例: 全滅した
バスは 1 秒に 1 軸だけ試す）を入れれば詰まりの回数が 1/3 になり、モータ電源 OFF
のまま `legs` を眺める使い方も現実的になる。復帰の検出を残す必要があるので、
「全滅時だけ間引く」形にすること。

## 未実施の計画

決定性を上位に置いた優先順。

| # | 項目 | 状態 | 理由 |
|---|---|---|---|
| 1 | `performance` governor 固定 | **適用済み**（ただし IPA が上書きするため単独では不十分） | ループのジッタに直結 |
| 1b | 実アプリでの周波数観測 | **次にやる** | `step_wise` 化の要否を判定する |
| 2 | RT 優先度の確保 | **開発時は適用済み**（`limits.d` + `realtime` グループ、`ulimit -r`=95）。本番ユニットは未インストール（下記 9） | ジッタ対策 |
| 2b | PREEMPT_RT | **不要と判定**（cyclictest 実測: 負荷時 206 µs / 周期 2000 µs） | — |
| 3 | USB autosuspend 無効化 | **適用済み**（`usbcore.autosuspend=-1`） | CH348 の経路保護 |
| 4 | `multi-user.target` 化 | **適用済み**（メモリ 2.2→1.0 GiB） | X + KDE が制御ループから CPU/メモリを奪う構図をなくす |
| 5 | ウォッチドッグ有効化 | 未 | プロポで動くロボットのハングは危険 |
| 6 | ch9344 の DKMS 登録 | **適用済み** | `/dev/ttyCH9344USB0-7` 生成確認済み |
| 6b | `dialout` グループ追加 | 適用（要再ログイン） | 無いと全ポート `Permission denied` |
| 6c | Rust 1.85+ (`rustup`) | 未 | `misa-run` のビルドに必要 |
| 7 | 不要サービス無効化 | **適用済み**（常駐 17→14、タイマー 5→2） | 調査3 を参照。`irqbalance` のみ保留 |
| 8 | `lkmotor-driver` の 20 ms 問題 | **パッチ適用済み・コンパイル検証済み**（実機未検証） | 500 Hz の縮退耐性を決める。他のどの項目より影響が大きい |
| 9 | `misa-run.service` の投入 | **保留（意図的）** — 機体を組み立て中。まず手動でセンサ確認 | 準備は完了済み。調査3「残った未整備」を参照 |
| 9b | `ch9344` 先読み + `WantedBy=` 見直し | **保留** — 9 とセットで実施 | 電源投入→S.BUS 応答が -1.65 s。`boot_config.md` 参照 |

4〜7 の詳細は [`boot_config.md`](boot_config.md) の「運用: ロボット組み込み用途への最適化」。

## 未検証・今後の課題

- **実ループ周期の実測** — `misa-run` 側で周期のヒストグラムを取り、governor 変更の効果を確認する。
  システム側の数値ではなくアプリ側の実測が最終判断材料
- **ファンのトリップポイント引き下げ** — `step_wise` 化する場合、Device Tree の修正で
  `pwm-fan` の active トリップを 60 °C より下げられるか
- **CH348 の USB レイテンシ** — ドライバ導入後、tty の低レイテンシ設定
  （FTDI でいう `latency_timer` 相当）が `ch9344` にあるか要確認。
  8 UART × 500 Hz の USB 転送が間に合うかは実測が必要
- **CPU 親和性** — 制御スレッドを big クラスタ（cpu6/7）に固定するか、
  `isolcpus` で専有するか。IRQ の親和性も含めて検討
- **PREEMPT_RT** — 上記で決定性が不足する場合の最終手段。カーネル再ビルドが必要で影響が大きい

## 再現コマンド

```sh
# CPU 周波数の状態
for p in /sys/devices/system/cpu/cpufreq/policy*; do
  echo "$p: cpus=$(cat $p/affected_cpus) gov=$(cat $p/scaling_governor) \
cur=$(cat $p/scaling_cur_freq) max=$(cat $p/scaling_max_freq)"
done

# thermal の cooling device と governor
for c in /sys/class/thermal/cooling_device*; do
  echo "$c type=$(cat $c/type) cur=$(cat $c/cur_state) max=$(cat $c/max_state)"
done
cat /sys/class/thermal/thermal_zone0/policy
cat /sys/class/thermal/thermal_zone0/sustainable_power

# 温度
for z in /sys/class/thermal/thermal_zone*; do
  echo "$(cat $z/type): $(awk '{printf "%.1f C", $1/1000}' $z/temp)"
done

# RT / USB
cat /proc/sys/kernel/sched_rt_runtime_us
ulimit -r
cat /sys/module/usbcore/parameters/autosuspend
```
