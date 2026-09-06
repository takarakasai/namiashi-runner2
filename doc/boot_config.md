# 起動時間レポート / 最適化記録

対象ホスト: `radxa-cubie-a7z`
Debian GNU/Linux 11 (bullseye) / Linux 5.15.147-21-a733 / aarch64 / 8 core / RAM 7.7 GiB
ストレージ: SoC 直付け UFS (`4520000.ufs`, BWU2ASV46A256G 238.5G)

計測日: 2026-08-20

制御ループの決定性・実行時チューニングは [`runtime_tuning.md`](runtime_tuning.md) を参照。

## 結果サマリ

| 項目 | 初回 | 案A | +B | +C | +D | +E,F | +G | 総差分 |
|---|---|---|---|---|---|---|---|---|
| **合計起動時間** | **23.177 s** | 7.233 s | 6.400 s | 4.135 s | 3.936 s | 3.789 s | **3.489 s** | **-19.688 s (-84.9%)** |
| カーネル | 5.596 s | 5.090 s | 3.947 s | **1.999 s** | 1.727 s | 1.744 s | **1.494 s** | -4.102 s (-73.3%) |
| ユーザースペース | 17.580 s | 2.142 s | 2.452 s | 2.136 s | 2.208 s | 2.044 s | **1.994 s** | -15.586 s (-88.7%) |
| ログインプロンプト (`getty@tty1`) | — | — | — | — | 2.134 s | **1.931 s** | — | — |
| `hdmi-toggle-once.service` | 15.022 s | 0.033 s | 0.036 s | 0.036 s | **無効化** | 無効化 | 無効化 | -15.022 s |
| initrd サイズ | 31.1 MiB | 31.1 MiB | **12.8 MiB** | 12.8 MiB | 12.8 MiB | 12.8 MiB | **6.7 MiB** | -78.5% |
| initrd 展開時間 | 0.746 s | 0.746 s | **0.371 s** | 0.371 s | 0.371 s | 0.371 s | — | -50.3% |
| メモリ使用量 | 2.2 GiB | — | — | 2.2 GiB | **1.0 GiB** | 1.0 GiB | — | **-1.2 GiB** |
| 起動失敗ユニット | 0 件 | 0 件 | 0 件 | 0 件 | 0 件 | 0 件 | 0 件 | — |

> 案G の initrd 展開時間とメモリ使用量が「—」なのは未計測のため。`dmesg` が
> `dmesg_restrict` で読めず、メモリは VSCode Remote 接続中で比較にならなかった。

- **案A** = `hdmi-toggle-once.service` の固定 sleep 削除
- **案B** = initramfs `MODULES=most` → `dep`
- **案C** = PCIe 無効化 + `quiet splash` 除去 + `usbcore.autosuspend=-1`
- **案D** = `multi-user.target` 化 + plymouth mask（デスクトップ廃止）
- **案E** = `systemd-user-sessions` から `network.target` 依存を除去
- **案F** = 不要サービスの無効化（効果は不確定。後述）
- **案G** = 常駐デーモンの棚卸し + `plymouth` purge（2026-08-21）。
  詳細は [`runtime_tuning.md`](runtime_tuning.md) 「調査3」。
  **起動時間短縮を狙った施策ではなく、運転中の常駐プロセスを削る作業の副産物**として
  カーネル時間が -250 ms 縮んだ（initrd 12.8 → 6.7 MiB の展開時間分）

> **計測のばらつきについて。** `systemd-journal-flush` はブート毎に 226〜762 ms の範囲で
> 変動する。案E・案F の効果（合計で約 -150 ms）はこのばらつきと同程度であり、
> **1 回の計測では因果を断定できない**。案A〜Dのような桁違いの改善とは性質が異なる。

> **この表は `systemd-analyze` の値なので、U-Boot の時間は入っていない。**
> 電源投入からの実時間は、これに **U-Boot メニューの待ち 1.0 秒**（`timeout 10`、
> デシ秒単位）と U-Boot 自体の初期化時間（未測定）が乗る。
> 「残る改善余地」の U-Boot の節を参照。

- **案A** = `hdmi-toggle-once.service` の `ExecStartPre=/bin/sleep 15` 削除 → ユーザースペースを削減
- **案B** = initramfs の `MODULES=most` → `dep` → カーネル時間を削減

ユーザースペースが案A後 2.142 s → 案A+B後 2.452 s と 0.31 s 増えているが、これは案Bとは無関係の
ブート毎のばらつき（`upower.service` 513ms→1.067s、`accounts-daemon` 1.118s→1.358s 等が変動）。
いずれも並列実行される非クリティカルパス上のサービス。

---

## 案A: `hdmi-toggle-once.service` の固定 sleep 削除【適用済み】

### 原因

HDMI 解像度を 1280x1024 → 1920x1080 に切り替えるワンショット処理のユニットに、固定 15 秒の
待機が入っていた:

```ini
# /lib/systemd/system/hdmi-toggle-once.service
[Service]
Type=oneshot
ExecStartPre=/bin/sleep 15          # X/SDDM の準備待ち
ExecStart=/usr/bin/hdmi-toggle-once
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
```

`Type=oneshot` + `WantedBy=multi-user.target` のため、`multi-user.target` → `graphical.target`
の到達がこの 15 秒間まるごとブロックされていた。

**この sleep は冗長だった。** `/usr/bin/hdmi-toggle-once` は既に自前でポーリングしている:

- step 2: SDDM の X:0 ソケットと xauth cookie を最大 20 秒待つ（`xrandr --query` の成功を実確認）
- step 3: HDMI が `connected` になるまで最大 10 秒待つ

さらに `mode set` チェックは step 1、つまりこれらの待機ループより**前**にある。`mode set = yes`
なら数ミリ秒で `exit 0` する。すなわち sleep 15 は「何もしないと確定した後に 15 秒待つ」という
完全な無駄になっていた。

### 適用内容

`/lib/systemd/system/` を直接編集するとパッケージ更新で上書きされるため drop-in で対応:

```ini
# /etc/systemd/system/hdmi-toggle-once.service.d/override.conf
# Drop-in: remove the hard-coded 15s pre-start sleep.
# The script /usr/bin/hdmi-toggle-once already polls for the SDDM X:0
# socket + xauth cookie (up to 20s) and for HDMI "connected" (up to 10s),
# so ExecStartPre=/bin/sleep 15 was redundant and blocked multi-user.target.
[Service]
ExecStartPre=
TimeoutStartSec=45
```

- `ExecStartPre=` の空代入が既存の `sleep 15` をクリアする
- `TimeoutStartSec=45` を併せて設定。変更前は `TimeoutStartUSec=infinity` で上限が無く、
  スクリプト内ポーリング（最大 20+10 秒）が詰まった場合にブートが無限に待つ状態だった

ロールバック:

```sh
sudo rm -rf /etc/systemd/system/hdmi-toggle-once.service.d && sudo systemctl daemon-reload
```

### 検証結果

| 検証 | 結果 |
|---|---|
| `systemd-analyze verify` | exit=0（警告は無関係な既存の plymouth `KillMode=none` のみ） |
| 手動 `systemctl restart` 実測 | 15.022 s → **0.067 s** |
| 実ブート後の blame | **33〜36 ms** |
| ユニット状態 | `Result=success` / `ExecMainStatus=0` / `active (exited)` |
| HDMI 出力 | `HDMI-1 connected primary 1920x1080+0+0`、`60.00*+` |
| `hdmi_source` attr | `enable=yes` / `mode set=yes` / `mode info=1920x1080` / `force=off` |

---

## 案B: initramfs の `MODULES=dep`【適用済み】

### 判断材料

initrd 展開時間をカーネルログで実測してから着手した:

```
[    0.185735] Trying to unpack rootfs image as initramfs...
[    0.932101] Freeing initrd memory: 31828K
```

**展開に 0.746 秒**（31 MB gzip → 78 MB）。これがこの方面から取れる上限だった。

### ストレージ構成の確認（リスク評価の根拠）

一般に `MODULES=dep` は「生成後にディスクの接続先を変えると必要なドライバが initrd に無く
`VFS: Unable to mount root fs on unknown-block(0,0)` で panic する」リスクがある
（USB 3.0 `xhci_hcd` → USB 2.0 `ehci_hcd` への挿し替え、USB → SATA 変換アダプタへの変更、
SDカード `mmc_block` から USB SSD への移行、`usb_storage` から `uas` への切り替わり など）。

**本機ではこのシナリオが発生しない。** ルートは USB でも SATA でもなく SoC 直付けの UFS:

```
/sys/block/sda → /sys/devices/platform/soc@3000000/4520000.ufs/host0/target0:0:0/0:0:0:0/block/sda
/dev/disk/by-path/platform-4520000.ufs-scsi-0:0:0:0 -> ../../sda
MODEL: BWU2ASV46A256G (238.5G)
```

`sd` ドライバ経由で SCSI として見えるため `lsblk` 上は通常のディスクに見えるが、**差し替える
ポートが物理的に存在しない**。`sdb` / `sdc`（各 4MB）は同一 UFS チップの boot LUN。

さらに、ルート到達に必要なドライバは全てカーネルビルトイン:

```
$ grep -iE 'ufs|ext4' /lib/modules/5.15.147-21-a733/modules.builtin
kernel/fs/ext4/ext4.ko
kernel/drivers/scsi/ufs/ufshcd-core.ko
kernel/drivers/scsi/ufs/ufshcd-pltfrm.ko
kernel/bsp/drivers/ufs/sunxi-ufs-platform.ko
```

`.ko` ファイルとしては存在せず（`find /lib/modules/... -name '*ufs*'` はヒットなし）、
カーネル本体に組み込み済み。**ルートのマウントに initrd 内のモジュールは 1 つも必要ない。**

### 適用内容

```sh
sudo cp -a /boot/initrd.img-5.15.147-21-a733 /boot/initrd.img-5.15.147-21-a733.gzip.bak
sudo sed -i 's/^MODULES=most/MODULES=dep/' /etc/initramfs-tools/initramfs.conf
sudo update-initramfs -u
```

`COMPRESS` は `gzip` のまま（後述の理由により変更していない）。

ロールバック:

```sh
sudo cp -a /boot/initrd.img-5.15.147-21-a733.gzip.bak /boot/initrd.img-5.15.147-21-a733
sudo sed -i 's/^MODULES=dep/MODULES=most/' /etc/initramfs-tools/initramfs.conf
```

### 生成物の差分

| | 変更前 (`most`) | 変更後 (`dep`) |
|---|---|---|
| initrd サイズ | 32,594,122 B (31.1 MiB) | **13,399,270 B (12.8 MiB)** |
| 非圧縮サイズ | 77,862,912 B (74.3 MiB) | **29,554,688 B (28.2 MiB)** |
| 総エントリ数 | 1,504 | 427 |
| モジュール数 | 128 | **3** |

**削除された 1,077 エントリの内訳:**

| 分類 | 件数 | 内容 |
|---|---|---|
| `usr/lib/firmware` | 898 | **amdgpu**（bonaire / carrizo / fiji 等）の GPU ファームウェア |
| `usr/lib/modules` | 66 | モジュールとディレクトリ |
| `var/cache/fontconfig` | 6 | フォントキャッシュ |

Allwinner の aarch64 SBC に AMD GPU ファームウェアが約 900 ファイル入っていた。
`MODULES=most` が積んでいた無駄がそのまま出ている。

**残ったモジュール（3個）:**

```
usr/lib/modules/5.15.147-21-a733/kernel/drivers/md/dm-mod.ko.xz
usr/lib/modules/5.15.147-21-a733/kernel/fs/fuse/fuse.ko.xz
usr/lib/modules/5.15.147-21-a733/updates/dkms/pvrsrvkm.ko.xz
```

> **追記 (2026-08-21):** `pvrsrvkm.ko.xz`（PowerVR GPU）が initrd に居たのは
> **plymouth の initramfs hook がスプラッシュ描画用に DRM ドライバを引き込んでいた**ため。
> plymouth を purge した結果このモジュールも消え、initrd は 427 → 208 エントリ /
> 13.4 MB → 7.0 MB になった。root は `sda3` の ext4（LVM 無し）で GPU は起動に
> 関与しないため影響なし。[`runtime_tuning.md`](runtime_tuning.md) 調査3 を参照。

**必須コンポーネントは全て残存（再起動前に確認済み）:**

```
usr/bin/sh                              シェル
usr/sbin/blkid + libblkid.so.1          root=UUID= の解決
usr/sbin/modprobe                       ✓
scripts/local, scripts/init-*           ✓
usr/bin/nuke, usr/bin/resume            ✓
```

**むしろ増えたもの:**

```
usr/sbin/e2fsck + libext2fs / libe2p / libcom_err
```

`MODULES=dep` はルートが ext4 だと認識するため fsck ツールを同梱する。`most` には
入っていなかったので改善。

なお `update-initramfs` 実行時の
`find: '/var/tmp/mkinitramfs_XXXXXX/lib/modules/5.15.147-21-a733/kernel': No such file or directory`
はモジュールコピー前にフックが走ったことによるもので、最終イメージには当該ディレクトリが
存在する。無害。

### 検証結果

initrd 展開時間（`dmesg`）:

```
変更前: [0.185735] Trying to unpack rootfs image as initramfs...
        [0.932101] Freeing initrd memory: 31828K      → 0.746 s

変更後: [0.185570] Trying to unpack rootfs image as initramfs...
        [0.556269] Freeing initrd memory: 13084K      → 0.371 s
```

**展開時間 -0.375 s**（-50.3%）。非圧縮サイズの削減率 -62% とおおむね比例している。

一方、カーネル時間全体は **5.090 s → 3.947 s（-1.143 s）** と、展開時間の削減分 0.375 s を
0.77 s ほど上回って縮んだ。差分の説明:

`systemd-analyze` の「kernel」時間は、Debian の initramfs-tools が systemd ではなく
シェルスクリプト `/init` を使うため、**initrd 内のユーザースペース処理も含んでいる**
（udev coldplug、modprobe、blkid による root=UUID= 解決、fsck、`switch_root`）。
モジュールが 128 → 3 個、ファームウェアが 898 ファイル削減されたことで、initrd 内の
udev coldplug と modprobe の処理量も大幅に減っている。展開時間の短縮はその一部でしかない。

他の検証項目:

| 検証 | 結果 |
|---|---|
| 起動失敗ユニット | 0 件 |
| HDMI 出力 | `HDMI-1 connected primary 1920x1080+0+0` / `mode set=yes` |
| `hdmi-toggle-once.service` | `Result=success` / 36 ms |
| ルートのマウント | 正常（`root=UUID=` の解決に問題なし） |

---

## 見送った案: `COMPRESS=zstd`

当初 `MODULES=dep` と併せて検討したが**見送った**。経緯を記録する。

### 前提条件は全て満たしていた

| 確認項目 | 結果 |
|---|---|
| `zstd` バイナリ | v1.4.8 (`/usr/bin/zstd`) |
| initramfs-tools | 0.140 — `mkinitramfs:187` に `zstd) compress="zstd -q -19 -T0"` |
| カーネル | `CONFIG_RD_ZSTD=y`（`CONFIG_RD_GZIP=y` も併存） |
| ブートローダ | extlinux が raw initrd を直接参照。uInitrd ラッパーや mkimage 変換なし |

### しかし適用されなかった

`initramfs.conf` を `COMPRESS=zstd` にして `update-initramfs -u` しても、生成物は gzip のままだった。
原因は `conf.d` の読み込み順:

```
/usr/sbin/mkinitramfs:91   . "${CONFDIR}/initramfs.conf"       ← 先に読む
/usr/sbin/mkinitramfs:113  for i in "${CONFDIR}"/conf.d/*; do  ← 後で読む＝こちらが勝つ
```

```
$ cat /etc/initramfs-tools/conf.d/compress-as-gzip
COMPRESS=gzip

$ dpkg -S /etc/initramfs-tools/conf.d/compress-as-gzip
radxa-system-config-common: /etc/initramfs-tools/conf.d/compress-as-gzip
```

**Radxa のボードサポートパッケージが意図的に gzip を強制していた。** ファイル名も
`compress-as-gzip` と明示的で偶然ではない。

### 見送りの判断

- 期待できる削減は **0.5 秒前後**（zstd の展開は gzip の 3〜5 倍速だが、元が 0.746 秒）
- ベンダーが gzip を強制する理由はシステム上から判断できなかった。フラッシュツールや
  復旧経路が gzip 前提である可能性を否定できない
- 同じ 0.746 秒を狙うなら `MODULES=dep` の方が有利:
  展開時間は非圧縮サイズに比例するため、圧縮方式を変えるより展開する中身を減らす方が効く。
  さらに U-Boot が UFS から initrd を読む時間（`systemd-analyze` には現れない）も短縮される

**理由の分からないベンダー設定を 0.5 秒のために上書きするのは割に合わない**と判断した。

### 将来やる場合

ベンダーファイルは触らず、glob 順で後に来る drop-in を置けば勝てる:

```sh
echo 'COMPRESS=zstd' | sudo tee /etc/initramfs-tools/conf.d/zz-compress-zstd
sudo update-initramfs -u
```

---

## 案C: PCIe 無効化ほか【適用済み】

カーネル時間 3.947 s の内訳を `dmesg` のギャップ解析で調べたところ、**2 箇所に集中**していた。

```
2.367  gap=1.458  [0.908553] sunxi-pcie 6000000.pcie: MEM 0x0022000000..0x0027ffffff
3.040  gap=0.674  [2.366797] [drm] sunxi-hdmi: hdmi drv detect hpd connect
```

合計 2.13 s で、**カーネル時間の 54%**。ユーザースペース全体（2.1 s）より PCIe 単独の方が大きかった。

### PCIe は空きスロットで 1.458 秒待っていた

```
$ lspci
00:00.0 PCI bridge [0604]: Device [1f6d:abcd] (rev 01)     ← ルートブリッジのみ
$ ls /sys/bus/pci/devices/0000:00:00.0/ | grep '^0000:'
（エンドポイントなし）
```

`dmesg` を 0.9〜2.4 秒の範囲で見ると**ログが 1 行も無い**。ホストブリッジがレンジを表示した直後に
probe がブロックし、「link up」のメッセージが最後まで出ないまま次へ進んでいる。
空きスロットでのリンクトレーニング・タイムアウト。

**ストレージは SoC 直付け UFS、Wi-Fi は USB（aic8800）** なので PCIe に依存するものは何も無い。

### Device Tree overlay で無効化

`sunxi-pcie` はカーネルビルトイン（`modules.builtin` に
`kernel/bsp/drivers/pcie/pcie_sunxi_host.ko`）なので blacklist は使えない。DT overlay で対応した。

```dts
/dts-v1/;
/plugin/;

/ {
    metadata {
        title = "Disable PCIe controller";
        compatible = "radxa,cubie-a7z";
        category = "misc";
        exclusive = "pcie@6000000";
        description = "...";
    };

    fragment@0 {
        target-path = "/soc@3000000/pcie@6000000";
        __overlay__ {
            status = "disabled";
        };
    };
};
```

```sh
dtc -q -@ -I dts -O dtb -o /boot/dtbo/pcie-off.dtbo pcie-off.dts
u-boot-update
```

#### overlay の適用機構

`u-boot-update` は `/boot/dtbo/` を **`*.dtbo` で glob** して `extlinux.conf` の
`fdtoverlays` 行に書き出す（`/usr/sbin/u-boot-update:220-236`）。
ベンダー製 overlay が全て `*.dtbo.disabled` という名前なのはこのため。

```
U_BOOT_FDT_OVERLAYS_DIR="${U_BOOT_FDT_OVERLAYS_DIR:-/boot/dtbo}"
# U_BOOT_FDT_OVERLAYS が空なら *.dtbo を全て拾う
```

**無効化はリネームだけ:**

```sh
sudo mv /boot/dtbo/pcie-off.dtbo /boot/dtbo/pcie-off.dtbo.disabled
sudo u-boot-update
```

#### 安全性: rescue エントリには overlay が付かない

```
label l0
	fdtoverlays  /boot/dtbo/pcie-off.dtbo     ← overlay あり
label l0r  (rescue target)
	（fdtoverlays なし）                       ← overlay なし
```

overlay が原因で起動しなくなった場合、**U-Boot メニューで rescue エントリ `l0r` を選ぶだけで
overlay 無しの状態で起動できる**。ファイル操作なしで切り分けられる。

### カーネルコマンドラインの変更

```sh
# /etc/kernel/cmdline から
-  quiet splash
+  usbcore.autosuspend=-1
sudo u-boot-update
```

- `quiet splash` 除去 — plymouth のスプラッシュを止める
- `usbcore.autosuspend=-1` — **CH348（500 Hz 制御の経路）を USB autosuspend させない**。
  詳細は [`runtime_tuning.md`](runtime_tuning.md)

### 結果

| 項目 | 前 | 後 |
|---|---|---|
| 合計 | 6.028 s | **4.135 s** |
| カーネル | 3.928 s | **1.999 s** |
| PCI デバイス数 | 1（ルートブリッジ） | **0** |
| `usbcore.autosuspend` | 2 | **-1** |

**PCIe の 1.458 s に加え、HDMI の 0.674 s も消えた。** probe 順序が変わった副次効果と思われる。
再計測後の `dmesg` ギャップは最大 0.240 s で、突出したストールは無くなった。
**カーネル側からこれ以上大きく削るのは難しい。**

## 案D: `multi-user.target` 化【適用済み】

用途がロボット組み込み（デスクトップ不要、HDMI は CLI ターミナルで十分）であるため、
デスクトップ環境を廃止した。KDE Plasma がフル導入されていた。

```sh
sudo systemctl set-default multi-user.target
sudo systemctl disable hdmi-toggle-once.service
sudo systemctl mask plymouth-start.service plymouth-quit-wait.service                     plymouth-quit.service plymouth-read-write.service
```

### `hdmi-toggle-once` の無効化は必須

```ini
# /lib/systemd/system/hdmi-toggle-once.service
Wants=display-manager.service     ← multi-user でも sddm を引き戻す
```

`WantedBy=multi-user.target` かつ `Wants=display-manager.service` なので、
デフォルトターゲットを変えるだけでは sddm が起動してしまう。

なお xrandr ベースで X 前提のサービスなので、X を使わなくなれば存在意義自体が無くなる。
**後述の awk フィールドずれのバグも、これで無関係になった。**

### `sddm` は enabled のまま残した

`graphical.target` が来ないので起動しない。デスクトップに戻したくなったら
`systemctl set-default graphical.target` だけで復活する。

### 結果

| 項目 | 前 | 後 |
|---|---|---|
| 合計 | 4.135 s | **3.936 s** |
| `udisks2`（クリティカルチェーン上） | +314 ms | チェーンから消滅 |
| plymouth | 124 ms | **blame に現れない** |
| Xorg / sddm | 起動 | 起動せず |
| **メモリ使用量** | **2.2 GiB** | **1.0 GiB** |

起動時間の短縮（-0.2 s）より、**メモリ 1.2 GiB の解放**の方が 500 Hz 制御ループには効く。

### 補足: ブートメッセージが HDMI に出ない件

`quiet splash` を外し plymouth も mask したが、HDMI にブートメッセージは流れなかった。
原因はカーネルコマンドラインの **`loglevel=4`**。

```
$ cat /proc/sys/kernel/printk
4	7	1	7
```

1 列目が console_loglevel で、`4` は「KERN_ERR 以上のみコンソールへ出力」。
通常の info/notice は抑制される。見たい場合は `loglevel=7` にして `u-boot-update`。
エラーだけ見えれば十分なら現状のままでよく、詳細は `journalctl -b` / `dmesg` で読める。

ログインプロンプト自体は `tty1` に CUI で出ており、意図どおり。

## 案E: コンソールログインをネットワーク待ちから解放【適用済み】

### 症状

`tty1` のログインプロンプトが 2.134 s まで出なかった。原因は以下の連鎖。

```
getty@tty1 → systemd-user-sessions → network.target → NetworkManager (1.199s)
```

`systemd-user-sessions.service` の定義:

```ini
After=remote-fs.target nss-user-lookup.target network.target home.mount
```

この `network.target` は NIS/LDAP 等の**リモート認証**を想定したもの。本機はローカル
アカウントのみ。`network.target` は「ネットワーク管理が起動した」の意味で「接続済み」では
ないが、`NetworkManager.service` が `Before=network.target` を持つため、その起動完了
（WiFi アソシエーションを含む 1.199 s）まで待たされていた。

### ハマった点: `After=` は drop-in で削除できない

最初 drop-in で空代入によるリセットを試みたが**効かなかった**。

```ini
# /etc/systemd/system/systemd-user-sessions.service.d/override.conf （効果なし）
[Unit]
After=
After=remote-fs.target nss-user-lookup.target home.mount
```

`systemctl cat` で drop-in が読まれていることは確認できるのに、
`systemctl show -p After` には `network.target` が残ったままだった。

**systemd では `After=` / `Before=` / `Wants=` / `Requires=` などの依存指示子は追記のみで、
空代入によるリセットができない。** `ExecStart=` / `ExecStartPre=` などの `Exec*=` 系は
リセットできるため混同しやすい（案A の drop-in はこの性質を使っている）。

### 対処: ユニットファイル全体のオーバーライド

`/etc/systemd/system/` に完全なコピーを置いてベンダーファイルを隠す。

```ini
# /etc/systemd/system/systemd-user-sessions.service
[Unit]
Description=Permit User Sessions
Documentation=man:systemd-user-sessions.service(8)
After=remote-fs.target nss-user-lookup.target home.mount

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/lib/systemd/systemd-user-sessions start
ExecStop=/lib/systemd/systemd-user-sessions stop
```

`/lib/systemd/system/multi-user.target.wants/systemd-user-sessions.service` の
シンボリックリンクはユニット名で解決されるため、`/etc` 側が使われる
（`FragmentPath` で確認済み）。

**副作用:** systemd パッケージ更新でこのユニットが変更されても反映されない。
6 行の小さなユニットなので許容範囲と判断した。元に戻すには本ファイルを削除して
`daemon-reload`。

### 結果

`systemd-user-sessions` は 1.856 s → 1.718 s となり、**`network.target`（2.043 s）より
前に完了**するようになった。依存の除去は成功。

ただし `getty@tty1` は 2.134 s → 2.049 s と 85 ms しか改善しなかった。
依存関係ではなく**ジョブのスケジューリング遅延**が残っていたため
（依存が 0.913 s で揃っているのに開始は 1.496 s だった）。これが案F の動機になった。

### 見送り: `NetworkManager` の `Before=multi-user.target`

NM は自身の `Before=` に `multi-user.target` を含むため、「起動完了」の判定は NM を待つ。
しかし `multi-user.target` を待っているものを調べると実質何も無い。

```
$ systemctl show multi-user.target -p Before
graphical.target  shutdown.target  systemd-update-utmp-runlevel.service
```

`graphical.target` は使わず、残り 2 つはブート完了の記録処理。つまり外しても
**縮むのは `systemd-analyze` の表示値だけ**。`NetworkManager.service`（36 行）を丸ごと
オーバーライドするとベンダー更新が反映されなくなるため、割に合わないと判断した。

将来 `misa-run.service` を作る場合も `After=basic.target` 等で直接順序を指定すれば
`multi-user.target` の到達時刻とは無関係に起動する。

## 案F: 不要サービスの無効化【適用済み・効果は不確定】

`multi-user.target` 化により `accounts-daemon` / `udisks2` / `upower` は既に起動しなく
なっていた（`graphical.target` が引いていたため）。残っていたものを整理した。

```sh
sudo systemctl disable --now avahi-daemon lm-sensors rtkit-daemon cups                              accounts-daemon udisks2 upower bluetooth
sudo systemctl disable --now cups.path cups.socket
sudo systemctl mask systemd-rfkill.service systemd-rfkill.socket
sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer
```

| サービス | blame（前） | 判断 |
|---|---|---|
| `systemd-rfkill` | **1.702 s** | `static` のため mask。WiFi/BT の soft-block 状態を保存/復元するだけで、ヘッドレス運用では不要。socket 側も塞がないと udev イベントで起こされる |
| `avahi-daemon` | 1.080 s | mDNS 不要（SSH は IP 直打ち） |
| `lm-sensors` | 695 ms | 温度は `/sys/class/thermal/` から直読み、ファンはカーネルの cooling device が制御。このサービスは `sensors -s` を 1 回実行するだけ |
| `rtkit-daemon` | 647 ms | PulseAudio 用。**`disable` では止まらなかった**（後述の訂正を参照） |
| `cups` / `bluetooth` | — | 印刷・BT とも用途なし（操縦はプロポ + S.BUS、開発は SSH） |
| `apt-daily*` タイマー | — | 現場で勝手に apt が走るのを防ぐ |

### 結果: 効果を証明できなかった

| 項目 | 前 | 後 |
|---|---|---|
| 合計 | 3.897 s | 3.789 s（-108 ms） |
| `getty@tty1` | 2.049 s | 1.931 s（-118 ms） |
| blame 上位のサービス | rfkill / avahi / lm-sensors / rtkit | **全て消滅** |

サービスが消えたことは確実。しかし**同じブートで他の項目が逆に伸びている**。

| 項目 | 前 | 後 |
|---|---|---|
| `systemd-journal-flush` | 264 ms | 334 ms |
| `systemd-tmpfiles-setup` | 54 ms | 148 ms |
| `systemd-timesyncd` | 162 ms | 231 ms |
| `basic.target` 到達 | 0.913 s | 1.399 s |

`journal-flush` はこれまでの計測でも 226〜762 ms の範囲で変動しており、
**-108 ms という改善幅はブート毎のばらつきと同程度**。競合緩和が効いたのか偶然かは
1 回の計測では判定できない。

**起動時間短縮の施策としては効果不明。** ただしロボット組み込み用途では
常駐プロセスと CPU 競合が減る意味があり、500 Hz 制御ループには有利に働く。
この理由だけでも維持する価値はある。

ロールバック:

```sh
sudo systemctl unmask systemd-rfkill.service systemd-rfkill.socket
sudo systemctl enable --now avahi-daemon lm-sensors rtkit-daemon cups bluetooth
sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer
```

### 訂正 (2026-08-21): `rtkit-daemon` は `disable` では止まっていなかった

上表の `rtkit-daemon` は `disable` したが、その後の棚卸しで **`is-enabled` = disabled /
`is-active` = active** という状態で生き残っていたことが判明した。ユーザセッションの
`pulseaudio.service` からの **D-Bus activation** で起動するため、`disable` では止まらない。

**`disable` は「target から引かれるのを止める」だけで、D-Bus / socket 活性化は止まらない。**
この種のサービスは `mask` が要る。同じ理由で `packagekit` も後から上がっていた。
対処は [`runtime_tuning.md`](runtime_tuning.md) 「調査3: 常駐デーモンの棚卸し」。

## 起動時間短縮の打ち止め判断

案F 適用後のクリティカルチェーン:

```
multi-user.target @2.023s
└─ssh.service @1.940s +81ms
  └─network.target @1.929s
    └─wpa_supplicant.service @1.874s +53ms
      └─dbus.service @1.524s
        └─basic.target @1.399s
          └─sysinit.target @1.379s
            └─systemd-timesyncd.service @1.099s +231ms
              └─systemd-tmpfiles-setup.service @779ms +148ms
                └─systemd-journal-flush.service @440ms +334ms
                  └─systemd-journald.service @342ms +93ms
```

- **カーネル 1.744 s** — `dmesg` のギャップは最大 0.240 s まで分散済み。単一の削減対象は無い
- **`dev-sda3.device` 1.539 s**（blame 最大）— UFS の列挙待ち。ソフト側で短縮不可
- 残るユーザースペースの項目は journal-flush / timesyncd / tmpfiles-setup で、
  いずれも数百 ms かつ**ブート毎のばらつきが同程度**

**これ以上は削減幅に対してリスクと手間が見合わない。ここを打ち止めとする。**

> **追記 (2026-08-21):** この判断自体は妥当だったが、その後**別の目的**（運転中の常駐
> プロセス削減）で `plymouth` を purge した結果、initrd が 12.8 → 6.7 MiB になり
> **カーネル時間が -250 ms、合計 3.789 → 3.489 s になった**。
> 「カーネル 1.744 s に単一の削減対象は無い」は、initrd の中身に plymouth の
> テーマ・フォント・DRM ドライバが残っていた点を見落としていた。案G を参照。
>
> さらに 2026-08-21 の調査で、**U-Boot メニューが 1.0 秒待っている**ことが
> 分かった（「残る改善余地」の U-Boot の節）。
>
> **打ち止め判断の穴は 2 件とも「`systemd-analyze` が測っていない領域」だった。**
> この指標だけを見ていると、initrd の中身と U-Boot の時間は最後まで見えない。
> 次に起動時間を詰めるときは、まず**測定範囲の外**を疑うこと。

## 最適化後の内訳（案A+B 適用後）

### blame 上位

```
1.673s systemd-rfkill.service
1.643s dev-sda3.device
1.358s accounts-daemon.service
1.174s avahi-daemon.service
1.107s lm-sensors.service
1.067s upower.service
1.053s udisks2.service
 980ms systemd-resolved.service
 892ms NetworkManager.service
 849ms systemd-logind.service
  36ms hdmi-toggle-once.service
```

### クリティカルチェーン

```
graphical.target @2.424s
└─upower.service @1.356s +1.067s
  └─basic.target @847ms
    └─sockets.target @847ms
      └─dbus.socket @846ms
        └─sysinit.target @838ms
          └─systemd-timesyncd.service @682ms +154ms
            └─systemd-tmpfiles-setup.service @619ms +47ms
              └─systemd-journal-flush.service @453ms +162ms
                └─systemd-journald.service @364ms +86ms
                  └─systemd-journald.socket @341ms
                    └─-.mount @148ms
```

`hdmi-toggle-once.service` はクリティカルチェーンから完全に消え、最長の枝は `upower.service`
(1.067 s) になった。

> **注意:** blame 上位の多く（`systemd-rfkill`, `dev-sda3.device`, `accounts-daemon`,
> `avahi-daemon` 等）はクリティカルチェーンに乗っておらず並列実行される。
> これらを止めても総時間はほとんど変わらない。

---

## 残る改善余地

ユーザースペースは 2.4 秒まで削れており、**残るボトルネックはカーネル側の 3.947 秒**。

| 案 | 見込み | リスク | 備考 |
|---|---|---|---|
| `COMPRESS=zstd` | -0.2 s 程度 | 中 | 展開が 0.371 s まで縮んだため取り分も縮小。ベンダー設定との衝突あり |
| `avahi-daemon` 無効化 | ほぼ 0 | 低 | mDNS 不要なら。並列実行のため wall clock は変わらない |
| `lm-sensors` 無効化 | ほぼ 0 | 低 | 同上 |
| `dev-sda3.device` (1.643 s) | — | — | UFS 列挙待ち。ソフト側で短縮不可 |

案C（不要サービスの整理）はいずれも並列実行のため、止めても総時間はほぼ変わらない。
CPU 競合が減る程度。

### U-Boot メニューの待ち時間は 1.0 秒【調査済み・保留】

**この 1 秒は `systemd-analyze` の数字に含まれていない。** カーネル以降しか
測っていないため。

```
/boot/extlinux/extlinux.conf:
  prompt 1
  timeout 10        ← 1/10 秒単位。= 1.0 秒
```

`timeout` の単位は**デシ秒**。`man u-boot-update` に明記されている:

> `U_BOOT_TIMEOUT="50"` — Values are in **decisecond** greater than 0
> (e.g. '10' for a 1 second timeout), **0 specifies to wait forever**. The default is 50.

**`timeout 0` は「即起動」ではなく「無限に待つ」。** ヘッドレスのロボットで
踏むと起動しなくなる。直感と逆なので注意。

#### 設定の出所

`/etc/default/u-boot` は全行コメントアウトのままで、実際に効いているのは
**Radxa がパッケージで置いているフラグメント**:

```sh
# /usr/share/u-boot-menu/conf.d/radxa.conf
U_BOOT_PROMPT=1
U_BOOT_TIMEOUT=10
```

u-boot-menu の既定は 50（5 秒）なので、Radxa が既に 1/5 に縮めてある。

`u-boot-update` は設定をこの順に読む（`/usr/sbin/u-boot-update:45-58`）:

```
/etc/default/u-boot
  → /usr/share/u-boot-menu/conf.d/*.conf     ← radxa.conf はここ
    → /etc/u-boot-menu/conf.d/*.conf         ← 後勝ち。上書きするならここ
```

**`/usr/share/` 側を直接編集してはいけない。** パッケージ所有なので更新で
戻る。恒久的に変えるなら `/etc/u-boot-menu/conf.d/*.conf` に置く
（現時点でこのディレクトリは存在しない）。

#### 保留の理由

`timeout 1` にすれば 0.1 秒で、**-0.9 秒**。残っているユーザースペースの
削り代（journal-flush / timesyncd の数百 ms、しかもブレ幅同程度）より
はるかに大きい単一項目である。

にもかかわらず保留にしたのは、この 1 秒が「緊急時の起動手段」の要だから。
rescue エントリ `l0r` を選ぶ唯一の手段で、initrd や overlay で起動しなく
なったときの復帰経路になっている。0.1 秒では事実上押せない。

電源投入時からキーを押しっぱなしにすれば拾える可能性はあるが、**U-Boot が
メニュー開始前に入力バッファを捨てるかは本ボードで未確認。** 縮めるなら
先に「実際に rescue に入れること」を試すこと。

判断の順番としては、機体の組み立てと立ち上げが終わって構成が固まってから、
0.3 秒（`timeout 3`）あたりで rescue に入れるかを実測して決める。
これから initrd や overlay をいじる可能性がある段階で復帰経路を細くするのは
割に合わない。

#### 未測定: U-Boot 自体の初期化時間

メニューの待ちは U-Boot 時間の一部でしかない。SPL / DRAM 初期化 / UFS 列挙 /
initrd 読み出しは**どれも測っていない**（`systemd-analyze` には現れず、
Linux 側からは観測できない）。

```
3.489 s   systemd-analyze（カーネル + ユーザースペース）
+ 1.0 s   U-Boot メニューの待ち
+ α       U-Boot 自体の初期化        ← 未測定
```

α がメニューの 1 秒より大きければ、削る優先順位が変わる。
**次にシリアルコンソール（`115200n8`）を繋いで再起動するとき、U-Boot の
出力タイムスタンプを採ること。**

### CH348 の tty 生成が 1.764 秒待たされている【調査済み・保留】

**本来の指標は「電源投入 → 制御ループが S.BUS フレームを処理し始めるまで」**
であって `multi-user.target` ではない。そこで測り直したところ、
`systemd-analyze` だけを見ていては見えない支配項が出てきた。

```
電源投入
  ├─ U-Boot（メニュー待ち 1.0 s + 初期化 α 未測定）
  ├─ カーネル → ユーザースペース            1.495 s
  ├─ CH348 の tty が生える                  3.398 s  (+1.903 s)  ← 支配項
  ├─ multi-user.target                      3.489 s  (+0.091 s)
  └─ misa-run 起動 → 初フレーム処理                 (+0.145 s)  実測
                                           ≈ 3.63 s + U-Boot
```

`misa-run` 自身の起動（設定 + モデル読み込み + ポート open + スレッド開始 +
初フレーム）は **0.145 秒**。ここは既に小さく、削る対象ではない。

#### 1.764 秒の正体は USB 列挙ではなくドライバのロード待ち

```
[0.655]  xhci 登録
[1.479]  usb 1-1 検出                    (+0.824 s)
[1.627]  CH348 列挙完了 (1a86:55d9)      (+0.148 s)   ← ここまでは速い
[3.392]  ch9344 が bind → tty 生成       (+1.764 s)   ← 支配項
```

**デバイスは 1.627 秒に列挙し終わっていて、1.764 秒間ドライバを待っているだけ。**

原因は udev の順番待ち。根拠:

- `ch9344.ko.xz` は **16.9 KB**。ファイル読み出しで 1.76 秒かかるはずがない
- ドライバ初期化自体は約 100 µs（`3.391621` で attach、`3.391724` で driver 登録）
- 同じ窓で **AIC の WiFi ファームウェア 14 MB**（`/lib/firmware/aic8800_fw`）が
  ロードされている。`aic_load_fw` が 2.823 s に登録、WiFi チップが 3.647 s に
  再列挙、`aic_btusb` が 3.845 s
- `ch9344` 専用の udev ルールは無く、`modalias`
  （`usb:v1A86p55D9d*...`）経由の遅延ロード

つまり **14 MB の WiFi ファームウェア転送の後ろに、16 KB のシリアルドライバが
並んでいる**という構図。

#### 手は 2 つあり、片方だけでは効かない

**① `ch9344` を先読みする。** `systemd-modules-load.service` は **1.811 s** に
走っている（所要 28 ms）。ここでロードすれば列挙済みのデバイスに即 bind して
tty は約 **1.84 s**。`/etc/initramfs-tools/modules` に入れて initrd に積めば
さらに早く、列挙完了と同時の約 **1.63 s**。

```sh
# 案1: /etc/modules-load.d/ch9344.conf に "ch9344" の 1 行（initrd の再生成が不要）
# 案2: /etc/initramfs-tools/modules に "ch9344" → update-initramfs -u（さらに 0.2 s 速い）
```

**② `misa-run.service` の起動契機を変える。** 現在のテンプレートは
`WantedBy=multi-user.target` なので **3.489 s まで待つ**。
①だけやっても tty が早く出るだけで misa-run は待たされ、**効果はゼロ**。
device ユニットに引かせれば tty 生成と同時に起動できる。

```
現状        3.398 s (tty) → 3.489 s (multi-user) → misa-run
① だけ      1.84 s  (tty) → 3.489 s (multi-user) → misa-run    効果なし
① + ②       1.84 s  (tty) → misa-run                           -1.65 s
```

> **訂正。** 調査の途中で「`misa-run.service` の起動契機を変えても 91 ms 程度」と
> 見積もったが、これは誤り。tty が 3.398 s に出る**現状での**差でしかない。
> ①で tty が早く出るようになると律速が `multi-user.target` 側へ移るため、
> ②の取り分が 1.6 秒に化ける。**①②は片方ずつ評価すると両方とも「効果が薄い」
> と見えてしまう。**

#### 副次的な項目

- **`aic_btusb`（Bluetooth ドライバ）が 3.845 s にロードされている。**
  `bluetooth.service` は案F で無効化済みだが、**ドライバとファームウェアは
  毎回ロードされている**。使わないならブラックリストすれば udev の混雑が減る。
  WiFi は SSH に要るので触れない
- **`/etc/modules-load.d/` にゴミがある。** 5 つとも実際にはロードされていない
  （このカーネルに存在しない）。毎回失敗する読み込みを試しているだけ:

  ```
  /etc/modules-load.d/cups-filters.conf:  lp ppdev parport_pc   ← パラレルポート
  /etc/modules-load.d/modules.conf:       rockchip-cpufreq      ← Allwinner なのに Rockchip
  ```

  `cups` が入ったままなのが原因。時間としては小さい

#### 保留の理由

**②は `misa-run.service` の構造変更**であり、2026-08-21 時点で機体は組み立て中・
サービスは未インストール。①は単独では効果が無く、②とセットで初めて意味を持つ。

したがって **`misa-run.service` を投入するときに①②を一緒に設計する**。
そのとき `BindsTo=` / `After=` に加えて `WantedBy=` をどうするかが論点になる。

見込みは①②で **-1.65 s**、カーネル基準 3.63 s → 約 2.0 s。
U-Boot の -0.9 s を足せば約 1.1 s まで届く計算になる。

---

## 既知の問題（未修正）

### `/usr/bin/hdmi-toggle-once` の awk フィールド番号ずれ

ガード条件が意図と異なるフィールドを読んでいる。

```sh
# コメント: "Parse the 8th column ... | enable | mode set | mode info | force |"
MODESET="$(awk -F'|' '/\| *state *\|/ {gsub(/^ +| +$/,"",$8); print $8; exit}' "$ATTR_NODE")"
```

`/sys/class/hdmi/hdmi/attr/hdmi_source` の実際のフィールド対応:

```
$8  = enable      ← スクリプトが読んでいる
$9  = mode set    ← 読むべきフィールド
$10 = mode info
$11 = force
```

`$8` は `mode set` ではなく **`enable`**。現在は両方 `yes` のため判定結果が偶然一致しているが、
このスクリプトが本来対処すべき「`enable = yes` かつ `mode set = no`（有効なのにモード未設定＝
画面が出ない）」というまさにその状況で `[ yes = no ]` が false になりスキップされる。
**ワークアラウンドが必要な時に発動しない。**

修正は `$8` → `$9` の 1 文字だが、`/usr/bin/hdmi-toggle-once` の直接編集になり drop-in で
回避できないため、パッケージ更新で上書きされうる。現在 HDMI は正常動作中で緊急性は無い。
修正すると、これまで発動していなかったモード切り替えが将来のブートで発動しうる（挙動が変わる）。

### RTC の時刻保持

`sunxi-rtc` (`7090000.rtc`, `/dev/rtc0`) は存在し、暖機再起動では時刻を保持する
（再起動後 `who -b` は正しく `2026-08-20 17:26` を示した）。

一方、初回計測時（16:39:39 のブート）は `who -b` / `wtmp` が **1970-01-01 00:00** を記録していた。
これは電源断を伴うコールドブート後、RTC がバックアップ電源を持たず時刻を失った状態で
起動したことを示す。その後 `systemd-timesyncd` により補正される
（`timedatectl`: `System clock synchronized: yes` / `NTP service: active`）。

影響: コールドブート直後の数秒間、ログのタイムスタンプ・`last` の履歴・証明書検証がずれる。
起動時刻は `btime` ベース（`uptime -s`）の値を正とみなすこと。

---

## 運用メモ

### CH348 ドライバ導入予定について

`MODULES=dep` は CH348（WCH の USB 8ポート UART）の運用に影響しない。

1. `MODULES=dep` が決めるのは **initrd に何を入れるか**だけ。運用時のモジュールはルート
   マウント後に udev が `/lib/modules/` から読み込む。CH348 は USB シリアルデバイスで
   ルートのマウントに関与しない
2. ルートは直付け UFS + 全ドライバビルトインなので、initrd に CH348 が入る余地は元々ない
3. ブートコンソールは `console=ttyAS0,115200n8`（SoC 内蔵 UART）と `console=tty1`。
   CH348 経由のポートを早期ブートで使う設定にはなっていない

導入時に注意すべきは `MODULES` 設定ではなく以下:

- **`dkms` が未インストール**（`dkms: command not found`）。DKMS 形式で入れるなら
  `sudo apt install dkms` が先に必要。手動 `make install` ならカーネル更新のたびに再ビルド
- ドライバ配置後の `sudo depmod -a`（DKMS なら自動）

万一 initrd に入れたくなった場合、`/etc/initramfs-tools/modules`（現在空）に書けば
`MODULES` の設定に関わらず強制同梱される:

```sh
echo ch348 | sudo tee -a /etc/initramfs-tools/modules
sudo update-initramfs -u
```

### 緊急時の起動手段

シリアルコンソール `115200n8`、U-Boot メニューは `prompt 1` / `timeout 10`
（**デシ秒単位なので 1.0 秒**）で停止可能。
extlinux には通常エントリ `l0` と rescue エントリ `l0r`（`single`）がある。

**この 1 秒を縮めるとここが細くなる。** 起動時間の観点では削りたい項目だが、
復帰経路とのトレードオフになる。詳細は「残る改善余地」の U-Boot の節。

1. バックアップから戻す:
   `cp -a /boot/initrd.img-5.15.147-21-a733.gzip.bak /boot/initrd.img-5.15.147-21-a733`
2. U-Boot メニューで rescue エントリ `l0r` を選択
3. 最終手段: initrd なしで直接起動。ext4/UFS ドライバは全てビルトインなので initrd は本来不要。
   ただし `root=UUID=` は initrd が必要なため、デバイス名指定に変える: `root=/dev/sda3`

---

## 運用: ロボット組み込み用途への最適化

2026-08-20 に用途が判明したため、方針を見直した。

**用途:** ロボットに組み込んで制御に使用。デスクトップ環境は不要で、デバッグ用に HDMI で
CLI ターミナルが表示されれば足りる。SSH でのアクセスあり（IP 直打ち、`.local` 名は使わない）。
CH348（WCH の USB 8ポート UART）ドライバを今後導入予定。

現状インストールされているのは KDE Plasma のフルデスクトップ
（`plasma-desktop`, `task-kde-desktop`, `sddm`, `xserver-xorg-core`, `firefox-esr`）で、
用途に対して過剰。

### 優先度1: SSH が起動直後に繋がらない問題【対応済み】

#### 症状

電源投入後、コンソールでユーザーがログインするまで WiFi に接続されず、SSH できない。
ロボット組み込みでは致命的。

#### 原因

WiFi プロファイルがユーザー限定接続になっていた。

```
$ nmcli -f connection.permissions connection show koya24
connection.permissions:  user:takara

$ grep permissions /etc/NetworkManager/system-connections/koya24-*.nmconnection
permissions=user:takara:;
```

NetworkManager はこの設定の接続を**そのユーザーのセッションが存在する間だけ**有効化する。
GUI で WiFi を設定した際の「すべてのユーザーに使用を許可する」がオフだったのが原因。

#### 対応

```sh
sudo nmcli connection modify koya24 connection.permissions ""
sudo nmcli connection modify koyags connection.permissions ""
```

**併せて孤立ファイルを削除:**
`/etc/NetworkManager/system-connections/koya24.nmconnection`（Apr 13 作成）は NM に
読み込まれていない残骸だった。実際に使われていたのは
`koya24-97fe0f11-bf56-4d35-9a7d-2642e7177e38.nmconnection`。

```sh
sudo rm -f /etc/NetworkManager/system-connections/koya24.nmconnection
sudo nmcli connection reload
```

#### 注意: 1回目の適用は失敗した

孤立ファイルが存在する状態では `id=koya24` が重複しており、`nmcli connection modify koya24`
（id で解決する）がどちらに当たるか不定だった。書き込みが孤立ファイル側に行き、その直後の
`rm` で変更ごと消えた。`koyags` だけ適用され `koya24` が元のままという状態になった。

**教訓:** `nmcli connection modify` を id 指定で使う前に、id の重複が無いことを確認する。
確実を期すなら UUID を指定する:

```sh
sudo nmcli connection modify 97fe0f11-bf56-4d35-9a7d-2642e7177e38 connection.permissions ""
```

検証は必ずファイル内容で行う（`nmcli` の表示だけでは足りない）:

```sh
sudo grep -H -E '^\[|^id=|^permissions=' /etc/NetworkManager/system-connections/*.nmconnection
```

#### 検証結果【成功】

再起動後、**コンソールで一切ログインせずに** SSH 接続成功。

| 確認項目 | 結果 |
|---|---|
| `connection.permissions` | `--`（空） |
| `permissions=` 行 | ファイルから消滅 |
| `wlan0` | `connected` / `192.168.0.21/24` |
| `loginctl list-sessions` | sddm 自身（UID 112, seat0）と SSH 経由の takara のみ。**コンソールログインなし** |
| SSH | 接続可 |

### 温度監視について（`lm-sensors` 無効化の根拠）

読めるセンサーは CPU だけではない。

| ゾーン | 実測値 |
|---|---|
| `cpul_thermal_zone` (little クラスタ) | 41.5 °C |
| `cpub_thermal_zone` (big クラスタ) | 40.7 °C |
| `gpu_thermal_zone` | 40.9 °C |
| `npu_thermal_zone` | 40.7 °C |
| `ddr_thermal_zone` | 41.5 °C |
| `skin_zone`（筐体表面） | 32.2 °C |

**`lm-sensors.service` は無効化して問題ない。** 理由:

1. 温度はカーネルの thermal framework が直接公開している。全て `-virtual-0`（仮想デバイス）で
   `/sys/class/thermal/thermal_zone*/temp` から常時読める。ユーザースペースのサービスは介在しない
2. ファン制御もカーネル側。PWM ファンは cooling device として登録済みで温度に応じて自動制御される:
   ```
   /sys/class/thermal/cooling_device3  type=pwm-fan  cur=4  max=4
   ```
3. `lm-sensors.service` の中身は oneshot で `sensors -s` と `sensors` を1回実行するだけ。
   監視も制御もしていない

ロボット側から温度を読むなら `/sys/class/thermal/thermal_zone*/temp` を直接読むのが確実で、
依存も減る。

### 未実施の計画

#### 優先度2: `multi-user.target` 化

```sh
sudo systemctl set-default multi-user.target
sudo systemctl disable hdmi-toggle-once.service
```

`graphical.target` が引いている `sddm` / `accounts-daemon` / `udisks2` / **`upower`** が
起動しなくなる。`upower` は現在クリティカルチェーンの最長枝（1.067 s）。

**HDMI の CLI ターミナルは維持される。** カーネルコマンドラインに `console=tty1` があり
`getty@tty1.service` も enabled なので、X を止めると素の CLI ログインプロンプトが HDMI に出る。

**罠:** `hdmi-toggle-once.service` は `Wants=display-manager.service` を持つため、
`multi-user.target` に変えても sddm を引っ張り出してしまう。同時に無効化が必要。
このサービスは xrandr ベースで X 前提なので、X を使わなければ無意味
（前述の awk バグ問題も自動的に消滅する）。

見込み: ユーザースペース 2.4 s → 1.2〜1.5 s、総計 5.3 s 前後

#### 優先度3: 不要サービスの無効化

| サービス | blame | 判断 |
|---|---|---|
| `accounts-daemon` | 1.358s | 無効化。ディスプレイマネージャ専用 |
| `upower` | 1.067s | 無効化。**クリティカルチェーン上** |
| `udisks2` | 1.053s | 無効化（USB ストレージ自動マウントが不要なら） |
| `avahi-daemon` | 1.174s | **無効化確定**（SSH は IP 直打ちのため mDNS 不要） |
| `lm-sensors` | 1.107s | **無効化確定**（上記の根拠より） |
| `rtkit-daemon` | 445ms | 無効化。PulseAudio 用 |
| `cups` | 76ms | 無効化。印刷 |
| `bluetooth` | 80ms | 使わないなら無効化。`systemd-rfkill` (1.673s) も軽くなる |
| `apt-daily` / `apt-daily-upgrade` timer | — | **無効化推奨**。現場で勝手に apt が走るのを防ぐ |

~~`plymouth`（スプラッシュ）はカーネルコマンドラインから `quiet splash` を外せば止まる。~~
**この記述は誤り。** 2026-08-21 の棚卸しで、`quiet` も `splash` も無い状態で
`plymouthd` が起動から居座り続けていることが判明した。起動元は systemd ではなく
**initramfs 側の hook** で、カーネルコマンドラインもユニットの mask も関係しない。
詳細と対処は [`runtime_tuning.md`](runtime_tuning.md) 「調査3: 常駐デーモンの棚卸し」。

**デバッグ用途ならブートメッセージが HDMI に出る方が有用。**

#### 優先度4: ロボット運用の信頼性

**ハードウェアウォッチドッグが未使用:**

```
/dev/watchdog, /dev/watchdog0   ← 存在する
#RuntimeWatchdogSec=0           ← 無効
```

制御プロセスごとハングした場合、現状は手動電源断しかない。有効化すれば自動復帰する。

```ini
# /etc/systemd/system.conf
RuntimeWatchdogSec=30s
RebootWatchdogSec=5min
```

**`fake-hwclock` が未インストール:** RTC にバックアップ電源が無いため、ネットワークの無い現場で
コールドブートすると時刻が 1970 から始まる。ログの時系列が壊れ、TLS 証明書の検証も失敗する。

```sh
sudo apt install fake-hwclock
```

シャットダウン時に時刻を保存し起動時に復元するので、少なくとも「前回停止時刻以降」にはなる。

### 要調査

#### CPU 周波数のクランプ【調査完了 → `runtime_tuning.md` へ】

アイドル時に cooling state が上限に張り付き、`scaling_max_freq` が最低周波数まで下がる現象を
確認した。**恒常的な throttle ではなく**（負荷をかけると解放される）、thermal governor
`power_allocator` (IPA) の電力バジェット配分による挙動と判明。

500 Hz 制御ループでのジッタ要因になるため、対処と併せて
[`runtime_tuning.md`](runtime_tuning.md) に移した。

---

## 再現コマンド

```sh
systemd-analyze
systemd-analyze blame
systemd-analyze critical-chain
systemctl --failed
uptime -s; grep btime /proc/stat; timedatectl
systemctl cat hdmi-toggle-once.service
cat /run/hdmi-toggle-once.log
journalctl --list-boots
sudo dmesg | grep -iE 'initramfs|Freeing initrd'    # initrd 展開時間
```

ストレージ構成の確認:

```sh
readlink -f /sys/block/sda                       # 物理パス（接続方式が分かる）
ls -l /dev/disk/by-path/ | grep sd
lsblk -o NAME,TRAN,MODEL,SIZE
grep -iE 'ufs|ext4' /lib/modules/$(uname -r)/modules.builtin   # ビルトイン確認
lsinitramfs /boot/initrd.img-$(uname -r) | grep -c '\.ko'      # initrd モジュール数
```

再起動が実際に起きたかの確認（`systemd-analyze` は前回ブートの記録値を返すため、
これだけでは判別できない）:

```sh
ps -o lstart= -p 1        # PID 1 の起動時刻
grep btime /proc/stat     # 変化していれば再起動済み
journalctl --list-boots   # 新しい boot ID の有無
```
