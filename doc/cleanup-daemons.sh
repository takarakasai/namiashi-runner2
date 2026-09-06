#!/bin/bash
# SBC 常駐デーモン整理 (radxa-cubie-a7z / namiashi 制御機)
# 調査: boot_config.md 案F の続き。SSH / NetworkManager / wpa_supplicant /
# getty 系には一切触れないので、デバッグ経路は維持される。
set -u
log() { printf '\n=== %s ===\n' "$*"; }

log "1. 音声系 (rtkit の復活元を断つ)"
# rtkit は disabled でも pulseaudio からの D-Bus activation で起動するため mask が要る。
systemctl mask --now rtkit-daemon.service
systemctl mask alsa-restore.service alsa-state.service
# takara 以外のユーザでも pulseaudio が上がらないよう system-wide にも mask を置く
mkdir -p /etc/systemd/user
ln -sf /dev/null /etc/systemd/user/pulseaudio.service
ln -sf /dev/null /etc/systemd/user/pulseaudio.socket

log "2. PackageKit (D-Bus activation で勝手に上がる。apt は影響を受けない)"
systemctl mask --now packagekit.service

log "3. 不要タイマー (fstrim は残す: 運転時刻を避ける運用で対処)"
#   fwupd-refresh: 毎日ネットワークからファーム更新メタデータを DL
#   man-db       : 毎日 mandb 全走査
#   e2scrub_all  : LVM 上の ext4 専用。この機体は LVM 無しで完全に無意味
systemctl disable --now fwupd-refresh.timer man-db.timer e2scrub_all.timer

log "4. haveged (poolsize=entropy_avail=256 で常に満杯。供給する余地が無い)"
systemctl disable --now haveged.service

log "5. plymouth (initramfs hook が起動し、quit 系が mask 済みで誰も殺さない)"
apt-get -y purge plymouth plymouth-label plymouth-themes
update-initramfs -u
pkill plymouthd 2>/dev/null || true

log "完了"
