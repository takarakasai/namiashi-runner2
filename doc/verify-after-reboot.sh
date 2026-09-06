#!/bin/bash
# 再起動後の検証 (2026-08-21 の常駐デーモン棚卸し / plymouth purge の確認)
# sudo 不要。SSH でログインし直したら実行する。
set -u
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
ng()   { printf '  \033[31mNG\033[0m   %s\n' "$*"; FAIL=1; }
FAIL=0

printf '\n=== 起動できたか ===\n'
ok "起動して SSH でここまで来ている (uptime: $(uptime -p 2>/dev/null))"
printf '  カーネル: %s\n' "$(uname -r)"

printf '\n=== 1. 消したデーモンが復活していないか ===\n'
for p in plymouthd pulseaudio rtkit-daemon packagekitd haveged; do
  if pgrep -x "$p" >/dev/null 2>&1; then ng "$p が復活している"; else ok "$p なし"; fi
done

printf '\n=== 2. mask/disable が効いているか ===\n'
for u in rtkit-daemon alsa-restore alsa-state packagekit; do
  s=$(systemctl is-active "$u" 2>&1)
  [ "$s" = "inactive" ] && ok "$u = inactive" || ng "$u = $s"
done
s=$(systemctl is-enabled haveged 2>&1)
[ "$s" = "disabled" ] && ok "haveged = disabled" || ng "haveged = $s"

printf '\n=== 3. デバッグ経路が生きているか ===\n'
for u in ssh NetworkManager wpa_supplicant getty@tty1 serial-getty@ttyAS0; do
  s=$(systemctl is-active "$u" 2>&1)
  [ "$s" = "active" ] && ok "$u = active" || ng "$u = $s"
done
ip -br addr show wlan0 2>/dev/null | sed 's/^/  /'

printf '\n=== 4. タイマー (fstrim と tmpfiles-clean だけのはず) ===\n'
systemctl list-timers --all --no-pager | awk '/\.timer/{print "  " $NF}'

printf '\n=== 5. 常駐サービス数 (棚卸し前 17 → 後 14) ===\n'
n=$(systemctl list-units --type=service --state=running --no-pager | grep -c '\.service')
printf '  running = %s\n' "$n"
systemctl list-units --type=service --state=running --no-pager | awk '/\.service/{print "    " $1}'

printf '\n=== 6. failed ユニット ===\n'
systemctl --failed --no-pager | sed 's/^/  /'

printf '\n=== 7. 起動時間 ===\n'
systemd-analyze 2>/dev/null | sed 's/^/  /'
systemd-analyze blame 2>/dev/null | head -8 | sed 's/^/  /'

printf '\n'
[ "$FAIL" = 0 ] && printf '\033[32m全項目 OK\033[0m\n' || printf '\033[31mNG あり。上を確認\033[0m\n'
exit "$FAIL"
