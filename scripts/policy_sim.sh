#!/usr/bin/env bash
# 学習済み namiashi 方策を MuJoCo で回し、articara へ配信しつつキーボードで操縦する。
#
#   端末1: ./scripts/policy_sim.sh [policy.onnx] [追加フラグ...]
#   端末2: cd ../articara && cargo run --release --features viz -- \
#            --model ../namiashi-runner2/models/namiashi/namiashi_meas.misa
#          GUI の「Live feed (Zenoh)」で topology=Connect,
#          endpoint tcp/127.0.0.1:7447 のまま Subscribe を ON
#
# キー: W/S = 前後, A/D = 旋回, R/F = 横, Space = 停止, q/Esc = 終了
set -eu
cd "$(dirname "$0")/.."
if [ -z "${MUJOCO_DYNAMIC_LINK_DIR:-}" ]; then
  for d in "${MUJOCO_HOME:-}/lib" "$HOME/.mujoco/mujoco-3.8.0/lib"; do
    if [ -e "$d/libmujoco.so.3.8.0" ]; then MUJOCO_DYNAMIC_LINK_DIR="$d"; break; fi
  done
fi
export MUJOCO_DYNAMIC_LINK_DIR="${MUJOCO_DYNAMIC_LINK_DIR:-$HOME/.mujoco/mujoco-3.8.0/lib}"
# libmujoco は cargo run でも自動では載らない。
export LD_LIBRARY_PATH="$MUJOCO_DYNAMIC_LINK_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# 既定は v24 s103@5500（デプロイ標準。go2_rl doc/namiashi_policy_architecture.md §18）。
DEFAULT_MODEL="$HOME/work/dp/go2_rl/logs/rsl_rl/namiashi_walk_flat_ref/2026-09-23_20-12-21_v24_long_s103/exported/policy_5500.onnx"
MODEL="${1:-${NAMIASHI_POLICY:-$DEFAULT_MODEL}}"
[ $# -gt 0 ] && shift
if [ ! -f "$MODEL" ]; then
  echo "ONNX が見つかりません: $MODEL" >&2
  echo "学習済みモデルはこのリポジトリに入っていない。go2_rl 側の" >&2
  echo "logs/rsl_rl/namiashi_walk_flat_ref/<run>/exported/policy_*.onnx を第1引数か NAMIASHI_POLICY で渡す。" >&2
  exit 1
fi
VIZ_EP="${NAMIASHI_VIZ_ENDPOINT:-tcp/127.0.0.1:7447}"
exec cargo run --release --features sim -- policy --sim --model "$MODEL" --keys \
  --heading-hold 2 0.5 --heading-hold-turn \
  --viz --viz-endpoint "$VIZ_EP" "$@"
