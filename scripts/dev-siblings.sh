#!/usr/bin/env bash
# ローカルの兄弟チェックアウト（misa-runner）を見るように `[patch]` を書き出す。
#
# 通常の依存は GitHub の git 依存なので、`git clone && cargo build` だけで
# 立ち上がる。要るのは **misa-runner を直しながら試すとき**だけ。
#
#   ./scripts/dev-siblings.sh          隣の misa-runner への [patch] を書く
#   ./scripts/dev-siblings.sh --off    上書きを消して git 依存へ戻す
#
# **clone も fetch もしない**（ネットワークに出ない）。`.cargo/config.toml` は
# 追跡していない。**Cargo.toml の git URL と一字一句合わせる**こと（合って
# いないと patch は黙って無視される）。
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$(cd .. && pwd)"
CONFIG=".cargo/config.toml"
if [ "${1:-}" = "--off" ]; then
  rm -f "$CONFIG"; echo "$CONFIG を消しました（git 依存へ戻る）"; exit 0
fi
# 手元では misa-runner のチェックアウトが namiashi-runner という名前で置いてある
# ことがある（remote は misa-runner.git）。どちらでも拾う。
for d in "$ROOT/misa-runner" "$ROOT/namiashi-runner"; do
  if [ -f "$d/crates/misa-runner/Cargo.toml" ]; then
    mkdir -p .cargo
    cat > "$CONFIG" <<EOT
[patch."ssh://git@github.com/takarakasai/misa-runner.git"]
misa-runner = { path = "$d/crates/misa-runner" }
EOT
    echo "$CONFIG を書きました → $d"; exit 0
  fi
done
echo "隣に misa-runner が見つかりません（$ROOT/misa-runner または $ROOT/namiashi-runner）"; exit 1
