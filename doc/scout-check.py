#!/usr/bin/env python3
"""Zenoh のマルチキャスト探索が届いているかを確認する。

zenoh は既定で `224.0.0.224:7446` に scout メッセージを撒く。ここへ**受動的に**
join して、誰から届いているかを表示するだけのスクリプト。zenoh 本体には触れない
ので、`misa-run legs --viz` を動かしたまま実行してよい。

    # SBC と PC の両方で走らせる
    python3 scout-check.py

PC で articara（または任意の zenoh アプリ）を起動した状態で SBC 側にこれを
走らせ、**PC の IP から届けばマルチキャストは通っている**。自分の IP しか
出なければ、経路のどこかで落ちている。

WiFi 越しは通らないことがよくある。AP のクライアント分離や IGMP snooping で
無線クライアント間のマルチキャストが落とされるため。その場合は
`--viz-endpoint tcp/0.0.0.0:7447` で固定ポート待ち受けに切り替える
（`viz_live.md` のネットワーク A）。
"""

import argparse
import socket
import struct
import sys
import time

GROUP = "224.0.0.224"
PORT = 7446


def local_addrs() -> set[str]:
    """自分の IPv4 アドレス。届いた packet が自分由来かを見分けるのに使う。"""
    out = {"127.0.0.1"}
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 実際には送らない。経路表を引いて送信元アドレスを得るための接続。
        s.connect(("8.8.8.8", 53))
        out.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--group", default=GROUP)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--secs", type=float, default=15.0, help="0 以下で Ctrl-C まで")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    s.bind(("", args.port))
    mreq = struct.pack("4sl", socket.inet_aton(args.group), socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    s.settimeout(1.0)

    mine = local_addrs()
    forever = args.secs <= 0
    limit = "Ctrl-C まで" if forever else f"{args.secs:.0f} 秒"
    print(f"{args.group}:{args.port} を受信中（{limit}）。自分の IP: {sorted(mine)}")
    print("他ホストの IP が出ればマルチキャストは通っている。\n")

    seen: dict[str, int] = {}
    start = time.monotonic()
    try:
        while forever or time.monotonic() - start < args.secs:
            try:
                data, (src, _) = s.recvfrom(65535)
            except socket.timeout:
                continue
            first = src not in seen
            seen[src] = seen.get(src, 0) + 1
            if first:
                tag = "自分" if src in mine else "**他ホスト**"
                print(f"  {src:15s} {tag}  ({len(data)} bytes)")
    except KeyboardInterrupt:
        pass

    print("\n--- 集計 ---")
    if not seen:
        print("  1 つも届かなかった。zenoh アプリがどこでも動いていないか、")
        print("  マルチキャストが落ちている。")
        return 1
    others = 0
    for src, n in sorted(seen.items()):
        tag = "自分" if src in mine else "**他ホスト**"
        others += src not in mine
        print(f"  {src:15s} {n:5d} packets  {tag}")
    if others:
        print("\n  → 他ホストから届いている。マルチキャスト探索は通っている。")
        return 0
    print("\n  → 自分のぶんだけ。相手まで届いていないか、相手が動いていない。")
    print("     相手側でも zenoh アプリを起動しているか確認する。")
    print("     起動しているのに届かないなら、--viz-endpoint で固定ポートに切り替える")
    print("     （viz_live.md のネットワーク A）。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
