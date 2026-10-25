#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线把采集到的原始日志重新解析成 CSV。

适用于：
  - 之前用 tio / minicom / 手机 APP 导出的纯文本日志
  - bt_serial_capture.py 生成的 .log（本身就是 "ISO\t原始行"）

用法：
    python3 tools/serial_parse.py --log logs/serial_20261025_165000.log
    python3 tools/serial_parse.py --log a.log b.log --outdir logs/parsed

输出与采集时一致：.loop.csv / .speed.csv / .gray.csv / .scene.csv
（不会覆盖原始 .log）
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import serial_proto  # noqa: E402
from serial_sink import TelemetrySink  # noqa: E402


def split_timestamp(line: str):
    """日志行可能是 "ISO\t原文" 或纯原文。返回 (iso, epoch, 原文)。"""
    if "\t" in line:
        head, rest = line.split("\t", 1)
        try:
            dt = datetime.fromisoformat(head)
            return head, dt.timestamp(), rest.strip()
        except ValueError:
            pass
    return "", 0.0, line.strip()


def parse_file(path: Path, outdir: Path) -> int:
    # 用文件名做前缀，避免多个日志互相覆盖
    prefix = path.stem + ".parsed"
    sink = TelemetrySink(str(outdir), prefix, write_raw=False)
    n = 0
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            raw = raw.rstrip("\n")
            if not raw.strip():
                continue
            iso, epoch, line = split_timestamp(raw)
            if not iso:
                epoch = time.time()
                iso = datetime.fromtimestamp(epoch).astimezone().isoformat(
                    timespec="milliseconds")
            parsed = serial_proto.parse_line(line)
            if parsed is not None:
                sink.write(iso, epoch, parsed)
                n += 1
    sink.close()
    print(f"[parse] {path} -> {n} 行，输出 {outdir}/{prefix}.*.csv",
          file=sys.stderr)
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description="离线解析遥测日志为 CSV")
    ap.add_argument("--log", nargs="+", required=True, help="一个或多个日志文件")
    ap.add_argument("--outdir", default="logs/parsed", help="输出目录")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    total = 0
    for name in args.log:
        p = Path(name)
        if not p.exists():
            print(f"[parse] 跳过不存在的文件：{p}", file=sys.stderr)
            continue
        total += parse_file(p, outdir)
    print(f"[parse] 合计解析 {total} 行", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
