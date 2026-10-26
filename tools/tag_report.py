#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 [T] 成功/失败标记切分原始日志，输出每段运行摘要。

车端（构建 s 起）：循迹模式下停在 MODE 页
    短按 KEY2 -> 串口打印 "[T] OK   #n t<mcu_ms>"    （刚跑完这段成功）
    短按 KEY1 -> 串口打印 "[T] FAIL #n t<mcu_ms>"    （刚跑完这段失败）

约定：**标记打在"这段数据跑完之后"**，所以一段数据 = 上一个标记之后
到本标记之前。第一次标记之前的数据没有标签（--）。

用法：
    python3 tools/tag_report.py logs/serial_20261026_*.log
    python3 tools/tag_report.py logs/xxx.log --csv logs/tags.csv
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from serial_proto import parse_line  # noqa: E402

_TS_RE = re.compile(r"^(\S+)\t(.*)$")


def _t(s: str) -> float:
    try:
        return datetime.fromisoformat(s).timestamp()
    except ValueError:
        return 0.0


def load_log(path: Path):
    """返回 (tags, line_samples, dist_samples)，都按日志顺序。"""
    tags = []       # (ts, ok, seq)
    lines = []      # (ts, dict)
    dist = []       # (ts, cm)

    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            m = _TS_RE.match(raw.rstrip("\n"))
            if not m:
                continue
            ts, body = m.group(1), m.group(2)
            d = parse_line(body)
            if d is None:
                continue
            if d["kind"] == "T":
                tags.append((ts, d["ok"], d["seq"]))
            elif d["kind"] == "L":
                lines.append((ts, d))
            elif d["kind"] == "A":
                dist.append((ts, d["distance"]))
    return tags, lines, dist


def summarize(lines, dist, t0, t1):
    seg = [(ts, d) for ts, d in lines if t0 < _t(ts) <= t1]
    if not seg:
        return None

    modes = Counter(d["mode"] for _, d in seg)
    scenes = Counter(d["sc"] for _, d in seg)
    n_corner = sum(1 for a, b in zip(seg, seg[1:])
                   if a[1]["cs"] == 0 and b[1]["cs"] == 1)
    n_lost = sum(1 for _, d in seg if d["sc"] == "NONE")
    errs = sorted(abs(d["err"]) for _, d in seg if d["mode"] == "LINE")
    med_e = errs[len(errs) // 2] if errs else 0.0

    ds = [cm for ts, cm in dist if t0 < _t(ts) <= t1]
    travel = (ds[-1] - ds[0]) if len(ds) >= 2 else 0.0

    return {
        "dur": t1 - t0,
        "n": len(seg),
        "modes": modes,
        "scenes": scenes,
        "corners": n_corner,
        "lost": n_lost,
        "med_e": med_e,
        "travel": travel,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="按 [T] 标记切分日志，输出每段摘要")
    ap.add_argument("logs", nargs="+", help="原始日志（可传多个/通配符）")
    ap.add_argument("--csv", help="把分段摘要额外写成 CSV")
    args = ap.parse_args()

    rows = []
    for pattern in args.logs:
        for p in sorted(Path().glob(pattern)) or [Path(pattern)]:
            if not p.is_file():
                continue
            tags, lines, dist = load_log(p)
            if not lines:
                print(f"{p}: 没有 [L] 数据")
                continue

            bounds = [(_t(lines[0][0]), None, None)]
            for ts, ok, seq in tags:
                bounds.append((_t(ts), ok, seq))
            bounds.append((_t(lines[-1][0]), "END", None))

            print(f"\n=== {p.name} ===")
            print(f"标记 {len(tags)} 个  [L] 样本 {len(lines)}")
            for i in range(len(bounds) - 1):
                t0 = bounds[i][0]
                t1 = bounds[i + 1][0]
                if i + 1 < len(bounds) - 1:
                    label = "OK" if bounds[i + 1][1] else "FAIL"
                    tagseq = bounds[i + 1][2]
                else:
                    label = "--"
                    tagseq = None

                s = summarize(lines, dist, t0, t1)
                if s is None or s["n"] < 5:
                    continue

                modes = " ".join(f"{k}:{v}" for k, v in s["modes"].most_common(3))
                sc = " ".join(f"{k}:{v}" for k, v in s["scenes"].most_common(3))
                ss = (f"[{label:4s} #{tagseq}] {datetime.fromtimestamp(t0).strftime('%H:%M:%S')}"
                      f"~{datetime.fromtimestamp(t1).strftime('%H:%M:%S')}"
                      f" {s['dur']:6.1f}s  n={s['n']:4d}"
                      f"  角点{s['corners']:2d} 丢线{s['lost']:3d}"
                      f"  |e|中位{s['med_e']:.2f}  里程{s['travel']:6.1f}cm")
                print(ss)
                print(f"       模式 {modes}")
                print(f"       场景 {sc}")
                rows.append({
                    "file": p.name, "label": label, "seq": tagseq,
                    "start": datetime.fromtimestamp(t0).isoformat(),
                    "end": datetime.fromtimestamp(t1).isoformat(),
                    "dur_s": round(s["dur"], 1), "n": s["n"],
                    "corners": s["corners"], "lost": s["lost"],
                    "med_e": round(s["med_e"], 2), "travel_cm": round(s["travel"], 1),
                })

    if args.csv and rows:
        import csv
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nCSV -> {args.csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
