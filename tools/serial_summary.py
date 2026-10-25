#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对采集到的 .loop.csv 做快速统计，用来验证角点防抖/转向是否按预期工作。

用法：
    python3 tools/serial_summary.py logs/serial_20261025_165000.loop.csv

输出重点：
  - sr 变化次数 vs sc 变化次数
      sr = 未防抖的瞬时场景，sc = 防抖后的稳定场景。
      锐角弯里 sr 应该频繁变化，而 sc 只在真正的场景切换时变。
      如果 sc 的变化次数接近 sr，说明防抖参数还需要调大。
  - 角点事件（cs 0->1）的次数、平均持续时间
  - 误差 / 差速 / 轮速的范围
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

# sr 是数字编号，sc 是场景名，比较前先统一
SCENE_NAMES = {0: "NONE", 1: "LINE", 2: "ALL", 3: "SIDE_L", 4: "SIDE_R", 5: "FORK"}


def percentile(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = (len(xs) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def main() -> int:
    ap = argparse.ArgumentParser(description="遥测 CSV 快速统计")
    ap.add_argument("csv", help="*.loop.csv")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"文件不存在：{path}", file=sys.stderr)
        return 2

    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append(r)

    if not rows:
        print("没有数据行", file=sys.stderr)
        return 1

    ts = [float(r["t"]) for r in rows]
    dur = ts[-1] - ts[0]

    sc_seq = [r["sc"].strip() for r in rows]
    sr_seq = [SCENE_NAMES.get(int(r["sr"]), f"?{r['sr']}") for r in rows]
    cs_seq = [int(r["cs"]) for r in rows]
    cd_seq = [int(r["cd"]) for r in rows]

    def changes(seq):
        return sum(1 for a, b in zip(seq, seq[1:]) if a != b)

    # 角点事件统计
    events = []
    start = None
    for i, cs in enumerate(cs_seq):
        if cs == 1 and start is None:
            start = i
        elif cs == 0 and start is not None:
            events.append((start, i - 1))
            start = None
    if start is not None:
        events.append((start, len(cs_seq) - 1))

    errs = [float(r["err"]) for r in rows]
    steers = [float(r["steer"]) for r in rows]
    lefts = [float(r["left"]) for r in rows]
    rights = [float(r["right"]) for r in rows]

    print(f"文件        : {path}")
    print(f"采样        : {len(rows)} 行 / {dur:.1f} s = {len(rows)/max(dur,1e-6):.1f} Hz")
    print(f"场景分布 sc : {dict(Counter(sc_seq))}")
    print(f"瞬时分布 sr : {dict(Counter(sr_seq))}")
    print(f"sc 变化次数  : {changes(sc_seq)}")
    print(f"sr 变化次数  : {changes(sr_seq)}")
    print(f"sr!=sc 占比  : {sum(1 for a,b in zip(sc_seq,sr_seq) if a!=b)/len(rows)*100:.1f} %")
    print(f"角点事件数   : {len(events)}")
    if events:
        lens = [(ts[b] - ts[a]) * 1000 for a, b in events]
        print(f"角点持续 ms  : 平均 {sum(lens)/len(lens):.0f}, "
              f"最长 {max(lens):.0f}, 最短 {min(lens):.0f}")
        print("角点明细（cd = 锁存的转弯方向，+1 左 / -1 右）:")
        for k, (a, b) in enumerate(events[:8]):
            rel = ts[a] - ts[0]
            scs = Counter(sc_seq[a:b + 1])
            dirs = Counter(cd_seq[a:b + 1])
            print(f"  #{k+1} 第 {rel:6.2f}s {lens[k]:5.0f}ms  "
                  f"cd={dict(dirs)}  sc={dict(scs)}")
    print(f"误差 err     : min {min(errs):.2f} / p05 {percentile(errs,0.05):.2f} / "
          f"中位 {percentile(errs,0.5):.2f} / p95 {percentile(errs,0.95):.2f} / max {max(errs):.2f}")
    print(f"差速 steer   : min {min(steers):.1f} / max {max(steers):.1f}")
    print(f"左轮 L       : min {min(lefts):.1f} / max {max(lefts):.1f}")
    print(f"右轮 R       : min {min(rights):.1f} / max {max(rights):.1f}")

    # 一句结论（样本太少时不下判断）
    if len(rows) < 30:
        print("\n样本不足 30 拍，结论仅供参考。")
    elif changes(sr_seq) > 0:
        if changes(sc_seq) <= changes(sr_seq) * 0.6:
            print("\n结论：sc 明显比 sr 稳定，防抖生效。")
        else:
            print("\n结论：sc 仍频繁跟着 sr 变化，建议调大 GRAY_SCENE_*_SAMPLES。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
