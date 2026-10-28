#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从遥测日志辨识"内环速度环"的电机模型，并给出 PID 建议参数。

方法（一阶模型 + 纯延迟）：
    v[k+1] = v[k] + (dt/τ)·(K·u[k-d] − v[k])

    v : 实测速度 cm/s（[A]/[B] 的 m）
    u : 占空比 %（[A]/[B] 的 o）
    K : 稳态增益 cm/s per %duty
    τ : 时间常数 s
    d : 纯延迟（拍）

用最小二乘拟合 K、τ，然后：
    KFF = 1/K                   （前馈：占空比 ≈ KFF × 目标速度）
    用 IMC/SIMC 公式给 KP、KI 初值，并用拟合模型仿真打分，扫描候选。

用法：
    .venv/bin/python tools/ident_speed.py logs/serial_*.log
    .venv/bin/python tools/ident_speed.py logs/xxx.log --wheel B --csv out.csv
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

_TS_RE = re.compile(r"^(\S+)\t(.*)$")
_AB_RE = re.compile(
    r"^\[(?P<wheel>[AB])\]\s+t(?P<t>-?\d+\.\d)\s+m(?P<m>-?\d+\.\d)\s+"
    r"o(?P<o>-?\d+\.\d)\s+i(?P<i>-?\d+\.\d)"
)


def load(path: Path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            m = _TS_RE.match(raw.rstrip("\n"))
            if not m:
                continue
            d = _AB_RE.match(m.group(2).strip())
            if not d:
                continue
            rows.append({
                "ts": m.group(1),
                "wheel": d.group("wheel"),
                "t": float(d.group("t")),
                "m": float(d.group("m")),
                "o": float(d.group("o")),
            })
    return rows


def pick_windows(rows, wheel, dt=0.2, min_len=8):
    """截取"目标速度稳定、实际在动"的窗口，用于拟合。"""
    seq = [r for r in rows if r["wheel"] == wheel]
    wins, cur = [], []
    for r in seq:
        moving = abs(r["t"]) > 3.0 and abs(r["m"]) > 2.0
        if moving:
            cur.append(r)
        else:
            if len(cur) >= min_len:
                wins.append(cur)
            cur = []
    if len(cur) >= min_len:
        wins.append(cur)
    return wins


def fit_model(wins, dt):
    """最小二乘拟合 v[k+1] = a·v[k] + b·u[k]。"""
    A, y = [], []
    for w in wins:
        for k in range(len(w) - 1):
            v, u = w[k]["m"], w[k]["o"]
            vn = w[k + 1]["m"]
            A.append([v, u])
            y.append(vn)
    if len(A) < 20:
        return None
    A = np.asarray(A, dtype=float)
    y = np.asarray(y, dtype=float)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, b = coef
    if not (0.0 < a < 1.0) or b == 0.0:
        return None
    tau = -dt / np.log(a) if a > 0 else float("nan")
    K = b * tau / dt
    return {"a": a, "b": b, "tau": tau, "K": K, "n": len(A)}


def simulate(K, tau, delay, kff, kp, ki, target, dt, sat=100.0):
    """用拟合出的一阶模型仿真 PI 闭环，返回 (v, err) 两个数组。"""
    n = int(3.0 / dt)
    v = 0.0
    integ = 0.0
    hist = []
    for k in range(n):
        e = target - v
        integ += ki * e * dt
        u = kff * target + kp * e + integ
        u = max(-sat, min(sat, u))
        # 延迟：用目标历史的简化（延迟一拍）
        v_prev = v
        v = v_prev + dt / tau * (K * u - v_prev)
        hist.append((v, e))
    return np.asarray(hist)


def main() -> int:
    ap = argparse.ArgumentParser(description="内环电机模型辨识 + PID 建议")
    ap.add_argument("log")
    ap.add_argument("--wheel", default="A", choices=["A", "B"])
    ap.add_argument("--dt", type=float, default=0.2, help="日志采样周期 s")
    args = ap.parse_args()

    rows = load(Path(args.log))
    wins = pick_windows(rows, args.wheel)
    if not wins:
        print(f"[{args.wheel}] 没找到可用的运动窗口（需要目标/实测都 > 3cm/s）")
        return 1

    fit = fit_model(wins, args.dt)
    if fit is None:
        print(f"[{args.wheel}] 拟合失败（样本太少或数据异常）")
        return 1

    K_arx, tau = fit["K"], fit["tau"]

    # 稳态增益用"目标≈实测"的样本直接取比值，比 ARX 受动态影响小
    steady = [r for r in rows
              if r["wheel"] == args.wheel and abs(r["t"]) > 5.0
              and abs(r["m"] - r["t"]) < 1.5 and abs(r["o"]) > 3.0]
    ratios = [r["m"] / r["o"] for r in steady]
    K = float(np.median(ratios)) if len(ratios) >= 20 else K_arx

    print(f"[{args.wheel}] 窗口 {len(wins)} 个，样本 {fit['n']} 条；稳态样本 {len(ratios)} 条")
    print(f"  稳态增益 K  = {K:.3f} cm/s per %duty   (ARX 估计 {K_arx:.3f})")
    print(f"  时间常数 τ  = {tau:.3f} s")
    print(f"  建议 KFF    = {1.0 / K:.3f}  (占空比/速度)")

    # IMC 一阶整定（λ 取 τ 的 1~2 倍，兼顾响应和无超调）
    lam = 2.0 * tau
    kp = tau / (K * lam)
    ki = kp / tau
    print(f"  IMC 初值    : KP={kp:.3f}  KI={ki:.3f} (λ=2τ)")

    # 扫描候选，用仿真模型打分（ITSE + 超调惩罚）
    best = None
    for kp_c in (0.5 * kp, 0.75 * kp, kp, 1.5 * kp, 2.0 * kp):
        for ki_c in (0.0, 0.25 * ki, 0.5 * ki, ki, 2.0 * ki):
            hist = simulate(K, tau, 1, 1.0 / K, kp_c, ki_c, 30.0, 0.02)
            v = hist[:, 0]
            over = max(0.0, float(np.max(v)) - 30.0)
            itse = float(np.sum((hist[:, 1] ** 2) * np.arange(1, len(v) + 1)))
            score = itse + (over ** 2) * 500.0
            if best is None or score < best[0]:
                best = (score, kp_c, ki_c, over, float(v[-1]))
    score, kp_b, ki_b, over, v_end = best
    print(f"  仿真最优    : KP={kp_b:.3f}  KI={ki_b:.3f}  (超调 {over:.1f}cm/s, 终值 {v_end:.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
