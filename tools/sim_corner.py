#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""90° 直角弯参数仿真：用实测的车辆/控制参数扫一遍 Kp/Kd/FF/速度。

模型（尽量贴近实车）：
  - 差速驱动：yaw_rate = (vR - vL) / W，W=16cm
  - 内环：轮速一阶滞后 τ=0.35s（由 SPEED_TEST 日志辨识）
  - 外环：40ms 一次，PD + 线角前馈，差速限幅 ±35，每拍最大变化 22
  - 速度规划：按 |e| / |steer| / 线角深度降速，最低到 corner_slow × v_base
  - 传感器：8 路 4.9cm 宽，线 2cm；|e| > 3.0cm 视为丢线
赛道：直线 → 90° 圆弧（半径 R）→ 直线。
判据：跑完不丢线、且最大 |e| 越小越好、用时越短越好。
"""

from __future__ import annotations

import itertools
import math
import sys

import numpy as np

W = 16.0          # 轮距 cm
TAU = 0.35        # 轮速时间常数 s
DT_CTRL = 0.040   # 外环周期 s
DT_SIM = 0.005    # 仿真步长
DIFF_LIMIT = 35.0
SLEW = 22.0
LOST_ERR = 3.0    # 超过这个横向误差就算丢线
E_MAX = 2.5
WHEEL_MAX = 90.0


def build_track(radius: float, straight: float = 40.0):
    pts = []
    for x in np.arange(0.0, straight, 0.1):
        pts.append((float(x), 0.0))
    # 左转 90°：圆心 (straight, radius)，入口 (straight, 0)，出口 (straight+radius, radius)
    cx, cy = straight, radius
    for a in np.arange(0.0, math.pi / 2, 0.005):
        pts.append((cx + radius * math.sin(a), cy - radius * math.cos(a)))
    x0, y0 = cx + radius, cy
    for y in np.arange(y0, y0 + straight, 0.1):
        pts.append((x0, float(y)))
    arr = np.asarray(pts)
    tang = np.zeros_like(arr)
    tang[1:-1] = arr[2:] - arr[:-2]
    tang[0] = arr[1] - arr[0]
    tang[-1] = arr[-1] - arr[-2]
    tang /= np.linalg.norm(tang, axis=1)[:, None] + 1e-9
    return arr, tang


def run(radius: float, v_base: float, kp: float, kd: float, ff: float,
        corner_slow: float, straight_max: float = 0.25):
    path, tang = build_track(radius)
    # 起点：直道中间，朝向 +x
    x, y, th = 15.0, 0.0, 0.0
    vl = vr = 0.0
    last_e = 0.0
    e_f = 0.0
    idx = int(15.0 / 0.1)
    t = 0.0
    max_e = 0.0
    lost = False
    step = 0
    if straight_max > 0:
        pass
    boost = 0.0
    t_ctrl = 0.0
    steer = 0.0
    v_cmd = 0.0

    while t < 6.0:
        # --- 传感器（每 40ms 更新一次）---
        if t_ctrl <= 0.0:
            t_ctrl = DT_CTRL
            # 最近路径点（局部搜索）
            i0 = max(0, idx - 60)
            i1 = min(len(path), idx + 200)
            d2 = (path[i0:i1, 0] - x) ** 2 + (path[i0:i1, 1] - y) ** 2
            k = int(np.argmin(d2))
            idx = i0 + k
            dist = math.sqrt(float(d2[k]))
            if dist > LOST_ERR:
                lost = True      # 线已不在探头阵列下
                break
            px, py = path[idx]
            tx, ty = tang[idx]
            # 横向误差（正=线在左侧）
            rx, ry = x - px, y - py
            # 线相对车的横向位置（正 = 线在车左侧）
            e = rx * ty - ry * tx
            v = 0.5 * (vl + vr)
            de = (e - last_e) / DT_CTRL
            e_f += 0.5 * (e - e_f)
            theta_line = math.atan2(ty, tx) - th
            theta_line = (theta_line + math.pi) % (2 * math.pi) - math.pi
            if abs(e_f) > LOST_ERR:
                lost = True
                break
            # PD + 前馈
            ff_term = ff * v * theta_line if abs(theta_line) < 0.7 else ff * v * np.sign(theta_line) * 0.7
            s_new = kp * e_f + kd * de + ff_term
            s_new = max(-DIFF_LIMIT, min(DIFF_LIMIT, s_new))
            steer += max(-SLEW, min(SLEW, s_new - steer))
            # 直道加速
            if (abs(e_f) < 0.35) and (abs(steer) < 8.0):
                boost = min(straight_max, boost + 0.03)
            else:
                boost = max(0.0, boost - 0.06)
            # 速度规划
            depth = max(abs(e_f) / E_MAX, abs(steer) / DIFF_LIMIT, abs(theta_line) / 0.7)
            depth = min(1.0, depth)
            slow = 1.0 - (1.0 - corner_slow) * depth
            v_cmd = v_base * slow * (1.0 + boost)
            last_e = e

        # --- 电机一阶滞后 ---
        tl = max(-WHEEL_MAX, min(WHEEL_MAX, v_cmd - steer))
        tr = max(-WHEEL_MAX, min(WHEEL_MAX, v_cmd + steer))
        vl += (tl - vl) * DT_SIM / TAU
        vr += (tr - vr) * DT_SIM / TAU
        v = 0.5 * (vl + vr)
        omega = (vr - vl) / W
        th += omega * DT_SIM
        x += v * math.cos(th) * DT_SIM
        y += v * math.sin(th) * DT_SIM
        t += DT_SIM
        t_ctrl -= DT_SIM
        step += 1
        # 到终点（路径末端）
        if idx >= len(path) - 3:
            break
    return dict(ok=not lost, t=round(t, 2), max_e=round(max_e, 2), lost=lost)


def main() -> int:
    print("半径 R / 基速 v_base / Kp / Kd / FF / 角点限速 -> 结果")
    best = []
    for radius, v_base in [(2, 25), (3, 25), (5, 30), (5, 40), (5, 55),
                           (8, 40), (8, 55), (10, 55), (15, 55)]:
        rows = []
        for kp, kd, ff, cs in itertools.product((8, 12, 16, 20), (0.6, 1.0, 1.4),
                                               (0.7, 0.9), (0.5, 0.62, 0.72)):
            r = run(radius, v_base, kp, kd, ff, cs)
            if r["ok"]:
                rows.append((r["t"], kp, kd, ff, cs, r["max_e"]))
        if rows:
            rows.sort()
            t, kp, kd, ff, cs, me = rows[0]
            print(f"R={radius:4.0f}cm v={v_base:2.0f}: 可行 {len(rows):3d} 组 | 最优 Kp={kp} Kd={kd} FF={ff} slow={cs} 用时{t}s")
            best.append((radius, v_base, kp, kd, ff, cs))
        else:
            print(f"R={radius:4.0f}cm v={v_base:2.0f}: 所有组合都丢线 -> 必须减速或靠原地转")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
