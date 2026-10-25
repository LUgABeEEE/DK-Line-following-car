#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DK 循迹小车 / 蓝牙遥测协议解析。

车端格式（USER/Src/Ble_task.c）：

    [Lj] LINE e-0.3 s1.2 L30.0 R28.8 sd+0 sc=LINE  cd+0 sr=1 cs=0
    [A] t30.0 m29.6 o27.5 i3.2 c12345 d678.9
    [B] t30.0 m29.9 o28.1 i2.9 c12001 d678.9
    [G] A 2950 2960 ...              （8 路模拟值）
    [G] T 3000 3000 ...              （8 路阈值）
    [S] ln4 rt0 cl1 sp4 lv-1.4       （场景判据原始特征，2 秒一次）

所有浮点都是 BleTask_Fmt1 输出的 "整数.一位小数"，负数形如 -1.2，
所以正则里统一用 -?\\d+\\.\\d。

本模块只做纯解析，不碰串口，方便离线复用：
  - bt_serial_capture.py 采集时调用
  - serial_parse.py 离线转换日志时调用
"""

from __future__ import annotations

import re
from typing import Any, Dict, Optional

# [L] 外环遥测。注意 mode 可能是 "SPD "（带尾空格），用 \S+ + \s+ 吃掉。
_L_RE = re.compile(
    r"^\[L(?P<tag>[^\]]*)\]\s+"
    r"(?P<mode>\S+)\s+"
    r"e(?P<err>-?\d+\.\d)\s+"
    r"s(?P<steer>-?\d+\.\d)\s+"
    r"L(?P<left>-?\d+\.\d)\s+"
    r"R(?P<right>-?\d+\.\d)\s+"
    r"sd(?P<sd>[+-]?\d+)\s+"
    r"sc=(?P<sc>[A-Z_]+)\s+"
    r"cd(?P<cd>[+-]?\d+)\s+"
    r"sr=(?P<sr>\d+)\s+"
    r"cs=(?P<cs>\d+)"
    r"(?:\s+y(?P<yaw>[+-]?\d+))?"     # p 版起：编码器偏航角（度，可为负）
)

# [A] / [B] 内环遥测
_AB_RE = re.compile(
    r"^\[(?P<kind>[AB])\]\s+"
    r"t(?P<target>-?\d+\.\d)\s+"
    r"m(?P<measure>-?\d+\.\d)\s+"
    r"o(?P<output>-?\d+\.\d)\s+"
    r"i(?P<integral>-?\d+\.\d)\s+"
    r"c(?P<count>\d+)\s+"
    r"d(?P<distance>-?\d+\.\d)"
)

# [G] 8 路模拟值 / 阈值
_G_RE = re.compile(r"^\[G\]\s+(?P<kind>[AT])\s+(?P<vals>(?:\s*\d+){8})")

# [S] 场景判据原始特征
_S_RE = re.compile(
    r"^\[S\]\s+ln(?P<ln>\d+)\s+rt(?P<rt>\d+)\s+"
    r"cl(?P<cl>\d+)\s+sp(?P<sp>\d+)\s+lv(?P<lv>-?\d+\.\d)"
)


def parse_line(line: str) -> Optional[Dict[str, Any]]:
    """解析一行遥测。不认识的行走返回 None（原始日志里仍然保留）。"""
    line = line.strip()
    if not line:
        return None

    m = _L_RE.match(line)
    if m:
        d = m.groupdict()
        return {
            "kind": "L",
            "tag": d["tag"],
            "mode": d["mode"],
            "err": float(d["err"]),
            "steer": float(d["steer"]),
            "left": float(d["left"]),
            "right": float(d["right"]),
            "sd": int(d["sd"]),
            "sc": d["sc"],
            "cd": int(d["cd"]),
            "sr": int(d["sr"]),
            "cs": int(d["cs"]),
            "yaw": int(d["yaw"]) if d.get("yaw") is not None else None,
        }

    m = _AB_RE.match(line)
    if m:
        d = m.groupdict()
        return {
            "kind": d["kind"],
            "target": float(d["target"]),
            "measure": float(d["measure"]),
            "output": float(d["output"]),
            "integral": float(d["integral"]),
            "count": int(d["count"]),
            "distance": float(d["distance"]),
        }

    m = _G_RE.match(line)
    if m:
        return {
            "kind": "G",
            "gray_type": m.group("kind"),
            "values": [int(v) for v in m.group("vals").split()],
        }

    m = _S_RE.match(line)
    if m:
        d = m.groupdict()
        return {
            "kind": "S",
            "ln": int(d["ln"]),
            "rt": int(d["rt"]),
            "cl": int(d["cl"]),
            "sp": int(d["sp"]),
            "lv": float(d["lv"]),
        }

    return None


if __name__ == "__main__":
    # 自测：把几种真实格式过一遍
    samples = [
        "[Lj] LINE e-0.3 s1.2 L30.0 R28.8 sd+0 sc=LINE  cd+0 sr=1 cs=0",
        "[Lj] SPD  e0.0 s0.0 L15.0 R15.0 sd-1 sc=FORK  cd-1 sr=5 cs=1",
        "[A] t30.0 m29.6 o27.5 i3.2 c12345 d678.9",
        "[G] A 2950 2960 2970 2980 2990 3000 3010 3020",
        "[S] ln4 rt0 cl1 sp4 lv-1.4",
        "[VER] hello",
    ]
    for s in samples:
        print(s, "->", parse_line(s))
