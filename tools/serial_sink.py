#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥测落盘：把解析结果写成 CSV，采集和离线转换共用。

输出文件（同一个 --outdir 下）：
    <prefix>.log          原始行（ISO 时间戳 + TAB + 原文），保底可回溯
    <prefix>.loop.csv     [L] 外环遥测，一行一拍（5 Hz）
    <prefix>.speed.csv    [A]/[B] 内环遥测（axis 列区分左右轮）
    <prefix>.gray.csv     [G] 8 路模拟值 / 阈值
    <prefix>.scene.csv    [S] 场景判据原始特征

设计原则：
  - 全部用标准库，不依赖 pandas / pyserial
  - 每写一行就 flush，采集过程中 AI 可以随时读文件
  - 列名用 ASCII，方便 awk / pandas 直接吃
"""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Any, Dict, Optional


class TelemetrySink:
    def __init__(self, outdir: str = "logs", prefix: str = "serial",
                 write_raw: bool = True) -> None:
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.base = self.outdir / prefix
        # 注意：不能对 base 用 Path.with_suffix()，它会把文件名里已有的
        # "." 之后的内容当成后缀替换掉（prefix 里可能带点）。
        self.log_path = self._name(".log")
        self._log = (open(self.log_path, "w", encoding="utf-8", buffering=1)
                     if write_raw else None)

        self._csv = {}
        self._csv["L"] = self._open_csv(
            self._name(".loop.csv"),
            ["t", "iso", "tag", "mode", "err", "steer", "left", "right",
             "sd", "sc", "cd", "sr", "cs"],
            "loop",
        )
        self._csv["AB"] = self._open_csv(
            self._name(".speed.csv"),
            ["t", "iso", "axis", "target", "measure", "output", "integral",
             "count", "distance"],
            "AB",
        )
        self._csv["G"] = self._open_csv(
            self._name(".gray.csv"),
            ["t", "iso", "type"] + [f"ch{i}" for i in range(1, 9)],
            "G",
        )
        self._csv["S"] = self._open_csv(
            self._name(".scene.csv"),
            ["t", "iso", "ln", "rt", "cl", "sp", "lv"],
            "S",
        )

    def _name(self, ext: str) -> Path:
        return self.base.parent / (self.base.name + ext)

    def _open_csv(self, path: Path, header, key):
        fh = open(path, "w", encoding="utf-8", newline="", buffering=1)
        w = csv.writer(fh)
        w.writerow(header)
        return {"path": path, "file": fh, "writer": w, "key": key}

    # ------------------------------------------------------------------
    def raw(self, iso: str, line: str) -> None:
        if self._log is not None:
            self._log.write(f"{iso}\t{line}\n")

    def write(self, iso: str, epoch: float, d: Dict[str, Any]) -> None:
        kind = d.get("kind")
        if kind == "L":
            self._csv["L"]["writer"].writerow([
                f"{epoch:.3f}", iso, d["tag"], d["mode"],
                d["err"], d["steer"], d["left"], d["right"],
                d["sd"], d["sc"], d["cd"], d["sr"], d["cs"],
            ])
        elif kind in ("A", "B"):
            self._csv["AB"]["writer"].writerow([
                f"{epoch:.3f}", iso, kind,
                d["target"], d["measure"], d["output"], d["integral"],
                d["count"], d["distance"],
            ])
        elif kind == "G":
            self._csv["G"]["writer"].writerow(
                [f"{epoch:.3f}", iso, d["gray_type"]] + d["values"]
            )
        elif kind == "S":
            self._csv["S"]["writer"].writerow([
                f"{epoch:.3f}", iso, d["ln"], d["rt"], d["cl"], d["sp"], d["lv"],
            ])

    def paths(self):
        return {
            "log": self.log_path if self._log is not None else None,
            "loop": self._csv["L"]["path"],
            "speed": self._csv["AB"]["path"],
            "gray": self._csv["G"]["path"],
            "scene": self._csv["S"]["path"],
        }

    def close(self) -> None:
        for c in self._csv.values():
            try:
                c["file"].close()
            except OSError:
                pass
        if self._log is not None:
            try:
                self._log.close()
            except OSError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
