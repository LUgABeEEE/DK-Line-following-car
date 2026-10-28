#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""蓝牙 BLE 串口模块采集（FFE0/FFE1 透传），把收到的行按 timestamp\\tbody 写入日志。

适用于 HM-10 / JDY-31 / AT-09 等 BLE 串口模块（不是经典蓝牙 SPP 的 HC-05）。

用法：
    .venv/bin/python -u tools/bt_ble_capture.py --name LFCar --outdir logs
    .venv/bin/python -u tools/bt_ble_capture.py --address 8F:39:C7:CD:2A:78
"""

from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import sys
from pathlib import Path

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


def now_iso() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


async def find_device(name: str | None, address: str | None, timeout: float = 15.0):
    seen = {}

    def cb(dev, adv):
        seen[dev.address.upper()] = dev

    scanner = BleakScanner(detection_callback=cb)
    await scanner.start()
    try:
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            if address and address.upper() in seen:
                return seen[address.upper()]
            if name and any((d.name or "") == name for d in seen.values()):
                return next(d for d in seen.values() if (d.name or "") == name)
            await asyncio.sleep(0.2)
        return None
    finally:
        await scanner.stop()


async def run(args) -> int:
    out_path = Path(args.outdir) / f"serial_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(out_path, "w", encoding="utf-8")
    print(f"[ble] 日志：{out_path}")

    buf = ""
    line_count = 0
    last_rx = asyncio.get_event_loop().time()

    def on_notify(_char, data: bytearray):
        nonlocal buf, line_count, last_rx
        last_rx = asyncio.get_event_loop().time()
        buf += data.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                fh.write(f"{now_iso()}\t{line}\n")
                fh.flush()
                line_count += 1

    backoff = 2.0
    while True:
        dev = await find_device(args.name, args.address)
        if dev is None:
            print("[ble] 没扫描到设备，2s 后重试（确认模块上电、不在被手机占用）")
            await asyncio.sleep(2.0)
            continue

        print(f"[ble] 找到 {dev.address} ({dev.name})，连接…")
        try:
            async with BleakClient(dev.address, timeout=20.0) as client:
                await client.start_notify(FFE1, on_notify)
                print("[ble] 已连接，开始采集（Ctrl-C 结束）")
                while True:
                    await asyncio.sleep(1.0)
                    if asyncio.get_event_loop().time() - last_rx > args.idle_timeout:
                        print(f"[ble] {args.idle_timeout}s 无数据，重连")
                        break
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[ble] 连接/接收失败：{e}")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 1.5, 5.0)

    fh.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=None)
    ap.add_argument("--address", default=None)
    ap.add_argument("--outdir", default="logs")
    ap.add_argument("--idle-timeout", type=float, default=5.0)
    args = ap.parse_args()
    if not args.name and not args.address:
        ap.error("--name 或 --address 至少给一个")
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[ble] 用户中断")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
