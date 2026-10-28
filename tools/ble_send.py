#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 BLE 透传模块给小车发一条命令，并打印接下来几秒的返回。

用法：
    .venv/bin/python tools/ble_send.py Z          # 静止重标定 IMU
    .venv/bin/python tools/ble_send.py I          # 扫描 I2C1/I2C2
    .venv/bin/python tools/ble_send.py --text "Z" --seconds 4
"""

import argparse
import asyncio

from bleak import BleakClient

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("text", nargs="?", default="Z", help="要发送的命令（单字符）")
    ap.add_argument("--name", default="LFCar")
    ap.add_argument("--address", default=None)
    ap.add_argument("--seconds", type=float, default=3.0, help="发送后监听时长")
    args = ap.parse_args()

    if args.address:
        target = args.address
    else:
        from bleak import BleakScanner
        dev = await BleakScanner.find_device_by_name(args.name, timeout=15.0)
        if dev is None:
            print(f"[send] 没找到 {args.name}")
            return 1
        target = dev.address

    buf = ""

    def cb(_c, data: bytearray):
        nonlocal buf
        buf += data.decode("utf-8", "replace")

    async with BleakClient(target, timeout=20.0) as c:
        await c.start_notify(FFE1, cb)
        await asyncio.sleep(0.4)
        await c.write_gatt_char(FFE1, args.text.encode(), response=True)
        print(f"[send] -> {args.text!r}")
        await asyncio.sleep(args.seconds)

    for line in [l for l in buf.splitlines() if l.strip()][-12:]:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
