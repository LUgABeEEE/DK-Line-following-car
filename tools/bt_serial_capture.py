#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HC-05（SPP）串口采集：把车上的蓝牙遥测写进 logs/，供 AI / 脚本分析。

两种连接方式，二选一：

  1) 直接连 RFCOMM（推荐，不需要 rfcomm 命令）
       python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF
     默认依次尝试 RFCOMM 通道 1~5，连上哪个用哪个。
     前提：HC-05 已在系统里配对（bluetoothctl pair / trust）。

  2) 连一个已经存在的 tty（USB-TTL，或内核已 rfcomm bind 出的 /dev/rfcomm0）
       python3 tools/bt_serial_capture.py --port /dev/rfcomm0 --baud 9600

典型用法：

    # 采集 30 秒，按 Ctrl-C 也可以提前结束
    python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF --duration 30

    # 只抓原始行，不生成 CSV
    python3 tools/bt_serial_capture.py --port /dev/ttyUSB0 --no-csv

    # 离线把之前抓的日志重新解析成 CSV
    python3 tools/serial_parse.py --log logs/serial_20261025_165000.log

输出（logs/ 已在 .gitignore 里）：
    serial_YYYYmmdd_HHMMSS.log        原始行 + ISO 时间戳（保底）
    serial_YYYYmmdd_HHMMSS.loop.csv   [L] 外环遥测（控制分析主表）
    serial_YYYYmmdd_HHMMSS.speed.csv  [A]/[B] 内环遥测
    serial_YYYYmmdd_HHMMSS.gray.csv   [G] 8 路模拟值/阈值
    serial_YYYYmmdd_HHMMSS.scene.csv  [S] 场景判据特征
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import serial_proto  # noqa: E402
from serial_sink import TelemetrySink  # noqa: E402


# ---------------------------------------------------------------------------
# 数据源
# ---------------------------------------------------------------------------
class TtySource:
    """普通 tty（USB-TTL / rfcomm）。用 termios 配成 raw 8N1。"""

    BAUDS = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }

    def __init__(self, path: str, baud: int = 9600) -> None:
        if baud not in self.BAUDS:
            raise SystemExit(f"不支持的波特率 {baud}，可选 {sorted(self.BAUDS)}")

        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                    # iflag：关掉软件流控/换行转换
        attrs[1] = 0                                    # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                    # lflag：非规范模式
        attrs[4] = self.BAUDS[baud]                     # ispeed
        attrs[5] = self.BAUDS[baud]                     # ospeed
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def fileno(self) -> int:
        return self.fd

    def read(self, n: int) -> bytes:
        return os.read(self.fd, n)

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass

    def describe(self) -> str:
        return f"tty {self.path}"


class RfcommSource:
    """直接开 RFCOMM socket 连 HC-05，不依赖 rfcomm 命令。"""

    def __init__(self, mac: str, channels) -> None:
        self.mac = mac.upper()
        last_err = None
        for ch in channels:
            s = socket.socket(socket.AF_BLUETOOTH,
                              socket.SOCK_STREAM,
                              socket.BTPROTO_RFCOMM)
            s.settimeout(8.0)
            try:
                s.connect((self.mac, ch))
            except OSError as exc:
                last_err = exc
                s.close()
                continue
            s.settimeout(None)
            self.sock = s
            self.channel = ch
            return
        raise OSError(
            f"连不上 {self.mac}（通道 {channels}）：{last_err}. "
            "先确认 HC-05 已上电、没有被手机占用、且已配对。"
        )

    def fileno(self) -> int:
        return self.sock.fileno()

    def read(self, n: int) -> bytes:
        return self.sock.recv(n)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def describe(self) -> str:
        return f"RFCOMM {self.mac} ch{self.channel}"


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
def open_source(args):
    if args.mac:
        return RfcommSource(args.mac, args.channel)
    return TtySource(args.port, args.baud)


def pump(src, sink: TelemetrySink, args) -> int:
    buf = ""
    lines = 0
    t0 = time.time()
    last_report = t0

    while True:
        if args.duration and (time.time() - t0) >= args.duration:
            break
        try:
            ready, _, _ = select.select([src.fileno()], [], [], 0.5)
        except (OSError, ValueError):
            break
        if not ready:
            continue
        try:
            chunk = src.read(4096)
        except (OSError, EOFError) as exc:
            print(f"\n[capture] 读取出错：{exc}", file=sys.stderr)
            break
        if not chunk:
            print("\n[capture] 对端关闭连接", file=sys.stderr)
            break

        buf += chunk.decode("utf-8", "replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.rstrip("\r")
            if not line:
                continue

            now = time.time()
            iso = datetime.fromtimestamp(now).astimezone().isoformat(
                timespec="milliseconds")
            sink.raw(iso, line)

            parsed = serial_proto.parse_line(line)
            if parsed is not None and not args.no_csv:
                sink.write(iso, now, parsed)

            if args.echo:
                print(line)

            lines += 1
            if now - last_report >= 5.0:
                rate = lines / max(1e-6, (now - t0))
                print(f"[capture] {lines} 行，{rate:.1f} 行/秒", file=sys.stderr)
                last_report = now

    return lines


def main() -> int:
    ap = argparse.ArgumentParser(
        description="采集 DK 循迹小车 HC-05 / tty 遥测到 logs/")
    ap.add_argument("--mac", help="HC-05 的蓝牙 MAC，例如 AA:BB:CC:DD:EE:FF")
    ap.add_argument("--channel", type=int, nargs="+", default=[1, 2, 3, 4, 5],
                    help="RFCOMM 通道候选（默认 1~5 依次尝试）")
    ap.add_argument("--port", help="tty 路径，例如 /dev/rfcomm0 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600,
                    help="tty 波特率（默认 9600，与 USART2 一致）")
    ap.add_argument("--outdir", default="logs", help="输出目录（默认 logs）")
    ap.add_argument("--prefix", default=None,
                    help="输出文件名前缀（默认 serial_日期_时间）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="采集多少秒后自动停止（0 = 一直采）")
    ap.add_argument("--reconnect", dest="reconnect", action="store_true",
                    default=True, help="断线自动重连（默认开）")
    ap.add_argument("--no-reconnect", dest="reconnect", action="store_false")
    ap.add_argument("--reconnect-delay", type=float, default=2.0,
                    help="重连间隔秒数（默认 2）")
    ap.add_argument("--no-csv", action="store_true", help="只写原始 .log")
    ap.add_argument("--echo", action="store_true", help="同时打印到终端")
    args = ap.parse_args()

    if bool(args.mac) == bool(args.port):
        ap.error("--mac 和 --port 必须且只能给一个")

    prefix = args.prefix or ("serial_" + datetime.now().strftime("%Y%m%d_%H%M%S"))

    # 等真正连上再创建输出文件：配对没做好时不留下空文件
    sink = None
    total = 0
    try:
        while True:
            try:
                src = open_source(args)
            except OSError as exc:
                print(f"[capture] 连接失败：{exc}", file=sys.stderr)
                if not args.reconnect:
                    return 2
                time.sleep(args.reconnect_delay)
                continue

            print(f"[capture] 已连接 {src.describe()}", file=sys.stderr)
            if sink is None:
                sink = TelemetrySink(args.outdir, prefix)
                print(f"[capture] 原始日志：{sink.paths()['log']}", file=sys.stderr)
                if not args.no_csv:
                    print(f"[capture] 解析 CSV：{sink.paths()['loop'].parent}/"
                          f"{prefix}.*.csv", file=sys.stderr)
            try:
                total += pump(src, sink, args)
            finally:
                src.close()

            if args.duration or not args.reconnect:
                break
            print(f"[capture] 断开，{args.reconnect_delay:g}s 后重连",
                  file=sys.stderr)
            time.sleep(args.reconnect_delay)
    except KeyboardInterrupt:
        print("\n[capture] 用户中断", file=sys.stderr)
    finally:
        if sink is not None:
            sink.close()
            print(f"[capture] 共 {total} 行，文件在 {args.outdir}/", file=sys.stderr)
        else:
            print("[capture] 没有建立连接，未生成日志文件", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
