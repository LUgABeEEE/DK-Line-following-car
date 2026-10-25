# tools/ — HC-05 遥测采集与离线分析

车上的 HC-05 是 **SPP（串口透传）** 模块，波特率 9600，对应 USART2。
这一套脚本的目标只有一个：**把车上的 ASCII 遥测稳定落到文件里**，
让 AI 或你自己可以直接读、画图、做统计，而不是盯着手机 APP 看。

全部是 Python 标准库，不需要 `pyserial`、`pandas`、`numpy`。

---

## 1. 为什么不用 `rfcomm` / 现成串口工具

BlueZ 从 5.65 起移除了 `rfcomm`、`sdptool`、`hcitool` 这些老工具，
Arch 的 `bluez-utils 5.87` 里已经找不到 `rfcomm` 了，
所以网上大多数 "`sudo rfcomm bind ...`" 的教程在这台机器上会直接失败。

本目录的 `bt_serial_capture.py` **直接用 Python 的 RFCOMM socket**
（`AF_BLUETOOTH + BTPROTO_RFCOMM`）连 HC-05，不需要 `rfcomm` 命令，
也不需要内核 tty 节点。配对仍然用系统自带的 `bluetoothctl`。

| 方案 | 需要什么 | 适合谁 |
|---|---|---|
| **本目录脚本** | 已配对 HC-05 | AI / 脚本 / 长时间采集 |
| `tio` | 一个 tty（`/dev/rfcomm0`、`/dev/ttyUSB0`） | 人在终端里看 |
| Serial Studio | 一个 tty | 人看波形、导 CSV |
| serial MCP server | DSH/Claude 配置 MCP | 让 agent 直接调工具（当前会话没有 MCP 通道） |

如果只是想手动看一眼数据、手边有 USB-TTL，`tio` 最短：

```bash
tio -b 9600 --timestamp -L logs/manual.log /dev/ttyUSB0
```

---

## 2. 配对 HC-05（只需一次）

车端：

- HC-05 上电，**EN 悬空或拉低**（透传数据模式；EN 拉高是 AT 命令模式）；
- 确认没有被手机 APP 连着（HC-05 同一时刻只能被一个主设备连接）。

PC 端：

```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# scan on
# 等到出现 [NEW] Device AA:BB:CC:DD:EE:FF HC-05
[bluetooth]# pair AA:BB:CC:DD:EE:FF
Request PIN code: 1234          # HC-05 默认 PIN，买回来没改过就是这个
[bluetooth]# trust AA:BB:CC:DD:EE:FF
[bluetooth]# scan off
[bluetooth]# quit
```

确认一下：

```bash
bluetoothctl devices            # 列表里应该有 HC-05
```

**本车实测**：HC-05 显示成 `Device 26:03:02:01:A5:F4 ?`（名字读出来是 `?`，
这是正常的；`Paired/Bonded/Trusted` 都是 yes，UUID 里有 `Serial Port`）。
扫描时还会看到一个 `26:03:02:02:A5:F4` —— 那是另一个 LE 设备
（带腾讯 `0xfee7` 服务），**不是** HC-05，别配错。

**不需要也不要 `rfcomm bind`** —— 本目录脚本直接连 MAC。

---

## 3. 采集

```bash
# 直接连 RFCOMM，采 60 秒
python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF --duration 60

# 一直采，Ctrl-C 停止
python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF

# 连已有的 tty（USB-TTL 或系统能提供 /dev/rfcomm0）
python3 tools/bt_serial_capture.py --port /dev/ttyUSB0 --baud 9600

# 只抓原始文本，不生成 CSV
python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF --no-csv

# 断线自动重连（默认就是开的，间隔 2s）
python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF --reconnect-delay 3
```

**空闲看门狗**：车上每秒约 10 行数据，如果链路静默超过 5 秒（`--idle-timeout`，
默认 5），脚本会判定链路失效并重连 —— 之前出现过 BlueZ 显示 `Connected: yes`
但 RFCOMM 已经死掉、`select` 永远不返回的情况，采集会静默假死。

后台跑（关掉终端也不停）：

```bash
nohup python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF \
      --prefix run1 >/tmp/cap.log 2>&1 &
```

输出（`logs/`，已 gitignore）：

| 文件 | 内容 |
|---|---|
| `serial_YYYYmmdd_HHMMSS.log` | 原始行 + ISO 时间戳，保底可回溯 |
| `*.loop.csv` | `[L]` 外环遥测，一行一拍（5 Hz），控制分析主表 |
| `*.speed.csv` | `[A]/[B]` 内环遥测，`axis` 列区分左右轮 |
| `*.gray.csv` | `[G]` 8 路模拟值/阈值 |
| `*.scene.csv` | `[S]` 场景判据原始特征 |

CSV 列名全是 ASCII，`pandas.read_csv` / `awk` 直接吃：

```python
import pandas as pd
df = pd.read_csv("logs/serial_20261025_165000.loop.csv")
print(df[df.cs == 1][["err", "steer", "sc", "sr", "cd"]].head())
```

---

## 4. 离线分析

```bash
# 之前用手机 APP / tio 导出的纯文本日志，也能重新解析
python3 tools/serial_parse.py --log logs/old.log --outdir logs/parsed

# 快速统计：sr/sc 变化次数、角点事件、误差/差速范围
python3 tools/serial_summary.py logs/serial_20261025_165000.loop.csv
```

`serial_summary.py` 的输出里最值得看的两行：

- `sc 变化次数` vs `sr 变化次数`：`sr` 是未防抖的瞬时场景，
  `sc` 是防抖后的场景。过锐角弯时 `sr` 应该频繁变、`sc` 应该明显更少。
  两者接近说明 `GRAY_SCENE_*_SAMPLES` 还要调大。
- `角点明细`：每个角点事件的锁存方向 `cd` 和持续时长。
  `cd` 与车的实际转向方向不一致，就是方向判据有问题。

---

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `bluetoothctl connect` 报 `BREDR.ProfileUnavailable` / `No more profiles` | **正常现象**。BlueZ 没有 SPP 守护进程，它只负责建 ACL 链路；SPP 数据要靠 RFCOMM socket。`bluetoothctl info <MAC>` 里 `Connected: yes` 就说明链路已通，直接用本目录脚本 |
| 日志里 `[L]` 在 `sr` 处截断，后面紧跟 `[A]` | 固件 `HAL_UART_Transmit` 超时 50ms，而 9600 下 60 字节的 `[L]` 行要约 62ms，每行都在第 50 字节被切断。固件已改为 `BLE_TX_TIMEOUT_MS = 250ms`（构建标记 j→k），旧固件需重刷 |
| 日志里 `[LinePD] line lost, s` 被截断、和 `[L]` 行粘在一起 | 旧固件的控制任务打印：27 字节要 28 ms，而发送超时只有 20 ms，且每 40 ms 打一次会把控制周期吃掉大半。l 版已改成 `[PD] lost`（每次丢线只打一次）和 40 ms 超时，重刷即可 |
| 连不上 … host is down | HC-05 没上电 / 被手机占着 / 没配对 |
| `Permission denied` | 少数内核上 RFCOMM socket 需要特权：`sudo python3 tools/...`，或改用 USB-TTL |
| 通道试完都失败 | HC-05 的 SPP 通道一般就是 1；若改过，用 `--channel 2` 指定 |
| 数据乱码 / 半行 | 波特率不对（USART2 是 9600）；或 EN 被拉高进了 AT 模式 |
| 只收到 `[VER]` 就没了 | 蓝牙连接建立了但车没进入循迹模式，`[L]` 行只在 LINE/SPD 模式输出 |
| CSV 里没有数据，log 有 | 协议格式对不上：先跑 `python3 tools/serial_proto.py` 自测，再对照车端 `Ble_task.c` 的格式字符串 |
