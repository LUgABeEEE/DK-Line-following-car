# DK 循迹小车

> 面向「电子科技协会 · 校庆智能小车竞速赛」的循迹小车工程
> STM32F103C8T6 + FreeRTOS + TB6612 + 8 路灰度，串级双环（循迹 PD + 编码器速度 PID）

[![build](https://github.com/LUgABeEEE/DK-Line-following-car/actions/workflows/build.yml/badge.svg)](https://github.com/LUgABeEEE/DK-Line-following-car/actions/workflows/build.yml)

---

## 1. 硬件构成

| 项目 | 型号 / 规格 | 说明 |
|---|---|---|
| 主控 | STM32F103C8T6 | 72 MHz，64 KB Flash / 20 KB RAM，HSE 8 MHz |
| 底板 | 小车控制底板 V1.0（嘉立创 EDA） | 原理图见 [`docs/小车控制底板V1.0-原理图.pdf`](docs/小车控制底板V1.0-原理图.pdf) |
| 电机驱动 | TB6612FNG 双路 H 桥模块 | 底板 `M-INPUT` 7P 接口直插 |
| 电机 | N20 减速电机（**6 V 额定**）+ 霍尔编码器 | 实测轮径 48 mm、约 1300 计数/轮转（PSC=0）；2S 7.4 V 供电，占空比 100 % ≈ 7.4 V（超压 23 %），见第 6 节 |
| 循迹传感器 | 8 路灰度，I2C 地址 `0x5D` | 实测探头间距 7 mm，I2C1 400 kHz |
| 显示 | 0.96" OLED（SSD1306，128×64） | I2C2，400 kHz |
| 无线 | HC-05 蓝牙透传 | USART2，9600 8N1 |
| 电源 | 2S 7.4 V 锂电 + 2S 保护板 | DCDC 7.4→5 V（LMR14030）→ LDO 5→3.3 V（TLV75733） |

> ⚠️ **USB 供电时禁止接电机**：Q1 体二极管会把 VBAT 反灌到 5 V 轨，可能损坏 USB 口/电脑。调试逻辑先拔电机。

---

## 2. 软件架构

```
Core/            CubeMX 生成（时钟、GPIO、I2C、TIM、UART、USB；I2C1 400 kHz + DMA 收）
Drivers/BSD/     板级驱动（手写）
  PWM.c          TB6612 方向 + PWM（含每轮方向系数）
  Encoder.c      正交编码器测速（20 ms 窗口 + 一阶低通 + 分轮修正）
  SpeedPID.c     内环：前馈 + PID（cm/s → 占空比）
  Gray.c         8 路灰度读取、加权质心位置解算、场景识别
  LinePD.c       外环：循迹 PD + 直角/分叉/全黑处理 + 丢线搜索
  Led/Key/Ble/Oled.c  外设驱动
USER/            FreeRTOS 任务层
  Control_task.c 实时控制任务（优先级 3）：20 ms 内环 / 40 ms 外环 / 20 ms HMI，含目标速度斜坡
  Ble_task.c     蓝牙任务（优先级 2）：急停命令 + 灰度/PID 遥测
  Oled_task.c    OLED 显示任务（优先级 1）：状态页与按键调参
FreeRTOS/       内核（Cortex-M3 端口，heap_4）
```

控制链路：

```
灰度场景+位置 ──► 外环 PD(40 ms) ──目标速度 cm/s──► 内环速度 PID(20 ms) ──占空比──► TB6612 ──► N20
   ▲                                                                                    │
   └────────────────────── 编码器测速 ◄────────────────────────────────────────────────┘
```

设计要点：

- 内环带**前馈** `duty = KFF × v_target`，PID 只修小偏差；没有前馈时直道输出全靠积分攒，必然饱和、一转弯就失控。
- 内环在**目标速度符号翻转**的那一拍清零积分。否则前进时攒下的正积分会把反转指令抵消掉，
  "原地右转"实际变成"左轮前进、右轮不动"，车只会平移；实车日志里 B 轮
  被要求 -24 cm/s 时实际只有 -3 cm/s，根源就是积分还背着 +22。
- 外环**刻意不含积分**：线本身就是绝对位置基准，不存在需要积分消除的静差。
- 外环周期必须是内环的 2 倍以上（现为 40 ms vs 20 ms），`LinePD_Update()` 内部会检查周期比，不满足直接拒绝运行。

已实现的赛道能力：

- **场景识别与防抖**（`Gray.c`）：正常单线 / 横向线或终点（全黑）/ 左直角 / 右直角 / 分叉 / 丢线，输出防抖后的 `scene` 供外环切换策略；直角与锐角的判断冲突处理见 2.1。
- **丢线搜索**（`LinePD.c`）：锁存最后偏离方向，超时（1.2 s）后双向扫描找回，搜索速度 24 cm/s。
- **目标速度斜坡**：内环 40 cm/s²、外环 50 cm/s²，避免起步时两轮不同步造成原地打转。
- **分轮标定**：`ENCODER_A_FIX / B_FIX` 修正两轮计数差异（解决直行跑偏），支持运行时调。
- **蓝牙遥测**：`X/S/Q` 急停、`I` 扫 I2C 总线，周期输出灰度 8 路模拟值/阈值与内环 PID 三列。

### 2.1 直角 / 锐角判断冲突是怎么解决的

8 路灰度只有"某一帧的通道分布"这一种信息，而 SIDE（半侧宽黑带）和
FORK（两簇）**不是互斥的物理类别**：车走到锐角弯顶点附近时，同一条黑线会
先被探头切成两段（FORK），再随车头前压并成一片（SIDE），偶尔还会全亮（ALL）。
如果逐帧直接分类，`scene` 就会在 SIDE 和 FORK 之间来回跳 ——
表现为"一会儿按直角处理、一会儿按锐角处理"，转向和搜索方向跟着乱翻。

现在分三层处理：

1. **传感器层（`Gray.c`）**
   - 相邻簇间隔 ≤ 1 路时合并成一簇：一条线中间偶尔断一路，不再被误判成分叉。
   - 两级时间防抖：先确认"场景族"（丢线 / 单线 / 角点），再确认角点内部的子类型。
     族确认 2 帧（40 ms）、角点→单线释放 3 帧（60 ms）、丢线立即生效。
     关键点：**角点的进入只看"是不是角点"**，所以 SIDE / FORK 交替出现时
     也能在 2 帧内确认进入；而子类型要连续 3 帧相同才更新，
     交替抖动时 `sc` 保持不变。
2. **决策层（`LinePD.c`）**
   - SIDE 和 FORK 统一成**角点事件**：事件一旦成立就锁存方向
     （SIDE 用"哪半边全黑"，FORK 用"另一条腿在哪一侧"），
     之后无论分类怎么变都不重算；连续 3 拍（120 ms）回到单线才释放。
   - 方向证据**分级**：SIDE 是最直接的观测（哪半边全黑 = 往哪边转），
     可以纠正之前由 FORK 得到的方向；反过来不行。
     实车日志里出现过 `SIDE_L` 却锁存 `cd=-1` 的情况（先来了一帧 FORK 把方向占住），
     导致丢线后往反方向搜索 4~6 秒，就是靠这条规则修掉的。
   - **出弯方向记忆**：角点事件释放后 `corner_dir` 不立刻清空，再保留 15 拍（600 ms）。
     实车锐角弯常见时序是「SIDE/FORK → 短暂 LINE → 立刻丢线」，
     如果方向在丢线前就被清掉，搜索只能退回默认方向盲扫。
   - **锐角弯"定角转弯"**：如果角点事件里出现过锐角证据（FORK 双簇 / ALL 全黑），
     丢线后先闭眼朝锁存方向**转到目标角度**（`LINE_ACUTE_TURN_DEG`，默认 115°），
     这段时间看到线也不认。原因：V 形顶点处探头扫过"刚走过的那条腿"只有几十毫秒，
     按 2 拍确认就会误判"找回来了"，然后顺着旧线往回跑。
   - **角度从哪来**：没有陀螺仪，用两个编码器做差分里程计
     `yaw += (v_right − v_left) / 轮距 × dt`（dt 用 `HAL_GetTick` 的真实间隔）。
     纯原地转时约 172°/s，115° 约 0.67 s。打滑时轮子转得比车身多，编码器算出的角度会偏大、物理转角偏小，所以另设
     `LINE_ACUTE_COMMIT_TIMEOUT_MS`（1.5 s）兜底，不会无限转。
     等 MPU6050 到货，只需把 `yaw` 的来源换成陀螺仪 Z 轴积分，上层逻辑不变。
   - **搜索方向只信锁存方向**：不再用实测位置去"纠正"它。
     车转过头之后实测位置会指回旧腿，按它修正等于主动把车送回走过的赛道
     （日志 17:36:54：`cd=+1`，丢线后却向右搜）。
   - 角点事件期间放宽差速限幅（×1.35）；遇到 ALL（全黑，质心必然落在正中）
     时改用锁存方向满舵，避免"直直地冲出弯心"。
3. **搜索层（`LinePD.c`）**
   - 角点里丢线时，搜索方向直接用锁存方向，而不是被过冲污染的"最后位置"。

**怎么验证**：看串口 `[L]` 行新增的两列 ——
`sr` 是**未防抖**的瞬时场景，`sc` 是防抖后的稳定场景，`cs` 表示是否处于角点事件。
锐角弯里 `sr` 应该在 3/4/5 之间乱跳，而 `sc` 稳定、`cs` 进弯置 1、出弯后回 0。
如果 `sc` 仍跟着 `sr` 跳，把 `Gray.h` 里的 `GRAY_SCENE_*_SAMPLES` 调大。

**调参入口**（`LinePD.h`）：`LINE_CORNER_DIFF_BOOST`（角点差速倍数）、
`LINE_CORNER_RELEASE_CYCLES`（出弯确认拍数）、`LINE_ALL_AS_CORNER`（全黑满舵开关）。

---

## 3. 引脚映射

| 功能 | 信号 | 引脚 | 备注 |
|---|---|---|---|
| 电机 A | PWMA | PA9 | TIM1_CH2，10 kHz |
| 电机 A | AIN1 / AIN2 | PB13 / PB12 | 方向，`IN1=IN2` 为刹车 |
| 电机 B | PWMB | PA8 | TIM1_CH1，10 kHz |
| 电机 B | BIN1 / BIN2 | PB14 / PB15 | 方向 |
| 驱动使能 | STBY | PA10 | 高电平工作，必须拉高 |
| 编码器 E1 | E1A / E1B | PB6 / PB7 | TIM4 编码器模式（TI12） |
| 编码器 E2 | E2A / E2B | PA6 / PA7 | TIM3 编码器模式（TI12） |
| 灰度 | GS_SCL / GS_SDA | PB8 / PB9 | I2C1，100 kHz，地址 0x5D |
| OLED | OLED_SCL / OLED_SDA | PB10 / PB11 | I2C2，400 kHz |
| 蓝牙 | BLE_TX / BLE_RX | PA2 / PA3 | USART2，9600 |
| 蓝牙 | BLE_EN / BLE_STATE | PB3 / PA1 | EN 拉低 = 透传模式 |
| 按键 | KEY1 / KEY2 / KEY3 | PB0 / PA4 / PA5 | 上拉输入，低有效 |
| LED | LED1 / LED2 | PB5 / PB4 | 低电平点亮 |
| 蜂鸣器 | BUZZ | PB1 | |
| 调试 | SWDIO / SWCLK | PA13 / PA14 | SWD 下载 |
| 预留 | ADC | PA0 | ADC1_IN0，当前未使用 |

PWM 参数：TIM1 `PSC=71, ARR=99` → 72 MHz / 72 / 100 = **10 kHz**。
改 ARR 必须同步改 `PWM.h` 的 `MOTOR_PWM_PERIOD`。

---

## 4. 编译与烧录

依赖：CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`（建议用 CLion 自带 toolchain 或 xPack 发行版），
且 `arm-none-eabi-gcc` 必须在 `PATH` 里（`cmake/gcc-arm-none-eabi.cmake` 只按前缀找编译器，不写死路径）。
STM32CubeCLT 用户可临时 `export PATH=$HOME/st/stm32cubeclt_1.21.0/bin:$PATH`。

**烧录用 Release**（体积最小，`-Os`）：

```bash
cmake --preset Release
cmake --build --preset Release
# 产物：build/Release/linefllowing_car.elf —— 用 ST-Link / DAP-Link 经 SWD 烧录
# 实测占用：FLASH 43 736 B (66.7 %)、RAM 17 344 B (84.7 %)
```

**需要单步调试时用 Debug**：

```bash
cmake --preset Debug
cmake --build --preset Debug
# 实测占用：FLASH 49 368 B (75.3 %)、RAM 17 360 B (84.8 %)
```

> ⚠️ Debug 的优化级别已从 CubeMX 默认的 `-O0` 改为 `-Og`（见 `cmake/gcc-arm-none-eabi.cmake`）。
> 用 `-O0` 时镜像约 73 KB，**超出 64 KB Flash**，链接会报
> `region 'FLASH' overflowed by 7464 bytes`；`-Og` 可通过。
> 若调试时要完整观察某个函数里的局部变量，把该文件的优化关掉单独编即可。

> 本仓库不含 `build/` 产物，首次编译需自行 configure。

### 4.1 CI（GitHub Actions）

`.github/workflows/build.yml`：推送 `main` / `simple`、PR 或手动触发时，
把 Debug 与 Release 两种预设都编一遍（apt 装 `gcc-arm-none-eabi` + cmake + ninja），
产出 `.hex` / `.bin` 并上传为 `firmware-Debug` / `firmware-Release`；同时用
`python -m compileall tools` 检查遥测脚本语法。

不装本地工具链也能拿到可烧录文件：Actions → 某次运行 → **Artifacts**。

---

## 5. 调试与标定流程（务必按顺序）

**先用安全模式**：`USER/Inc/Control_task.h` 里 `CTRL_TASK_ENABLE_MOTION = 0`，控制任务每个周期把目标锁 0，车不会动。

### 5.1 三步硬件自检（`Core/Src/main.c` 的 USER CODE 2 段里逐个取消注释）

1. `Motor_TestSequence()` —— 正转缓升 / 滑行 / 反转 / 刹车 / 换 B 轮，确认接线与方向系数
2. `Encoder_TestSequence()` —— 手拨轮子看计数，确认 A/B 相与方向
3. `Speed_PID_TestSequence()` —— 阶跃响应，串口 **9600** 打印「目标 / 实测 / 输出」（USART2 接的是 HC-05，各头文件注释里写的 115200 是早期旧值）

其余可选项：`Gray_TestSequence()`、`Led_TestSequence()`、`Key_TestSequence()`、`BLE_Verify()`、`OLED_TestSequence()`。

### 5.2 上车前钉死符号

`LinePD_VerifySigns(&g_line_pd, &g_gray, &hi2c1, 0)` —— 逐个确认「误差为正 = 线偏左」「差速为正 = 左转」「正速度为前进」。这一步偷懒，后面调参全是玄学。

### 5.3 内环（先 KFF，再 P，再 I）

| 步骤 | 做法 | 判据 |
|---|---|---|
| 前馈 | Kp=Ki=Kd=0，只留 KFF | 实测速度接近目标；偏小就按 `目标/实测` 比例放大 KFF |
| 比例 | 加 Kp | 阶跃能快速跟上、超调 < 20 % |
| 积分 | 加 Ki | 稳态无静差，不振荡 |
| 微分 | Kd 一般保持 0 | 电机惯性和编码器滤波已提供阻尼 |

参数在 `SpeedPID.h` 的 `SPEED_PID_DFLT_*`，也可运行时按键/蓝牙改（`Speed_PID_SetGain()`）。

### 5.4 外环（循迹 PD）

`LinePD.h`：`KP=20.0`、`KD=1.0`、`BASE_SPEED=30 cm/s`、`LINE_DIFF_LIMIT=35`、`LINE_SPEED_LIMIT=75`、`LINE_CURVE_SLOW_MIN=0.40`，外环周期 **40 ms**。
先把基础速度降到 20 cm/s 调 Kp（能跟线不画龙），再加 Kd 抑制过冲；`LINE_CURVE_SLOW_MIN` 控制急弯减速深度。
`LINE_STOP_IF_LINE_LOST=1` 是首次调试的正确选择——丢线即停，车不会冲出赛道；调顺之后再开 `LINE_SEARCH_ENABLE`，让丢线后按锁存方向双向扫描找回。

### 5.5 自动采集遥测（给 AI / 脚本分析用）

HC-05 是 SPP 透传，PC 端不用装 `rfcomm` 也能直接连。仓库里带了一套纯标准库的小工具：

```bash
# 1) 先在 bluetoothctl 里 pair/trust 一次（PIN 默认 1234），不需要 rfcomm bind

# 2) 采集：直接连 RFCOMM，默认依次尝试通道 1~5
python3 tools/bt_serial_capture.py --mac AA:BB:CC:DD:EE:FF --duration 60

# 或者连已有的 tty（USB-TTL / 系统提供的 /dev/rfcomm0）
python3 tools/bt_serial_capture.py --port /dev/ttyUSB0 --baud 9600

# 3) 分析
python3 tools/serial_summary.py logs/serial_YYYYmmdd_HHMMSS.loop.csv
```

输出在 `logs/`（已 gitignore）：原始 `.log` + `loop/speed/gray/scene` 四个 CSV。
`serial_summary.py` 会直接给出 `sr`（瞬时场景）和 `sc`（防抖后场景）的变化次数对比、
角点事件次数/时长/锁存方向，用来验证 2.1 节的防抖是否生效。

> 遥测节拍：BLE 任务 25 ms、PID 输出 160 ms，`[L]` 实测约 5 Hz
> （受 9600 波特率限制，60 字节一行需要 62 ms）。
> 控制任务里的调试打印已经压到最短、且只在状态变化时发一次 ——
> 以前 `[LinePD] line lost, stop` 每 40 ms 打一次（27 字节要 28 ms），
> 会把控制周期吃掉一大半，还会被 20 ms 超时截断成 `[LinePD] line lost, s`。
详细用法和排错见 [`tools/README.md`](tools/README.md)。

### 5.6 跑圈结果打标（KEY1 = 失败，KEY2 = 成功）

循迹运行时，OLED 停在 **MODE** 页会显示 `K1 FAIL K2 OK`：

| 操作 | 效果 |
|---|---|
| 短按 **KEY2** | 串口打印 `[T] OK #n t<ms>`，表示刚跑完的这段是**成功**的 |
| 短按 **KEY1** | 串口打印 `[T] FAIL #n t<ms>`，表示这段**失败**了 |
| 长按 **KEY2** | 急停（原有行为，**不会**顺带打标） |
| KEY1/KEY2 | 在其他页面仍是原来的调参功能 |

设计要点：

- **模式在循迹运行中锁定**：以前在 MODE 页按 KEY1/KEY2 会直接把模式切走（车还在跑时
  一不小心就切到 SPD/STOP），现在锁定；要退出循迹先长按 KEY2 停车。
- **只在松手时判定短按**：长按 KEY2 是急停，不能同时冒出一个假的 `OK`。
- **按下瞬间的状态决定要不要打标**：从 STOP/SPD 切进循迹的那一次按键不会误打标
  （否则一进循迹就凭空出现一个 `[T] OK`）。
- **打印统一在蓝牙任务里做**：控制任务只把 1/0 塞进队列，避免两条任务抢同一个串口
  把正在发送的 `[L]` 行截断。

标记的含义是"**这一段跑完了**"：一段数据 = 上一个标记之后到本标记之前。
分析脚本按 `[T]` 行切段：

```bash
python3 tools/tag_report.py logs/serial_20261026_*.log
# [FAIL #1] 10:00:00~10:00:09  9.0s  n=9  角点 0 丢线 0 ...
# [OK   #2] 10:00:09~10:00:19 10.0s  n=10 角点 1 丢线 3 ...
```

输出每段的时长、模式分布、场景分布、角点事件数、丢线样本数、`|e|` 中位和里程增量，
成功/失败一目了然；加 `--csv` 还能导出分段表。

---

## 6. N20 实测标定结果（6 V 额定 / 7.4 V 电池 / 自带编码器）

### 6.1 电压：占空比等于平均电压

TB6612 输出的是 PWM 平均电压 = 占空比 × VBAT：

| 占空比 | 平均电压 | 含义 |
|---|---|---|
| 81 % | 6.0 V | 正好等于电机额定 |
| 100 % | 7.4 V | 超压约 23 % |

竞速单次只跑几十秒，用满 100 % 能多拿约 1/4 转速，通常不用怕；训练时若发现电机发烫、换向火花明显，把 `PWM.h` 的 `MOTOR_DUTY_MAX` 改成 **81** 让软件限压。
电流方面 TB6612 单路 1.2 A 连续 / 3.2 A 峰值，N20 堵转 0.8~1.5 A（7.4 V 下更高），够用，但别长时间堵转。

### 6.2 机械参数（已按实车标定）

| 宏 | 值 | 依据 |
|---|---|---|
| `ENCODER_PPR` | 13 | 电机铭牌线数 |
| `ENCODER_EDGE_MULTIPLIER` | 4.0 | TIM 工作在 TI12 编码器模式，硬件本来就是四倍频 |
| `ENCODER_GEAR_RATIO` | 25.0 | 名义值：由 PSC=49 时代的 26 计数/轮转 × 50 反推 1300 / (13×4) |
| `ENCODER_WHEEL_DIAMETER_MM` | 48.0 | 卡尺实测 |
| `ENCODER_CALIBRATION` | 0.97 | 两次 100 cm 标定取平均（直线 0.943 / 循迹 1.002）；PSC 归零后比例不变，无需重标 |
| `ENCODER_A_FIX` / `B_FIX` | 0.995 / 1.0 | 长直道反复微调到走直 |

折算：**每轮转 ≈ 1300 计数、约 0.1125 mm/计数**（PSC=0）。

> ⚠️ `GEAR_RATIO = 25` 只是让「13 × 4 × 25 = 1300」这个乘积成立的**名义值**，真实减速箱大概率在 1:25~1:30 之间，5~10 % 的差由 `ENCODER_CALIBRATION` 吸收。它不影响循迹（循迹靠灰度位置），只影响速度读数的绝对准确度。

### 6.3 编码器预分频：49 → 0（已改，需实车复核）

编码器模式下 `PSC` 是对**编码器边沿**分频的，不是对 72 MHz 内部时钟分频。之前两路都是 49，等于每 50 个边沿才计 1 个数——两路一致（不会左右不对称），但分辨率白扔了 50 倍：

| | PSC = 49（旧） | PSC = 0（现） |
|---|---|---|
| 计数/轮转 | ~27 | **~1300** |
| mm/计数 | 5.63 | **0.1125** |
| 20 ms 窗口下 1 个计数 | ≈ 28 cm/s | ≈ 0.56 cm/s |
| 30 cm/s 时每 20 ms 计数 | ~1 | **~53** |

改法：`main.c` 的 `MX_TIM3_Init/MX_TIM4_Init` 与 `.ioc` 的 `TIM3/TIM4.Prescaler` 一起改成 0（已同步改好），换算侧把 `EDGE_MULTIPLIER` 恢复成 4、`GEAR_RATIO` 设为 25。

**烧录后请按这个顺序复核**（底座架空，车不动）：

1. 手转车轮**正好一圈**，看原始计数增量是不是 **~1300**（原来是 ~26）。
   - 只有 ~650 → A/B 相有一路没接进来，检查编码器线序；
   - 还是 ~26 → PSC 没真正写进去，检查 `.ioc` 是否被 CubeMX 覆盖回去。
2. 跑 100 cm 直行，看里程增量是不是 **~8900**（对应 0.1125 mm/计数）。
   - 偏差在 ±5 % 内属正常（轮径公差 + 编码器加工误差），需要更准就按 6.4 节微调 `CALIBRATION`。
3. 速度读数应当与改动前**基本一致**（同一目标值跑出来的 cm/s 数值不该差 50 倍）——如果差了一个数量级，说明换算没跟上，立刻停车检查。

### 6.4 换轮子 / 换电机后怎么重标

跑一段已知距离 `L`，记录里程增量 `Delta`：

```
ENCODER_CALIBRATION_new = ENCODER_CALIBRATION_old × L / Delta
```

分轮跑偏仍用 `ENCODER_A_FIX / B_FIX` 微调（每次 0.005 量级）：车往左偏 → 减小 `A_FIX`。

### 6.5 长时间运行的计数 / 内存审计

"跑久了开始抖"这类问题，先排除计数回绕和内存耗尽。逐项核对结果：

| 量 | 类型 | 实际上限 | 结论 |
|---|---|---|---|
| 编码器单拍增量 | `int16_t` | 20 ms 窗口内约 350 计数（200 cm/s），上限 32767 | 安全 |
| 累计里程 `s_distance_cm` | `float` | 1 km 处精度 0.008 cm、100 km 处 0.8 cm | 只影响显示 |
| 偏航角 `yaw_deg` | `float` | 5000° 处精度 0.0005°、10⁶° 处 0.06° | 单次转弯够用 |
| 内环积分 | `float` | 限幅 ±40（0.4 × 100） | 不会爆炸 |
| `search_ms` | `uint16_t` | 饱和在 65535 ms（65 s） | 安全（饱和后即停车） |
| `lost_count` / `corner_*` / `recover_count` | `uint8_t` | 全部有 `< 0xFF` 限幅 | 安全 |
| **`fail_count`（I2C 连续失败）** | 原 `uint8_t` | 无上限 → **连续失败 256 次回绕成 0** | ❌ 已修：`uint16_t` + 限幅 |
| `scene_flips` | `uint16_t` | 饱和（有保护） | 只影响统计 |
| `total_reads` / `total_fails` / `s_rx_total` | `uint32_t` | 50 Hz 下约 2.7 年；600 B/s 下约 79 天 | 安全 |
| 任务栈 | — | BLE 1280 B（帧链约 700 B）、Ctrl 1280 B（约 350 B）、OLED 1792 B（约 350 B） | 有余量，但原来溢出检测是关的 |
| 堆 | — | 10 KB，任务+信号量约 6 KB，剩约 4 KB | 够用 |

已修/已加：

1. **`fail_count` 位宽**：以前连续失败 256 次会回绕成 0，`fail_count >= GRAY_FAIL_LIMIT`
   又变 false —— 上层以为总线恢复了，继续拿陈旧数据开车。现在是 `uint16_t` 并限幅。
2. **打开 FreeRTOS 保护**：`configCHECK_FOR_STACK_OVERFLOW = 2`、
   `configUSE_MALLOC_FAILED_HOOK = 1`，并实现钩子（`USER/Src/app_hooks.c`）：
   - **LED1 常亮 + 车停住** = 栈溢出
   - **LED2 常亮 + 车停住** = 堆分配失败
   以前这两种情况是静默的内存破坏，表现就是"跑一会儿就抖/乱走"。
3. **新增运行期诊断行 `[M]`**（每 10 s 一行）：

   ```text
   [M] heap4120 stkC980 stkO1520 stkB760 i2c14/18000
   ```

   - `heap`：剩余堆字节。本工程运行期不再动态分配，**它应该是常数**；持续下降 = 泄漏；
   - `stkC/O/B`：三个任务栈的**历史最小剩余字节**（`uxTaskGetStackHighWaterMark`），
     低于 200 就该加大对应任务的 `*_STACK_WORDS`；
   - `i2c`：灰度传感器累计失败/总读取，长期错误率升高 = 接线/上拉问题。

---

## 7. 已知问题 / TODO

- [ ] **等待实车复核（PSC 49→0 之后）**：手转一圈应为 ~1300 计数、跑 100 cm 应为 ~8900 计数，速度读数应与改动前基本一致；若只有 ~650 计数，说明 A/B 相缺一路。复核数据回来后可能需要微调 `ENCODER_GEAR_RATIO`。
- [ ] **内环 KFF 未单独标定**：仍是默认 1.0。
      2026-10-25 的实车日志（`logs/serial_20261025_172300.log`）给出反证：
      目标 15~35 cm/s 区间里，**占空比/实测速度** 是 A 轮 1.35、B 轮 1.87，
      即 30 cm/s 实际需要约 40 %（A）/ 56 %（B）占空比，而 KFF=1.0 只给 30 %，
      差值全靠积分慢慢补 —— 追不上快速变化的差速目标。
      表现为 `m` 长期比 `t` 低 2~7 cm/s，而且 B 轮输出经常打到 70~100 % 仍不达标。
      先按 5.3 节把 KFF 调到 1.4~1.6，再核对两轮 `d` 增量是否一致
      （差得多就查 B 轮机械阻力 / 用 ENCA 平衡）。
- [ ] **外环参数绑定当前赛道**：`KP=20 / KD=1.0 / base=30` 是在现有场地调的，换赛道或换电池电压后需要重调。
      同一份日志里，循迹段 `steer` 符号翻转占 58 %（几乎每拍反向）、`|steer|≥30` 占 14 %，
      `|e|` 中位 0.50 cm / p90 1.90 cm —— 典型画龙。
      建议先降到 `LBAS=20 / LKP=12 / LKD=2.5` 跑顺，再逐步加回来。
- [ ] **角点防抖参数待实车确认**：`GRAY_SCENE_*_SAMPLES`（2/3/3/1）和
      `LINE_CORNER_RELEASE_CYCLES`（3 拍）是按 20 ms 采样 + 40 ms 控制律估的。
      跑锐角弯时看 `[L]` 行的 `sr/sc/cs`：目标是 `sr` 乱跳而 `sc` 稳定；
      如果入弯偏晚（车已经压到弯心才识别），把 `GRAY_SCENE_CONFIRM_SAMPLES` 降到 1；
      如果出弯后还长时间保持角点态（`cs=1` 拖太久），把 `LINE_CORNER_RELEASE_CYCLES` 降到 2。
- [ ] **角点差速倍数未实车验证**：`LINE_CORNER_DIFF_BOOST = 1.35` 沿用原 SIDE 的经验值，
      锐角弯如果甩尾就降到 1.2，如果转不过来就升到 1.5。
- [x] **`LINE_ACUTE_TURN_DEG` 首次标定**：2026-10-25 18:36~18:40 的日志显示，
      能顺利找回线的 V 弯 y 增量集中在 **130~145°**，失败的样本是 180~400°
      （115° 转完还没对上出弯腿，搜索继续扫过头）。默认值已从 115° 提到 **140°**。
- [ ] **继续微调 `LINE_ACUTE_TURN_DEG`**：现在 `[PD] ok xxxms y±deg` 会直接打印
      每次承诺转向实际转过的角度：
      反复出现"y 到了 140 还找不到线" -> 加到 150；
      出现"转过头、反向扫很久" -> 减到 130。
      打滑时轮子转得比车身多（编码器角度偏大、物理转角偏小），要往大调；
      换新轮胎/擦干净纸面之后可能又要往小调。
- [ ] **蓝牙只能急停 + 遥测**：无线改参数没做，调参仍靠按键 + OLED。

---

## 8. 目录速查

```
Core/             CubeMX 生成代码（main.c 里有全部自检入口）
Drivers/BSD/      手写板级驱动
  Inc/ PWM.h Encoder.h SpeedPID.h Gray.h LinePD.h Led.h Key.h Ble.h Oled.h OledFont.h
  Src/ 对应实现
USER/             FreeRTOS 任务
cmake/            gcc-arm-none-eabi 工具链文件 + CubeMX 子工程
docs/             原理图
tools/            HC-05 遥测采集/解析/统计脚本（见 tools/README.md）
linefllowing_car.ioc   CubeMX 工程（改外设从这里进）
```

> 注：工程名与目录沿用 CubeMX 生成时的 `linefllowing` 拼写，重命名会牵动 CMake、启动文件与 `.ioc`，比赛前不建议动。
