# DK 循迹小车

> 面向「电子科技协会 · 校庆智能小车竞速赛」的循迹小车工程
> STM32F103C8T6 + FreeRTOS + TB6612 + 8 路灰度，串级双环（循迹 PD + 编码器速度 PID）

---

## 1. 硬件构成

| 项目 | 型号 / 规格 | 说明 |
|---|---|---|
| 主控 | STM32F103C8T6 | 72 MHz，64 KB Flash / 20 KB RAM，HSE 8 MHz |
| 底板 | 小车控制底板 V1.0（嘉立创 EDA） | 原理图见 [`docs/小车控制底板V1.0-原理图.pdf`](docs/小车控制底板V1.0-原理图.pdf) |
| 电机驱动 | TB6612FNG 双路 H 桥模块 | 底板 `M-INPUT` 7P 接口直插 |
| 电机 | N20 减速电机（**6 V 额定**）+ 霍尔编码器 | 实测轮径 48 mm、约 27 计数/轮转；2S 7.4 V 供电，占空比 100 % ≈ 7.4 V（超压 23 %），见第 6 节 |
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
- 外环**刻意不含积分**：线本身就是绝对位置基准，不存在需要积分消除的静差。
- 外环周期必须是内环的 2 倍以上（现为 40 ms vs 20 ms），`LinePD_Update()` 内部会检查周期比，不满足直接拒绝运行。

已实现的赛道能力：

- **场景识别**（`Gray.c`）：正常单线 / 横向线或终点（全黑）/ 左直角 / 右直角 / 分叉 / 丢线，输出 `scene` 供外环切换策略。
- **丢线搜索**（`LinePD.c`）：锁存最后偏离方向，超时（1.2 s）后双向扫描找回，搜索速度 24 cm/s。
- **目标速度斜坡**：内环 40 cm/s²、外环 50 cm/s²，避免起步时两轮不同步造成原地打转。
- **分轮标定**：`ENCODER_A_FIX / B_FIX` 修正两轮计数差异（解决直行跑偏），支持运行时调。
- **蓝牙遥测**：`X/S/Q` 急停、`I` 扫 I2C 总线，周期输出灰度 8 路模拟值/阈值与内环 PID 三列。

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

依赖：CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`（建议用 CLion 自带 toolchain 或 xPack 发行版）。

```bash
cmake --preset Debug
cmake --build --preset Debug
# 产物：build/Debug/linefllowing_car.elf —— 用 ST-Link / DAP-Link 经 SWD 烧录
```

Release 构建把 `--preset Debug` 换成 `Release` 即可。

> 本仓库不含 `build/` 产物，首次编译需自行 configure。

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
| `ENCODER_EDGE_MULTIPLIER` | 2.0 | 100 cm 只走 183 个计数，反推约 26 计数/轮转 = 13 × 2 |
| `ENCODER_GEAR_RATIO` | 1.0 | 数据推翻了「30:1」：27.6 % 占空比下若真按 30:1 换算只有 0.58 cm/s，物理上不可能 |
| `ENCODER_WHEEL_DIAMETER_MM` | 48.0 | 卡尺实测 |
| `ENCODER_CALIBRATION` | 0.97 | 两次 100 cm 标定取平均（直线 0.943 / 循迹 1.002） |
| `ENCODER_A_FIX` / `B_FIX` | 0.995 / 1.0 | 长直道反复微调到走直 |

折算：**约 5.63 mm/计数、约 26~27 计数/轮转**。
所以速度读数曾经虚高：轮径从 65 mm 改到 48 mm 后同一转速的读数变成原来的 74 %——改之前显示 30 cm/s 时车其实只有约 22 cm/s。

### 6.3 一个还有余量的地方：编码器预分频

`TIM3` / `TIM4` 现在都是 `PSC = 49`。两路**已经一致**（这是先修好的关键问题，之前一路 0 一路 49，左右轮速度差 50 倍），但代价是：编码器模式下 PSC 直接对边沿分频，49 意味着**每 50 个边沿才计 1 个数**。

现在的后果：

- 20 ms 采样窗口下，1 个计数 ≈ **28 cm/s**；
- 30 cm/s 时每窗口只有 1 个计数，速度反馈基本靠低通滤波「平均」出中间值；
- 低速（<15 cm/s）时速度读数会明显跳变，这是内环调不顺的主因之一。

如果把两路 PSC 都改成 **0**：

| | PSC = 49（现状） | PSC = 0 |
|---|---|---|
| 计数/轮转 | ~27 | ~1380 |
| mm/计数 | 5.63 | 0.11 |
| 30 cm/s 时每 20 ms 计数 | ~1 | ~53 |

代价是**所有换算系数要同步 ×50**，属于「改完必须重新标定」的改动，所以没有擅自动。要做的步骤：

1. CubeMX 里把 TIM3/TIM4 的 Prescaler 改成 0，重新生成（`main.c` 与 `.ioc` 一起变）
2. 跑 100 cm，记录 `d` 增量（应约为原来的 50 倍，≈ 9150）
3. 把 `ENCODER_EDGE_MULTIPLIER / GEAR_RATIO` 调到与新的 counts/轮转 对上
4. 按 6.4 节重算 `ENCODER_CALIBRATION`

### 6.4 换轮子 / 换电机后怎么重标

跑一段已知距离 `L`，记录里程增量 `Delta`：

```
ENCODER_CALIBRATION_new = ENCODER_CALIBRATION_old × L / Delta
```

分轮跑偏仍用 `ENCODER_A_FIX / B_FIX` 微调（每次 0.005 量级）：车往左偏 → 减小 `A_FIX`。

---

## 7. 已知问题 / TODO

- [ ] **编码器分辨率**：两路 `PSC=49` 一致但损失 50 倍分辨率（见 6.3），改 `PSC=0` 需连带重标定，待决定。
- [ ] **内环 KFF 未单独标定**：仍是默认 1.0。实测 30.9 cm/s 对应 27.6 % 占空比，量级基本吻合，但没有做过阶跃标定。
- [ ] **外环参数绑定当前赛道**：`KP=20 / KD=1.0 / base=30` 是在现有场地调的，换赛道或换电池电压后需要重调。
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
linefllowing_car.ioc   CubeMX 工程（改外设从这里进）
```

> 注：工程名与目录沿用 CubeMX 生成时的 `linefllowing` 拼写，重命名会牵动 CMake、启动文件与 `.ioc`，比赛前不建议动。
