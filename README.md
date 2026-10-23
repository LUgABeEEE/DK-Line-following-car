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
| 电机 | N20 减速电机（**6 V 额定**）+ 霍尔编码器 | 电池 2S 7.4 V，占空比 100 % ≈ 7.4 V（超压 23 %），见第 6 节 |
| 循迹传感器 | 8 路灰度，I2C 地址 `0x5D` | 探头间距 10 mm |
| 显示 | 0.96" OLED（SSD1306，128×64） | I2C2，400 kHz |
| 无线 | HC-05 蓝牙透传 | USART2，9600 8N1 |
| 电源 | 2S 7.4 V 锂电 + 2S 保护板 | DCDC 7.4→5 V（LMR14030）→ LDO 5→3.3 V（TLV75733） |

> ⚠️ **USB 供电时禁止接电机**：Q1 体二极管会把 VBAT 反灌到 5 V 轨，可能损坏 USB 口/电脑。调试逻辑先拔电机。

---

## 2. 软件架构

```
Core/            CubeMX 生成（时钟、GPIO、I2C、TIM、UART、USB）
Drivers/BSD/     板级驱动（手写）
  PWM.c          TB6612 方向 + PWM（含每轮方向系数）
  Encoder.c      正交编码器测速（20 ms 窗口 + 一阶低通）
  SpeedPID.c     内环：前馈 + PID（cm/s → 占空比）
  Gray.c         8 路灰度 I2C 读取与位置解算（加权质心 → cm 误差）
  LinePD.c       外环：循迹 PD（误差 cm → 左右轮目标速度）
  Led/Key/Ble/Oled.c  外设驱动
USER/            FreeRTOS 任务层
  Control_task.c 实时控制任务（优先级 3）：20 ms 内环 / 100 ms 外环 / 20 ms HMI
  Ble_task.c     蓝牙任务（优先级 2）：命令解析、OLED 调参联动
  Oled_task.c    OLED 显示任务（优先级 1）
FreeRTOS/       内核（Cortex-M3 端口，heap_4）
```

控制链路：

```
灰度位置 ──► 外环 PD(100 ms) ──目标速度 cm/s──► 内环速度 PID(20 ms) ──占空比──► TB6612 ──► N20
   ▲                                                                                    │
   └────────────────────── 编码器测速 ◄────────────────────────────────────────────────┘
```

设计要点：

- 内环带**前馈** `duty = KFF × v_target`，PID 只修小偏差；没有前馈时直道输出全靠积分攒，必然饱和、一转弯就失控。
- 外环**刻意不含积分**：线本身就是绝对位置基准，不存在需要积分消除的静差。
- 外环周期必须是内环的 3~5 倍，`LinePD_Update()` 内部会检查周期比，不满足直接拒绝运行。

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
3. `Speed_PID_TestSequence()` —— 阶跃响应，串口 115200 打印「目标 / 实测 / 输出」

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

`LinePD.h`：`LINE_PD_DFLT_KP=8.0`、`KD=3.0`、`BASE_SPEED=30 cm/s`、`LINE_DIFF_LIMIT=25`。
先把基础速度降到 20 cm/s 调 Kp（能跟线不画龙），再加 Kd 抑制过冲；`LINE_CURVE_SLOW_MIN` 控制急弯减速深度。
`LINE_STOP_IF_LINE_LOST=1` 是首次调试的正确选择——丢线即停，车不会冲出赛道。

---

## 6. N20 电机适配（6 V 额定 / 7.4 V 电池 / 自带编码器）

### 6.1 电压：占空比等于平均电压

TB6612 输出的是 PWM 平均电压 = 占空比 × VBAT：

| 占空比 | 平均电压 | 含义 |
|---|---|---|
| 81 % | 6.0 V | 正好等于电机额定 |
| 100 % | 7.4 V | 超压约 23 % |

竞速单次只跑几十秒，用满 100 % 能多拿约 1/4 转速，通常不用怕；训练时若发现电机发烫、换向火花明显，把 `PWM.h` 的 `MOTOR_DUTY_MAX` 改成 **81** 让软件限压。
电流方面 TB6612 单路 1.2 A 连续 / 3.2 A 峰值，N20 堵转 0.8~1.5 A（7.4 V 下更高），够用，但别长时间堵转。

### 6.2 机械参数：量「轮子转一圈的计数」

`Encoder.h` 已改成 **方式 A**：只填一个数 `ENCODER_COUNTS_PER_WHEEL_REV`，不用管线数和减速比。

标定方法（1 分钟）：

1. 车架空，串口跑 `Encoder_TestSequence()`
2. 在轮子上做个记号，手转**正好一圈**回到原点
3. 把读到的计数增量填进 `ENCODER_COUNTS_PER_WHEEL_REV`

参考量级（7 线编码器 × 4 倍频，43 mm 轮，counts/轮转 = 28 × 减速比）：

| 减速比 | counts/轮转 | mm/count | 7.4 V 空载转速\* | 最高车速\* | KFF 初值 | base_speed 建议 |
|---|---|---|---|---|---|---|
| 1:30 | 840 | 0.161 | ~490 rpm | ~111 cm/s | 0.9 | 60（受 `LINE_SPEED_LIMIT` 限制） |
| 1:50 | 1400 | 0.097 | ~296 rpm | ~67 cm/s | 1.5 | 43 |
| 1:100 | 2800 | 0.048 | ~148 rpm | ~33 cm/s | 3.0 | 22 |
| 1:150 | 4200 | 0.032 | ~99 rpm | ~22 cm/s | 4.5 | 14 |
| 1:210 | 5880 | 0.023 | ~70 rpm | ~16 cm/s | 6.3 | 10 |
| 1:298 | 8344 | 0.016 | ~49 rpm | ~11 cm/s | 9.0 | 7 |

\* 按 6 V 铭牌转速 × 7.4/6 估算，各厂家差异较大，**以实测为准**。

两条硬结论：

- **KFF 初值 ≈ 100 ÷ 最高车速**（实际再加 20 % 补摩擦与压降）。默认 `KFF=1.0` 是给 MG513 那种「30 cm/s 只需 30 % 占空比」的电机准备的，直接套到 N20 上，车会慢吞吞爬到积分攒够才动。
- **1:100 及更慢的减速比跑不到默认的 30 cm/s**。24 cm/s 就需要约 107 rpm，1:100 的 N20 在 7.4 V 下也才 148 rpm 空载、带载更低。目标超过电机能力时 PID 会长期饱和，必须把 `LinePD.h` 的 `LINE_PD_DFLT_BASE_SPEED` 降到表里的建议值。

### 6.3 其余三条

- **编码器**：已确认自带（两路正交霍尔），内环速度 PID 可用，无需降级为开环。
- **轮径**：`ENCODER_WHEEL_DIAMETER_MM` 填实测值（滚动一圈量直线距离 ÷ π 比卡尺量直径更准，含轮胎压缩）；跑过 2 秒直行后还能用 `ENCODER_CALIBRATION` 做比例修正。
- **方向系数**：换电机/重新接线后重跑 `Motor_TestSequence()`，必要时改 `PWM.h` 的 `MOTOR_A_DIR_SIGN / MOTOR_B_DIR_SIGN`（现有设置是 A 相取 -1）。

---

## 7. 已知问题 / TODO

- [ ] **TIM3 与 TIM4 的编码器预分频不一致**：`MX_TIM3_Init()` 里 `Prescaler = 49`，`MX_TIM4_Init()` 里 `Prescaler = 0`，而 `Encoder.h` 注释写的是「两路都用 49」。编码器模式下 PSC 直接对边沿分频，这会让 E1、E2 的计数差 50 倍，左/右轮实测速度不可比。建议两路统一为 `PSC = 0`（此时每轮 counts 恰好 = PPR × 4 × 减速比，与 `ENCODER_EDGE_MULTIPLIER = 4` 自洽）。
- [ ] **N20 参数待实测**：现有 `counts/rev = 2800`（7 线 × 4 × 1:100）与 `轮径 = 43 mm` 是占位值，需按 6.2 节手转一圈核对；`KFF` 与 `base_speed` 需上车实测后定稿。
- [ ] 循迹外环实车参数未定稿。

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
