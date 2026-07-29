# MSPM0G3507 八路灰度循迹小车

基于 TI MSPM0G3507、Code Composer Studio（CCS）和 DriverLib 的循迹小车工程。当前工程包含双电机驱动、A/B 相编码器测速、8 路灰度巡线与直角转弯、BMI088 姿态解算、OLED 显示，以及 UART1 蓝牙收发与接收链路诊断。

> 引脚、外设、时钟和中断以 `empty.syscfg` 为唯一配置源。不要手动修改 `Debug/ti_msp_dl_config.*` 等生成文件。

## 当前功能

- 两路 PWM 电机控制：支持正转、反转、停止；速度输入范围为 `-2000`～`+2000`。
- 左右各一组 A/B 相编码器：GPIO 双边沿状态机解码，10 ms 读取并清零一次脉冲增量。
- 8 路灰度：CD4051 将 8 路传感器复用到 ADC0 的 PA27；ADC + DMA 连续采样。
- 自动灰度标定：上电后在白底采样 50 次，黑线阈值取白底均值的 25%。
- 巡线闭环：基础速度 `300`；编码器速度 PID 与灰度位置 PID 共同输出左右轮差速。
- 直角转弯：左侧 3 路灰度全部检测到黑线时触发；先直行约 910 ms，再原地左转，右侧 4 路出现黑线并连续确认后回到巡线。
- BMI088：PA0/PA1 使用 I2C0；驱动包含硬件 I2C 传输、总线恢复及软件 I2C 备用路径。启动时进行静止零偏校准并计算偏航角。
- OLED：显示轮速增量、灰度状态、Yaw、IMU 状态和 UART 接收诊断。
- 蓝牙：UART1 使用 PB6/PB7，9600、8N1；每秒发送本机 Yaw，接收端由 FIFO + UART 中断缓存。
- 按键：PB8 内部上拉，低电平表示按下；支持短按、长按和双击。

## 目录

```text
hengtuobei2025/
├─ empty.c                 应用入口、10 ms 定时器中断、OLED、UART1 蓝牙逻辑
├─ empty.syscfg            SysConfig 外设、引脚、时钟和中断配置源
├─ Hardware/
│  ├─ motor.*              电机方向与 TIMG6 PWM 比较值控制
│  ├─ encoder.*            A/B 相编码器状态机与脉冲增量读取
│  ├─ huidu.*              灰度标定、巡线 PID、直角转弯状态机
│  ├─ graysensor.*         灰度传感器数据处理
│  ├─ adc_dma.*            ADC DMA 数据读取
│  ├─ pid.*                位置式 PID 实现及电机 PID 参数
│  ├─ icm42688.*           文件名沿用历史名称，实际为 BMI088 驱动
│  ├─ imu.* / ahrs.*       零偏校准、姿态更新与 Yaw 计算
│  ├─ oled.*               OLED 驱动和字库
│  ├─ KEY.*                PB8 按键事件识别
│  └─ board.*              延时等基础支持
├─ targetConfigs/          CCS 调试目标配置
└─ Debug/                  CCS/SysConfig 生成的构建输出
```

## 引脚与外设

| 功能 | 外设/信号 | 引脚 | 说明 |
|---|---|---|---|
| BMI088 | I2C0 SDA / SCL | PA0 / PA1 | 需要外部 I2C 上拉与共地 |
| 左电机 PWM | TIMG6 CCP0 | PB2 | PWM 比较值由 `Motor_SetSpeed(1, ...)` 设置 |
| 右电机 PWM | TIMG6 CCP1 | PB3 | PWM 比较值由 `Motor_SetSpeed(2, ...)` 设置 |
| 左电机方向 | AIN1 / AIN2 | PA13 / PA14 | 高低电平组合控制方向 |
| 右电机方向 | BIN1 / BIN2 | PA17 / PA16 | 高低电平组合控制方向 |
| 左编码器 | E1A / E1B | PA25 / PA26 | GPIO 中断输入 |
| 右编码器 | E2A / E2B | PB20 / PB24 | GPIO 中断输入 |
| 灰度模拟量 | ADC0 CH0 | PA27 | CD4051 公共输出端 |
| CD4051 地址 | S0 / S1 / S2 | PA22 / PA7 / PA24 | 选择 8 路灰度通道 |
| OLED | RST / DC / SCL / SDA | PB14 / PB15 / PA28 / PA31 | 软件时序接口 |
| 用户按键 | KEY | PB8 | 内部上拉，按键另一端应接地 |
| 蓝牙串口 | UART1 TX / RX | PB6 / PB7 | PB6 → 模块 RXD，模块 TXD → PB7 |
| 备用串口 | UART2 TX / RX | PB17 / PB16 | 9600、8N1，当前未使用 |
| 调试下载 | SWDIO / SWCLK | PA19 / PA20 | J-Link SWD |

## 上电与按键

1. `SYSCFG_DL_init()` 后等待 5 秒，让外部供电和模块稳定。
2. 灰度自动标定时 OLED 显示 `CALIB...`；此时应将灰度模块置于白底。
3. 标定完成后，BMI088 初始化并进行静止校准；保持小车静止。
4. 需要长按 PB8 才会置位 `huidu_pid_flag` 并开始巡线。

PB8 的接法与判定：

```text
未按下：PB8 被内部上拉为高电平
按下：  按键将 PB8 接地，读取为低电平
```

- 短按：`target_lap` 加 1，在 0～5 间循环；0 表示不按圈数停止。
- 长按：启动巡线。
- 双击：切换正常页和 IMU 调试页。

## OLED 显示

正常页：

```text
L:+0000 R:+0000     左右轮最近 10 ms 编码器增量
G:00000000          8 路灰度状态，显示顺序为 bit7 到 bit0
YAW:+000            BMI088 Yaw；IMU 失败时显示 I2C 诊断
RX:hello            最近收到的 UART1 可显示文本
```

UART 尚未接收到可显示文本时，第四行显示：

```text
RX:--- I:00 B:00
```

- `I`：UART1 中断进入次数（后两位）。
- `B`：从 UART1 FIFO 实际取出的字节数（后两位）。
- `I:00 B:00` 说明 PB7 尚未收到可触发 UART 接收的有效数据；此时优先检查模块 TXD→PB7、共地、模块数据模式及波特率。

调试页显示 Yaw、Z 轴角速度 `GZ`、零偏 `B`、IMU 更新周期 `DT`、校准状态 `CAL`，第四行仍显示 UART 接收状态。

## 蓝牙 UART1 协议与接收

UART1 配置为 9600、8N1，RX FIFO 阈值为 1 字节，接收中断将数据写入 32 字节软件队列，主循环再做解析和 OLED 显示，避免 OLED 刷新期间丢字节。

- 每秒发送一次本机偏航角：`<Y:+012.3>`。
- 收到同样格式的帧时，会解析并保存对方 Yaw。
- OLED 原始接收监视器独立于协议解析：手机发送 `hello`、`123` 或 `<Y:+000.0>` 都应直接显示。
- `\r`/`\n` 视为普通文本包结束；单包最多显示 18 个可显示 ASCII 字符。

HC-05 类模块接线：

```text
MSPM0 PB6 (TX)  →  蓝牙模块 RXD
MSPM0 PB7 (RX)  ←  蓝牙模块 TXD
MSPM0 GND       ↔  蓝牙模块 GND
```

## 控制流程

```text
main()
  ├─ SysConfig 初始化
  ├─ 延时 5 s
  ├─ 编码器、灰度、ADC、OLED、UART 中断使能
  └─ while (1)
       ├─ 灰度自动标定完成前显示 CALIB...
       ├─ 维持灰度采样
       ├─ BMI088 初始化、校准与姿态更新
       ├─ 处理 UART1 接收队列及周期发送
       └─ 每 100 ms 刷新 OLED

TIMG0 中断（10 ms）
  ├─ 增加 `sys_10ms_tick`
  ├─ 读取并清零左右编码器增量
  ├─ 若已启动巡线，执行 `HuiDu_PID()`
  └─ 扫描按键短按、长按、双击事件
```

## 构建与下载

在 CCS 导入工程后选择 **Debug** 构建。构建输出：

```text
Debug/hengtuobei2025.out
Debug/hengtuobei2025.hex
```

命令行构建：

```powershell
gmake -C Debug -j2 all
```

下载配置文件为 `targetConfigs/MSPM0G3507.ccxml`，使用 J-Link SWD。

## 已知配置提示

- 当前 `empty.syscfg` 使用 `MSPM0G350X` 器件族元数据；SysConfig 构建会提示自动选择 `MSPM0G3505`，而 CCS 调试目标配置为 `MSPM0G3507`。当前工程可以构建，但后续整理工程时应统一 SysConfig 的具体器件型号与实际芯片。
- 本 README 记录的是源码与构建配置状态；巡线速度、灰度阈值、电机方向和 BMI088 安装方向仍需以实车调试为准。

## 维护规则

- 调整引脚、外设、时钟、DMA 或中断时，先改 `empty.syscfg`，再重新生成并构建。
- 不要手动编辑 `Debug/` 下的 `ti_msp_dl_config.c`、`ti_msp_dl_config.h`、`.out`、`.hex`、`.map` 等生成文件。
- 速度、巡线和转弯参数集中于 `Hardware/huidu.c`、`Hardware/pid.c` 与 `Hardware/motor.c`。
