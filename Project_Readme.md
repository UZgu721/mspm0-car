# MSPM0G3507 智能循迹小车

基于 TI MSPM0G3507、Code Composer Studio 和 DriverLib 的八路灰度循迹小车工程。当前版本包含双轮电机控制、GPIO 中断编码器、灰度巡线与直角转弯、速度闭环、BMI088 六轴 IMU、OLED 状态/调试显示，以及蓝牙与备用串口配置。

> 当前 CCS 工程目标为 `MSPM0G3507`；`empty.syscfg` 使用 MSPM0G350X 器件族配置。引脚和外设配置以 `empty.syscfg` 为唯一来源。

## 功能概览

- 两路 PWM 电机驱动，支持正反转和停车。
- 两组 A/B 相编码器 GPIO 中断计数，10 ms 读取左右轮增量。
- 八路灰度传感器经 CD4051 复用到 ADC0，自动标定并输出黑线二值状态。
- 灰度位置 PID 差速循迹；左侧三路全黑触发直角转弯，右侧四路用于回线检测。
- 左右轮速度 PID 闭环，巡线基础速度为 `300`。
- BMI088 通过 I2C0 采样，启动三轴零偏校准，使用三轴角速度在静态重力方向上的投影计算偏航角。
- OLED 正常巡线页与 IMU 调试页；PB8 双击切换页面。
- UART1 用于蓝牙，UART2 预留为备用串口。

## 目录

```text
hengtuobei2025/
├─ empty.c                 应用入口、10 ms 中断、按键和 OLED 页面
├─ empty.syscfg            SysConfig 外设、引脚、时钟和中断配置源
├─ Hardware/
│  ├─ motor.*              电机方向与 PWM 比较值控制
│  ├─ encoder.*            两组 A/B 相编码器 GPIO 中断计数
│  ├─ huidu.*              灰度采样、二值化、巡线与转弯状态机
│  ├─ adc_dma.*            ADC 结果读取与平均处理
│  ├─ pid.*                位置式 PID 通用实现及左右轮参数
│  ├─ icm42688.*           ICM42688 初始化与 I2C 突发读取
│  ├─ imu.*                三轴零偏、重力轴、偏航角速度与调试数据
│  ├─ ahrs.*               Roll/Pitch 互补更新与 Yaw 积分
│  ├─ oled.*               4 线 SPI 风格 OLED 驱动
│  ├─ KEY.*                PB8 单击、长按、双击识别
│  └─ board.*              延时等基础支持
├─ targetConfigs/          CCS 调试目标配置
└─ Debug/                  CCS 生成的构建输出（不要手工修改）
```

## 引脚与外设

| 功能 | 外设/信号 | 引脚 |
|---|---|---|
| BMI088 | I2C0 SDA / SCL，Fast 模式 | PA0 / PA1 |
| 左电机 PWM | TIMG6 CCP0 | PB2 |
| 右电机 PWM | TIMG6 CCP1 | PB3 |
| 左电机方向 | AIN1 / AIN2 | PA13 / PA14 |
| 右电机方向 | BIN1 / BIN2 | PA17 / PA16 |
| 左编码器 | E1A / E1B | PA25 / PA26 |
| 右编码器 | E2A / E2B | PB20 / PB24 |
| 灰度 ADC | ADC0 输入 | PA27 |
| CD4051 地址 | S0 / S1 / S2 | PA22 / PA7 / PA24 |
| OLED | RST / DC / SCL / SDA | PB14 / PB15 / PA28 / PA31 |
| 按键 | KEY | PB8 |
| 蓝牙串口 | UART1 TX / RX | PB6 / PB7 |
| 备用串口 | UART2 TX / RX | PB17 / PB16 |
| 调试下载 | SWDIO / SWCLK | PA19 / PA20 |

## 上电、按键与显示

1. 上电后先执行灰度自动标定；标定阶段 OLED 显示 `CALIB...`，请按实际灰度模块标定要求放置传感器。
2. 随后 OLED 显示 `IMU CAL...`，BMI088 会丢弃 100 个样本并对 300 个静止样本求平均。此时保持车体静止，约 1 秒。
3. 按键 PB8 的行为：
   - 短按：目标圈数 `target_lap` 加 1，超过 5 回到 0；0 表示不按圈数停止。
   - 长按：置位 `huidu_pid_flag`，启动巡线。
   - 双击：切换 OLED 正常页和 IMU 调试页。

正常页显示左右轮 10 ms 编码器增量、八路灰度二值状态、Yaw 和目标圈数。IMU 调试页格式如下：

```text
Y:+012.3
GZ:+0.04 B:+0.12
DT:010 CAL:OK
LATE:000
```

- `Y`：偏航角，单位为度。
- `GZ`：零偏校正后的传感器 Z 轴角速度，单位为 dps。
- `B`：启动校准得到的 Z 轴零偏，单位为 dps。
- `DT`：相邻 IMU 更新的实际间隔，单位为 ms。
- `LATE`：`DT > 20 ms` 的累计次数。正常运行时应尽量保持为 0。

OLED 物理刷新频率为 10 Hz；IMU 更新优先于 OLED 绘制，不在定时器中断内执行 I2C。

## 控制流程

```text
main()
  ├─ SysConfig 初始化、编码器/灰度/ADC/OLED 初始化
  ├─ 灰度自动标定
  ├─ BMI088 初始化和静态三轴校准
  └─ while (1)
       ├─ 更新灰度采样和二值状态
       ├─ 按 10 ms 节拍读取 IMU 并更新姿态
       └─ 每 100 ms 刷新 OLED

TIMG0 10 ms 中断
  ├─ 累加系统节拍
  ├─ 获取并清零左右编码器增量
  ├─ 若已启动，执行 HuiDu_PID()
  └─ 扫描 PB8 的短按、长按和双击事件
```

`HuiDu_PID()` 的关键状态：

1. 直线巡线：速度 PID 维持左右轮速度，灰度 PID 生成差速转向量。
2. 触发转弯：最左三路灰度均检测到黑线。
3. 前进阶段：保持两轮正转一段时间。
4. 转弯阶段：左轮反转、右轮正转；右侧四路检测到黑线后回到直线巡线。
5. 每完成 4 次转弯记为 1 圈；达到非零目标圈数后停车。

## BMI088 姿态说明

BMI088 的加速度计与陀螺仪是两个独立的 I2C 从设备。`icm42688.c` 保留的是 CCS 自动生成构建规则所需的历史文件名，文件实际实现为 BMI088 驱动；它分别以两个 6 字节 I2C 突发读取获取加速度和陀螺仪数据。初始化采用官方示例的 ±6 g、±1000 dps、100 Hz 配置，并自动探测 `0x18/0x19` 与 `0x68/0x69` 两组合法地址。`imu.c` 在启动静止校准期间同时得到：

- 三轴陀螺仪零偏；
- 静态重力方向在传感器坐标系中的单位向量。

偏航角速度不是直接使用 `gz`，而是使用三轴校正角速度对该重力方向的投影。因此 BMI088 模块固定倾斜安装时，仍可获得车体平面转动的角速度。加速度数据进入 AHRS 前统一转换为 `g`。

这是六轴 IMU，没有磁力计或外部绝对航向参考；短期角度精度会改善，但长期航向角不能保证绝对无漂移。若转向方向与实际相反，需要依据 BMI088 模块箭头与车头方向调整坐标符号。

## 构建与下载

推荐使用 Code Composer Studio 导入工程并执行 **Debug** 构建。构建会根据 `empty.syscfg` 生成 `Debug/ti_msp_dl_config.*` 等文件；这些生成文件不要手工修改。

当前 Debug 输出为：

```text
Debug/hengtuobei2025.out
Debug/hengtuobei2025.hex
```

命令行构建（需使用本机 CCS 安装的 gmake 与 TI Arm Clang 工具链）：

```powershell
gmake -C Debug -j2 all
```

下载时使用 `targetConfigs/MSPM0G3507.ccxml` 中配置的调试后端，并确认实际连接的 SWD 探针与该配置匹配。

## 验证状态

- 已通过 SysConfig 生成和 CCS Debug 编译、链接、HEX 生成。
- 未在本仓库中记录对具体实车、灰度模块、BMI088 安装朝向和电机接线的统一硬件验证结果；首次下载后请先在支架上确认电机方向、灰度位序和偏航角正负方向。

## 维护规则

- 修改引脚、外设、时钟、DMA 或中断时，先修改 `empty.syscfg`，再重新生成并构建。
- 不要直接修改 `Debug/` 下的 `ti_msp_dl_config.c`、`ti_msp_dl_config.h`、`.out`、`.hex`、`.map` 等生成文件。
- 速度、巡线和转弯参数集中在 `Hardware/huidu.c`、`Hardware/pid.c` 与 `Hardware/motor.c`；调整前建议记录当前数值和实车效果。
