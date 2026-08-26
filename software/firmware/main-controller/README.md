# 主控固件

这是基于 Infineon AURIX TC2xx 的 ADS/TASKING 工程，包含纯惯导路径记录与复现、速度环、航向控制、路径上传、光电与滑移观测相关模块。

## 打开与构建

1. 安装与工程匹配的 AURIX Development Studio、TASKING TriCore 工具链和芯片 SDK。
2. 在 ADS 中导入本目录为现有 Eclipse 工程。
3. 选择 `Mad_Circuits TriCore Debug (TASKING)` 构建配置。
4. 在未接电机的条件下先检查引脚、时钟、PWM、编码器、IMU 和串口配置。

工程链接脚本为 `Lcf_Tasking_Tricore_Tc.lsl`。构建产物应生成在本机目录，已由仓库根目录 `.gitignore` 排除。

主控原理图位于仓库根目录的 `hardware/schematics/`，不再混放在固件目录。

## 主要模块

| 路径 | 职责 |
| --- | --- |
| `src/app/module/module_vehicle_control/` | 速度、姿态、起步状态机与 PWM 输出 |
| `src/app/module/module_vehicle_path/` | 路径记录、规划、上传、复现与位置估计 |
| `src/app/module/module_vehicle_encoder/` | 轮速和编码器观测 |
| `src/app/module/module_vehicle_gyro/` | IMU 与姿态观测 |

## 参数与安全

实车参数和 PID 不应直接照搬。首次移植应从低速、低加速度、架空测试开始，逐项校验轮周长、轮距、减速比、编码器方向、电机方向、IMU 轴向和急停逻辑。
