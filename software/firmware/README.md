# 固件工程

## 子项目

| 目录 | 目标 | 工具链 |
| --- | --- | --- |
| `main-controller/` | 车辆主控、惯导路径、速度与姿态控制、串口协议 | Infineon AURIX Development Studio + TASKING |
| `drive-esc-ai8051u/` | 行进电机的 AI8051U 无刷电调 | Keil C251 |
| `suction-esc-ai8051u/` | 负压电机的 AI8051U 无刷电调 | Keil C251 |

三个固件工程需要分别编译、烧录。两块 AI8051U 板不能混刷：行进电调接收主控的行进 PWM，负压电调只驱动负压电机。每次烧录前都应核对目标板、接线、电源与对应目录。

## 第三方代码

`main-controller/Libraries/`、两套电调的逐飞组件、芯片头文件、SDK 及 IDE 工程模板可能带有独立许可证。公开发布前必须保留各文件原有版权声明，并按原许可证确认是否允许再分发和商用。
