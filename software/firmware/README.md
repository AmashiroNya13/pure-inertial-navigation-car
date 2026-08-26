# 固件工程

## 子项目

| 目录 | 目标 | 工具链 |
| --- | --- | --- |
| `main-controller/` | 车辆主控、惯导路径、速度与姿态控制、串口协议 | Infineon AURIX Development Studio + TASKING |
| `esc-ai8051u/` | AI8051U 无刷电调 | Keil C251 |

两个固件工程需要分别编译、烧录。主控发送 PWM 输入，电调执行无感六步换相；请先确认两端的 PWM 频率、量程和失控保护配置一致。

## 第三方代码

`main-controller/Libraries/`、芯片头文件、SDK、逐飞组件及 IDE 工程模板可能带有独立许可证。公开发布前必须保留各文件原有版权声明，并按原许可证确认是否允许再分发和商用。
