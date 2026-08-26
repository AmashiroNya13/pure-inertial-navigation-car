# 21届纯惯导车开源项目

本仓库整理了纯惯导小车的控制软件、上位机工具、硬件资料和机械资料。项目用于路径采集、节点地图规划、路径上传、纯惯导复现与数据可视化。

> 安全提示：本项目涉及高速电机、无刷电调、负压和移动平台。首次使用必须架空车辆，使用保守的速度与加速度参数，并确认急停可用。

## 目录

```text
.
├── software/
│   ├── firmware/
│   │   ├── main-controller/ # TC2xx 主控：惯导、路径、速度和姿态控制
│   │   └── esc-ai8051u/     # AI8051U 无刷电调控制
│   └── tools/
│       ├── PathCapture/     # 串口上位机、路径上传与复现数据可视化
│       └── path-tools/      # 节点坐标规划器、路径日志查看器
├── hardware/              # 原理图、PCB、BOM 与接线资料
├── mechanical/            # CAD、装配图与机械 BOM
├── docs/                  # 使用、接口与调参说明
└── archive/               # 本地导入归档，不应推送到公开仓库
```

## 快速开始

### PathCapture 上位机

需要 Python 3.8 或更高版本。Windows 下双击：

```text
software/tools/PathCapture/launch.bat
```

首次启动会创建本地 `venv` 并安装依赖，浏览器随后打开 `http://localhost:5000`。更多使用方法见 [software/tools/PathCapture/README.md](software/tools/PathCapture/README.md)。

### 节点坐标规划器

直接使用浏览器打开：

```text
software/tools/path-tools/tools/node_coordinate_viewer.html
```

它可导入节点坐标、手动连接路线、做切弯与速度规划，并导出供 PathCapture 使用的 JSON。

## 发布前待补资料

- 在 `hardware/` 放入原理图、PCB、BOM、供电与接线说明。
- 在 `mechanical/` 放入 CAD 源文件、导出 PDF、装配说明和紧固件清单。
- 填写实车参数与安全范围，尤其是轮距、轮周长、减速比、PWM 上限和负压配置。

## 授权状态

当前尚未指定仓库整体许可证。公开前请逐项核对芯片 SDK、IDE 模板、逐飞库及其他第三方内容的原始许可证与再分发条件，再为你拥有版权的内容添加 `LICENSE`。

`archive/` 存放本次整理前的原始导入压缩包，仅供本地追溯，已由 `.gitignore` 排除。
