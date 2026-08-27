# 21届纯惯导车开源项目

本仓库整理了纯惯导小车的控制软件、上位机工具、硬件资料和机械资料。项目用于路径采集、节点地图规划、路径上传、纯惯导复现与数据可视化。

> 安全提示：本项目涉及高速电机、无刷电调、负压和移动平台。首次使用必须架空车辆，使用保守的速度与加速度参数，并确认急停可用。

> **重要文档**：
> - [纯惯导软件设计说明](docs/INERTIAL_NAVIGATION_SOFTWARE.md)：IMU、编码器、双自由度角度环、速度前馈、路径与修正思路。
> - [纯惯导机械设计说明](docs/INERTIAL_NAVIGATION_MECHANICAL.md)：电机、负压、结构、减振、底盘和装配经验。
> - [纯惯导硬件设计说明](docs/INERTIAL_NAVIGATION_HARDWARE.md)：电调、双 IMU、屏蔽、编码器连接和后续硬件方向。
> - [机械采购与加工清单（Excel）](docs/MECHANICAL_PROCUREMENT_LIST.xlsx)：采购链接、CNC/3D 打印需求、在售核验与下单规格。

## 目录

```text
.
├── software/
│   ├── firmware/
│   │   ├── main-controller/ # TC2xx 主控：惯导、路径、速度和姿态控制
│   │   ├── drive-esc-ai8051u/   # 行进电机 AI8051U 电调
│   │   └── suction-esc-ai8051u/ # 负压电机 AI8051U 电调
│   └── tools/
│       ├── PathCapture/     # 串口上位机、路径上传与复现数据可视化
│       └── path-tools/      # 节点坐标规划器、路径日志查看器
├── hardware/
│   ├── schematics/        # 主控原理图
│   └── README.md          # PCB、BOM 与接线资料的发布约定
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

已识别到的第三方组件和发布边界见 [docs/THIRD_PARTY_NOTICES.md](docs/THIRD_PARTY_NOTICES.md)。

## 致谢与上游

主控工程的早期基础参考了 [XCELINE / MotorCycle](https://gitee.com/xceline566/motorcycle) 的第二十一届智能车单车定向开源项目。感谢 XCELINE 公开基础工程与文档。

本仓库不是该项目的官方分支：路径复现、车辆控制、速度与姿态策略、串口协议、路径工具及上位机均已进行了大量改动和扩展。上游来源、版本与许可证边界见 [docs/UPSTREAM.md](docs/UPSTREAM.md)。

`archive/` 存放本次整理前的原始导入压缩包，仅供本地追溯，已由 `.gitignore` 排除。
