# 第三方组件与发布边界

本文件记录已识别的第三方内容。它不是许可证替代品；发布者须在推送公开远端前确认每个来源的具体再分发条件。

| 位置 | 来源/用途 | 当前处理 |
| --- | --- | --- |
| `software/firmware/main-controller/` 的早期工程基础 | [XCELINE / MotorCycle](https://gitee.com/xceline566/motorcycle)，GPL-3.0 | 本项目为大幅修改后的独立仓库，保留上游归因并遵守适用的 GPL-3.0 条款 |
| `software/firmware/main-controller/Libraries/` | Infineon AURIX TC26B iLLD、寄存器定义与平台组件 | 保留原始版权头；根据各文件原始许可证发布 |
| `software/firmware/main-controller/user/` 与工程模板 | Infineon/ADS 工程模板 | 保留原始版权头；不纳入未来自有代码许可证 |
| `software/firmware/drive-esc-ai8051u/` | 行进电机：逐飞科技 AI8051U 库与无刷电调工程 | 文件头说明含非商业限制，不能被仓库未来的 MIT/GPL 许可证覆盖 |
| `software/firmware/suction-esc-ai8051u/` | 负压电机：逐飞科技 AI8051U 库与无刷电调工程 | 文件头说明含非商业限制，不能被仓库未来的 MIT/GPL 许可证覆盖 |
| `software/tools/PathCapture/` | 本项目上位机与网页工具 | 作者需确认所有文件均为自有或兼容许可后再授权 |
| `software/tools/path-tools/` | 本项目节点规划与路径查看工具 | 作者需确认所有文件均为自有或兼容许可后再授权 |

## 发布建议

1. 自有代码与第三方目录分开授权，不要在根目录许可证中声称覆盖整个仓库。
2. 若逐飞代码没有明确的公开再分发授权，公开仓库应改为仅保留你的差异补丁、配置文件与获取说明。
3. 每次新增 SDK、算法库、字体、图片或数据文件时，都在本表补充来源和许可证。
