# 路径规划工具

本目录提供无需构建的本地工具。

| 文件 | 用途 |
| --- | --- |
| `tools/node_coordinate_viewer.html` | 节点坐标导入、手动路线连接、切弯、速度规划与 JSON 导出 |
| `tools/amr_path_viewer.html` | 路径与惯导数据查看 |
| `tools/path_log_viewer.py` | 日志辅助查看工具 |

## 使用

直接用现代浏览器打开 `tools/node_coordinate_viewer.html`。规划完成后导出 JSON，再在 PathCapture 的“外部路径”页面导入。

## 测试

安装 Node.js 后，在本目录执行：

```powershell
node tools/test_node_coordinate_viewer.js
node tools/test_amr_path_viewer.js
node tools/test_node_radius_cdp.js
```
