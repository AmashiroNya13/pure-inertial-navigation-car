# 已知问题与验证状态

更新日期：2026-08-26。

## 已通过

- `software/path-tools/tools/test_amr_path_viewer.js`
- `software/path-tools/tools/test_node_radius_cdp.js`
- `software/PathCapture/tests/test_command_templates.js`

## 需要处理

### 节点规划器浏览器测试

`software/path-tools/tools/test_node_coordinate_viewer.js` 在当前 Node.js 22 + CDP 环境中加载页面脚本时出现 `SyntaxError: Invalid or unexpected token`，测试尚未进入功能断言。公开发布前应定位并修复该页面脚本的解析问题，然后重新运行完整测试。

### PathCapture Python 测试环境

`pytest` 在当前系统 Python 环境中未找到 `flask`，因此无法收集 `test_parser.py` 和 `test_binary_telemetry.py`。这不是已确认的功能失败；请先在 `software/PathCapture/` 下运行 `launch.bat`，或创建虚拟环境并执行 `pip install -r requirements.txt`，再运行测试。
