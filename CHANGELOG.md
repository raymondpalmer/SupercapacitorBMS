### 2025-08-14

- 新增 — 数据后端兼容：
  - **--backend sim|ros2|can** 三种数据源模式。
  - **CAN(socketcan)**：支持通过 `python-can` 读取真实 CAN，总线接口由 `--can-if` 指定（默认 `can0`），并可用 `--can-map` 传入 JSON/YAML 映射解析 SOC/电压/电流/温度/电容参数等。
  - **ROS2**：订阅 `sensor_msgs/BatteryState`（话题由 `--ros2-batt-topic` 指定，默认 `/battery_state`）。

- 新增 — 显示与多屏：
  - **--screen <index>** 指定全屏输出的 HDMI 屏幕索引，配合 `--fullscreen` 使用；窗口模式使用 `--window`。

- 视觉 — 光效重构：
  - 移除粒子与弧头扫光，全面改为沿指针长边的**光剑式拖影**。
  - 拖影由“多帧历史”连成**连续光带**，引入**细分插值**以增强相邻帧的过渡混合，拖尾更丝滑、连贯。
  - 整体透明度进一步下调，采用 Screen 叠加，保证只增亮不变暗。
  - 充电时光带为绿色，其余按各仪表渐变头色。

- 观感与一致性：
  - 低电量警示改为 Screen 叠加的红色呼吸光，不再压暗表盘。
  - 修复 0 刻度起始色与弧线“左侧”不一致问题：闭环锥形渐变 + `t-ε` 采样 + 起点小补丁弧。
  - 保留轻刻度与分区着色（绿/黄/红，低透明度），提升读数认知但不喧宾夺主。
  - 顶部动态胶囊（含 Time）与下方风格统一、宽度自适应。

- 运行示例：
  - 真实 HDMI（屏幕 1）+ ROS2：
    - `python3 scripts/preview_ui.py --fullscreen --screen 1 --ios18 --backend ros2 --ros2-batt-topic /battery_state`
  - 真实 HDMI（屏幕 1）+ CAN（can0）+ JSON 映射：
    - `python3 scripts/preview_ui.py --fullscreen --screen 1 --ios18 --backend can --can-if can0 --can-map /path/to/map.json`
  - 窗口预览（模拟）：
    - `python3 scripts/preview_ui.py --window --ios18`

提示：CAN 模式需安装 `python-can`，如使用 YAML 映射需安装 `PyYAML`；ROS2 模式需正确 source ROS 环境并有 BatteryState 发布。


