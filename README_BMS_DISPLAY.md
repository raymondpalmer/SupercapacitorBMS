# BMS Display System - HDMI接口电池管理系统显示界面

这是一个专为电池管理系统(BMS)设计的现代化HDMI显示界面，提供 CarPlay/iOS18 风格的精致可视化、优雅的动画与真实数据后端（ROS2/CAN）对接能力。

## 功能特性

### 🎨 现代化UI设计
- **CarPlay Ultra风格**: 采用苹果CarPlay的设计语言，简洁美观
- **渐变背景**: 深色主题配合蓝色渐变，专业感十足
- **圆角设计**: 现代化的圆角元素，符合当前设计趋势

### ⚡ 充电/指针动画
- **光剑式拖影**: 指针沿边缘的连续光带，随运动产生丝滑拖尾（Screen 叠加，只增亮不变暗）
- **低电量呼吸光**: additive 红色呼吸，不压暗底层
- **渐变圆环**: 统一亮度、圆角端帽、起点色严格一致

### 📊 电池信息显示
- **实时数据**: 显示电池电量、电压、电流、温度
- **状态指示**: 充电/放电状态实时显示
- **颜色编码**: 根据电量状态显示不同颜色（绿色/橙色/红色）

### 🖥️ 多种显示模式
- **HDMI 全屏**: 可通过 `--screen <index>` 指定输出到目标屏幕
- **窗口预览**: 窗口化显示，便于开发和调试

## 系统要求

### 软件依赖（Python 预览 UI）
- Python 3.8+
- PyQt5
- （可选）python-can（如使用 CAN 后端）
- （可选）PyYAML（如使用 YAML 格式映射）
- （可选）can-utils（socketcan 调试）

### 硬件要求
- 支持HDMI输出的显卡
- 至少2GB显存
- 1920x1080或更高分辨率

## 安装说明

### 1. 安装依赖（二选一）
```bash
# 方式A：系统包（推荐一键）
sudo apt update
sudo apt install -y python3-pyqt5 python3-can python3-yaml can-utils

# 方式B：用户级 pip
python3 -m pip install --user PyQt5 python-can PyYAML
```

### 2. 构建项目
```bash
# 进入工作空间
cd ~/ros2_ws

# 构建项目
colcon build --packages-select bms_can_bridge_cpp

# 设置环境
source install/setup.bash
```

## 使用方法（Python 预览 UI）

```bash
# 窗口预览（模拟数据）
python3 scripts/preview_ui.py --window --ios18

# HDMI 全屏到屏幕1（根据实际改索引）+ 模拟数据
python3 scripts/preview_ui.py --fullscreen --screen 1 --ios18

# 使用 ROS2 作为数据源（需已 source ROS 环境并有 BatteryState 发布）
python3 scripts/preview_ui.py --window --ios18 --backend ros2 --ros2-batt-topic /battery_state

# 使用 CAN(socketcan) 作为数据源（示例接口 can0 + JSON 映射）
python3 scripts/preview_ui.py --fullscreen --screen 1 --ios18 \
  --backend can --can-if can0 --can-map /path/to/map.json
```

#### 方法3: 使用虚拟显示器脚本
```bash
# 交互式虚拟显示器
python3 scripts/virtual_display.py --interactive

# 命令行模式
python3 scripts/virtual_display.py --mode preview --resolution 1920x1080

# 启动VNC服务器
python3 scripts/virtual_display.py --mode preview --vnc
```

### 虚拟HDMI显示器

如果没有物理HDMI显示器，可以使用虚拟显示器进行预览：

```bash
# 使用bash脚本
chmod +x scripts/virtual_hdmi_display.sh
./scripts/virtual_hdmi_display.sh

# 使用Python脚本
chmod +x scripts/virtual_display.py
./scripts/virtual_display.py --interactive
```

## 界面说明

### 主要区域
1. **状态栏**: 显示时间、日期和连接状态
2. **电池图标**: 大型电池图标，显示电量百分比
3. **信息面板**: 显示详细的电池参数
4. **CarPlay区域**: 底部动态波浪效果

### 颜色方案
- **主色调**: iOS蓝色 (#007AFF)
- **辅助色**: 紫色 (#5856D6)
- **强调色**: 橙色 (#FF9500)
- **背景色**: 深灰色 (#1C1C1E)
- **文字色**: 白色 (#FFFFFF)

### 动画效果
- **光剑式拖影**: 连续光带，细分插值，过渡丝滑
- **波浪效果**: 底部动态波浪
- **低电量呼吸**: additive 只增亮

## 开发说明

### Python 预览 UI（当前推荐）
- 主文件：`scripts/preview_ui.py`
- 关键参数：`--backend sim|ros2|can`、`--screen`、`--ios18`/`--hmi`
- CAN 映射：支持 JSON/YAML，字段示例参见源码注释

### C++/Qt + ROS2（历史说明，暂作参考）

### 项目结构
```
bms_can_bridge_cpp/
├── src/
│   ├── bms_display_ui.h          # UI头文件
│   ├── bms_display_ui.cpp        # UI实现
│   ├── bms_display_main.cpp      # 主程序
│   └── ...
├── scripts/
│   ├── virtual_hdmi_display.sh   # Bash虚拟显示器脚本
│   └── virtual_display.py        # Python虚拟显示器脚本
├── launch/
│   └── bms_display.launch.py     # ROS2启动文件
└── CMakeLists.txt                 # 构建配置
```

### 自定义开发
- Python 预览：在 `scripts/preview_ui.py` 中调整颜色/布局/动画即可
- C++：编辑 `src/bms_display_ui.cpp` 对应函数

## 故障排除

### 常见问题

#### 1. Python 报 “无法解析导入 can”
```bash
# 安装 python-can（系统包或 pip）
sudo apt install -y python3-can   # 或
python3 -m pip install --user python-can
```

#### 2. 显示权限问题
```bash
# 确保X11权限正确
xhost +local:
```

#### 3. 虚拟显示器创建失败
```bash
# 检查xrandr是否可用
xrandr --version

# 手动添加显示器模式
xrandr --addmode <display> 1920x1080_60.00
```

#### 4. 字体显示问题
```bash
# 安装系统字体
sudo apt install fonts-sf-pro
# 或者使用系统默认字体
```

### 调试模式
```bash
# 启用调试输出
export QT_LOGGING_RULES="qt.qpa.*=true"
./build/bms_can_bridge_cpp/bms_display_system --preview
```

## 性能优化

### 渲染优化
- 启用硬件加速: 确保显卡驱动正确安装
- 减少重绘频率: 调整定时器间隔
- 优化动画: 使用缓动函数减少计算量

### 内存管理
- 及时释放资源: 在析构函数中清理
- 避免内存泄漏: 使用智能指针管理Qt对象

## 扩展功能

### 未来计划
- [ ] 触摸屏与手势
- [ ] C++/ROS2 集成完善与发布
- [ ] 丰富更多主题与布局预设

### 贡献指南
欢迎提交Issue和Pull Request来改进这个项目！

## 许可证

本项目采用BSD-3-Clause许可证。

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目Issues: GitHub Issues
- 邮箱: you@example.com

---

**注意**: 这是一个演示系统，实际部署时请根据具体需求调整参数和配置。详见 `CHANGELOG.md`。
