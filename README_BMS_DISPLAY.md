# BMS Display System - HDMI接口电池管理系统显示界面

这是一个专为电池管理系统(BMS)设计的现代化HDMI显示界面，具有CarPlay Ultra风格的设计和流畅的充电动画。

## 功能特性

### 🎨 现代化UI设计
- **CarPlay Ultra风格**: 采用苹果CarPlay的设计语言，简洁美观
- **渐变背景**: 深色主题配合蓝色渐变，专业感十足
- **圆角设计**: 现代化的圆角元素，符合当前设计趋势

### ⚡ 充电动画效果
- **闪电图标**: 充电时显示动态闪电图标
- **脉冲环**: 多层脉冲环动画，增强视觉效果
- **透明度变化**: 平滑的透明度动画，提升用户体验

### 📊 电池信息显示
- **实时数据**: 显示电池电量、电压、电流、温度
- **状态指示**: 充电/放电状态实时显示
- **颜色编码**: 根据电量状态显示不同颜色（绿色/橙色/红色）

### 🖥️ 多种显示模式
- **HDMI模式**: 专为HDMI显示器优化的全屏模式
- **全屏模式**: 主显示器全屏显示
- **预览模式**: 窗口化显示，便于开发和调试

## 系统要求

### 软件依赖
- Ubuntu 20.04+ / ROS2 Humble+
- Qt5 或 Qt6
- X11 显示系统
- Python 3.8+

### 硬件要求
- 支持HDMI输出的显卡
- 至少2GB显存
- 1920x1080或更高分辨率

## 安装说明

### 1. 安装系统依赖
```bash
# 安装Qt开发包
sudo apt update
sudo apt install qt6-base-dev qt6-base-dev-tools
# 或者Qt5
sudo apt install qtbase5-dev qt5-qmake

# 安装X11工具
sudo apt install x11-xserver-utils

# 安装VNC服务器（可选，用于远程查看）
sudo apt install x11vnc
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

## 使用方法

### 启动BMS显示系统

#### 方法1: 直接运行可执行文件
```bash
# 预览模式（窗口化）
./build/bms_can_bridge_cpp/bms_display_system --preview

# 全屏模式
./build/bms_can_bridge_cpp/bms_display_system --fullscreen

# HDMI模式
./build/bms_can_bridge_cpp/bms_display_system --hdmi
```

#### 方法2: 使用ROS2 launch文件
```bash
# 预览模式
ros2 launch bms_can_bridge_cpp bms_display.launch.py preview:=true

# HDMI模式
ros2 launch bms_can_bridge_cpp bms_display.launch.py hdmi:=true

# 全屏模式
ros2 launch bms_can_bridge_cpp bms_display.launch.py fullscreen:=true
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
- **充电动画**: 闪电图标 + 脉冲环
- **波浪效果**: 底部动态波浪
- **淡入淡出**: 平滑的透明度变化

## 开发说明

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
- 修改颜色方案: 编辑 `bms_display_ui.cpp` 中的颜色定义
- 添加新动画: 在 `setupAnimations()` 函数中添加新的动画
- 修改布局: 调整 `paintEvent()` 中的绘制函数

## 故障排除

### 常见问题

#### 1. Qt库未找到
```bash
# 确保安装了Qt开发包
sudo apt install qt6-base-dev
# 或者
sudo apt install qtbase5-dev
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
- [ ] 支持触摸屏操作
- [ ] 添加更多动画效果
- [ ] 支持多语言界面
- [ ] 集成实时数据源
- [ ] 添加配置界面

### 贡献指南
欢迎提交Issue和Pull Request来改进这个项目！

## 许可证

本项目采用BSD-3-Clause许可证。

## 联系方式

如有问题或建议，请通过以下方式联系：
- 项目Issues: GitHub Issues
- 邮箱: you@example.com

---

**注意**: 这是一个演示系统，实际部署时请根据具体需求调整参数和配置。
