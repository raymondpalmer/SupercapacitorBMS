#!/bin/bash

# 虚拟HDMI显示器脚本
# 用于在没有物理HDMI显示器的情况下预览BMS显示界面

echo "=== BMS Virtual HDMI Display ==="
echo "This script creates a virtual display for previewing the BMS interface"
echo ""

# 检查是否安装了必要的工具
check_dependencies() {
    echo "Checking dependencies..."
    
    if ! command -v xrandr &> /dev/null; then
        echo "Error: xrandr not found. Please install x11-xserver-utils"
        exit 1
    fi
    
    if ! command -v x11vnc &> /dev/null; then
        echo "Warning: x11vnc not found. Install with: sudo apt install x11vnc"
        echo "This will limit remote viewing capabilities."
    fi
    
    echo "Dependencies check passed."
    echo ""
}

# 创建虚拟显示器
create_virtual_display() {
    echo "Creating virtual HDMI display..."
    
    # 获取当前显示器信息
    CURRENT_DISPLAY=$(xrandr --current | grep " connected" | head -1 | awk '{print $1}')
    
    if [ -z "$CURRENT_DISPLAY" ]; then
        echo "Error: No connected display found"
        exit 1
    fi
    
    echo "Current display: $CURRENT_DISPLAY"
    
    # 添加虚拟显示器 (1920x1080)
    xrandr --addmode $CURRENT_DISPLAY 1920x1080_60.00 2>/dev/null || true
    
    # 设置虚拟显示器分辨率
    xrandr --output $CURRENT_DISPLAY --mode 1920x1080_60.00
    
    echo "Virtual display created: 1920x1080"
    echo ""
}

# 启动BMS显示界面
start_bms_display() {
    echo "Starting BMS Display System..."
    echo "Options:"
    echo "  1. Preview mode (windowed)"
    echo "  2. Fullscreen mode"
    echo "  3. HDMI mode"
    echo ""
    
    read -p "Select mode (1-3): " choice
    
    case $choice in
        1)
            echo "Starting in preview mode..."
            cd /home/ray/ros2_ws/src/bms_can_bridge_cpp
            ./build/bms_display_system --preview
            ;;
        2)
            echo "Starting in fullscreen mode..."
            cd /home/ray/ros2_ws/src/bms_can_bridge_cpp
            ./build/bms_display_system --fullscreen
            ;;
        3)
            echo "Starting in HDMI mode..."
            cd /home/ray/ros2_ws/src/bms_can_bridge_cpp
            ./build/bms_display_system --hdmi
            ;;
        *)
            echo "Invalid choice. Starting in preview mode..."
            cd /home/ray/ros2_ws/src/bms_can_bridge_cpp
            ./build/bms_display_system --preview
            ;;
    esac
}

# 启动VNC服务器（可选）
start_vnc_server() {
    if command -v x11vnc &> /dev/null; then
        echo ""
        echo "Starting VNC server for remote viewing..."
        echo "You can connect using VNC client to: localhost:5900"
        echo "Press Ctrl+C to stop VNC server"
        
        x11vnc -display :0 -nopw -listen localhost -xkb -ncache 10 -ncache_cr -forever
    else
        echo ""
        echo "VNC server not available. Install x11vnc for remote viewing."
    fi
}

# 清理函数
cleanup() {
    echo ""
    echo "Cleaning up..."
    
    # 恢复原始显示器设置
    CURRENT_DISPLAY=$(xrandr --current | grep " connected" | head -1 | awk '{print $1}')
    if [ ! -z "$CURRENT_DISPLAY" ]; then
        xrandr --output $CURRENT_DISPLAY --auto
    fi
    
    echo "Virtual display removed."
    echo "Goodbye!"
}

# 设置信号处理
trap cleanup EXIT INT TERM

# 主程序
main() {
    echo "BMS Virtual HDMI Display Setup"
    echo "================================"
    echo ""
    
    check_dependencies
    create_virtual_display
    
    echo "Virtual HDMI display is ready!"
    echo "You can now run the BMS display system."
    echo ""
    
    # 询问是否立即启动BMS显示
    read -p "Start BMS display now? (y/n): " start_now
    
    if [[ $start_now =~ ^[Yy]$ ]]; then
        start_bms_display
    else
        echo ""
        echo "To start BMS display later, run:"
        echo "  ./build/bms_display_system --preview"
        echo ""
        echo "Virtual display will remain active until you close this script."
        echo "Press Enter to exit..."
        read
    fi
}

# 运行主程序
main
