#!/bin/bash

# BMS Display System Quick Start Script
# 快速启动BMS显示系统

echo "=== BMS Display System Quick Start ==="
echo ""

# 检查是否在工作空间目录
if [[ ! -f "package.xml" ]] || [[ ! -d "src" ]]; then
    echo "Error: Please run this script from the bms_can_bridge_cpp package directory"
    exit 1
fi

# 检查是否已构建
if [[ ! -d "../build" ]] || [[ ! -f "../build/bms_can_bridge_cpp/bms_display_system" ]]; then
    echo "Building BMS Display System..."
    cd ..
    colcon build --packages-select bms_can_bridge_cpp
    if [[ $? -ne 0 ]]; then
        echo "Build failed! Please check the error messages above."
        exit 1
    fi
    cd src/bms_can_bridge_cpp
    echo "Build completed successfully!"
    echo ""
fi

# 设置环境
source ../../install/setup.bash

echo "Select startup mode:"
echo "  1. Preview mode (windowed) - Recommended for development"
echo "  2. Fullscreen mode"
echo "  3. HDMI mode"
echo "  4. Virtual display + Preview"
echo "  5. Exit"
echo ""

while true; do
    read -p "Enter your choice (1-5): " choice
    
    case $choice in
        1)
            echo "Starting in preview mode..."
            ../../build/bms_can_bridge_cpp/bms_display_system --preview
            break
            ;;
        2)
            echo "Starting in fullscreen mode..."
            ../../build/bms_can_bridge_cpp/bms_display_system --fullscreen
            break
            ;;
        3)
            echo "Starting in HDMI mode..."
            ../../build/bms_can_bridge_cpp/bms_display_system --hdmi
            break
            ;;
        4)
            echo "Starting virtual display + preview mode..."
            echo "This will create a virtual HDMI display and run the BMS interface."
            echo ""
            
            # 检查Python脚本
            if [[ -f "scripts/virtual_display.py" ]]; then
                python3 scripts/virtual_display.py --interactive
            else
                echo "Error: virtual_display.py not found"
                exit 1
            fi
            break
            ;;
        5)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please enter 1-5."
            ;;
    esac
done

echo ""
echo "BMS Display System has been started!"
echo "Press Ctrl+C to exit the display interface."
