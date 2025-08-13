#!/usr/bin/env python3
"""
BMS Virtual HDMI Display
Python script for creating virtual displays and previewing BMS interface
"""

import os
import sys
import subprocess
import time
import signal
import argparse
from pathlib import Path

class VirtualDisplay:
    def __init__(self):
        self.current_display = None
        self.original_mode = None
        self.vnc_process = None
        
    def check_dependencies(self):
        """检查必要的依赖工具"""
        print("Checking dependencies...")
        
        # 检查xrandr
        try:
            subprocess.run(['xrandr', '--version'], 
                         capture_output=True, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("Error: xrandr not found. Please install x11-xserver-utils")
            print("  sudo apt install x11-xserver-utils")
            return False
            
        # 检查x11vnc（可选）
        try:
            subprocess.run(['x11vnc', '-version'], 
                         capture_output=True, check=True)
            self.vnc_available = True
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("Warning: x11vnc not found. Install with: sudo apt install x11vnc")
            self.vnc_available = False
            
        print("Dependencies check passed.")
        return True
        
    def get_display_info(self):
        """获取当前显示器信息"""
        try:
            result = subprocess.run(['xrandr', '--current'], 
                                  capture_output=True, text=True, check=True)
            
            lines = result.stdout.split('\n')
            for line in lines:
                if ' connected' in line:
                    parts = line.split()
                    self.current_display = parts[0]
                    self.original_mode = parts[1] if len(parts) > 1 else None
                    break
                    
            if not self.current_display:
                print("Error: No connected display found")
                return False
                
            print(f"Current display: {self.current_display}")
            if self.original_mode:
                print(f"Original mode: {self.original_mode}")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error getting display info: {e}")
            return False
            
    def create_virtual_display(self, resolution="1920x1080"):
        """创建虚拟显示器"""
        print(f"Creating virtual HDMI display: {resolution}")
        
        try:
            # 添加虚拟显示器模式
            subprocess.run(['xrandr', '--addmode', self.current_display, 
                          f"{resolution}_60.00"], 
                         capture_output=True)
        except subprocess.CalledProcessError:
            print(f"Mode {resolution}_60.00 already exists or failed to add")
            
        # 设置虚拟显示器分辨率
        try:
            subprocess.run(['xrandr', '--output', self.current_display, 
                          '--mode', f"{resolution}_60.00"], 
                         check=True)
            print(f"Virtual display created: {resolution}")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error setting display mode: {e}")
            return False
            
    def start_vnc_server(self):
        """启动VNC服务器"""
        if not self.vnc_available:
            print("VNC server not available")
            return None
            
        print("Starting VNC server for remote viewing...")
        print("You can connect using VNC client to: localhost:5900")
        
        try:
            vnc_cmd = [
                'x11vnc', '-display', ':0', '-nopw', 
                '-listen', 'localhost', '-xkb', '-ncache', '10', 
                '-ncache_cr', '-forever'
            ]
            
            self.vnc_process = subprocess.Popen(vnc_cmd)
            return self.vnc_process
        except subprocess.CalledProcessError as e:
            print(f"Error starting VNC server: {e}")
            return None
            
    def start_bms_display(self, mode="preview"):
        """启动BMS显示系统"""
        print(f"Starting BMS Display System in {mode} mode...")
        
        # 获取项目路径
        project_path = Path(__file__).parent.parent
        build_path = project_path / "build"
        
        if not build_path.exists():
            print("Error: Build directory not found. Please build the project first:")
            print("  cd /home/ray/ros2_ws")
            print("  colcon build --packages-select bms_can_bridge_cpp")
            return False
            
        # 构建可执行文件路径
        executable = build_path / "bms_display_system"
        if not executable.exists():
            print(f"Error: Executable not found: {executable}")
            return False
            
        # 启动BMS显示系统
        try:
            cmd = [str(executable)]
            if mode == "preview":
                cmd.append("--preview")
            elif mode == "fullscreen":
                cmd.append("--fullscreen")
            elif mode == "hdmi":
                cmd.append("--hdmi")
                
            print(f"Running command: {' '.join(cmd)}")
            subprocess.run(cmd, cwd=str(project_path))
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error starting BMS display: {e}")
            return False
            
    def cleanup(self):
        """清理虚拟显示器"""
        print("\nCleaning up...")
        
        if self.vnc_process:
            print("Stopping VNC server...")
            self.vnc_process.terminate()
            self.vnc_process.wait()
            
        if self.current_display and self.original_mode:
            print("Restoring original display mode...")
            try:
                subprocess.run(['xrandr', '--output', self.current_display, 
                              '--mode', self.original_mode], 
                             capture_output=True)
            except subprocess.CalledProcessError:
                print("Warning: Failed to restore original display mode")
                
        print("Virtual display removed.")
        
    def interactive_mode(self):
        """交互模式"""
        print("\nBMS Virtual HDMI Display Setup")
        print("================================")
        
        if not self.check_dependencies():
            return False
            
        if not self.get_display_info():
            return False
            
        if not self.create_virtual_display():
            return False
            
        print("\nVirtual HDMI display is ready!")
        print("You can now run the BMS display system.")
        
        # 询问是否立即启动BMS显示
        while True:
            choice = input("\nStart BMS display now? (y/n): ").lower().strip()
            if choice in ['y', 'yes']:
                break
            elif choice in ['n', 'no']:
                print("\nTo start BMS display later, run:")
                print("  ./build/bms_display_system --preview")
                print("\nVirtual display will remain active until you close this script.")
                input("Press Enter to exit...")
                return True
            else:
                print("Please enter 'y' or 'n'")
                
        # 选择显示模式
        print("\nSelect display mode:")
        print("  1. Preview mode (windowed)")
        print("  2. Fullscreen mode")
        print("  3. HDMI mode")
        
        while True:
            try:
                mode_choice = int(input("Select mode (1-3): "))
                if mode_choice == 1:
                    mode = "preview"
                    break
                elif mode_choice == 2:
                    mode = "fullscreen"
                    break
                elif mode_choice == 3:
                    mode = "hdmi"
                    break
                else:
                    print("Please enter 1, 2, or 3")
            except ValueError:
                print("Please enter a valid number")
                
        # 启动BMS显示
        self.start_bms_display(mode)
        return True

def signal_handler(signum, frame):
    """信号处理器"""
    print(f"\nReceived signal {signum}")
    if hasattr(signal_handler, 'virtual_display'):
        signal_handler.virtual_display.cleanup()
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(
        description="BMS Virtual HDMI Display Setup"
    )
    parser.add_argument(
        '--mode', 
        choices=['preview', 'fullscreen', 'hdmi'],
        default='preview',
        help='Display mode for BMS interface'
    )
    parser.add_argument(
        '--resolution',
        default='1920x1080',
        help='Virtual display resolution (default: 1920x1080)'
    )
    parser.add_argument(
        '--vnc',
        action='store_true',
        help='Start VNC server for remote viewing'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='Run in interactive mode'
    )
    
    args = parser.parse_args()
    
    # 创建虚拟显示器实例
    virtual_display = VirtualDisplay()
    
    # 设置信号处理器
    signal_handler.virtual_display = virtual_display
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        if args.interactive:
            # 交互模式
            virtual_display.interactive_mode()
        else:
            # 命令行模式
            if not virtual_display.check_dependencies():
                sys.exit(1)
                
            if not virtual_display.get_display_info():
                sys.exit(1)
                
            if not virtual_display.create_virtual_display(args.resolution):
                sys.exit(1)
                
            # 启动VNC服务器（如果请求）
            if args.vnc:
                virtual_display.start_vnc_server()
                
            # 启动BMS显示
            virtual_display.start_bms_display(args.mode)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        virtual_display.cleanup()
        print("Goodbye!")

if __name__ == "__main__":
    main()
