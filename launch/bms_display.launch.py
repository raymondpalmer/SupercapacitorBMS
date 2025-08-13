#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 声明启动参数
    preview_mode = DeclareLaunchArgument(
        'preview',
        default_value='false',
        description='Run in preview mode (windowed)'
    )
    
    hdmi_mode = DeclareLaunchArgument(
        'hdmi',
        default_value='false',
        description='Run in HDMI mode (fullscreen)'
    )
    
    fullscreen_mode = DeclareLaunchArgument(
        'fullscreen',
        default_value='false',
        description='Run in fullscreen mode'
    )
    
    # 启动BMS显示系统
    bms_display = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'bms_can_bridge_cpp', 'bms_display_system',
            '--preview' if LaunchConfiguration('preview') == 'true' else '',
            '--hdmi' if LaunchConfiguration('hdmi') == 'true' else '',
            '--fullscreen' if LaunchConfiguration('fullscreen') == 'true' else ''
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('preview') == 'false' and 
                                LaunchConfiguration('hdmi') == 'false' and 
                                LaunchConfiguration('fullscreen') == 'false')
    )
    
    # 启动虚拟显示器（可选）
    virtual_display = ExecuteProcess(
        cmd=['python3', 'scripts/virtual_display.py', '--interactive'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('preview') == 'true')
    )
    
    return LaunchDescription([
        preview_mode,
        hdmi_mode,
        fullscreen_mode,
        virtual_display,
        bms_display
    ])
