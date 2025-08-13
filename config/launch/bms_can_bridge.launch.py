from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    interface = LaunchConfiguration('interface')
    filters = LaunchConfiguration('filters')
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('filters', default_value='0x2F4:0x7FF,0x4F4:0x7FF,0x5F4:0x7FF,0x7F4:0x7FF,'
                                                '0x18F128F4:0x1FFFFFFF,0x18F228F4:0x1FFFFFFF,0x18F328F4:0x1FFFFFFF,'
                                                '0x18F428F4:0x1FFFFFFF,0x18F528F4:0x1FFFFFFF,0x18E028F4:0x1FFFFFFF,'
                                                '0x1806E5F4:0x1FFFFFFF,0x18F0F428:0x1FFFFFFF'),
        Node(
            package='ros2_socketcan',
            executable='socketcan_receiver_node',
            name='socketcan_receiver',
            parameters=[{'interface': interface, 'filters': filters}],
            output='screen'
        ),
        Node(
            package='bms_can_bridge_cpp',
            executable='bms_can_decoder',
            name='bms_can_decoder',
            output='screen'
        ),
    ])
