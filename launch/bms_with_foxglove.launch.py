from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    cells_total = LaunchConfiguration('cells_total')
    temps_total = LaunchConfiguration('temps_total')
    rate_hz     = LaunchConfiguration('rate_hz')
    from_topic  = LaunchConfiguration('from_topic')

    return LaunchDescription([
        DeclareLaunchArgument('cells_total', default_value='12'),
        DeclareLaunchArgument('temps_total', default_value='6'),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),
        DeclareLaunchArgument('from_topic', default_value='/from_can'),

        Node(
            package='bms_can_bridge_cpp',
            executable='fake_can_pub',
            name='fake_can_pub',
            output='screen',
            parameters=[{
                'cells_total': cells_total,
                'temps_total': temps_total,
                'rate_hz': rate_hz
            }]
        ),
        Node(
            package='bms_can_bridge_cpp',
            executable='bms_state_node',
            name='bms_state_node',
            output='screen',
            parameters=[{'from_topic': from_topic}]
        ),
        # Foxglove bridge (websocket default 8765)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        ),
    ])
