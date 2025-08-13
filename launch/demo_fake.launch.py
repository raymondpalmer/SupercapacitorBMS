from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='bms_can_bridge_cpp', executable='bms_can_decoder', name='bms_decoder', output='screen'),
        Node(package='bms_can_bridge_cpp', executable='fake_can_pub',   name='fake_can_pub', output='screen'),
    ])
