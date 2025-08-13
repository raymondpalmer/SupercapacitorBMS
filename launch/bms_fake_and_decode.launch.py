from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cells_total = LaunchConfiguration('cells_total')
    temps_total = LaunchConfiguration('temps_total')
    rate_hz     = LaunchConfiguration('rate_hz')
    from_topic  = LaunchConfiguration('from_topic')

    return LaunchDescription([
        DeclareLaunchArgument('cells_total', default_value='12',
                              description='Number of series cells'),
        DeclareLaunchArgument('temps_total', default_value='6',
                              description='Number of temperature probes'),
        DeclareLaunchArgument('rate_hz', default_value='20.0',
                              description='Publishing rate of fake CAN (Hz)'),
        DeclareLaunchArgument('from_topic', default_value='/from_can',
                              description='CAN frames topic for decoder'),

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
            parameters=[{
                'from_topic': from_topic
            }]
        ),
    ])
