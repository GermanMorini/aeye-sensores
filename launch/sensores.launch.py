from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='sensores',
                executable='pixhawk_driver',
                name='pixhawk_driver',
                output='screen',
            ),
            Node(
                package='sensores',
                executable='sensores_web',
                name='sensores_web',
                output='screen',
            ),
        ]
    )
