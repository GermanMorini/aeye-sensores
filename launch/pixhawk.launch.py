from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_web = LaunchConfiguration('launch_web')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_web',
                default_value='false',
                description='Launch the sensores_web node',
            ),
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
                condition=IfCondition(launch_web),
            ),
        ]
    )
