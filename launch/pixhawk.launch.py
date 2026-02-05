from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_web = LaunchConfiguration('launch_web')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    odom_frame = LaunchConfiguration('odom_frame')
    base_link_frame = LaunchConfiguration('base_link_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    gps_frame = LaunchConfiguration('gps_frame')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_web',
                default_value='false',
                description='Launch the sensores_web node',
            ),
            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/ttyACM0',
                description='Pixhawk serial port',
            ),
            DeclareLaunchArgument(
                'baudrate',
                default_value='921600',
                description='Pixhawk MAVLink baudrate',
            ),
            DeclareLaunchArgument(
                'odom_frame',
                default_value='odom',
                description='Odometry frame id',
            ),
            DeclareLaunchArgument(
                'base_link_frame',
                default_value='base_footprint',
                description='Robot base frame id (child of odom)',
            ),
            DeclareLaunchArgument(
                'imu_frame',
                default_value='imu_link',
                description='IMU frame id',
            ),
            DeclareLaunchArgument(
                'gps_frame',
                default_value='gps_link',
                description='GPS frame id',
            ),
            Node(
                package='sensores',
                executable='pixhawk_driver',
                name='pixhawk_driver',
                output='screen',
                parameters=[
                    {'serial_port': serial_port},
                    {'baudrate': baudrate},
                    {'odom_frame': odom_frame},
                    {'base_link_frame': base_link_frame},
                    {'imu_frame': imu_frame},
                    {'gps_frame': gps_frame},
                ],
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
