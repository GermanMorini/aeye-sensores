from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_web = LaunchConfiguration('launch_web')
    launch_wheel_odom = LaunchConfiguration('launch_wheel_odom')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    odom_frame = LaunchConfiguration('odom_frame')
    base_link_frame = LaunchConfiguration('base_link_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    gps_frame = LaunchConfiguration('gps_frame')
    wheel_odom_port = LaunchConfiguration('wheel_odom_port')
    wheel_rx_backend = LaunchConfiguration('wheel_rx_backend')
    wheel_gpio_rx_pin = LaunchConfiguration('wheel_gpio_rx_pin')
    wheel_odom_baudrate = LaunchConfiguration('wheel_odom_baudrate')
    wheel_odom_invert_bytes = LaunchConfiguration('wheel_odom_invert_bytes')
    wheel_odom_log_rx_frames = LaunchConfiguration('wheel_odom_log_rx_frames')
    wheel_odom_log_raw_hex = LaunchConfiguration('wheel_odom_log_raw_hex')
    wheel_odom_invert_signal = LaunchConfiguration('wheel_odom_invert_signal')
    wheel_pigpiod_host = LaunchConfiguration('wheel_pigpiod_host')
    wheel_pigpiod_port = LaunchConfiguration('wheel_pigpiod_port')
    wheel_odom_frame_gap_us = LaunchConfiguration('wheel_odom_frame_gap_us')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    wheel_velocity_topic = LaunchConfiguration('wheel_velocity_topic')
    wheel_speed_topic = LaunchConfiguration('wheel_speed_topic')
    wheel_throttle_topic = LaunchConfiguration('wheel_throttle_topic')

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
                'launch_wheel_odom',
                default_value='false',
                description='Launch the wheel_odom_uart node',
            ),
            DeclareLaunchArgument(
                'wheel_odom_port',
                default_value='/dev/ttyAMA2',
                description='UART port for wheel odometry sniffer',
            ),
            DeclareLaunchArgument(
                'wheel_rx_backend',
                default_value='serial',
                description='Wheel odometry backend: serial (default) or pigpio',
            ),
            DeclareLaunchArgument(
                'wheel_gpio_rx_pin',
                default_value='5',
                description='GPIO pin for pigpio bit-bang UART RX',
            ),
            DeclareLaunchArgument(
                'wheel_odom_baudrate',
                default_value='2000',
                description='Wheel UART baudrate (2000 or 2083)',
            ),
            DeclareLaunchArgument(
                'wheel_odom_invert_bytes',
                default_value='false',
                description='Invert each UART byte with bitwise NOT',
            ),
            DeclareLaunchArgument(
                'wheel_odom_log_rx_frames',
                default_value='false',
                description='Print Arduino-like RX trace per frame',
            ),
            DeclareLaunchArgument(
                'wheel_odom_log_raw_hex',
                default_value='false',
                description='Include full frame hex in RX trace',
            ),
            DeclareLaunchArgument(
                'wheel_odom_invert_signal',
                default_value='false',
                description='Invert physical UART signal (pigpio backend)',
            ),
            DeclareLaunchArgument(
                'wheel_pigpiod_host',
                default_value='localhost',
                description='pigpiod host',
            ),
            DeclareLaunchArgument(
                'wheel_pigpiod_port',
                default_value='8888',
                description='pigpiod port',
            ),
            DeclareLaunchArgument(
                'wheel_odom_frame_gap_us',
                default_value='12000',
                description='Frame delimiter gap in microseconds',
            ),
            DeclareLaunchArgument(
                'wheel_odom_topic',
                default_value='/wheel/odom',
                description='Wheel odometry topic',
            ),
            DeclareLaunchArgument(
                'wheel_velocity_topic',
                default_value='/wheel/velocity',
                description='Wheel velocity topic',
            ),
            DeclareLaunchArgument(
                'wheel_speed_topic',
                default_value='/wheel/speed_kmh',
                description='Wheel decoded speed topic',
            ),
            DeclareLaunchArgument(
                'wheel_throttle_topic',
                default_value='/wheel/throttle',
                description='Wheel decoded throttle topic',
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
            Node(
                package='sensores',
                executable='wheel_odom_uart',
                name='wheel_odom_uart',
                output='screen',
                condition=IfCondition(launch_wheel_odom),
                parameters=[
                    {'rx_backend': wheel_rx_backend},
                    {'serial_port': wheel_odom_port},
                    {'gpio_rx_pin': wheel_gpio_rx_pin},
                    {'baudrate': wheel_odom_baudrate},
                    {'invert_bytes': wheel_odom_invert_bytes},
                    {'log_rx_frames': wheel_odom_log_rx_frames},
                    {'log_raw_hex': wheel_odom_log_raw_hex},
                    {'invert_signal': wheel_odom_invert_signal},
                    {'pigpiod_host': wheel_pigpiod_host},
                    {'pigpiod_port': wheel_pigpiod_port},
                    {'frame_gap_us': wheel_odom_frame_gap_us},
                    {'odom_topic': wheel_odom_topic},
                    {'velocity_topic': wheel_velocity_topic},
                    {'speed_topic': wheel_speed_topic},
                    {'throttle_topic': wheel_throttle_topic},
                    {'odom_frame': odom_frame},
                    {'base_link_frame': base_link_frame},
                ],
            ),
        ]
    )
