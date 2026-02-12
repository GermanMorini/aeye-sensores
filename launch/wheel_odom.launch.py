from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rx_backend = LaunchConfiguration('rx_backend')
    serial_port = LaunchConfiguration('serial_port')
    gpio_rx_pin = LaunchConfiguration('gpio_rx_pin')
    baudrate = LaunchConfiguration('baudrate')
    frame_gap_us = LaunchConfiguration('frame_gap_us')
    invert_bytes = LaunchConfiguration('invert_bytes')
    log_rx_frames = LaunchConfiguration('log_rx_frames')
    log_raw_hex = LaunchConfiguration('log_raw_hex')
    invert_signal = LaunchConfiguration('invert_signal')
    pigpiod_host = LaunchConfiguration('pigpiod_host')
    pigpiod_port = LaunchConfiguration('pigpiod_port')
    poll_rate_hz = LaunchConfiguration('poll_rate_hz')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    odom_topic = LaunchConfiguration('odom_topic')
    velocity_topic = LaunchConfiguration('velocity_topic')
    speed_topic = LaunchConfiguration('speed_topic')
    throttle_topic = LaunchConfiguration('throttle_topic')

    odom_frame = LaunchConfiguration('odom_frame')
    base_link_frame = LaunchConfiguration('base_link_frame')

    speed_scale = LaunchConfiguration('speed_scale')
    speed_timeout_s = LaunchConfiguration('speed_timeout_s')
    use_throttle_gate = LaunchConfiguration('use_throttle_gate')
    forward_sign = LaunchConfiguration('forward_sign')

    return LaunchDescription(
        [
            DeclareLaunchArgument('rx_backend', default_value='serial', description='RX backend: serial (default) or pigpio'),
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA2', description='UART port for wheel sniffer stream'),
            DeclareLaunchArgument('gpio_rx_pin', default_value='5', description='GPIO pin for pigpio bit-bang UART RX'),
            DeclareLaunchArgument('baudrate', default_value='2000', description='UART baudrate (2000 or 2083)'),
            DeclareLaunchArgument('frame_gap_us', default_value='12000', description='Idle gap that marks frame end (us)'),
            DeclareLaunchArgument('invert_bytes', default_value='false', description='Invert each received byte with bitwise NOT'),
            DeclareLaunchArgument('log_rx_frames', default_value='false', description='Print Arduino-like RX trace per frame'),
            DeclareLaunchArgument('log_raw_hex', default_value='false', description='Include full frame hex in RX trace'),
            DeclareLaunchArgument('invert_signal', default_value='false', description='Invert physical UART signal (pigpio backend)'),
            DeclareLaunchArgument('pigpiod_host', default_value='localhost', description='pigpiod host'),
            DeclareLaunchArgument('pigpiod_port', default_value='8888', description='pigpiod TCP port'),
            DeclareLaunchArgument('poll_rate_hz', default_value='500.0', description='UART polling loop rate'),
            DeclareLaunchArgument('publish_rate_hz', default_value='50.0', description='Odometry publish rate'),
            DeclareLaunchArgument('odom_topic', default_value='/wheel/odom', description='Wheel odometry topic'),
            DeclareLaunchArgument('velocity_topic', default_value='/wheel/velocity', description='Wheel velocity topic'),
            DeclareLaunchArgument('speed_topic', default_value='/wheel/speed_kmh', description='Decoded speed topic (km/h)'),
            DeclareLaunchArgument('throttle_topic', default_value='/wheel/throttle', description='Decoded throttle topic'),
            DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame id'),
            DeclareLaunchArgument('base_link_frame', default_value='base_footprint', description='Robot base frame id'),
            DeclareLaunchArgument('speed_scale', default_value='1.0', description='Scale decoded speed in km/h'),
            DeclareLaunchArgument('speed_timeout_s', default_value='0.5', description='Speed becomes 0 when stale (seconds)'),
            DeclareLaunchArgument('use_throttle_gate', default_value='true', description='Force speed=0 when throttle is OFF'),
            DeclareLaunchArgument('forward_sign', default_value='1.0', description='Direction sign (1.0 forward, -1.0 reverse)'),
            Node(
                package='sensores',
                executable='wheel_odom_uart',
                name='wheel_odom_uart',
                output='screen',
                parameters=[
                    {'rx_backend': rx_backend},
                    {'serial_port': serial_port},
                    {'gpio_rx_pin': gpio_rx_pin},
                    {'baudrate': baudrate},
                    {'frame_gap_us': frame_gap_us},
                    {'invert_bytes': invert_bytes},
                    {'log_rx_frames': log_rx_frames},
                    {'log_raw_hex': log_raw_hex},
                    {'invert_signal': invert_signal},
                    {'pigpiod_host': pigpiod_host},
                    {'pigpiod_port': pigpiod_port},
                    {'poll_rate_hz': poll_rate_hz},
                    {'publish_rate_hz': publish_rate_hz},
                    {'odom_topic': odom_topic},
                    {'velocity_topic': velocity_topic},
                    {'speed_topic': speed_topic},
                    {'throttle_topic': throttle_topic},
                    {'odom_frame': odom_frame},
                    {'base_link_frame': base_link_frame},
                    {'speed_scale': speed_scale},
                    {'speed_timeout_s': speed_timeout_s},
                    {'use_throttle_gate': use_throttle_gate},
                    {'forward_sign': forward_sign},
                ],
            ),
        ]
    )
