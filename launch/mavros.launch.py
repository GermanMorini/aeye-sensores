import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensores_share_dir = get_package_share_directory("sensores")
    mavros_share_dir = get_package_share_directory("mavros")

    launch_web = LaunchConfiguration("launch_web")
    launch_legacy_compat = LaunchConfiguration("launch_legacy_compat")
    enable_rtk = LaunchConfiguration("enable_rtk")
    enable_rtcm_tcp = LaunchConfiguration("enable_rtcm_tcp")
    rtcm_tcp_host = LaunchConfiguration("rtcm_tcp_host")
    rtcm_tcp_port = LaunchConfiguration("rtcm_tcp_port")
    rtcm_topic = LaunchConfiguration("rtcm_topic")
    send_rtcm_topic = LaunchConfiguration("send_rtcm_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    namespace = LaunchConfiguration("namespace")
    fcu_protocol = LaunchConfiguration("fcu_protocol")
    pluginlists_yaml = LaunchConfiguration("pluginlists_yaml")
    apm_config_yaml = LaunchConfiguration("apm_config_yaml")
    config_yaml = LaunchConfiguration("config_yaml")

    default_pluginlists_yaml = os.path.join(
        sensores_share_dir, "config", "mavros_sensor_only_pluginlists.yaml"
    )
    default_apm_config_yaml = os.path.join(mavros_share_dir, "launch", "apm_config.yaml")
    default_config_yaml = os.path.join(
        sensores_share_dir, "config", "mavros_apm_overrides.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_web",
                default_value="false",
                description="Launch the sensores_web node",
            ),
            DeclareLaunchArgument(
                "launch_legacy_compat",
                default_value="true",
                description="Republish MAVROS native topics to the legacy contract",
            ),
            DeclareLaunchArgument(
                "enable_rtk",
                default_value="false",
                description="Launch the RTK bridge that feeds RTCM to the MAVROS gps_rtk plugin",
            ),
            DeclareLaunchArgument(
                "enable_rtcm_tcp",
                default_value="true",
                description="Read RTCM corrections from a TCP source",
            ),
            DeclareLaunchArgument(
                "rtcm_tcp_host",
                default_value="127.0.0.1",
                description="Host for the incoming RTCM TCP stream",
            ),
            DeclareLaunchArgument(
                "rtcm_tcp_port",
                default_value="2102",
                description="Port for the incoming RTCM TCP stream",
            ),
            DeclareLaunchArgument(
                "rtcm_topic",
                default_value="/rtcm",
                description="Optional ROS topic carrying RTCM corrections",
            ),
            DeclareLaunchArgument(
                "send_rtcm_topic",
                default_value="/mavros_node/send_rtcm",
                description="Topic consumed by the MAVROS gps_rtk plugin; override if mavros_node runs in a namespace",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/global_position/raw/fix",
                description="Native GPS topic used for RTK diagnostics and the web dashboard",
            ),
            DeclareLaunchArgument(
                "fcu_url",
                default_value="/dev/ttyACM0:921600",
                description="FCU connection URL used by MAVROS",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="",
                description="Optional GCS connection URL used by MAVROS",
            ),
            DeclareLaunchArgument(
                "tgt_system",
                default_value="1",
                description="MAVLink target system id",
            ),
            DeclareLaunchArgument(
                "tgt_component",
                default_value="1",
                description="MAVLink target component id",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for mavros_node; empty keeps canonical root-level topics",
            ),
            DeclareLaunchArgument(
                "fcu_protocol",
                default_value="v2.0",
                description="MAVLink protocol version",
            ),
            DeclareLaunchArgument(
                "pluginlists_yaml",
                default_value=default_pluginlists_yaml,
                description="Path to MAVROS plugin allow/deny list config",
            ),
            DeclareLaunchArgument(
                "config_yaml",
                default_value=default_config_yaml,
                description="Path to MAVROS runtime config",
            ),
            DeclareLaunchArgument(
                "apm_config_yaml",
                default_value=default_apm_config_yaml,
                description="Path to base MAVROS APM config",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("mavros_node/data", "/imu/data"),
                    ("mavros_node/raw/fix", "/global_position/raw/fix"),
                    ("mavros_node/velocity_local", "/local_position/velocity_local"),
                    ("mavros_node/odom", "/local_position/odom"),
                ],
                parameters=[
                    pluginlists_yaml,
                    apm_config_yaml,
                    config_yaml,
                    {
                        "fcu_url": fcu_url,
                        "gcs_url": gcs_url,
                        "tgt_system": tgt_system,
                        "tgt_component": tgt_component,
                        "fcu_protocol": fcu_protocol,
                    },
                ],
            ),
            Node(
                package="sensores",
                executable="mavros_compat_bridge",
                name="mavros_compat_bridge",
                output="screen",
                condition=IfCondition(launch_legacy_compat),
            ),
            Node(
                package="sensores",
                executable="sensores_web",
                name="sensores_web",
                output="screen",
                condition=IfCondition(launch_web),
                parameters=[
                    {"imu_topic": "/imu/data"},
                    {"gps_topic": gps_topic},
                    {"velocity_topic": "/local_position/velocity_local"},
                    {"odom_topic": "/local_position/odom"},
                ],
            ),
            Node(
                package="sensores",
                executable="rtk_bridge",
                name="rtk_bridge",
                output="screen",
                condition=IfCondition(enable_rtk),
                parameters=[
                    {"enable_rtcm_tcp": enable_rtcm_tcp},
                    {"rtcm_tcp_host": rtcm_tcp_host},
                    {"rtcm_tcp_port": rtcm_tcp_port},
                    {"rtcm_topic": rtcm_topic},
                    {"send_rtcm_topic": send_rtcm_topic},
                    {"gps_topic": gps_topic},
                ],
            ),
        ]
    )
