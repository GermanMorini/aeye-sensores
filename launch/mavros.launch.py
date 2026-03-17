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
                default_value="mavros",
                description="Namespace for mavros_node",
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
                # Compat layer: keep legacy project topic contract while MAVROS
                # publishes under mavros_node/* in recent ROS 2 Humble builds.
                remappings=[
                    ("mavros_node/data", "imu/data"),
                    ("mavros_node/raw/fix", "global_position/raw/fix"),
                    ("mavros_node/velocity_local", "local_position/velocity_local"),
                    ("mavros_node/odom", "local_position/odom"),
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
                executable="sensores_web",
                name="sensores_web",
                output="screen",
                condition=IfCondition(launch_web),
                parameters=[
                    {"imu_topic": "/mavros/imu/data"},
                    {"gps_topic": "/mavros/global_position/raw/fix"},
                    {"velocity_topic": "/mavros/local_position/velocity_local"},
                    {"odom_topic": "/mavros/local_position/odom"},
                ],
            ),
        ]
    )
