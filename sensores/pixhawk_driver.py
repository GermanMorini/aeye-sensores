#!/usr/bin/env python3
"""
ROS 2 node that reads Pixhawk6X data via MAVLink and publishes standard
ROS 2 topics (sensor_msgs/Imu, sensor_msgs/NavSatFix).
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from pymavlink import mavutil


class PixhawkReader(Node):
    def __init__(self):
        super().__init__('sensores')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('frame_id_gps', 'gps')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('velocity_topic', '/velocity')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('use_raw_imu', False)
        self.declare_parameter('use_mavlink_odometry', True)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id_imu = self.get_parameter('frame_id_imu').value
        self.frame_id_gps = self.get_parameter('frame_id_gps').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        imu_topic = self.get_parameter('imu_topic').value
        gps_topic = self.get_parameter('gps_topic').value
        velocity_topic = self.get_parameter('velocity_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.use_raw_imu = self.get_parameter('use_raw_imu').value
        self.use_mavlink_odometry = self.get_parameter('use_mavlink_odometry').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, velocity_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # Connect to Pixhawk
        self.get_logger().info(
            f'Connecting to Pixhawk at {serial_port}:{baudrate}...'
        )
        try:
            self.mav = mavutil.mavlink_connection(
                f'{serial_port}',
                baud=baudrate,
                source_system=255,
                source_component=0,
            )

            # Wait for heartbeat
            self.get_logger().info('Waiting for Pixhawk heartbeat...')
            self.mav.wait_heartbeat(timeout=10)
            self.get_logger().info(
                'Heartbeat received from system %s, component %s',
                self.mav.target_system,
                self.mav.target_component,
            )

            # Request data streams
            self.request_data_streams()

        except Exception as exc:
            self.get_logger().error(f'Error connecting to Pixhawk: {exc}')
            raise

        # Timer to read MAVLink messages
        self.create_timer(0.01, self.read_mavlink)  # 100 Hz

        # State for orientation / angular velocity (Pixhawk EKF)
        self.current_orientation = None
        self.current_angular_velocity = None
        self.warned_odom_frame = False

        self.get_logger().info('Pixhawk Reader started successfully')

    def request_data_streams(self):
        """Request data streams from Pixhawk."""
        # IMU at 50 Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            50,
            1,
        )

        # GPS at 10 Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1,
        )

        # Quaternion attitude at ~50 Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            50,
            1,
        )

        # EKF odometry via message interval (if supported)
        try:
            self._request_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ODOMETRY,
                50,
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Unable to request ODOMETRY message interval: {exc}'
            )

        self.get_logger().info('Data streams requested')

    def read_mavlink(self):
        """Read MAVLink messages and publish ROS 2 topics."""
        try:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return

            msg_type = msg.get_type()

            # Raw IMU data (optional; disabled by default)
            if msg_type in ('SCALED_IMU', 'SCALED_IMU2'):
                if self.use_raw_imu:
                    self.publish_imu(msg)
            # Attitude quaternion
            elif msg_type == 'ATTITUDE_QUATERNION':
                self.publish_attitude(msg)
            # EKF odometry (pose/velocity)
            elif msg_type == 'ODOMETRY':
                if self.use_mavlink_odometry:
                    self.publish_mavlink_odometry(msg)
            # Local position/velocity in NED
            elif msg_type == 'LOCAL_POSITION_NED':
                if not self.use_mavlink_odometry:
                    self.publish_local_velocity(msg)
            # GPS data
            elif msg_type == 'GPS_RAW_INT':
                self.publish_gps(msg)

        except Exception as exc:
            self.get_logger().error(f'Error reading MAVLink: {exc}')

    def publish_imu(self, msg):
        """Publish IMU data."""
        imu_msg = Imu()

        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id_imu

        # Linear acceleration: milli-g to m/s^2. FRD -> FLU.
        imu_msg.linear_acceleration.x = msg.xacc / 1000.0 * 9.81
        imu_msg.linear_acceleration.y = -msg.yacc / 1000.0 * 9.81
        imu_msg.linear_acceleration.z = -msg.zacc / 1000.0 * 9.81

        # Angular velocity: milli-rad/s to rad/s. FRD -> FLU.
        imu_msg.angular_velocity.x = msg.xgyro / 1000.0
        imu_msg.angular_velocity.y = -msg.ygyro / 1000.0
        imu_msg.angular_velocity.z = -msg.zgyro / 1000.0

        # Covariances (tune if needed)
        imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]

        # Orientation not provided here
        imu_msg.orientation_covariance[0] = -1

        self.imu_pub.publish(imu_msg)

    def publish_attitude(self, msg):
        """Publish IMU orientation from ATTITUDE_QUATERNION."""
        imu_msg = Imu()

        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id_imu

        # Orientation: Pixhawk NED -> ROS ENU.
        q_ned = (msg.q1, msg.q2, msg.q3, msg.q4)
        q_enu = self._quat_ned_to_enu(q_ned)
        imu_msg.orientation.w = q_enu[0]
        imu_msg.orientation.x = q_enu[1]
        imu_msg.orientation.y = q_enu[2]
        imu_msg.orientation.z = q_enu[3]

        # Angular velocity: FRD -> FLU.
        imu_msg.angular_velocity.x = msg.rollspeed
        imu_msg.angular_velocity.y = -msg.pitchspeed
        imu_msg.angular_velocity.z = -msg.yawspeed

        # Covariances (tune if needed)
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]

        # Linear acceleration not available here
        imu_msg.linear_acceleration_covariance[0] = -1

        self.imu_pub.publish(imu_msg)
        self.current_orientation = imu_msg.orientation
        self.current_angular_velocity = imu_msg.angular_velocity

    def publish_gps(self, msg):
        """Publish GPS data."""
        gps_msg = NavSatFix()

        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = self.frame_id_gps

        # GPS position: lat/lon in deg, alt in meters
        gps_msg.latitude = msg.lat / 1e7
        gps_msg.longitude = msg.lon / 1e7
        gps_msg.altitude = msg.alt / 1000.0

        # Status
        gps_msg.status.status = (
            NavSatStatus.STATUS_FIX if msg.fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        )
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariance based on eph/epv (cm -> m)
        eph_m = msg.eph / 100.0 if msg.eph != 65535 else 10.0
        epv_m = msg.epv / 100.0 if msg.epv != 65535 else 10.0

        gps_msg.position_covariance = [
            eph_m**2, 0.0, 0.0,
            0.0, eph_m**2, 0.0,
            0.0, 0.0, epv_m**2,
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(gps_msg)

        # Log every 5 seconds
        if int(time.time()) % 5 == 0:
            self.get_logger().info(
                'GPS: lat=%.6f, lon=%.6f, alt=%.1fm, fix_type=%s, sats=%s',
                gps_msg.latitude,
                gps_msg.longitude,
                gps_msg.altitude,
                msg.fix_type,
                msg.satellites_visible,
            )

    def publish_local_velocity(self, msg):
        """Publish global velocity (ENU) using Pixhawk LOCAL_POSITION_NED."""
        # Velocity in NED (m/s)
        vx_n = msg.vx
        vy_e = msg.vy
        vz_d = msg.vz

        # Convert to ENU (ROS): x=East, y=North, z=Up
        vel_world_x = vy_e
        vel_world_y = vx_n
        vel_world_z = -vz_d

        vel_msg = TwistStamped()
        vel_msg.header = Header()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = self.odom_frame
        vel_msg.twist.linear.x = vel_world_x
        vel_msg.twist.linear.y = vel_world_y
        vel_msg.twist.linear.z = vel_world_z

        if self.current_angular_velocity is not None:
            vel_msg.twist.angular = self.current_angular_velocity

        self.velocity_pub.publish(vel_msg)

        # Position in NED (m)
        pos_n = msg.x
        pos_e = msg.y
        pos_d = msg.z

        # Convert to ENU
        pos_world_x = pos_e
        pos_world_y = pos_n
        pos_world_z = -pos_d

        odom_msg = Odometry()
        odom_msg.header = vel_msg.header
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.pose.pose.position.x = pos_world_x
        odom_msg.pose.pose.position.y = pos_world_y
        odom_msg.pose.pose.position.z = pos_world_z
        if self.current_orientation is not None:
            odom_msg.pose.pose.orientation = self.current_orientation
        else:
            odom_msg.pose.pose.orientation.w = 1.0

        if self.current_orientation is not None:
            yaw = self._quat_to_yaw(self.current_orientation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            vel_body_x = vel_world_x * cos_yaw + vel_world_y * sin_yaw
            vel_body_y = -vel_world_x * sin_yaw + vel_world_y * cos_yaw
        else:
            vel_body_x = vel_world_x
            vel_body_y = vel_world_y

        odom_msg.twist.twist.linear.x = vel_body_x
        odom_msg.twist.twist.linear.y = vel_body_y
        odom_msg.twist.twist.linear.z = vel_world_z
        if self.current_angular_velocity is not None:
            odom_msg.twist.twist.angular = self.current_angular_velocity

        self.odom_pub.publish(odom_msg)

    def publish_mavlink_odometry(self, msg):
        """Publish odometry using MAVLink ODOMETRY (EKF state)."""
        # NOTE: Most Pixhawk setups publish MAV_FRAME_LOCAL_NED. Convert to ENU.
        if msg.frame_id != mavutil.mavlink.MAV_FRAME_LOCAL_NED:
            if not self.warned_odom_frame:
                self.get_logger().warning(
                    'ODOMETRY frame_id=%s not LOCAL_NED; conversion may be wrong.',
                    msg.frame_id,
                )
                self.warned_odom_frame = True

        pos_n = msg.x
        pos_e = msg.y
        pos_d = msg.z

        pos_world_x = pos_e
        pos_world_y = pos_n
        pos_world_z = -pos_d

        # Orientation: NED -> ENU.
        q_ned = (msg.q1, msg.q2, msg.q3, msg.q4)
        q_enu = self._quat_ned_to_enu(q_ned)

        # Velocity in NED (m/s) -> ENU (world)
        vx_n = msg.vx
        vy_e = msg.vy
        vz_d = msg.vz
        vel_world_x = vy_e
        vel_world_y = vx_n
        vel_world_z = -vz_d

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.pose.pose.position.x = pos_world_x
        odom_msg.pose.pose.position.y = pos_world_y
        odom_msg.pose.pose.position.z = pos_world_z
        odom_msg.pose.pose.orientation.w = q_enu[0]
        odom_msg.pose.pose.orientation.x = q_enu[1]
        odom_msg.pose.pose.orientation.y = q_enu[2]
        odom_msg.pose.pose.orientation.z = q_enu[3]

        # Covariances: MAVLink uses a 6x6 upper-triangular array (21 values).
        if len(msg.pose_covariance) == 21:
            odom_msg.pose.covariance = self._unpack_upper_triangular(msg.pose_covariance)
        if len(msg.velocity_covariance) == 21:
            odom_msg.twist.covariance = self._unpack_upper_triangular(msg.velocity_covariance)

        # Twist in body frame is not guaranteed; publish world-frame linear velocity.
        odom_msg.twist.twist.linear.x = vel_world_x
        odom_msg.twist.twist.linear.y = vel_world_y
        odom_msg.twist.twist.linear.z = vel_world_z
        odom_msg.twist.twist.angular.x = msg.rollspeed
        odom_msg.twist.twist.angular.y = msg.pitchspeed
        odom_msg.twist.twist.angular.z = msg.yawspeed

        self.odom_pub.publish(odom_msg)

        vel_msg = TwistStamped()
        vel_msg.header = odom_msg.header
        vel_msg.twist.linear.x = vel_world_x
        vel_msg.twist.linear.y = vel_world_y
        vel_msg.twist.linear.z = vel_world_z
        vel_msg.twist.angular.x = msg.rollspeed
        vel_msg.twist.angular.y = msg.pitchspeed
        vel_msg.twist.angular.z = msg.yawspeed
        self.velocity_pub.publish(vel_msg)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Convert quaternion to yaw (assumes planar motion)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _request_message_interval(self, message_id: int, rate_hz: float):
        """Request a MAVLink message at a specific rate using command_long."""
        if rate_hz <= 0:
            return
        interval_us = int(1_000_000 / rate_hz)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    @staticmethod
    def _quat_multiply(q1, q2):
        """Multiply two quaternions (w, x, y, z)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    @classmethod
    def _quat_ned_to_enu(cls, q_ned):
        """Convert quaternion from NED to ENU."""
        q_enu_ned = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
        q_enu = cls._quat_multiply(q_enu_ned, q_ned)
        norm = math.sqrt(sum(c * c for c in q_enu))
        if norm == 0.0:
            return (1.0, 0.0, 0.0, 0.0)
        return tuple(c / norm for c in q_enu)

    @staticmethod
    def _unpack_upper_triangular(upper_tri):
        """Convert 6x6 upper-triangular (21) to full row-major (36)."""
        cov = [0.0] * 36
        idx = 0
        for row in range(6):
            for col in range(row, 6):
                cov[row * 6 + col] = upper_tri[idx]
                cov[col * 6 + row] = upper_tri[idx]
                idx += 1
        return cov


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PixhawkReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Error: {exc}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
