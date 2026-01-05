#!/usr/bin/env python3
"""
ROS 2 node: Pixhawk (ArduPilot) MAVLink -> ROS topics

Publishes:
- /imu/data        (sensor_msgs/Imu)      : accel+gyro from SCALED_IMU2 (or SCALED_IMU), orientation from ATTITUDE_QUATERNION
- /gps/fix         (sensor_msgs/NavSatFix): raw GNSS from GPS_RAW_INT
- /odom            (nav_msgs/Odometry)    : pose+twist from LOCAL_POSITION_NED + attitude (EKF)
- /velocity        (geometry_msgs/TwistStamped): linear+angular velocity (in base_link)

Frames (ROS standard):
- odom_frame:      ENU world frame (x=E, y=N, z=Up)
- base_link_frame: FLU body frame (x=Forward, y=Left, z=Up)

This assumes ArduPilot world is NED and body is FRD.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from pymavlink import mavutil


# ----------------------------
# Math helpers (no numpy needed)
# ----------------------------

def quat_norm(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)

def quat_mul(q1, q2):
    """Hamilton product. Quaternions are (w,x,y,z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def rotvec_by_quat(v: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Rotate vector v by quaternion q (active rotation)."""
    q = quat_norm(q)
    vx, vy, vz = v
    vq = (0.0, vx, vy, vz)
    rq = quat_mul(quat_mul(q, vq), quat_conj(q))
    return (rq[1], rq[2], rq[3])

def ros_quat_from_tuple(q) -> Quaternion:
    w, x, y, z = q
    out = Quaternion()
    out.w, out.x, out.y, out.z = w, x, y, z
    return out

# Frame transforms:
# NED -> ENU for vectors: [xE, yN, zU] = [y, x, -z]
def vec_ned_to_enu(v_ned: Tuple[float, float, float]) -> Tuple[float, float, float]:
    xN, yE, zD = v_ned
    return (yE, xN, -zD)

# FRD -> FLU for body vectors: [xF, yL, zU] = [x, -y, -z]
def vec_frd_to_flu(v_frd: Tuple[float, float, float]) -> Tuple[float, float, float]:
    xF, yR, zD = v_frd
    return (xF, -yR, -zD)

# Quaternion conversions:
# We want q_enu_flu: rotation from base_link (FLU) to world (ENU).
#
# ArduPilot attitude quaternion in ATTITUDE_QUATERNION is the vehicle attitude in NED with body FRD.
# Interpreting as q_ned_frd: rotation from body(FRD) to world(NED).
#
# Convert frames using fixed 90deg/axis swaps:
# world: NED -> ENU is a basis change with matrix:
#   ENU = [ 0 1 0;
#           1 0 0;
#           0 0 -1 ] * NED
# body:  FRD -> FLU:
#   FLU = [ 1 0 0;
#           0 -1 0;
#           0 0 -1 ] * FRD
#
# The corresponding quaternions for these basis changes:
# - q_enu_ned : rotate NED axes into ENU axes (passive basis change)
# - q_flu_frd : rotate FRD axes into FLU axes
#
# Easiest robust approach without matrix libs:
# Convert by sandwiching: q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu
# where q_frd_flu is inverse of q_flu_frd.
#
# For these specific axis flips/swaps, the quaternions are:
# q_flu_frd = (0, 1, 0, 0)?? No—axis flips aren't a pure rotation if you treat it as a handedness change,
# but here both frames remain right-handed (FRD and FLU are both right-handed, NED and ENU are right-handed).
# They are related by 180° about X for FRD->FLU, and 180° about (1,1,0)/sqrt(2) then something for NED->ENU.
#
# To avoid subtle mistakes, we compute q_enu_flu by transforming basis vectors explicitly.

def quat_from_rotation_matrix(R) -> Tuple[float,float,float,float]:
    """Convert 3x3 rotation matrix to quaternion (w,x,y,z)."""
    # R is list of lists [[r00,r01,r02],...]
    r00,r01,r02 = R[0]
    r10,r11,r12 = R[1]
    r20,r21,r22 = R[2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (r21 - r12) / S
        y = (r02 - r20) / S
        z = (r10 - r01) / S
    elif (r00 > r11) and (r00 > r22):
        S = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / S
        x = 0.25 * S
        y = (r01 + r10) / S
        z = (r02 + r20) / S
    elif r11 > r22:
        S = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / S
        x = (r01 + r10) / S
        y = 0.25 * S
        z = (r12 + r21) / S
    else:
        S = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / S
        x = (r02 + r20) / S
        y = (r12 + r21) / S
        z = 0.25 * S
    return quat_norm((w,x,y,z))

def rotation_matrix_from_quat(q: Tuple[float,float,float,float]):
    w,x,y,z = quat_norm(q)
    return [
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)],
    ]

def mat_mul(A,B):
    out = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
    return out

def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]

def quat_ned_frd_to_enu_flu(q_ned_frd: Tuple[float,float,float,float]) -> Tuple[float,float,float,float]:
    """
    Convert rotation from body(FRD)->world(NED) into body(FLU)->world(ENU).
    Using: R_enu_flu = T_world * R_ned_frd * T_body^-1
    where:
      T_world maps NED vectors into ENU vectors.
      T_body maps FRD vectors into FLU vectors.
    """
    # T_world: ENU = T_world * NED
    T_world = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0,-1.0],
    ]
    # T_body: FLU = T_body * FRD
    T_body = [
        [1.0, 0.0, 0.0],
        [0.0,-1.0, 0.0],
        [0.0, 0.0,-1.0],
    ]

    R_ned_frd = rotation_matrix_from_quat(q_ned_frd)
    # R_enu_flu = T_world * R_ned_frd * (T_body^-1)
    # T_body is orthonormal with det=+1 here, so inverse = transpose
    R_enu_flu = mat_mul(mat_mul(T_world, R_ned_frd), transpose(T_body))
    return quat_from_rotation_matrix(R_enu_flu)


# ----------------------------
# ROS node
# ----------------------------

class PixhawkMavlinkNode(Node):
    def __init__(self):
        super().__init__('pixhawk_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('gps_frame', 'gps')
        self.declare_parameter('publish_rate_hz', 200.0)  # loop tick (not MAVLink rate)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 20)
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_vel  = self.create_publisher(TwistStamped, '/velocity', 20)

        # MAVLink connect
        self.get_logger().info(f'Connecting MAVLink on {port} @ {baud}...')
        self.mav = mavutil.mavlink_connection(
            port,
            baud=baud,
            source_system=255,
            source_component=0,
            autoreconnect=True,
        )

        self.get_logger().info('Waiting for heartbeat...')
        hb = self.mav.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError('No heartbeat from Pixhawk (timeout). Check port/baud/cable.')
        self.get_logger().info(f'Heartbeat OK (sys={self.mav.target_system}, comp={self.mav.target_component})')

        # Ask message rates (Hz)
        self._set_message_rate('ATTITUDE_QUATERNION', 50)
        self._set_message_rate('SCALED_IMU2', 100)
        self._set_message_rate('SCALED_IMU', 100)   # fallback if IMU2 not sent
        self._set_message_rate('LOCAL_POSITION_NED', 50)
        self._set_message_rate('GPS_RAW_INT', 10)

        # Latest state cache
        self._last_accel_flu: Optional[Tuple[float,float,float]] = None
        self._last_gyro_flu: Optional[Tuple[float,float,float]] = None
        self._last_orientation_enu_flu: Optional[Tuple[float,float,float,float]] = None

        self._last_pos_enu: Optional[Tuple[float,float,float]] = None
        self._last_vel_enu: Optional[Tuple[float,float,float]] = None
        self._last_angvel_flu: Optional[Tuple[float,float,float]] = None

        self._last_gps_log = 0.0

        # Timer loop
        tick = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(tick, self._spin_once)

        self.get_logger().info('Node running.')

    def _set_message_rate(self, msg_name: str, hz: float):
        """Use MAV_CMD_SET_MESSAGE_INTERVAL to request a message at hz."""
        try:
            msg_id = getattr(mavutil.mavlink, f'MAVLINK_MSG_ID_{msg_name}')
        except AttributeError:
            # Some dialects might not include the constant
            return

        # Interval in microseconds. 0 disables, -1 default.
        interval_us = int(1e6 / hz) if hz > 0 else 0
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    def _spin_once(self):
        # Drain a few MAVLink messages per tick to keep up
        for _ in range(50):
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            t = msg.get_type()
            if t in ('SCALED_IMU2', 'SCALED_IMU'):
                self._handle_scaled_imu(msg)
            elif t == 'ATTITUDE_QUATERNION':
                self._handle_attitude_quaternion(msg)
            elif t == 'LOCAL_POSITION_NED':
                self._handle_local_position_ned(msg)
            elif t == 'GPS_RAW_INT':
                self._handle_gps_raw_int(msg)

        # Publish combined outputs when possible
        self._publish_imu_if_ready()
        self._publish_odom_if_ready()

    def _handle_scaled_imu(self, msg):
        # accel in mg -> m/s^2, gyro in mrad/s -> rad/s
        # msg fields: xacc,yacc,zacc (mg), xgyro,ygyro,zgyro (mrad/s)
        ax_frd = (msg.xacc / 1000.0) * 9.80665
        ay_frd = (msg.yacc / 1000.0) * 9.80665
        az_frd = (msg.zacc / 1000.0) * 9.80665

        gx_frd = (msg.xgyro / 1000.0)
        gy_frd = (msg.ygyro / 1000.0)
        gz_frd = (msg.zgyro / 1000.0)

        # body FRD -> FLU
        self._last_accel_flu = vec_frd_to_flu((ax_frd, ay_frd, az_frd))
        self._last_gyro_flu  = vec_frd_to_flu((gx_frd, gy_frd, gz_frd))

    def _handle_attitude_quaternion(self, msg):
        # MAVLink ATTITUDE_QUATERNION: q1,q2,q3,q4 correspond to w,x,y,z
        q_ned_frd = quat_norm((msg.q1, msg.q2, msg.q3, msg.q4))
        q_enu_flu = quat_ned_frd_to_enu_flu(q_ned_frd)
        self._last_orientation_enu_flu = q_enu_flu

        # Angular rates in rad/s (rollspeed,pitchspeed,yawspeed) are body rates.
        # They are in FRD convention. Convert to FLU.
        self._last_angvel_flu = vec_frd_to_flu((msg.rollspeed, msg.pitchspeed, msg.yawspeed))

    def _handle_local_position_ned(self, msg):
        # Position and velocity in NED (meters, m/s)
        pos_ned = (msg.x, msg.y, msg.z)      # x=N, y=E, z=D
        vel_ned = (msg.vx, msg.vy, msg.vz)   # vx=N, vy=E, vz=D

        self._last_pos_enu = vec_ned_to_enu(pos_ned)
        self._last_vel_enu = vec_ned_to_enu(vel_ned)

    def _handle_gps_raw_int(self, msg):
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = self.gps_frame

        gps.latitude = msg.lat / 1e7
        gps.longitude = msg.lon / 1e7
        gps.altitude = msg.alt / 1000.0

        gps.status.status = NavSatStatus.STATUS_FIX if msg.fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS

        eph_m = (msg.eph / 100.0) if getattr(msg, 'eph', 65535) != 65535 else 10.0
        epv_m = (msg.epv / 100.0) if getattr(msg, 'epv', 65535) != 65535 else 10.0
        gps.position_covariance = [
            eph_m*eph_m, 0.0,      0.0,
            0.0,      eph_m*eph_m, 0.0,
            0.0,      0.0,      epv_m*epv_m
        ]
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_gps.publish(gps)

        now = time.time()
        if now - self._last_gps_log > 5.0:
            self._last_gps_log = now
            self.get_logger().info(
                f'GPS fix={msg.fix_type} sats={msg.satellites_visible} '
                f'lat={gps.latitude:.6f} lon={gps.longitude:.6f} alt={gps.altitude:.1f}m'
            )

    def _publish_imu_if_ready(self):
        if self._last_accel_flu is None or self._last_gyro_flu is None:
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame

        ax, ay, az = self._last_accel_flu
        gx, gy, gz = self._last_gyro_flu

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        if self._last_orientation_enu_flu is not None:
            imu.orientation = ros_quat_from_tuple(self._last_orientation_enu_flu)
            imu.orientation_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  0.01
            ]
        else:
            imu.orientation_covariance[0] = -1.0

        imu.angular_velocity_covariance = [
            0.001, 0.0,   0.0,
            0.0,   0.001, 0.0,
            0.0,   0.0,   0.001
        ]
        imu.linear_acceleration_covariance = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.02
        ]

        self.pub_imu.publish(imu)

    def _publish_odom_if_ready(self):
        if self._last_pos_enu is None or self._last_vel_enu is None:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        px, py, pz = self._last_pos_enu
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz

        if self._last_orientation_enu_flu is not None:
            odom.pose.pose.orientation = ros_quat_from_tuple(self._last_orientation_enu_flu)
        else:
            odom.pose.pose.orientation.w = 1.0

        # Twist: express linear velocity in base_link (FLU) for common ROS tooling expectations
        vx_e, vy_n, vz_u = self._last_vel_enu  # world ENU
        if self._last_orientation_enu_flu is not None:
            # Convert world vel -> body vel: v_body = R^T * v_world
            q = self._last_orientation_enu_flu
            # Using quaternion inverse to rotate into body frame
            v_body = rotvec_by_quat((vx_e, vy_n, vz_u), quat_conj(q))
            odom.twist.twist.linear.x = v_body[0]
            odom.twist.twist.linear.y = v_body[1]
            odom.twist.twist.linear.z = v_body[2]
        else:
            odom.twist.twist.linear.x = vx_e
            odom.twist.twist.linear.y = vy_n
            odom.twist.twist.linear.z = vz_u

        if self._last_angvel_flu is not None:
            wx, wy, wz = self._last_angvel_flu
            odom.twist.twist.angular.x = wx
            odom.twist.twist.angular.y = wy
            odom.twist.twist.angular.z = wz

        self.pub_odom.publish(odom)

        vel = TwistStamped()
        vel.header = odom.header
        vel.header.frame_id = self.base_link_frame
        vel.twist = odom.twist.twist
        self.pub_vel.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PixhawkMavlinkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
