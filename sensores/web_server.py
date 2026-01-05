#!/usr/bin/env python3
"""
ROS 2 node that serves a local HTML dashboard and exposes Pixhawk data as JSON.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def _stamp_to_float(stamp):
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PixhawkWebServer(Node):
    def __init__(self):
        super().__init__('sensores_web')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('velocity_topic', '/velocity')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8000)
        self.declare_parameter('html_path', '')

        imu_topic = self.get_parameter('imu_topic').value
        gps_topic = self.get_parameter('gps_topic').value
        velocity_topic = self.get_parameter('velocity_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        http_host = self.get_parameter('http_host').value
        http_port = self.get_parameter('http_port').value
        html_path = self.get_parameter('html_path').value

        if not html_path:
            share_dir = Path(get_package_share_directory('sensores'))
            html_path = str(share_dir / 'pixhawk_dashboard.html')

        self._html_content = self._load_html(html_path)
        self._data_lock = threading.Lock()
        self._data = {
            'imu': None,
            'gps': None,
            'velocity': None,
            'odom': None,
        }

        self.create_subscription(Imu, imu_topic, self._imu_cb, 20)
        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, 20)
        self.create_subscription(TwistStamped, velocity_topic, self._velocity_cb, 20)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)

        self._httpd = self._start_http_server(http_host, int(http_port))
        self.get_logger().info(
            f'Web server running at http://{http_host}:{http_port}'
        )

    def _load_html(self, html_path: str) -> str:
        try:
            return Path(html_path).read_text(encoding='utf-8')
        except Exception as exc:
            self.get_logger().error(f'Failed to read HTML: {exc}')
            return '<html><body>Missing HTML file.</body></html>'

    def _start_http_server(self, host: str, port: int) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                if self.path in ('/', '/index.html'):
                    self._send_response(200, 'text/html', node._html_content)
                    return
                if self.path.startswith('/data'):
                    payload = node._get_snapshot()
                    self._send_response(200, 'application/json', payload)
                    return
                self._send_response(404, 'text/plain', 'Not Found')

            def _send_response(self, code, content_type, body):
                if isinstance(body, str):
                    body = body.encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):  # noqa: A003
                return

        httpd = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        return httpd

    def _get_snapshot(self) -> str:
        with self._data_lock:
            return json.dumps(self._data)

    def _imu_cb(self, msg: Imu):
        data = {
            'stamp': _stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
        }
        with self._data_lock:
            self._data['imu'] = data

    def _gps_cb(self, msg: NavSatFix):
        data = {
            'stamp': _stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'status': int(msg.status.status),
            'service': int(msg.status.service),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
        }
        with self._data_lock:
            self._data['gps'] = data

    def _velocity_cb(self, msg: TwistStamped):
        data = {
            'stamp': _stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'linear': {
                'x': msg.twist.linear.x,
                'y': msg.twist.linear.y,
                'z': msg.twist.linear.z,
            },
            'angular': {
                'x': msg.twist.angular.x,
                'y': msg.twist.angular.y,
                'z': msg.twist.angular.z,
            },
        }
        with self._data_lock:
            self._data['velocity'] = data

    def _odom_cb(self, msg: Odometry):
        data = {
            'stamp': _stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w,
            },
            'linear': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z,
            },
            'angular': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z,
            },
        }
        with self._data_lock:
            self._data['odom'] = data

    def destroy_node(self):
        if hasattr(self, '_httpd') and self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkWebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
