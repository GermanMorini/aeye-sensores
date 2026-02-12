#!/usr/bin/env python3
"""ROS 2 wheel odometry node from UART sniffer frames."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

import serial
from serial import SerialException


class WheelUartOdomNode(Node):
    """Decode wheel speed/throttle from UART frames and publish ROS odometry."""

    def __init__(self) -> None:
        super().__init__('wheel_odom_uart')

        self.declare_parameter('rx_backend', 'pigpio')
        self.declare_parameter('serial_port', '/dev/ttyAMA2')
        self.declare_parameter('baudrate', 2000)
        self.declare_parameter('frame_gap_us', 12000)
        self.declare_parameter('poll_rate_hz', 500.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_frame_bytes', 256)
        self.declare_parameter('read_chunk_size', 128)
        self.declare_parameter('invert_bytes', False)  # Debug-only, byte-level post decode.
        self.declare_parameter('log_rx_frames', False)  # Arduino-like RX trace per decoded frame.
        self.declare_parameter('log_raw_hex', False)  # Include full frame payload in hex when log_rx_frames=true.
        self.declare_parameter('invert_signal', True)  # Physical UART logic inversion (pigpio backend).
        self.declare_parameter('gpio_rx_pin', 5)
        self.declare_parameter('pigpiod_host', 'localhost')
        self.declare_parameter('pigpiod_port', 8888)
        self.declare_parameter('pigpio_data_bits', 8)

        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('velocity_topic', '/wheel/velocity')
        self.declare_parameter('speed_topic', '/wheel/speed_kmh')
        self.declare_parameter('throttle_topic', '/wheel/throttle')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_footprint')

        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('forward_sign', 1.0)
        self.declare_parameter('speed_timeout_s', 0.5)
        self.declare_parameter('use_throttle_gate', True)

        self.rx_backend = str(self.get_parameter('rx_backend').value).strip().lower()
        port = str(self.get_parameter('serial_port').value)
        baudrate = int(self.get_parameter('baudrate').value)

        self.frame_gap_s = float(self.get_parameter('frame_gap_us').value) / 1e6
        self.max_frame_bytes = int(self.get_parameter('max_frame_bytes').value)
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.invert_bytes = bool(self.get_parameter('invert_bytes').value)
        self.log_rx_frames = bool(self.get_parameter('log_rx_frames').value)
        self.log_raw_hex = bool(self.get_parameter('log_raw_hex').value)
        self.invert_signal = bool(self.get_parameter('invert_signal').value)
        self.gpio_rx_pin = int(self.get_parameter('gpio_rx_pin').value)
        self.pigpiod_host = str(self.get_parameter('pigpiod_host').value)
        self.pigpiod_port = int(self.get_parameter('pigpiod_port').value)
        self.pigpio_data_bits = int(self.get_parameter('pigpio_data_bits').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.velocity_topic = str(self.get_parameter('velocity_topic').value)
        self.speed_topic = str(self.get_parameter('speed_topic').value)
        self.throttle_topic = str(self.get_parameter('throttle_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_link_frame = str(self.get_parameter('base_link_frame').value)

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.forward_sign = float(self.get_parameter('forward_sign').value)
        self.speed_timeout_s = float(self.get_parameter('speed_timeout_s').value)
        self.use_throttle_gate = bool(self.get_parameter('use_throttle_gate').value)

        poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 20)
        self.pub_vel = self.create_publisher(TwistStamped, self.velocity_topic, 20)
        self.pub_speed = self.create_publisher(Float32, self.speed_topic, 20)
        self.pub_throttle = self.create_publisher(Bool, self.throttle_topic, 20)

        self._frame = bytearray()
        self._last_byte_t = 0.0

        self._last_speed_kmh: Optional[float] = None
        self._last_speed_update_t = 0.0

        self._have_throttle = False
        self._last_throttle = False

        self._x = 0.0
        self._last_publish_ros_time = self.get_clock().now()

        self.serial = None
        self.pigpio = None

        if self.rx_backend == 'serial':
            if self.invert_signal:
                raise RuntimeError(
                    'invert_signal=true is not supported by Linux UART via pyserial. '
                    'Use rx_backend:=pigpio for software inversion or external hardware inverter.'
                )
            try:
                self.serial = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0,
                )
            except SerialException as exc:
                raise RuntimeError(f'Cannot open UART {port} @ {baudrate}: {exc}') from exc
        elif self.rx_backend == 'pigpio':
            try:
                import pigpio as pigpio_mod
            except ImportError as exc:
                raise RuntimeError(
                    'rx_backend=pigpio requires python module pigpio and pigpiod daemon.'
                ) from exc

            self.pigpio = pigpio_mod.pi(self.pigpiod_host, self.pigpiod_port)
            if not self.pigpio.connected:
                raise RuntimeError(
                    f'Cannot connect to pigpiod at {self.pigpiod_host}:{self.pigpiod_port}. '
                    'Start pigpiod service first.'
                )

            open_status = self.pigpio.bb_serial_read_open(self.gpio_rx_pin, baudrate, self.pigpio_data_bits)
            if open_status != 0:
                self.pigpio.stop()
                self.pigpio = None
                raise RuntimeError(
                    f'pigpio bb_serial_read_open failed on GPIO{self.gpio_rx_pin} with code {open_status}.'
                )

            if not hasattr(self.pigpio, 'bb_serial_invert'):
                self.pigpio.bb_serial_read_close(self.gpio_rx_pin)
                self.pigpio.stop()
                self.pigpio = None
                raise RuntimeError(
                    'Your pigpio version does not support bb_serial_invert; '
                    'update pigpio or use a hardware inverter.'
                )

            invert_status = self.pigpio.bb_serial_invert(self.gpio_rx_pin, 1 if self.invert_signal else 0)
            if invert_status != 0:
                self.pigpio.bb_serial_read_close(self.gpio_rx_pin)
                self.pigpio.stop()
                self.pigpio = None
                raise RuntimeError(
                    f'pigpio bb_serial_invert failed on GPIO{self.gpio_rx_pin} with code {invert_status}.'
                )
        else:
            raise RuntimeError(f'Unsupported rx_backend: {self.rx_backend} (use "serial" or "pigpio").')

        self.create_timer(1.0 / max(poll_rate_hz, 1.0), self._poll_uart)
        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._publish_odom)

        if self.rx_backend == 'serial':
            self.get_logger().info(
                f'Wheel UART odom backend=serial on {port} @ {baudrate} '
                f'(gap={self.frame_gap_s * 1e3:.1f} ms, invert_bytes={self.invert_bytes})'
            )
        else:
            self.get_logger().info(
                f'Wheel UART odom backend=pigpio gpio_rx=GPIO{self.gpio_rx_pin} @ {baudrate} '
                f'(gap={self.frame_gap_s * 1e3:.1f} ms, invert_signal={self.invert_signal}, '
                f'invert_bytes={self.invert_bytes})'
            )

    @staticmethod
    def _speed_from_b16_b17(b16: int, b17: int) -> int:
        if b16 == 0xDB:
            if b17 in (0xB2, 0x92):
                return 2
        elif b16 == 0x5B:
            if b17 in (0xD9, 0x59):
                return 3
            if b17 in (0xC9, 0x49, 0x96):
                return 4
            if b17 in (0xB2, 0x92):
                return 5
        elif b16 == 0xCB:
            if b17 in (0x59, 0x5B, 0x4B):
                return 6
            if b17 in (0xB6, 0x49, 0xB2):
                return 7
            if b17 in (0x92, 0x5B, 0xDB):
                return 8
        elif b16 == 0x4B:
            if b17 in (0xB6, 0xC9, 0x59, 0x49):
                return 9
            if b17 in (0x92, 0x96, 0xB2):
                return 10
        elif b16 == 0xD9:
            if b17 in (0x9B, 0x5B, 0xD9):
                return 11
            if b17 in (0xB6, 0xC9, 0x59):
                return 12
            if b17 in (0xB2, 0x96, 0x92):
                return 13
        elif b16 == 0x59:
            if b17 == 0xD9:
                return 14
        return -1

    def _poll_uart(self) -> None:
        now = time.monotonic()

        try:
            raw = b''
            if self.rx_backend == 'serial' and self.serial is not None:
                available = self.serial.in_waiting
                if available > 0:
                    raw = self.serial.read(min(available, self.read_chunk_size))
            elif self.rx_backend == 'pigpio' and self.pigpio is not None:
                count, data = self.pigpio.bb_serial_read(self.gpio_rx_pin)
                if count < 0:
                    self.get_logger().error(f'pigpio bb_serial_read error code: {count}')
                    return
                if count > 0:
                    raw = bytes(data[:count])

            if raw:
                if self.invert_bytes:
                    raw = bytes((~b) & 0xFF for b in raw)

                for b in raw:
                    if len(self._frame) >= self.max_frame_bytes:
                        self._frame.clear()
                        self._last_byte_t = 0.0
                        self.get_logger().warn('Frame overflow; dropping buffered bytes.')
                        break
                    self._frame.append(b)
                    self._last_byte_t = now

            if self._frame and self._last_byte_t > 0.0 and (now - self._last_byte_t) >= self.frame_gap_s:
                self._handle_frame(bytes(self._frame))
                self._frame.clear()
                self._last_byte_t = 0.0

        except SerialException as exc:
            self.get_logger().error(f'UART read error: {exc}')
        except Exception as exc:
            self.get_logger().error(f'RX backend error: {exc}')

    def _handle_frame(self, frame: bytes) -> None:
        if len(frame) < 19:
            return

        b8 = frame[8]
        b16 = frame[16]
        b17 = frame[17]
        speed_kmh_int = self._speed_from_b16_b17(b16, b17)

        throttle = b8 == 0x5B
        if self.log_rx_frames:
            speed_txt = f'{speed_kmh_int} km/h' if speed_kmh_int >= 0 else 'NA'
            log_line = (
                f'RX frame len={len(frame)} byte8=0x{b8:02X} '
                f'b16=0x{b16:02X} b17=0x{b17:02X} throttle={"ON" if throttle else "OFF"} '
                f'speed={speed_txt}'
            )
            if self.log_raw_hex:
                log_line += f' raw={frame.hex(" ")}'
            self.get_logger().info(log_line)

        if (not self._have_throttle) or (throttle != self._last_throttle):
            self._have_throttle = True
            self._last_throttle = throttle
            self.pub_throttle.publish(Bool(data=throttle))
            self.get_logger().info(f'Throttle: {"ON" if throttle else "OFF"} (byte8=0x{b8:02X})')

        if speed_kmh_int >= 0:
            speed_kmh = speed_kmh_int * self.speed_scale
            speed_changed = (self._last_speed_kmh is None) or (abs(speed_kmh - self._last_speed_kmh) > 1e-6)

            self._last_speed_kmh = speed_kmh
            self._last_speed_update_t = time.monotonic()

            if speed_changed:
                self.get_logger().info(
                    f'Wheel speed: {speed_kmh:.2f} km/h (b16=0x{b16:02X}, b17=0x{b17:02X})'
                )

    def _get_current_speed_mps(self, now_mono: float) -> float:
        if self._last_speed_kmh is None:
            return 0.0

        if self.speed_timeout_s > 0.0 and (now_mono - self._last_speed_update_t) > self.speed_timeout_s:
            return 0.0

        if self.use_throttle_gate and self._have_throttle and (not self._last_throttle):
            return 0.0

        return self.forward_sign * (self._last_speed_kmh / 3.6)

    def _publish_odom(self) -> None:
        now_ros = self.get_clock().now()
        dt = (now_ros - self._last_publish_ros_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_publish_ros_time = now_ros

        speed_mps = self._get_current_speed_mps(time.monotonic())
        self._x += speed_mps * dt

        odom = Odometry()
        odom.header.stamp = now_ros.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.orientation.w = 1.0

        odom.twist.twist.linear.x = speed_mps

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
        ]

        odom.twist.covariance = [
            0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
        ]

        self.pub_odom.publish(odom)

        vel = TwistStamped()
        vel.header = odom.header
        vel.header.frame_id = self.base_link_frame
        vel.twist = odom.twist.twist
        self.pub_vel.publish(vel)

        self.pub_speed.publish(Float32(data=speed_mps * 3.6))

    def destroy_node(self) -> bool:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        if self.pigpio is not None:
            try:
                self.pigpio.bb_serial_read_close(self.gpio_rx_pin)
            except Exception:
                pass
            self.pigpio.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WheelUartOdomNode] = None
    try:
        node = WheelUartOdomNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
