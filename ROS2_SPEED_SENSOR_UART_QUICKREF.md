# ROS2 Speed Sensor UART Quick Reference

Minimal guide to implement the new ROS2 speed sensor in `sensores` using the
current ESP32 <-> Raspberry protocol.

## 1. Scope (what matters for ROS2)
- Input on Raspberry: ESP32 TX frames (`0x55 status telemetry crc`), 4 bytes.
- UART settings: `/dev/serial0`, `460800`, `8N1`, no flow control.
- CRC: Dallas/Maxim CRC-8 (`poly=0x31`, `init=0x00`, MSB-first).

## 2. Speed decoding
- `telemetry_u8` in byte 2.
- `0..254` -> `speed_kmh` (direct value).
- `255` -> `N/A` (no valid speed).

## 3. ROS2 output contract (keep compatibility)
- `/wheel/speed_kmh` (`std_msgs/Float32`)
- `/wheel/velocity` (`geometry_msgs/TwistStamped`)
- `/wheel/odom` (`nav_msgs/Odometry`)

Recommended defaults:
- `speed_timeout_s = 0.5`
- `forward_sign = 1.0`

Behavior for `N/A` (`telemetry_u8=255`):
- Keep last valid speed until timeout (`speed_timeout_s`).
- After timeout, force speed to `0`.

## 4. Parser requirements
1. Resync by header `0x55`.
2. Read exactly 4-byte frames.
3. Validate CRC before publish.
4. Count metrics: valid frames, CRC errors, last frame age.

## 5. Important test note (manual throttle)
If Raspberry transmits control frames (`0xAA ...`) while testing, ESP32 can
prioritize Pi control and manual RC throttle may look blocked.

For passive speed tests:
1. Stop service: `sudo systemctl stop salus-ws.service`
2. Verify: `systemctl is-active salus-ws.service` -> `inactive`
3. Run RX-only capture (do not transmit to ESP32)
4. Restore service after test if needed

## 6. Extended docs
- Canonical protocol (this workspace):
  - `/home/leo/codigo/aeye-ros-workspace/src/sensores/ESP32_UART_PROTOCOL.md`
- Raspberry side (extended operational docs):
  - `/home/salus/codigo/RASPY_SALUS/COMMS.md`
  - `/home/salus/codigo/RASPY_SALUS/PI_COMMS_README.md`
  - `/home/salus/codigo/RASPY_SALUS/ESP32_UART_PROTOCOL.md`
- ESP32 side (firmware details):
  - `/home/leo/codigo/ESP32_SALUS/COMMS.md`
  - `/home/leo/codigo/ESP32_SALUS/PI_COMMS_README.md`
