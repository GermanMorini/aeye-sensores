# sensores

Paquete ROS 2 para integración con Pixhawk y utilidades auxiliares de telemetría y cámara.

## Ejecutables reales
- `pixhawk_driver`
- `sensores_web`
- `camara`

## Launches reales
- `ros2 launch sensores pixhawk.launch.py`
- `ros2 launch sensores rs16.launch.py`

## `pixhawk_driver`
Publica:
- `/imu/data`
- `/gps/fix`
- `/odom`
- `/velocity`
- `/gps/rtk_status`
- `/gps/fix_type`
- `/gps/satellites_visible`
- `/gps/hdop`
- `/gps/rtcm_age_s`
- `/gps/rtcm_received_count`
- `/gps/rtcm_sequence_id`

Parámetros principales:
- `serial_port` (default `/dev/ttyACM0`)
- `baudrate` (default `921600`)
- `odom_frame` (default `odom`)
- `base_link_frame` (default `base_footprint`)
- `imu_frame` (default `imu_link`)
- `gps_frame` (default `gps_link`)
- `publish_rate_hz` (default `200.0`)
- `enable_gps_rtk`
- `enable_rtcm_tcp`
- `rtcm_tcp_host`
- `rtcm_tcp_port`
- `rtcm_topic`
- `yaw_correction_deg`

Ejecución directa:
```bash
ros2 run sensores pixhawk_driver --ros-args -p serial_port:=/dev/ttyACM0 -p baudrate:=921600
```

## `sensores_web`
Sirve `pixhawk_dashboard.html` y expone datos por HTTP y WebSocket.

Parámetros:
- `imu_topic`
- `gps_topic`
- `velocity_topic`
- `odom_topic`
- `http_host`
- `http_port`
- `ws_host`
- `ws_port`
- `html_path`

Ejecutar:
```bash
ros2 run sensores sensores_web
```

## `camara`
Nodo opcional para control ISAPI de cámara.

Servicios:
- `/camara/camera_pan`
- `/camara/camera_zoom_toggle`
- `/camara/camera_status`

Ejecutar:
```bash
ros2 run sensores camara
```

## Launch de Pixhawk
Solo driver:
```bash
ros2 launch sensores pixhawk.launch.py
```

Driver + dashboard web:
```bash
ros2 launch sensores pixhawk.launch.py launch_web:=true
```

## Launch de RS16
Este launch envuelve `rslidar_sdk`, que se considera una dependencia vendorizada del workspace.

Driver:
```bash
ros2 launch sensores rs16.launch.py
```

Driver + RViz:
```bash
ros2 launch sensores rs16.launch.py rviz:=true
```

Config personalizado:
```bash
ros2 launch sensores rs16.launch.py config_path:=/ruta/a/rs16.yaml
```

## Notas
- El nombre correcto del ejecutable Pixhawk es `pixhawk_driver`; `ros2 run sensores sensores` ya no aplica en este checkout.
- Los defaults de `pixhawk.launch.py` usan `base_footprint` y `gps_link`; mantén README y launch alineados si cambias esos frames.
