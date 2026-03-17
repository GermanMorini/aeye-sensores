# sensores

Paquete ROS 2 para integración con Pixhawk, dashboard web y utilidades auxiliares de cámara. Mantiene dos caminos de telemetría: el driver propio `pixhawk_driver` y un launch alternativo basado en MAVROS.

## Ejecutables reales
- `pixhawk_driver`
- `sensores_web`
- `camara`

## Launches reales
- `ros2 launch sensores pixhawk.launch.py`
- `ros2 launch sensores mavros.launch.py`
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

Payload adicional cuando hay orientación disponible:
- `imu.yaw_enu_rad`, `imu.yaw_enu_deg`
- `odom.yaw_enu_rad`, `odom.yaw_enu_deg`
- `diagnostics.yaw_delta_deg`
- `topics.{imu,gps,velocity,odom}`

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

## Launch de MAVROS
Este launch usa `mavros` + `mavros_extras` y deja `sensores_web` suscrito a la salida de MAVROS en lugar del driver propio.

Dashboard con MAVROS:
```bash
ros2 launch sensores mavros.launch.py launch_web:=true
```

Parámetros principales:
- `fcu_url` (default `/dev/ttyACM0:921600`)
- `gcs_url`
- `tgt_system`
- `tgt_component`
- `namespace` (default `mavros`)
- `fcu_protocol` (default `v2.0`)
- `pluginlists_yaml`
- `config_yaml`
- `apm_config_yaml`

Bindings por defecto del dashboard:
- IMU: `/mavros/imu/data`
- GPS: `/mavros/global_position/raw/fix`
- Velocidad: `/mavros/local_position/velocity_local`
- Odometría: `/mavros/local_position/odom`

`mavros.launch.py` agrega remaps de compatibilidad para MAVROS Humble:
- `mavros_node/data` -> `imu/data`
- `mavros_node/raw/fix` -> `global_position/raw/fix`
- `mavros_node/velocity_local` -> `local_position/velocity_local`
- `mavros_node/odom` -> `local_position/odom`

El perfil por defecto es `sensor-only` para no exponer interfaces de control desde ROS.

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
- El nombre correcto del ejecutable Pixhawk es `pixhawk_driver`; `ros2 run sensores sensores` no aplica en este checkout.
- Los defaults de `pixhawk.launch.py` usan `base_footprint` y `gps_link`; si los cambias, mantén README y launch alineados.
- `mavros.launch.py` requiere que `mavros` y `mavros_extras` estén instalados en el entorno ROS 2.
