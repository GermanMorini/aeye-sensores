# sensores

Paquete ROS 2 que se conecta a un Pixhawk (ej. Pixhawk 6X con ArduPilot)
por MAVLink y publica datos de IMU, GPS, velocidad y odometria en topicos
estandar de ROS 2.

## Funcionalidad
- Lee MAVLink por puerto serie y solicita streams de datos.
- Publica IMU en `/imu/data` y GPS en `/gps/fix`.
- Publica velocidad en `/velocity` y odometria en `/odom`.
- Convierte marcos: NED -> ENU y FRD -> FLU para compatibilidad ROS.
- Incluye servidor web con WebSocket para visualizar telemetria.

## Dependencias
- ROS 2 con `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`.
- `pymavlink` (paquete `python3-pymavlink`).

## Compilar
```bash
colcon build --packages-select sensores
source install/setup.bash
```

## Ejecutar
```bash
ros2 run sensores sensores
```

Servidor web (sirve `pixhawk_dashboard.html`, JSON en `/data` y WebSocket):
```bash
ros2 run sensores sensores_web
```
Abrir en el navegador: `http://localhost:8000`.

Lanzar ambos nodos (incluye servidor web):
```bash
ros2 launch sensores pixhawk.launch.py launch_web:=true
```

Solo el driver:
```bash
ros2 launch sensores pixhawk.launch.py
```

Lanzar LiDAR RS-LiDAR-16 (rslidar_sdk):
```bash
ros2 launch sensores rs16.launch.py
```

Lanzar LiDAR + RViz:
```bash
ros2 launch sensores rs16.launch.py rviz:=true
```

Usar un config personalizado:
```bash
ros2 launch sensores rs16.launch.py config_path:=/ruta/a/config.yaml
```

Con parametros personalizados:
```bash
ros2 run sensores sensores --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p imu_frame:=imu_link \
  -p gps_frame:=gps \
  -p odom_frame:=odom \
  -p base_link_frame:=base_link
```

## Topicos publicados
- `/imu/data` (`sensor_msgs/Imu`)
  - Aceleracion lineal y gyro desde `SCALED_IMU2` (o `SCALED_IMU` si no hay IMU2).
  - Orientacion desde `ATTITUDE_QUATERNION` (EKF).
  - Unidades: aceleracion en `m/s^2`, gyro en `rad/s`.
- `/gps/fix` (`sensor_msgs/NavSatFix`)
  - Posicion GPS desde `GPS_RAW_INT`.
- `/velocity` (`geometry_msgs/TwistStamped`)
  - Velocidad lineal en marco del cuerpo (FLU) y angular si esta disponible.
  - Se publica a partir de `/odom` (mismo timestamp).
- `/odom` (`nav_msgs/Odometry`)
  - Pose en `odom_frame` (ENU) con `child_frame_id` en `base_link_frame`.
  - Usa `LOCAL_POSITION_NED` + actitud (EKF) convertido a ENU/FLU.

## Parametros
- `serial_port` (string, default `/dev/ttyACM0`)
- `baudrate` (int, default `921600`)
- `odom_frame` (string, default `odom`)
- `base_link_frame` (string, default `base_link`)
- `imu_frame` (string, default `imu_link`)
- `gps_frame` (string, default `gps`)
- `publish_rate_hz` (float, default `200.0`)

## Parametros del servidor web
- `imu_topic` (string, default `/imu/data`)
- `gps_topic` (string, default `/gps/fix`)
- `velocity_topic` (string, default `/velocity`)
- `odom_topic` (string, default `/odom`)
- `http_host` (string, default `0.0.0.0`)
- `http_port` (int, default `8000`)
- `ws_host` (string, default `0.0.0.0`)
- `ws_port` (int, default `8001`)
- `html_path` (string, default vacio; usa `pixhawk_dashboard.html` del paquete)

## Streams MAVLink solicitados
- `ATTITUDE_QUATERNION` a 50 Hz.
- `SCALED_IMU2` a 100 Hz (fallback `SCALED_IMU`).
- `LOCAL_POSITION_NED` a 50 Hz.
- `GPS_RAW_INT` a 10 Hz.

## Notas y solucion de problemas
- Verifica permisos del puerto serie (grupo `dialout` en Linux).
- El nodo espera un heartbeat de MAVLink; si no aparece en 10 s, revisa el
  puerto y el baudrate.
- `/odom` y `/velocity` dependen de `LOCAL_POSITION_NED` (EKF). En ArduPilot
  puede requerir habilitar `SR0_POSITION` (USB) para que se publique.
