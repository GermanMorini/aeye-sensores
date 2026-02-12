# sensores

Paquete ROS 2 que se conecta a un Pixhawk (ej. Pixhawk 6X con ArduPilot)
por MAVLink y publica datos de IMU, GPS, velocidad y odometria en topicos
estandar de ROS 2.

## Funcionalidad
- Lee MAVLink por puerto serie y solicita streams de datos.
- Publica IMU en `/imu/data` y GPS en `/gps/fix`.
- Publica velocidad en `/velocity` y odometria en `/odom`.
- Lee odometria de ruedas por UART (sniffer) y publica `/wheel/odom`.
- Convierte marcos: NED -> ENU y FRD -> FLU para compatibilidad ROS.
- Incluye servidor web con WebSocket para visualizar telemetria.

## Dependencias
- ROS 2 con `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`.
- `pymavlink` (paquete `python3-pymavlink`).
- `pyserial` (paquete `python3-serial`).

## Compilar
```bash
colcon build --packages-select sensores
source install/setup.bash
```

## LiDAR RS-LiDAR-16 (driver)
Clonar los repositorios del driver y mensajes del LiDAR **dentro de** `src` (**importante**)

Recuerda estar en el workspace de ROS:
```bash
git clone 'https://github.com/RoboSense-LiDAR/rslidar_msg' src/rslidar_msg
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git src/rslidar_sdk
cd src/rslidar_sdk
git submodule update --init --recursive
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

Lanzar odometria de ruedas por UART:
```bash
ros2 launch sensores wheel_odom.launch.py \
  rx_backend:=serial serial_port:=/dev/ttyAMA2 baudrate:=2000
```

Ver en consola (estilo Arduino) que se esta recibiendo:
```bash
ros2 launch sensores wheel_odom.launch.py \
  rx_backend:=serial serial_port:=/dev/ttyAMA2 baudrate:=2000 \
  log_rx_frames:=true
```

Ver ademas el frame completo en hex:
```bash
ros2 launch sensores wheel_odom.launch.py \
  rx_backend:=serial serial_port:=/dev/ttyAMA2 baudrate:=2000 \
  log_rx_frames:=true log_raw_hex:=true
```

Lanzar Pixhawk + odometria de ruedas:
```bash
ros2 launch sensores pixhawk.launch.py \
  launch_wheel_odom:=true \
  wheel_rx_backend:=serial wheel_odom_port:=/dev/ttyAMA2
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
- `/wheel/odom` (`nav_msgs/Odometry`)
  - Odometria 1D integrada desde velocidad de ruedas decodificada por UART.
  - Pose en `odom_frame` con `child_frame_id` en `base_link_frame`.
- `/wheel/velocity` (`geometry_msgs/TwistStamped`)
  - Velocidad lineal de ruedas en `base_link`.
- `/wheel/speed_kmh` (`std_msgs/Float32`)
  - Velocidad decodificada final en km/h.
- `/wheel/throttle` (`std_msgs/Bool`)
  - Estado de acelerador (`true` cuando byte 8 == `0x5B`).

## Parametros
- `serial_port` (string, default `/dev/ttyACM0`)
- `baudrate` (int, default `921600`)
- `odom_frame` (string, default `odom`)
- `base_link_frame` (string, default `base_link`)
- `imu_frame` (string, default `imu_link`)
- `gps_frame` (string, default `gps`)
- `publish_rate_hz` (float, default `200.0`)

## Parametros del nodo wheel_odom_uart
- `rx_backend` (string, default `serial`) (`serial` o `pigpio`)
- `serial_port` (string, default `/dev/ttyAMA2`)
- `gpio_rx_pin` (int, default `5`) (solo backend `pigpio`)
- `baudrate` (int, default `2000`) (`2083` opcional)
- `frame_gap_us` (int, default `12000`)
- `poll_rate_hz` (float, default `500.0`)
- `publish_rate_hz` (float, default `50.0`)
- `invert_bytes` (bool, default `false`)
- `log_rx_frames` (bool, default `false`) (imprime una linea por frame recibido)
- `log_raw_hex` (bool, default `false`) (agrega payload de frame en hex)
- `invert_signal` (bool, default `false`) (solo backend `pigpio`)
- `pigpiod_host` (string, default `localhost`) (solo backend `pigpio`)
- `pigpiod_port` (int, default `8888`) (solo backend `pigpio`)
- `odom_topic` (string, default `/wheel/odom`)
- `velocity_topic` (string, default `/wheel/velocity`)
- `speed_topic` (string, default `/wheel/speed_kmh`)
- `throttle_topic` (string, default `/wheel/throttle`)
- `odom_frame` (string, default `odom`)
- `base_link_frame` (string, default `base_footprint`)
- `speed_scale` (float, default `1.0`)
- `forward_sign` (float, default `1.0`)
- `speed_timeout_s` (float, default `0.5`)
- `use_throttle_gate` (bool, default `true`)

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
- En `rx_backend:=serial`, `invert_signal=true` no esta soportado por `pyserial`.
- Si la UART del display llega invertida, invierte la senal en hardware antes
  del RX de la Raspberry Pi.
- El nodo espera un heartbeat de MAVLink; si no aparece en 10 s, revisa el
  puerto y el baudrate.
- `/odom` y `/velocity` dependen de `LOCAL_POSITION_NED` (EKF). En ArduPilot
  puede requerir habilitar `SR0_POSITION` (USB) para que se publique.
