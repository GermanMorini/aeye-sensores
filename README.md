# sensores

Paquete ROS 2 que se conecta a un Pixhawk (ej. Pixhawk6X) por MAVLink y publica
datos de IMU, GPS, velocidad y odometria en topicos estandar de ROS 2.

## Funcionalidad
- Lee MAVLink por puerto serie y solicita streams de datos.
- Publica IMU en `/imu/data` y GPS en `/gps/fix`.
- Publica velocidad en `/velocity` (TwistStamped) y odometria en `/odom`.
- Convierte marcos: NED -> ENU y FRD -> FLU para compatibilidad ROS.

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

Con parametros personalizados:
```bash
ros2 run sensores sensores --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=921600 \
  -p frame_id_imu:=imu_link \
  -p frame_id_gps:=gps \
  -p odom_topic:=/odom \
  -p odom_frame:=odom \
  -p base_link_frame:=base_footprint
```

## Topicos publicados
- `/imu/data` (`sensor_msgs/Imu`)
  - Por defecto usa solo la orientacion del EKF (`ATTITUDE_QUATERNION`).
  - Aceleracion/gyro crudos desde `SCALED_IMU` / `SCALED_IMU2` si `use_raw_imu:=true`.
  - Nota: se publican mensajes distintos en el mismo topico (unos con
    aceleracion y otros con orientacion). Los campos no disponibles se marcan
    con covarianza `-1`.
- `/gps/fix` (`sensor_msgs/NavSatFix`)
  - Posicion GPS desde `GPS_RAW_INT`.
- `/velocity` (`geometry_msgs/TwistStamped`)
  - Velocidad lineal en ENU usando `ODOMETRY` (si esta disponible).
  - Si `use_mavlink_odometry:=false`, usa `LOCAL_POSITION_NED`.
- `/odom` (configurable, `nav_msgs/Odometry`)
  - Pose en marco `odom` y `child_frame_id` en `base_link_frame`.
  - EKF via `ODOMETRY` si esta disponible (incluye covarianzas).
  - Si `use_mavlink_odometry:=false`, usa `LOCAL_POSITION_NED`.

## Parametros
- `serial_port` (string, default `/dev/ttyACM0`)
- `baudrate` (int, default `921600`)
- `frame_id_imu` (string, default `imu_link`)
- `frame_id_gps` (string, default `gps`)
- `imu_topic` (string, default `/imu/data`)
- `gps_topic` (string, default `/gps/fix`)
- `velocity_topic` (string, default `/velocity`)
- `odom_topic` (string, default `/odom`)
- `odom_frame` (string, default `odom`)
- `base_link_frame` (string, default `base_link`)
- `use_raw_imu` (bool, default `false`)
- `use_mavlink_odometry` (bool, default `true`)

## Streams MAVLink solicitados
- `MAV_DATA_STREAM_RAW_SENSORS` a 50 Hz (IMU).
- `MAV_DATA_STREAM_POSITION` a 10 Hz (GPS).
- `MAV_DATA_STREAM_EXTRA1` a 50 Hz (cuaterniones de actitud).
- `ODOMETRY` (pose/velocidad EKF, si el autopiloto lo publica).

## Notas y solucion de problemas
- Verifica permisos del puerto serie (grupo `dialout` en Linux).
- El nodo espera un heartbeat de MAVLink; si no aparece en 10 s, revisa el
  puerto y el baudrate.
- `/imu/data` mezcla mensajes de aceleracion y de orientacion; si necesitas un
  flujo unico, considera fusionarlo en un nodo aparte.
