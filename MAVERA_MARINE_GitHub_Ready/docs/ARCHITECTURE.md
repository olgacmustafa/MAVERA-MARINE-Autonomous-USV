# Architecture and interfaces

## Links and environments

- **UAV / İHA**: AR0234 RAW10 (V4L2/GStreamer), YOLO + HSV, Jetson, Cube/RC7, MAVLink uplink.
- **Ground / YKİ**: Two USB RFD radios, PyMAVLink serial connections, a UDP bridge for Mission Planner and a small colour relay state machine.
- **USV / İDA**: Cube connected through MAVROS; MAVROS owns the serial port. A ROS 2 `haberlesme` node consumes MAVROS raw messages, publishes a target colour and returns `COMMAND_ACK` via MAVROS.

## MAVLink protocol

| Item | Value in supplied files |
| --- | --- |
| UAV colour report | `NAMED_VALUE_INT` name `TGT_COLOR` |
| Ground → USV command | `MAV_CMD_USER_1` (`31010` fallback in receiver) |
| Version | `1` |
| Signature / marker | `2026` (not a cryptographic signature) |
| Colour 1 | `SIYAH` → `siyah` |
| Colour 2 | `KIRMIZI` → `kirmizi` |
| Colour 3 | `YESIL` → `yesil` |
| YKİ MAVLink source | `201/190` |
| USV Jetson MAVLink source | `202/191` |
| UAV Jetson MAVLink source | `200/191` |
| USV colour topic | `/carpilacakhedef` (`std_msgs/String`) |
| USV input/output | `/mavros/mavlink/from`, `/mavros/mavlink/to` |

The source implements three repeated reports at the ground station and target-colour locking on the USV. An application-level marker is **not an authentication mechanism**; use trusted telemetry links and appropriate security measures for deployment.

## ROS node responsibilities

| Node | Main responsibility |
| --- | --- |
| `haberlesme` | MAVLink command receipt, validation, acknowledgement and target-colour publication |
| `los_node` | Mission waypoint tracking and guidance |
| `obstacle_avoidance_node` | ZED-based YOLO/depth obstacle interpretation and steering/speed arbitration |
| `pid_node` | Guidance command conversion into MAVROS setpoints |
| `estop` | RC channel 9 → software E-stop and GPIO |
| `data_logger_node` | Video, depth/costmap and telemetry output |

The provided launch does not start a LiDAR driver or `slam_toolbox`. The `/camera_local_costmap` output is a camera-relative local occupancy representation, not a persistent global SLAM reconstruction.
