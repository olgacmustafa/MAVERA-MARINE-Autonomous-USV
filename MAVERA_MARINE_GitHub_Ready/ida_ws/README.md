# USV ROS 2 workspace

This workspace contains the team's original `usv_navigation` ROS 2 Python package, cleaned for publication. The package includes the source archive's `best.pt` model. Third-party Stereolabs ZED packages, the MAVROS installation and the Slamtec driver are **not vendored** in the repository.

## Prerequisites

- Ubuntu with ROS 2 Humble and a functioning MAVROS installation.
- ZED 2 camera, compatible ZED SDK and ROS 2 `zed_wrapper` package.
- Python packages appropriate for the ROS environment, including `rclpy`, `mavros_msgs`, `numpy`, `opencv`, `pymavlink`, `ultralytics`, `torch` and (for hardware E-stop) `Jetson.GPIO`.
- A Cube Orange+ or compatible MAVLink autopilot, appropriately configured hardware safety chain.

Install ROS dependencies in your own supported environment. The complete Jetson/CUDA/ROS/MAVROS setup is platform dependent; **`pip install -r iha/requirements.txt` alone is not enough to run the USV.**

## Build

```bash
cd ida_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select usv_navigation
source install/setup.bash
ros2 launch usv_navigation usv_otonom.launch.py fcu_url:=serial:///dev/ttyACM0:115200
```

Check `ls -l /dev/serial/by-id/` first and, where possible, pass the actual device path in `fcu_url`. The supplied default `/dev/ttyACM0` is only illustrative.

The launch file starts the following team nodes: `haberlesme`, `estop`, `los_node`, `obstacle_avoidance_node`, `pid_node`, and `data_logger_node`, along with MAVROS, ZED and static transforms. The model is installed to `share/usv_navigation/models/best.pt` by `setup.py`.

The runtime destination of JSON target-colour state is configured through the ROS parameter `kayit_yolu` on `haberlesme`; its default is under `~/mavera_ida/data/`.

**Note:** Static TF offsets, GPIO logic, autopilot setpoint semantics, thresholds and closed-loop guidance all require real-hardware validation. They have **not** been revalidated in this publication pass.
