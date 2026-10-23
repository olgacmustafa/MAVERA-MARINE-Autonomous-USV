# GitHub source release notes

This repository was assembled from four uploads: a UAV camera script, a ground-station bridge script, a stand-alone revised USV MAVROS colour receiver, and a USV ROS 2 ZIP archive.

## Changes limited to publication and integration

1. Moved source into `iha/`, `yki/`, and `ida_ws/src/usv_navigation/`, with stable generic file names.
2. Removed third-party Stereolabs and Slamtec vendor source trees (including their nested Git histories), old backup files, temporary cache files, an empty artifact and internal review notes. Third-party dependencies are installed separately.
3. Removed a second copy of the USV YOLO model; preserved the archive's model as `ida_ws/src/usv_navigation/best.pt`.
4. Used the separately supplied revised colour receiver as `usv_navigation/haberlesme.py` (the one installed by `setup.py` and started by the launch file). This is an **integration revision**, not merely a comment change, and has not been bench tested.
5. Removed a unique Cube USB serial identifier and a fixed home-directory path from the USV launch; launch now uses `/dev/ttyACM0` as an illustrative port and receiver's default per-user data path.
6. Changed ground-station RFD discovery from hardcoded individual USB serial numbers to configurable environment variables, with manual COM-port overrides and clear error messages.
7. Simplified first-person/temporary tutorial comments in the UAV script without altering its detection, exposure, flight gating or messaging thresholds.
8. Added English/Turkish project overview, dedicated setup guides, protocol documentation, static tests, CI and Git ignore rules.
9. Updated the placeholder package maintainer email to `maintainer@example.com`; replace it with a real contact before publishing.

## Unchanged by design

- USV movement control algorithms, ROS topics, parameter values, waypoint logic, PID gains, camera thresholds and E-stop GPIO polarity.
- UAV RC7 gating, ML inference thresholds, colour-lock logic and MAVLink message behaviour.
- Ground-station command retries/ACK logic and Mission Planner UDP ports, except for how radio identifiers are configured.

## Items not included

- Jetson services/systemd units and machine-specific OS configuration.
- Separate UAV plate-detection model: `iha/models/best.pt` must be supplied by the project owner.
- Bundled vendor SDKs or compiled ROS workspace artifacts.
