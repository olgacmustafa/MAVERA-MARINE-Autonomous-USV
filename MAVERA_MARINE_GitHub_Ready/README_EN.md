# MAVERA MARINE — Autonomous Marine Vehicle Software

Source code for the **EGM–MAVERA MARINE** unmanned surface vehicle (USV/İDA) project developed for **TEKNOFEST 2026**, including UAV-assisted colour detection, ground-station relay and ROS 2 navigation. The team achieved first place in the 2026 unmanned marine vehicle competition.

> **Repository status:** Source release and static review. This is a curated publication of project files, **not** a newly validated field-deployment build. Use in simulations and controlled tests before integrating with propulsion systems.

[Türkçe README](README_TR.md) · [Architecture](docs/ARCHITECTURE.md) · [Testing & safety](docs/TESTING.md) · [Release notes](docs/RELEASE_NOTES.md)

## System overview

```text
UAV: AR0234 camera → YOLO/HSV → MAVLink (TGT_COLOR)
                                     │ RFD
                                     ▼
Ground station (YKİ): colour validation + MAVLink/UDP bridge
                                     │ RFD / MAV_CMD_USER_1
                                     ▼
USV: Cube Orange+ → MAVROS → colour receiver → /carpilacakhedef
                              │
ZED 2 depth + MAVROS + waypoint guidance → obstacle avoidance → PID → setpoints
                                                 │
                                   camera local occupancy map + telemetry
```

## Repository structure

| Path | Function |
| --- | --- |
| `iha/src/canli_plaka_tespit.py` | UAV camera capture, plate-colour inference and MAVLink reporting |
| `yki/yki_iha_ida_koprusu.py` | Ground station with two RFD links, Mission Planner UDP bridge and ACK handling |
| `ida_ws/src/usv_navigation/` | ROS 2 USV autonomy package, launch/config, control, safety, logging and a model |
| `docs/` | Architecture, validation caveats and source-release notes |
| `tests/` | Dependency-free source and interface consistency checks |

Each component runs in a **different runtime environment**; it is not one program started from the repository root. Follow the component-specific README files: [UAV](iha/README.md), [ground station](yki/README.md), [USV](ida_ws/README.md).

## Core interface

The ground station interprets MAVLink `NAMED_VALUE_INT` with name `TGT_COLOR`, then sends `MAV_CMD_USER_1` to the USV. Target colour identifiers are **1 = black, 2 = red, 3 = green**. The USV publishes a lowercase Turkish `std_msgs/String` on `/carpilacakhedef`; downstream autonomy subscribes to it. Message IDs, source/target IDs, protocol version and signature have been retained from the supplied code and must remain consistent between components.

## Technical boundaries

- The supplied USV launch uses the **ZED camera** for perception. No active LiDAR scanner or LiDAR-based SLAM is started by the supplied launch file. The original upload contained separate Slamtec source code, excluded from this team-code release.
- `ida_ws/src/usv_navigation/best.pt` is retained from the uploaded USV archive. The UAV script needs its **own** model at `iha/models/best.pt` and none was supplied as a separate identifiable asset.
- Third-party repositories and nested `.git` directories, old backups and `__pycache__` files were removed.
- Device-specific unique USB IDs were not published. Use local configuration to select radios and MAVROS serial ports.

## Validation

The static tests run without ROS or Jetson hardware:

```bash
python -m unittest discover -s tests -v
python -m compileall -q iha yki ida_ws/src/usv_navigation
```

**No water trials, propeller tests, radio tests, Jetson inference tests or end-to-end MAVLink tests were executed for this release.** See [TESTING.md](docs/TESTING.md) before real-vehicle use.

## Licensing and attribution

The team's supplied `setup.py` and `package.xml` designate the code **Proprietary**. An open-source license was not supplied, so this release **does not grant an open-source usage license**. Repository visibility on GitHub is not equivalent to permission to redistribute or modify. The team should select an explicit license if it intends to invite community reuse. Verify the redistribution rights and provenance of `best.pt` before public distribution. External packages (ROS 2, MAVROS, ZED SDK/wrapper, Slamtec) keep their respective original licenses and are not included here.
