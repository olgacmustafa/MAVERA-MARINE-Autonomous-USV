# UAV plate-colour detection

`src/canli_plaka_tespit.py` uses an AR0234 global-shutter camera (RAW10 over V4L2/GStreamer), YOLO and HSV filtering, an RC channel gate, and MAVLink messages to report a locked target colour to the ground station.

## Required assets and environment

- An AR0234 camera with the expected driver and Bayer layout (GB), normally `/dev/video0`.
- CUDA-compatible PyTorch, Ultralytics, OpenCV, NumPy, PyMAVLink, GStreamer Python bindings, `v4l2-ctl` and optionally Jetson.GPIO.
- The **UAV plate-classification model** at `iha/models/best.pt`. **Not supplied** in this repository: the `.pt` file under `ida_ws` comes from the USV archive and must not automatically be assumed compatible.
- MAVLink connection to a Cube/telemetry system and channel 7 control as described in the source.

Run from the repository root:

```bash
python3 iha/src/canli_plaka_tespit.py
```

Set `MAVERA_EKRAN=0` for headless execution. The script creates camera captures and recordings under `iha/`; these folders are ignored by Git. Always test with motors disarmed and telemetry isolated first.

The source retains platform-specific exposure, GPIO and colour thresholds. They are **not** defaults intended for arbitrary cameras or airframes.
