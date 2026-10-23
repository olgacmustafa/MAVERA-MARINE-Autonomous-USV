# Validation, deployment and safety

## What was checked for this source release

- Python syntax and AST parsing for all published Python source files.
- XML and YAML parsing where a suitable parser was available.
- ROS `setup.py` entry points match published source modules.
- All three codebases retain matching target-colour protocol IDs and codes.
- No vendored nested Git repositories, stale backups or compiled Python cache are shipped.

These checks do **not** establish the safety of an autonomous vessel or make hardware assertions.

## Controlled integration checklist

- [ ] Confirm camera hardware/driver (`AR0234` UAV, `ZED 2` USV) and YOLO model classes; supply the separate UAV `best.pt`.
- [ ] Configure USB devices and MAVROS `fcu_url`, avoid competing serial owners, verify message routing and ACK with motors disarmed.
- [ ] Check `MAV_CMD_USER_1`, `TGT_COLOR`, three-colour mapping and `/carpilacakhedef` including late target arrival and duplicate messages.
- [ ] Validate timeouts / fail-safe behavior by disconnecting each radio, the RC receiver, the ZED stream and MAVROS separately.
- [ ] **Validate E-stop physical circuit polarity**. The original GPIO/RC semantics were preserved; the watchdog forces a ROS stop on RC timeout, but its physical output behavior must be inspected and physically tested rather than assumed safe.
- [ ] Measure camera/GPS TF offsets on the real vehicle.
- [ ] Test setpoint masks, propeller directions and PID settings on a restrained test setup before water trials.
- [ ] Check model file redistribution rights, dependency licenses and applicable competition/safety constraints.
- [ ] Validate all recordings, logs and mission termination conditions on a realistic dry-run, then a supervised water test.

## Limitations

The uploader supplied code and one USV archive, not hardware logs or a reproducible execution environment. The source is therefore published as a **historical project/educational reference with a publication cleanup**, not a turnkey safety-certified autopilot. The updated standalone USV colour receiver was chosen over the receiver embedded in the old ZIP; that integration requires a fresh bench test.
