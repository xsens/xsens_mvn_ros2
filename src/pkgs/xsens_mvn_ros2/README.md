# xsens_mvn_ros2 — Node Reference

This document provides the complete parameter, topic, service, and action reference for all nodes in the `xsens_mvn_ros2` package.

All nodes are **ROS2 managed (lifecycle) nodes**. The Python launch files (`.launch.py`) automatically configure and activate each node at startup. For manual lifecycle control, see the [Lifecycle Management](../../../README.md#lifecycle-management) section in the workspace README.

For installation, architecture and the multi-avatar walkthrough, see the [workspace README](../../../README.md).

---

## Table of Contents

- [xsens_mvn_ros2_stream_node](#xsens_mvn_ros2_stream_node)
- [xsens_mvn_ros2_xme_node](#xsens_mvn_ros2_xme_node)
- [xsens_mvn_ros2_urdf_publisher_node](#xsens_mvn_ros2_urdf_publisher_node)

---

## xsens_mvn_ros2_stream_node

A lightweight UDP client that receives pre-processed motion capture data from Xsens MVN Studio and publishes it to ROS2 topics. Implemented as a lifecycle node.

**Launch:**
```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py
```

**Config file:** `config/xsens_stream_node.yaml`

Configure Xsens MVN Studio to stream data via UDP to the host running this node.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/joint_states` | `sensor_msgs/msg/JointState` | XYZ Euler angles (radians) for every joint MVN streams (22 body joints plus MVN's ergonomic extras) |
| `~/link_states` | `xsens_mvn_msgs/msg/LinkStateArray` | Full kinematic state (pose, twist, accel) for the 23 body segments, plus 20 finger segments per hand when MANUS gloves are streamed |
| `~/com` | `geometry_msgs/msg/Point` | Centre of mass position in the reference frame |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF transforms for all segments (relative parent→child), frames named `<model_name>_<segment>` |

These are the topics of the **primary** avatar (`avatar_id`). With `track_all_avatars` every other avatar gets the same set under its own name, e.g. `~/actor_b/link_states`, `~/actor_b/joint_states`, `~/actor_b/com`, with TF frames `actor_b_<segment>`. A tracked object publishes `link_states` and `com` only (frame `<name>_base_link`). See [Multiple Avatars](../../../README.md#multiple-avatars).

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | `"skeleton"` | TF prefix of the primary avatar (overridden by its `avatar_names` entry, if any) |
| `reference_frame` | string | `"world"` | Root TF frame for the kinematic tree |
| `udp_port` | int | `9763` | UDP port to listen on for MVN Studio data |
| `update_frequency` | int | `100` | Publishing rate in Hz |
| `avatar_id` | int | `0` | MVN avatar published on the unprefixed topics; others are discarded unless `track_all_avatars` |
| `track_all_avatars` | bool | `false` | Publish every avatar MVN streams |
| `avatar_names` | string[] | `[]` | TF prefix per avatar id, e.g. `["actor_a", "actor_b", "prop"]`. Unnamed ids fall back to `model_name` (primary) or `<model_name>_<id>` |
| `avatar_stale_timeout` | double | `1.0` | Seconds without data before an avatar stops being published |

### Launch Arguments

`xsens_mvn_ros2_stream`'s own `xsens_stream.launch.py` starts only the node and takes `namespace`, `auto_activate`, `model_name`, `reference_frame`, `udp_port`, `avatar_id`, `track_all_avatars` and `avatar_names`. The top-level `xsens_mvn_ros2` launch adds:

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_rviz` | `true` | Whether to start RViz (one RobotModel display per body avatar) |
| `launch_description` | `true` | Whether to start a URDF publisher per body avatar |
| `object_avatars` | `[]` | Entries of `avatar_names` that are tracked objects; they get no URDF publisher or RViz model |
| `discovery_range` | `LOCALHOST` | DDS discovery scope |
| `rviz_config_file` | built-in | Path to a custom RViz config |

List arguments accept YAML (`"[a, b]"`) or a comma-separated string.

---

## xsens_mvn_ros2_xme_node

Direct hardware interface using the Xsens XME SDK. Provides full suit control, real-time calibration, hardware diagnostics, and streaming without requiring Xsens MVN Studio. Implemented as a lifecycle node.

> **Session setup — required before each new subject:**
> Edit `config/body_dimensions.yaml` to match the subject's body measurements before launching. These values (height, arm span, leg lengths, etc.) are passed directly to the XME biomechanical solver and affect the accuracy of all kinematic output. Using the wrong dimensions for a subject will produce incorrect joint angles and segment poses. See the [Body Dimensions](#body-dimensions) parameter table below for a full list of fields.

> **Lifecycle note:** The calibration service and action server reject requests when the node is not in the `active` state. The `on_configure` callback performs the hardware scan (30 s timeout) — if the hardware is not found, configure will fail and the node stays `unconfigured`, allowing a retry.

**Launch:**
```bash
ros2 launch xsens_mvn_ros2 xsens_xme.launch.py
```

**Config files:**
- `config/xsens_xme_node.yaml` — node parameters
- `config/body_dimensions.yaml` — subject body dimensions (edit before each session)

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/joint_states` | `sensor_msgs/msg/JointState` | XYZ Euler angles (radians) for all joints |
| `~/link_states` | `xsens_mvn_msgs/msg/LinkStateArray` | Full kinematic state for all body segments |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF transforms for all body segments |
| `~/com` | `geometry_msgs/msg/Point` | Centre of mass position in the reference frame |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Hardware status, battery, sensor health, calibration quality |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/set_calibration_path` | `xsens_mvn_msgs/srv/LoadCalibrationPath` | Load a previously saved `.mvn` calibration file |

**LoadCalibrationPath.srv:**
```
# Request
string calibration_file_path
---
# Response
string message
bool success
```

**Example:**
```bash
ros2 service call /xsens_mvn_ros2_xme_node/set_calibration_path \
  xsens_mvn_msgs/srv/LoadCalibrationPath \
  "{calibration_file_path: '/home/user/my_calibration.mvn'}"
```

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `~/perform_calibration` | `xsens_mvn_msgs/action/PerformCalibration` | Execute N-Pose or T-Pose calibration |

**PerformCalibration.action:**
```
# Goal
int8 NPOSE = 0
int8 TPOSE = 1
int8   pose             # 0 = N-Pose, 1 = T-Pose
string output_file_name # Must include .mvn extension
---
# Result
string calibration_result
---
# Feedback
string status           # Real-time status updates during calibration
```

**Example — N-Pose calibration:**
```bash
ros2 action send_goal /xsens_mvn_ros2_xme_node/perform_calibration \
  xsens_mvn_msgs/action/PerformCalibration \
  "{pose: 0, output_file_name: 'calibration.mvn'}"
```

### Parameters

#### General

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | `"skeleton"` | Prefix for all frame and joint names |
| `reference_frame` | string | `"world"` | Root TF frame |
| `userCalibrationPreparationTimeMs` | int | `5000` | Preparation countdown before calibration starts (ms) |

#### SDK Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `awindaChannel` | int | `-1` | Awinda wireless channel; `-1` = auto-select (range 11–25) |
| `settingsPath` | string | `$(find-pkg-share xsens_xme_sdk)` | XME SDK settings directory |
| `userPath` | string | `$(find-pkg-share xsens_xme_sdk)` | XME user data directory (calibration files saved here) |
| `logPath` | string | `$(env HOME)/.ros/log` | XME SDK log output directory |
| `sampleRate` | double | `240.0` | Hardware sampling rate in Hz |
| `suitConfiguration` | string | `"FullBody"` | Suit configuration type |
| `biomechanicalModel` | string | `"Legacy"` | Biomechanical solver: `"Legacy"`, `"Female"`, or `"Male"` |

#### Body Dimensions

All values in **metres**. Used for biomechanical solving. Edit `config/body_dimensions.yaml` to match your subject.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `bodyDimension.bodyHeight` | double | `1.80` | Total standing height |
| `bodyDimension.footSize` | double | `0.26` | Foot length |
| `bodyDimension.shoulderHeight` | double | `1.45` | Height to top of shoulders |
| `bodyDimension.shoulderWidth` | double | `0.38` | Distance between shoulders |
| `bodyDimension.elbowSpan` | double | `0.94` | Arm span to elbows |
| `bodyDimension.wristSpan` | double | `1.43` | Arm span to wrists |
| `bodyDimension.armSpan` | double | `1.80` | Total arm span (fingertip to fingertip) |
| `bodyDimension.hipHeight` | double | `0.87` | Height to hip joints |
| `bodyDimension.hipWidth` | double | `0.24` | Distance between hip joints |
| `bodyDimension.kneeHeight` | double | `0.49` | Height to knees |
| `bodyDimension.ankleHeight` | double | `0.08` | Height to ankles |
| `bodyDimension.shoeSoleHeight` | double | `0.0` | Shoe sole thickness (added to ankle height) |

> These are the built-in defaults from `body_dimensions_parameters.yaml`. The shipped `config/body_dimensions.yaml` overrides them with one example subject's measurements; edit that file for yours.

Body dimensions can also be changed at runtime on a running node using `ros2 param set`:

```bash
ros2 param set /xsens_mvn_ros2_xme_node bodyDimension.bodyHeight 1.75
ros2 param set /xsens_mvn_ros2_xme_node bodyDimension.shoulderWidth 0.42
```

The new values are pushed to the XME SDK immediately. **Changing any body dimension automatically invalidates the current calibration** — the node will stop publishing pose data and report an error in `/diagnostics` until a new calibration is performed or a calibration file is loaded:

```bash
# Re-calibrate after changing dimensions
ros2 action send_goal /xsens_mvn_ros2_xme_node/perform_calibration \
  xsens_mvn_msgs/action/PerformCalibration \
  "{pose: 0, output_file_name: 'calibration.mvn'}"

# Or load an existing calibration file
ros2 service call /xsens_mvn_ros2_xme_node/set_calibration_path \
  xsens_mvn_msgs/srv/LoadCalibrationPath \
  "{calibration_file_path: '/path/to/calibration.mvn'}"
```

### Diagnostics

The node publishes on `/diagnostics` with diagnostic name `"Suit status"`.

**Status levels:**

| Level | Condition |
|-------|-----------|
| `OK` | All sensors connected, no hardware errors |
| `WARN` | Poor calibration quality on N-Pose or T-Pose |
| `ERROR` | Hardware disconnected, missing/duplicate sensors, or SDK error |

**Key-value fields published:**

| Field | Description |
|-------|-------------|
| `Total Sensors` | Number of active IMU sensors |
| `Battery level %` | Suit battery percentage |
| `Radio quality %` | Wireless link quality percentage |
| `Hardware Status` | Raw XME hardware status string |
| `Last Error` | Most recent error message (if any) |
| `Missing Sensors` | Count of expected but absent sensors (only if > 0) |
| `Duplicate Sensors` | Count of duplicate sensor IDs detected (only if > 0) |
| `Calibration quality (NPose)` | N-Pose calibration score |
| `Calibration quality (TPose)` | T-Pose calibration score |

**Monitor diagnostics:**
```bash
# Terminal
ros2 topic echo /diagnostics

# RQT GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

## xsens_mvn_ros2_urdf_publisher_node

Dynamically generates a body-proportioned URDF for the skeleton by measuring distances between TF frames published by the stream or XME node, then scaling each mesh segment accordingly. Implemented as a lifecycle node.

The URDF is published on `robot_description` (relative, so `/robot_description` or `/<namespace>/robot_description`) with **transient-local QoS** so that late-joining subscribers (such as RViz) receive it immediately. Each body avatar needs its own instance; the top-level `xsens_stream.launch.py` starts one per named body, the non-primary ones in a namespace named after the avatar (`/actor_b/xsens_urdf_publisher` -> `/actor_b/robot_description`).

> **Lifecycle note:** The `~/republish_urdf` service rejects requests when the node is not in the `active` state. Scale computation and URDF publishing only happen while the node is active.

**Launch:**

Launched automatically by both `xsens_stream.launch.py` and `xsens_xme.launch.py` when `launch_description:=true` (the default). Can also be launched standalone:

```bash
ros2 launch xsens_mvn_ros2 description.launch.py model_name:=skeleton
# a second actor, alongside the first
ros2 launch xsens_mvn_ros2 description.launch.py model_name:=actor_b namespace:=actor_b
```

**Config file:** `config/xsens_urdf_publisher_node.yaml`

**RViz:** `xsens_mvn_ros2_description/launch/rviz.launch.py` starts RViz with one RobotModel display per description topic:

```bash
ros2 launch xsens_mvn_ros2_description rviz.launch.py \
  robot_description_topics:="[/robot_description, /actor_b/robot_description]"
```

### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `robot_description` | `std_msgs/msg/String` | Transient-local | Full URDF XML of the scaled skeleton; `<robot name>` is `model_name` |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/republish_urdf` | `std_srvs/srv/Trigger` | Force immediate re-publication of the URDF |

**Example:**
```bash
ros2 service call /xsens_urdf_publisher/republish_urdf std_srvs/srv/Trigger
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | `"skeleton"` | Must match the TF prefix of the avatar to describe (the Xsens node's `model_name`, or its `avatar_names` entry) |
| `republish_period` | double | `10.0` | Interval in seconds for periodic re-publication after the first successful publish |

Body proportions are measured from TF, not from parameters: the pelvis width comes from the hip-joint distance in TF, so the node takes no `bodyDimension.*` parameters.

### Scaling Algorithm

The node waits for TF data to become available, then computes per-segment scale factors from live TF distances:

| Segment group | Scale axis | Method |
|---------------|------------|--------|
| Pelvis (width) | Y | Bilateral hip-joint distance / 0.16 m reference |
| Pelvis (height) | Z | TF(pelvis→l5).z / 0.096 m reference |
| Spine / leg segments | Z | TF segment length / neutral mesh length |
| Arm / shoulder segments | Y | TF segment length / neutral mesh length |
| Feet | Uniform (XYZ) | TF(foot→toe).x / 0.1526 m reference |

All scale factors are clamped to **[0.5, 2.0]**.

The static URDF template is `urdf/humanoid.urdf.xacro`; body meshes are in `urdf/meshes/neutral/`.

---

## Error Handling

The driver uses three complementary layers for reporting errors. Understanding all three allows you to monitor health at different levels of granularity.

### Layer 1 — ROS2 Logging (`ros2 topic echo /rosout`)

All nodes use the standard `RCLCPP_*` macros. The following severity rules apply across the entire codebase:

| Situation | Macro | Behaviour |
|-----------|-------|-----------|
| Unrecoverable — node cannot function | `RCLCPP_ERROR` | Logged once; node does not retry automatically |
| Transient — retrying automatically | `RCLCPP_WARN_THROTTLE` (5 s) | Suppressed between retries to avoid log spam |
| Expected degradation (low battery, stale data) | `RCLCPP_WARN` | Logged once per state change |
| Normal lifecycle events (connected, ready, config applied) | `RCLCPP_INFO` | Logged once per event |
| High-frequency data paths | `RCLCPP_DEBUG` | Never used on hot paths at INFO or above |

Library code (`XsensStreamClient`, `HumanDataHandler`, `Socket`) uses the same macros via an injected `rclcpp::Logger` so all messages appear under the owning node's logger name.

### Layer 2 — Diagnostics (`ros2 topic echo /diagnostics`)

All three nodes publish to `/diagnostics` via `diagnostic_updater::Updater` (1 Hz). Use `rqt_robot_monitor` for a visual overview.

| Node | Hardware ID | Key/value fields |
|------|-------------|-----------------|
| `xsens_mvn_ros2_stream_node` | `Xsens MVN Stream` | UDP Port, Links, Joints, Last data received (ms ago) |
| `xsens_mvn_ros2_xme_node` | `Xsens MVN XME` | Total Sensors, Battery level (%), Radio Quality (%), Npose/Tpose calibration result, Last Error, Hardware Status |
| `xsens_urdf_publisher_node` | `Xsens URDF Publisher` | Last published (s ago) |

**Status levels:**

- `OK` — nominal operation
- `WARN` — degraded but still running: stale stream data (> 2× expected period), low battery (< 20%), low radio quality (< 20%), poor calibration quality, stale URDF (> 3× republish period)
- `ERROR` — not functional: stream client not initialised, hardware disconnected, hardware error, URDF not yet published (TF unavailable)

### Layer 3 — Service and Action Response Fields

Errors in discrete operations are reported through their ROS2 response types:

| Operation | Type | Error field |
|-----------|------|-------------|
| Load calibration file | `LoadCalibrationPath` service | `bool success` + `string message` |
| Perform calibration | `PerformCalibration` action | `string calibration_result` in the `Result`; `abort()` or `canceled()` terminal states |
| Set parameters | `SetParameters` | `bool successful` + `string reason` in `SetParametersResult` |
| Republish URDF | `std_srvs/Trigger` service | `bool success` + `string message` |

---
