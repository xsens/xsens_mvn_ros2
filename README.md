# Xsens MVN ROS2 Driver

ROS2 driver for Xsens motion capture suits. Supports two operational modes: receiving streamed data from Xsens MVN Studio over UDP, or connecting directly to the suit hardware via the Xsens XME SDK.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Lifecycle Management](#lifecycle-management)
- [Architecture](#architecture)
- [Nodes](#nodes)
- [Custom Messages](#custom-messages)
- [Configuration](#configuration)
- [Multiple Avatars](#multiple-avatars)
- [URDF and Visualization](#urdf-and-visualization)
- [Launch Files](#launch-files)
- [Code Style Guidelines](#code-style-guidelines)
- [Testing](#testing)
- [Contributing](#contributing)

---

## Overview

The driver publishes skeletal kinematics as ROS2 topics and TF transforms, making Xsens MVN motion data directly usable in any ROS2 application. All nodes are implemented as **ROS2 managed (lifecycle) nodes**, providing explicit control over initialization, activation, and shutdown. Two complementary modes are available:

| Mode | Node | When to use |
|------|------|-------------|
| **Stream** | `xsens_mvn_ros2_stream_node` | MVN Studio (Windows) is running and streaming data over UDP |
| **XME** | `xsens_mvn_ros2_xme_node` | Direct Linux connection to the suit hardware (no Windows PC needed) |

A third node — the **URDF publisher** — dynamically generates a body-proportioned robot description from the live TF data, enabling accurate mesh-scaled visualization in RViz.

**ROS Distribution:** Jazzy Jalisco
**Build system:** colcon / ament_cmake
**Language:** C++17

---

## Package Structure

```
xsens_mvn_ros2/                      <- workspace root
├── src/
│   ├── pkgs/
│   │   ├── xsens_mvn_msgs/          <- custom ROS2 message/service/action definitions
│   │   ├── xsens_mvn_ros2/          <- meta-package: top-level launch files, exec_depend on all below
│   │   ├── xsens_mvn_ros2_common/   <- shared library: SkeletonPublisher, xsens_model, IMotionCaptureSource
│   │   ├── xsens_mvn_ros2_stream/   <- stream node: UDP client for MVN Studio (multi-avatar aware)
│   │   ├── xsens_mvn_ros2_xme/      <- XME node: direct hardware via XME SDK
│   │   └── xsens_mvn_ros2_description/ <- URDF publisher node, RViz launch, meshes/neutral, xacro, rviz config
│   └── deps/
│       ├── xsens_mvn_sdk/            <- custom MVN UDP protocol parser (ament package)
│       └── xsens_xme_sdk/            <- vendored XME SDK binaries and headers
├── Puppet.mvn / Puppet.mvna          <- example MVN calibration files
└── colcon_defaults.yaml              <- workspace build defaults
```

### Package Dependency Graph

```
xsens_mvn_ros2 (meta-package)
├── xsens_mvn_ros2_stream
│   ├── xsens_mvn_ros2_common
│   │   └── xsens_mvn_msgs
│   └── xsens_mvn_sdk
├── xsens_mvn_ros2_xme
│   ├── xsens_mvn_ros2_common
│   │   └── xsens_mvn_msgs
│   └── xsens_xme_sdk
└── xsens_mvn_ros2_description
    └── xsens_mvn_msgs
```

### Key Components

| Component | Package | Type | Purpose |
|-----------|---------|------|---------|
| `xsens_mvn_ros2_stream_node` | `xsens_mvn_ros2_stream` | Lifecycle Node | UDP client for MVN Studio streaming |
| `xsens_mvn_ros2_xme_node` | `xsens_mvn_ros2_xme` | Lifecycle Node | Direct XME SDK hardware interface |
| `xsens_mvn_ros2_urdf_publisher_node` | `xsens_mvn_ros2_description` | Lifecycle Node | Dynamic, body-scaled URDF generation |
| `SkeletonPublisher` | `xsens_mvn_ros2_common` | Library | TF broadcasting and topic publishing (shared by stream and XME) |
| `IMotionCaptureSource` | `xsens_mvn_ros2_common` | Interface | Abstract interface for dependency injection |
| `HumanDataHandler` | `xsens_mvn_ros2_stream` | Library | Thread-safe skeleton data container |
| `XsensStreamClient` | `xsens_mvn_ros2_stream` | Library | UDP socket and MVN protocol handling |

---

## Installation

### Prerequisites

- **OS:** Ubuntu 24.04 LTS
- **ROS2:** [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) (ros-base or desktop)

Install ROS2 and development tools:

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-ros-base ros-dev-tools
```

For XME mode (direct hardware connection), the XME SDK is vendored in `src/deps/xsens_xme_sdk/` — no additional SDK installation is needed. However, a **valid XME SDK license** is required on the host machine. Without it, the XME node will fail during configuration with a `"License system not constructed"` error. The Linux license manager can be downloaded from the [Xsens Software & Documentation](https://www.xsens.com/support/software-documentation) page. Contact Xsens for license information.

### Clone and Build

```bash
# 1. Clone the repository
git clone <repository-url>
cd xsens_mvn_ros2

# 2. Initialize submodules (if any)
git submodule update --init --recursive

# 3. Source ROS2
source /opt/ros/jazzy/setup.bash

# 4. Install ROS dependencies
sudo rosdep init   # only needed once per system
rosdep update
rosdep install -y -i --from-paths $(colcon list --paths-only)

# 5. Build
colcon build

# 6. Source the workspace
source install/setup.bash
```

### Build Options

The `colcon_defaults.yaml` in the repo root pre-configures common settings:

| Setting | Default | Description |
|---------|---------|-------------|
| Build type | `Debug` | CMake build type |
| Optimization | `-O2` | Compiler optimization level |
| Code coverage | `--coverage` | gcov instrumentation enabled |
| Testing | `ON` | Build and run tests |
| Install mode | Symlink | Faster iteration during development |

To build in Release mode without coverage:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O2
```

### Verify the Installation

```bash
# Run tests
source install/setup.bash
colcon test
colcon test-result --verbose

# Check that nodes are available
ros2 pkg executables xsens_mvn_ros2_stream
ros2 pkg executables xsens_mvn_ros2_xme
ros2 pkg executables xsens_mvn_ros2_description
```

---

## Quick Start

### Stream Mode (MVN Studio)

Configure Xsens MVN Studio to stream data via UDP, then launch:

```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py
```

The Python launch file automatically configures and activates the lifecycle node. RViz and the URDF publisher are included by default. To disable:

```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py launch_rviz:=false launch_description:=false
```

If MVN streams more than one avatar (a second suit, or a tracked object), name them and the launch file brings up a URDF publisher and an RViz model per actor:

```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py \
  track_all_avatars:=true avatar_names:="[actor_a, actor_b, prop]" object_avatars:="[prop]"
```

See [Multiple Avatars](#multiple-avatars) for what this does and why it matters even with a single suit.

### XME Mode (Direct Hardware)

> **XME SDK License:** A valid license must be present on the host machine. If the node fails during configuration with `"License system not constructed"`, the license is missing or invalid. Download the Linux license manager from the [Xsens Software & Documentation](https://www.xsens.com/support/software-documentation) page to activate your license.

> **Before launching:** Edit `src/pkgs/xsens_mvn_ros2_xme/config/body_dimensions.yaml` to match your subject's measurements (all values in metres). Accurate body dimensions are required for correct biomechanical solving. See the [Body Dimensions](#body-dimensions) section for all available parameters.

With the suit connected (Awinda or USB), launch:

```bash
ros2 launch xsens_mvn_ros2 xsens_xme.launch.py
```

**Perform calibration** (N-Pose):

```bash
ros2 action send_goal /xsens_mvn_ros2_xme_node/perform_calibration \
  xsens_mvn_msgs/action/PerformCalibration \
  "{pose: 0, output_file_name: 'my_calibration.mvn'}"
```

**Load an existing calibration**:

```bash
ros2 service call /xsens_mvn_ros2_xme_node/set_calibration_path \
  xsens_mvn_msgs/srv/LoadCalibrationPath \
  "{calibration_file_path: '/path/to/calibration.mvn'}"
```

**Monitor hardware diagnostics**:

```bash
# Command line
ros2 topic echo /diagnostics

# Or with the RQT monitor
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

## Lifecycle Management

All three nodes are implemented as **ROS2 managed (lifecycle) nodes** (`rclcpp_lifecycle::LifecycleNode`). This provides explicit control over node state: initialization, activation, deactivation, and cleanup can be triggered independently via the standard lifecycle interface.

### State Machine

```
    ┌──────────────┐
    │ Unconfigured │ ◄── on_cleanup ── ┐
    └──────┬───────┘                   │
           │ on_configure              │
           ▼                           │
    ┌──────────────┐                   │
    │   Inactive   │ ──────────────────┘
    └──────┬───────┘
           │ on_activate
           ▼
    ┌──────────────┐
    │    Active    │
    └──────┬───────┘
           │ on_deactivate
           ▼
    ┌──────────────┐
    │   Inactive   │
    └──────────────┘
```

### What Happens in Each State

| Callback | Stream Node | XME Node | URDF Publisher |
|----------|-------------|----------|----------------|
| **on_configure** | Creates params, TF broadcaster, SkeletonPublisher, diagnostics, UDP client | Creates params, license, XmeControl, hardware scan (30 s timeout), publishers, services, action server | Creates params, TF buffer/listener, publisher, republish service, diagnostics |
| **on_activate** | Starts the polling timer (begins publishing) | Lifecycle publishers become active (SDK callbacks begin producing output) | Starts fast-poll timer to compute scales and publish URDF |
| **on_deactivate** | Stops the polling timer (UDP client stays alive) | Stops publishing; aborts in-progress calibration | Stops timers, resets published flag |
| **on_cleanup** | Destroys UDP client, resets all publishers and diagnostics | Removes SDK callback handler, disconnects hardware, terminates SDK, resets all ROS objects | Destroys TF listener/buffer, publishers, service, diagnostics |

### Default Behaviour with Python Launch Files

The provided Python launch files (`.launch.py`) automatically configure and activate each node at startup by default. From the user's perspective, the node starts publishing immediately — the same behaviour as a regular `rclcpp::Node`.

```bash
# These handle configure + activate automatically:
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py
ros2 launch xsens_mvn_ros2 xsens_xme.launch.py
ros2 launch xsens_mvn_ros2_description description.launch.py
```

### Starting in Unconfigured State

Set `auto_activate:=false` to launch all nodes in the `unconfigured` state. This is useful for staged bringup, debugging, or when you want full manual control over the lifecycle:

```bash
# Nodes start but remain unconfigured — no hardware scan, no publishing
ros2 launch xsens_mvn_ros2 xsens_xme.launch.py auto_activate:=false

# In another terminal, transition nodes manually when ready
ros2 lifecycle set /xsens_mvn_ros2_xme_node configure
ros2 lifecycle set /xsens_mvn_ros2_xme_node activate
ros2 lifecycle set /xsens_urdf_publisher configure
ros2 lifecycle set /xsens_urdf_publisher activate
```

The `auto_activate` argument is forwarded to all included launch files (e.g. the URDF publisher), so all nodes in the launch graph respect it.

### Manual Lifecycle Control

Regardless of `auto_activate`, you can always manage lifecycle transitions manually using `ros2 lifecycle`:

```bash
# Check current state
ros2 lifecycle get /xsens_mvn_ros2_stream_node

# Step through transitions manually
ros2 lifecycle set /xsens_mvn_ros2_stream_node configure
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate

# Pause publishing (deactivate) without disconnecting
ros2 lifecycle set /xsens_mvn_ros2_stream_node deactivate

# Resume publishing
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate

# Full teardown and return to unconfigured
ros2 lifecycle set /xsens_mvn_ros2_stream_node deactivate
ros2 lifecycle set /xsens_mvn_ros2_stream_node cleanup

# Reconfigure with new parameters
ros2 lifecycle set /xsens_mvn_ros2_stream_node configure
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate
```

> **Note:** Services and action servers on the XME node reject requests when the node is not in the `active` state.

---

## Architecture

### High-Level Overview

The driver follows a layered architecture with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────────────┐
│                          ROS2 Interface Layer                       │
│   ┌──────────────────┐  ┌──────────────────┐  ┌─────────────────┐  │
│   │  Stream Node     │  │  XME Node        │  │  URDF Publisher │  │
│   │  (xsens_mvn_     │  │  (xsens_mvn_     │  │  (xsens_mvn_    │  │
│   │   ros2_stream)   │  │   ros2_xme)      │  │   ros2_         │  │
│   └────────┬─────────┘  └────────┬─────────┘  │   description)  │  │
│            │                     │             └────────┬────────┘  │
│            │                     │                      │           │
│   ┌────────▼─────────────────────▼──────────┐          │           │
│   │         SkeletonPublisher               │◄─────────┘           │
│   │         (xsens_mvn_ros2_common)         │    reads /tf         │
│   └────────┬────────────────────────────────┘                      │
│            │  IMotionCaptureSource interface                       │
├────────────┼───────────────────────────────────────────────────────┤
│            │           Data Source Layer                            │
│   ┌────────▼─────────┐  ┌──────────────────┐                      │
│   │ XsensStreamClient│  │ XmeControl       │                      │
│   │ + HumanDataHandler│  │ (XME SDK)        │                      │
│   └────────┬─────────┘  └────────┬─────────┘                      │
│            │                     │                                  │
├────────────┼─────────────────────┼──────────────────────────────────┤
│            │       Protocol / Hardware Layer                        │
│   ┌────────▼─────────┐  ┌──────────────────┐                      │
│   │ xsens_mvn_sdk    │  │ xsens_xme_sdk    │                      │
│   │ (UDP parser)     │  │ (vendored binary) │                      │
│   └──────────────────┘  └──────────────────┘                      │
└─────────────────────────────────────────────────────────────────────┘
```

**Key design principles:**

- **Lifecycle nodes** — All nodes inherit from `rclcpp_lifecycle::LifecycleNode`, enabling explicit control over initialization, activation, and shutdown. Publishers use `LifecyclePublisher` which automatically gates output based on the node's state.
- **Dependency injection** — Both stream and XME nodes use `SkeletonPublisher` for all ROS publishing. The data source is decoupled from the publishing logic via the `IMotionCaptureSource` interface.
- **Thread safety** — `HumanDataHandler` provides a thread-safe container between the UDP receive thread and the ROS timer callback. XME SDK callbacks are similarly synchronized. The XME node removes the SDK callback handler before destroying ROS objects during cleanup to prevent use-after-free.
- **Generated parameters** — All nodes use `generate_parameter_library` for type-safe, validated parameters with zero manual `declare_parameter` calls.
- **Diagnostics-first** — Every node publishes to `/diagnostics` via `diagnostic_updater::Updater` at 1 Hz, making health monitoring uniform.

### Data Flow — Stream Mode

```
Xsens MVN Studio (Windows, UDP)
        │
        ▼
XsensStreamClient (UDP socket, background thread)
        │  MVN protocol datagrams
        ▼
ParserManager (xsens_mvn_sdk)
        │  parsed kinematics
        ▼
HumanDataHandler (thread-safe data container)
        │
        ▼
SkeletonPublisher
    ├──> /tf                  (TF2 transforms per segment)
    ├──> ~/joint_states       (Euler angles)
    ├──> ~/link_states        (full kinematic state)
    └──> ~/com                (centre of mass)
```

### Data Flow — XME Mode

```
Xsens Suit Hardware (Awinda / USB)
        │
        ▼
XmeControl (Xsens XME SDK)
        │  callbacks
        ▼
XsensXmeNode
    ├── CalibrationModule  ──> ~/perform_calibration (action server)
    │                      ──> ~/set_calibration_path (service)
    ├── DiagnosticsModule  ──> /diagnostics
    └── SkeletonPublisher
            ├──> /tf
            ├──> ~/joint_states
            ├──> ~/link_states
            └──> ~/com
```

### Data Flow — URDF Publisher

```
TF frames (published by either stream or XME node)
        │  measure joint distances
        ▼
XsensUrdfPublisherNode
        │  scale mesh per segment
        ▼
/robot_description  (std_msgs/String, transient-local QoS)
```

### Skeleton Definition

The driver uses the 23-segment MVN body model defined in `xsens_model.hpp`, plus an optional 20-segment finger block per hand when MANUS gloves are streamed (see [Finger segments](#finger-segments-manus-gloves)):

```
pelvis
  ├── l5 -> l3 -> t12 -> t8
  │              └── neck -> head
  │              ├── left_shoulder  -> left_upper_arm  -> left_forearm  -> left_hand
  │              └── right_shoulder -> right_upper_arm -> right_forearm -> right_hand
  ├── left_upper_leg  -> left_lower_leg  -> left_foot  -> left_toe
  └── right_upper_leg -> right_lower_leg -> right_foot -> right_toe
```

Frame names are prefixed with the `model_name` parameter (default: `skeleton`), e.g. `skeleton_pelvis`.

### Parameter System

Each node uses a **generated typed `ParamListener`** — no raw `declare_parameter` calls anywhere:

| YAML definition file | Generated class | Used by |
|---------------------|----------------|---------|
| `xsens_stream_node_parameters.yaml` | `xsens_stream_node::ParamListener` | Stream node |
| `xsens_xme_node_parameters.yaml` | `xsens_xme_node::ParamListener` | XME node |
| `body_dimensions_parameters.yaml` | `body_dimensions::ParamListener` | XME node (in common pkg) |
| `xsens_urdf_publisher_parameters.yaml` | `xsens_urdf_publisher::ParamListener` | URDF publisher |

The XME node additionally uses `add_on_set_parameters_callback` for runtime hardware-side updates (sampleRate, suitConfiguration, bodyDimension.*, biomechanicalModel).

### Error Handling

The driver uses three complementary layers for reporting errors:

**Layer 1 — ROS2 Logging** (`ros2 topic echo /rosout`)

| Situation | Macro | Behaviour |
|-----------|-------|-----------|
| Unrecoverable — node cannot function | `RCLCPP_ERROR` | Logged once; node does not retry |
| Transient — retrying automatically | `RCLCPP_WARN_THROTTLE` (5 s) | Suppressed between retries |
| Expected degradation (low battery, stale data) | `RCLCPP_WARN` | Logged once per state change |
| Normal lifecycle events | `RCLCPP_INFO` | Logged once per event |
| High-frequency data paths | `RCLCPP_DEBUG` | Never used at INFO or above on hot paths |

Library code uses the same macros via an injected `rclcpp::Logger` so all messages appear under the owning node's logger name.

**Layer 2 — Diagnostics** (`ros2 topic echo /diagnostics`)

All three nodes publish to `/diagnostics` via `diagnostic_updater::Updater` (1 Hz):

| Node | Hardware ID | Key/value fields |
|------|-------------|-----------------|
| Stream | `Xsens MVN Stream` | UDP Port, Links, Joints, Last data received (ms ago) |
| XME | `Xsens MVN XME` | Total Sensors, Battery level (%), Radio Quality (%), Calibration quality, Last Error |
| URDF Publisher | `Xsens URDF Publisher` | Last published (s ago) |

Status levels: `OK` = nominal, `WARN` = degraded but running, `ERROR` = not functional.

**Layer 3 — Service/Action Response Fields**

| Operation | Type | Error field |
|-----------|------|-------------|
| Load calibration file | `LoadCalibrationPath` srv | `bool success` + `string message` |
| Perform calibration | `PerformCalibration` action | `string calibration_result`; `abort()`/`canceled()` |
| Set parameters | `SetParameters` | `bool successful` + `string reason` |
| Republish URDF | `std_srvs/Trigger` srv | `bool success` + `string message` |

---

## Nodes

For full parameter and topic reference, see [`src/pkgs/xsens_mvn_ros2/README.md`](src/pkgs/xsens_mvn_ros2/README.md).

### xsens_mvn_ros2_stream_node

UDP client for Xsens MVN Studio streaming mode. Lifecycle node.

**Launch:** `ros2 launch xsens_mvn_ros2 xsens_stream.launch.py`
**Config:** `src/pkgs/xsens_mvn_ros2_stream/config/xsens_stream_node.yaml`

Key parameters: `udp_port` (default `9763`), `model_name`, `reference_frame`, `update_frequency`, `avatar_id` (default `0`), `track_all_avatars` (default `false`), `avatar_names`, `avatar_stale_timeout` (default `1.0` s). All but `update_frequency` and `avatar_stale_timeout` are also launch arguments, so a multi-suit scene needs no config-file edits. See [Multiple Avatars](#multiple-avatars).

### xsens_mvn_ros2_xme_node

Direct hardware interface via XME SDK. Provides calibration actions, live diagnostics, and configurable body dimensions. Lifecycle node. Services and actions reject requests when the node is not active.

**Launch:** `ros2 launch xsens_mvn_ros2 xsens_xme.launch.py`
**Config:** `src/pkgs/xsens_mvn_ros2_xme/config/xsens_xme_node.yaml` and `config/body_dimensions.yaml`

Key parameters: `awindaChannel`, `sampleRate` (default `240 Hz`), `biomechanicalModel` (`Legacy` / `Female` / `Male`), `suitConfiguration`.

### xsens_mvn_ros2_urdf_publisher_node

Reads live TF data to measure subject body proportions and publishes a scaled URDF to `/robot_description`. Lifecycle node.

**Launch:** Included automatically in `xsens_stream.launch.py` and `xsens_xme.launch.py` when `launch_description:=true` (one instance per body avatar in a multi-avatar scene).
**Config:** `src/pkgs/xsens_mvn_ros2_description/config/xsens_urdf_publisher_node.yaml`

Service `~/republish_urdf` (`std_srvs/Trigger`) forces immediate re-publication. Rejects requests when the node is not active.

The same package provides `rviz.launch.py`, which starts RViz with one RobotModel display per `robot_description` topic it is given. The top-level launch files use it so RViz follows the `namespace` argument and shows every actor.

---

## Custom Messages

### LinkState.msg

Full kinematic state for a single body segment.

```
std_msgs/Header header
geometry_msgs/Pose  pose   # position + orientation in reference frame
geometry_msgs/Twist twist  # linear and angular velocity
geometry_msgs/Accel accel  # linear and angular acceleration
```

### LinkStateArray.msg

```
std_msgs/Header header
xsens_mvn_msgs/LinkState[] states
```

### LoadCalibrationPath.srv

```
# Request
string calibration_file_path
---
# Response
string message
bool   success
```

### PerformCalibration.action

```
# Goal
int8 NPOSE = 0
int8 TPOSE = 1
int8   pose             # 0 = N-Pose, 1 = T-Pose
string output_file_name # must include .mvn extension
---
# Result
string calibration_result
---
# Feedback
string status           # real-time progress updates
```

---

## Configuration

### Body Dimensions

Edit `src/pkgs/xsens_mvn_ros2_xme/config/body_dimensions.yaml` to match your subject. All values are in metres.

```yaml
/**:
  ros__parameters:
    bodyDimension:
      bodyHeight:      1.85
      footSize:        0.31
      shoulderHeight:  1.4434
      shoulderWidth:   0.38
      elbowSpan:       0.94
      wristSpan:       1.43
      armSpan:         1.796
      hipHeight:       0.8744
      hipWidth:        0.24
      kneeHeight:      0.4861
      ankleHeight:     0.08
      shoeSoleHeight:  0.0
```

These values are used by the XME node for biomechanical solving. The URDF publisher derives body proportions independently from live TF data.

### Awinda Channel

If you experience wireless interference, set a fixed channel in `xsens_xme_node.yaml`:

```yaml
awindaChannel: 15   # range 11-25; -1 = auto
```

---

## Multiple Avatars

MVN streams **every avatar in the scene to the same UDP port** — each captured
subject plus every tracked object — distinguished only by an avatar id in the
datagram header. They share the datagram types: an object arrives as a
one-segment `PoseQuaternion` packet of exactly the same type as a subject's
63-segment one.

> **This matters even for a single-suit setup.** If MVN streams more than one
> avatar and the node is not told which to use, datagrams from the others
> overwrite the subject's pose and every segment they do not define reads as
> zero. Enabling object or multi-actor streaming in MVN without setting
> `avatar_id` will therefore corrupt the skeleton. The default (`avatar_id: 0`)
> is the captured subject, so single-suit setups are safe out of the box.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `avatar_id` | `0` | The primary avatar. Its data is published on the unprefixed topics and reported through the node's single-avatar API. Datagrams from other avatars are discarded unless `track_all_avatars` is set. |
| `track_all_avatars` | `false` | Publish every avatar MVN sends, not just `avatar_id`. |
| `avatar_names` | `[]` | TF prefix per avatar, indexed by avatar id, e.g. `["actor_a", "actor_b", "prop"]`. Ids without an entry fall back to `model_name` for the primary avatar and `<model_name>_<id>` for the rest. |
| `avatar_stale_timeout` | `1.0` | Seconds without data after which an avatar stops being published, so a dropped avatar does not leave a frozen frame in TF. |

### Topics and frames

The primary avatar keeps the unprefixed topics, so a single-suit setup sees
exactly what it always did. Every other avatar is namespaced under its name:

| Avatar | Topics | TF frames |
|--------|--------|-----------|
| primary (`avatar_id`) | `link_states`, `joint_states`, `com` | `<model_name>_<segment>` |
| another body | `<name>/link_states`, `<name>/joint_states`, `<name>/com` | `<name>_<segment>` |
| an object | `<name>/link_states`, `<name>/com` | `<name>_base_link` |

An object advertises no `joint_states`: it has no joints, so the topic could
only ever carry an empty message. Its pose, velocity and acceleration all
arrive on `link_states`, whose `LinkState` carries `Pose`, `Twist` and `Accel` —
MVN streams linear and angular kinematics for objects as well as for bodies.
(The primary avatar is set up before its kind is known, so it keeps a
`joint_states` topic even in the unusual case of `avatar_id` naming an object.)

### Bodies and objects

An avatar with fewer segments than the 23-segment body model is treated as a
rigid object: its segments are named `base_link` (then `link_2`, `link_3`, …)
and it is published as an absolute pose in `reference_frame`. Objects send no
joint-angle datagram, so the node does not wait for one.

Avatars are discovered while streaming — no restart is needed when one joins.
If an avatar's segment count changes (MVN recomposing a scene, an actor
joining, avatars being renumbered) its model is rebuilt, because the id may now
mean something entirely different.

`/diagnostics` lists every avatar with its name, kind and data age, and warns
when one goes stale:

```
Avatars tracked: 3
Avatar 0: actor_a (body), last data 5 ms ago
Avatar 1: actor_b (body), last data 5 ms ago
Avatar 2: prop (object), last data 5 ms ago
```

### Example: two actors and an object

```bash
ros2 launch xsens_mvn_ros2 xsens_stream.launch.py \
  track_all_avatars:=true \
  avatar_names:="[actor_a, actor_b, prop]" \
  object_avatars:="[prop]"
```

This one command brings up everything the scene needs:

| What | Where |
|------|-------|
| Stream node | `/xsens_mvn_ros2_stream_node`, publishing all three avatars |
| URDF publisher for `actor_a` (primary, id 0) | `/xsens_urdf_publisher` -> `/robot_description` |
| URDF publisher for `actor_b` | `/actor_b/xsens_urdf_publisher` -> `/actor_b/robot_description` |
| RViz | one **RobotModel** display per actor, on the topics above |

Each **body** avatar needs its own URDF publisher, since `robot_description` is
one topic per model, so the launch file starts one per *named* body: the
primary avatar in the launch `namespace`, every other body in a sub-namespace
named after it. That is the same layout the stream node uses for its topics, so
`/actor_b/link_states` and `/actor_b/robot_description` sit side by side.

`object_avatars` lists which names are tracked objects. An object has no pelvis
frame, so a URDF publisher started for it would wait forever and report an
error in `/diagnostics`; naming it here skips the publisher and the RobotModel
display. Objects are still published on their own topics and TF frames — show
them with a **TF** or **Axes** display in RViz.

Two things to keep in mind:

- **Name every body you want a URDF for.** An avatar without an
  `avatar_names` entry is still published (as `<model_name>_<id>`), but the
  launch file cannot start a URDF publisher for an id it does not know about.
- **`avatar_names` overrides `model_name` for the primary avatar.** With
  `avatar_names:="[actor_a, ...]"` the primary frames are `actor_a_*`, and the
  launch file points its URDF publisher at `actor_a` accordingly. When you run
  the nodes by hand, keep the two in sync yourself.

The same arguments exist on `xsens_mvn_ros2_stream`'s own `xsens_stream.launch.py`
if you only want the node, and can be given to the node directly:

```bash
ros2 run xsens_mvn_ros2_stream xsens_mvn_ros2_stream_node --ros-args \
  -p track_all_avatars:=true \
  -p avatar_names:="['actor_a','actor_b','prop']"
```

With `auto_activate:=false` there is one URDF publisher per body to transition,
e.g. `/xsens_urdf_publisher` and `/actor_b/xsens_urdf_publisher`.

---

## URDF and Visualization

The URDF publisher generates a body-proportioned skeleton by measuring TF distances between adjacent joints and scaling each mesh accordingly:

| Segment group | Scale axis | Reference distance |
|---------------|------------|-------------------|
| Pelvis (width) | Y | Bilateral hip-joint distance / 0.16 m |
| Spine / legs | Z | TF segment length / neutral mesh length |
| Arms / shoulders | Y | TF segment length / neutral mesh length |
| Feet | Uniform | Ankle-to-ball-of-foot distance / 0.1526 m |

Scales are clamped to the range [0.5, 2.0]. The static template is `src/pkgs/xsens_mvn_ros2_description/urdf/humanoid.urdf.xacro`; the meshes the generated URDF references are in `src/pkgs/xsens_mvn_ros2_description/meshes/neutral/`.

The segment tables that drive generation live in
`include/xsens_mvn_ros2_description/segment_defs.hpp`, shared by the node and
its tests so the two cannot drift apart.

### Finger segments (MANUS gloves)

When MVN streams finger tracking, each hand adds a 20-segment block. The block
is **not** a uniform 5 x 4 grid of phalanges — it is the MVN hand model, which
happens to also total 20:

```
carpus (1) + thumb MC/PP/DP (3) + four fingers x MC/PP/MP/DP (16)
```

Frame names mirror MVN's own, transliterated to snake_case exactly as the body
segments are, so they line up one-to-one with what MVN reports:

| MVN segment | TF frame (with `model_name: skeleton`) |
|-------------|----------------------------------------|
| `LeftCarpus` | `skeleton_left_carpus` |
| `LeftFirstMC` | `skeleton_left_first_mc` |
| `LeftFirstPP` | `skeleton_left_first_pp` |
| `LeftFirstDP` | `skeleton_left_first_dp` |
| `LeftSecondMC` | `skeleton_left_second_mc` |
| `LeftSecondPP` | `skeleton_left_second_pp` |
| … | … |

The thumb (`first`) has no middle phalange — that is why a hand is 20 segments
and not 21. The carpus sits on the wrist with zero offset from the hand, and
each finger's metacarpal runs back through the palm.

MVN sends a metacarpal for every finger, but the neutral mesh set ships no
`SecondMC` or `FifthMC`. Those two links per hand are emitted without geometry:
the kinematic chain is intact and the palm is covered by the carpus plus the
third and fourth metacarpal meshes.

Each finger mesh is authored in a shared hand frame whose origin is at
y = -0.0706 m (left) / +0.0706 m (right); the generated visual origin shifts
each mesh so its proximal end lands on the joint, and is scaled alongside it.

When a hand's finger block is live, that hand's closed-fist mesh is suppressed —
it spans the whole hand and would intersect the individual phalanges.

**RViz configuration** — a pre-built workspace is available at:
`src/pkgs/xsens_mvn_ros2_description/config/xsens_visualization.rviz`

### One URDF publisher per model

The generated URDF takes its `<robot name="...">` from `model_name`. RViz keys
its RobotModel display off that name, so two avatars sharing a name collide:
one display renders and the other reports an invalid model until the first is
switched off. Give every avatar a distinct `model_name`.

`robot_description` is a relative topic name, so running a publisher inside a
namespace moves it there (`-r __ns:=/actor_b` publishes
`/actor_b/robot_description`). Two publishers in the same namespace would
overwrite each other's description. The top-level `xsens_stream.launch.py`
does this namespacing for you; see [Multiple Avatars](#multiple-avatars) for a
worked two-actor example.

### RViz follows the topics

`xsens_mvn_ros2_description/launch/rviz.launch.py` starts RViz from a config
file and rewrites its **RobotModel** display into one display per
`robot_description` topic it is given (`robot_description_topics:="[/robot_description, /actor_b/robot_description]"`).
The top-level launch files use it, so RViz shows the model of a namespaced
driver and every actor of a multi-avatar scene without manual display setup.
The stock config, or one you pass with `rviz_config_file`, is left untouched
on disk; the rewritten copy is a temporary file. A custom config without a
RobotModel display is used as is.

---

## Launch Files

### Python Launch Files

The Python launch files are the primary entry point. They configure and activate the lifecycle nodes for you (unless `auto_activate:=false`) and handle the multi-avatar bring-up.

| Package | File | Description |
|---------|------|-------------|
| `xsens_mvn_ros2` (meta) | `xsens_stream.launch.py` | Stream node + a URDF publisher per body avatar + RViz showing them all |
| `xsens_mvn_ros2` (meta) | `xsens_xme.launch.py` | XME node + URDF publisher + RViz |
| `xsens_mvn_ros2` (meta) | `description.launch.py` | Wrapper that includes the description package launch |
| `xsens_mvn_ros2_stream` | `xsens_stream.launch.py` | Stream node only, with all avatar parameters as arguments |
| `xsens_mvn_ros2_xme` | `xsens_xme.launch.py` | XME node only |
| `xsens_mvn_ros2_description` | `description.launch.py` | One URDF publisher |
| `xsens_mvn_ros2_description` | `rviz.launch.py` | RViz with one RobotModel display per `robot_description` topic |

### XML Launch Files

The original XML launch files are still included alongside the Python ones. They start the lifecycle nodes in the **unconfigured** state, so you must manage state transitions manually. This is useful for debugging, staged bringup, or integration into a larger launch system that manages lifecycle externally.

| Package | File | Description |
|---------|------|-------------|
| `xsens_mvn_ros2_stream` | `xsens_stream.launch.xml` | Stream node + optional RViz + optional URDF publisher |
| `xsens_mvn_ros2_xme` | `xsens_xme.launch.xml` | XME node + optional RViz + optional URDF publisher |
| `xsens_mvn_ros2_description` | `description.launch.xml` | URDF publisher standalone |
| `xsens_mvn_ros2` (meta) | `xsens_stream.launch.xml` | Wrapper that includes the stream package launch |
| `xsens_mvn_ros2` (meta) | `xsens_xme.launch.xml` | Wrapper that includes the XME package launch |
| `xsens_mvn_ros2` (meta) | `description.launch.xml` | Wrapper that includes the description package launch |

The XML files start one URDF publisher and one RViz RobotModel on `/robot_description`. Multi-avatar bring-up (`avatar_names`, `object_avatars`, per-actor URDF publishers and RViz displays) is only in the Python launch files.

### Example: Stream Mode with XML

```bash
# 1. Launch the node (starts in unconfigured state)
ros2 launch xsens_mvn_ros2 xsens_stream.launch.xml

# 2. In another terminal, configure and activate the stream node
ros2 lifecycle set /xsens_mvn_ros2_stream_node configure
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate

# 3. Configure and activate the URDF publisher (if launch_description:=true)
ros2 lifecycle set /xsens_urdf_publisher configure
ros2 lifecycle set /xsens_urdf_publisher activate
```

### Example: XME Mode with XML

```bash
# 1. Launch the node
ros2 launch xsens_mvn_ros2 xsens_xme.launch.xml

# 2. Configure and activate (hardware scan happens during configure)
ros2 lifecycle set /xsens_mvn_ros2_xme_node configure
ros2 lifecycle set /xsens_mvn_ros2_xme_node activate

# 3. Activate the URDF publisher
ros2 lifecycle set /xsens_urdf_publisher configure
ros2 lifecycle set /xsens_urdf_publisher activate
```

### Example: Pause and Resume

The XML launch approach is useful when you want to pause and resume data publishing without restarting the node:

```bash
# Pause publishing (stream client stays connected, XME hardware stays connected)
ros2 lifecycle set /xsens_mvn_ros2_stream_node deactivate

# Resume publishing
ros2 lifecycle set /xsens_mvn_ros2_stream_node activate
```

### Python Launch Arguments

Run `ros2 launch xsens_mvn_ros2 xsens_stream.launch.py -s` for the live list. Both top-level launches take:

| Argument | Default | Description |
|----------|---------|-------------|
| `auto_activate` | `true` | Automatically configure and activate all lifecycle nodes on startup. Set to `false` for manual lifecycle control. |
| `namespace` | `""` | ROS2 namespace prefix for all nodes, topics, and services. RViz follows it. |
| `model_name` | `"skeleton"` | TF prefix of the (primary) avatar; passed to node and URDF publisher |
| `launch_rviz` | `true` | Whether to start RViz |
| `launch_description` | `true` | Whether to start the URDF publisher node(s) |
| `discovery_range` | `LOCALHOST` | DDS discovery scope |
| `rviz_config_file` | built-in | Path to a custom RViz config; its RobotModel display is repeated per actor |

`xsens_stream.launch.py` additionally takes the stream-node parameters:

| Argument | Default | Description |
|----------|---------|-------------|
| `udp_port` | `9763` | UDP port MVN streams to |
| `reference_frame` | `"world"` | Root TF frame |
| `avatar_id` | `0` | MVN avatar published on the unprefixed topics |
| `track_all_avatars` | `false` | Publish every avatar MVN streams |
| `avatar_names` | `[]` | TF prefix per avatar id, e.g. `[actor_a, actor_b, prop]`; a URDF publisher and RViz model is started per named body |
| `object_avatars` | `[]` | Entries of `avatar_names` that are tracked objects, so they get no URDF publisher or RViz model |

List arguments accept YAML (`"[a, b]"`) or a comma-separated string (`"a,b"`).

### XML Launch Arguments

The XML launches accept the scalar arguments of the Python versions, except `auto_activate` and `namespace` (XML launches always start in the unconfigured state with no namespace):

| Argument | Default | Description |
|----------|---------|-------------|
| `model_name` | `"skeleton"` | Passed to node and URDF publisher |
| `launch_rviz` | `true` | Whether to start RViz |
| `launch_description` | `true` | Whether to start the URDF publisher node |
| `discovery_range` | `LOCALHOST` | DDS discovery scope |
| `rviz_config_file` | built-in | Path to a custom RViz config |
| `udp_port`, `reference_frame`, `avatar_id`, `track_all_avatars` | as above | Stream launch only. `avatar_names` and `object_avatars` are Python-only. |

---

## Code Style Guidelines

### C++ Standard and Compiler Flags

- **Standard:** C++17
- **Compiler warnings:** `-Wall -Wextra -Wpedantic` (enforced in every `CMakeLists.txt`)
- **Linker safety:** `-Wl,--no-undefined` (catches missing symbols at link time)

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| **Files** | `snake_case` with `.hpp`/`.cpp` | `skeleton_publisher.hpp`, `xsens_stream_node.cpp` |
| **Classes / Structs / Enums** | `PascalCase` | `SkeletonPublisher`, `SegmentKinematics`, `ConfigurationResult` |
| **Functions / Methods** | `camelCase` | `initializeXsensStreamClient()`, `publishLinkStates()`, `diagnosticsCallback()` |
| **Member variables** | `m_` prefix + `camelCase` | `m_tfBroadcaster`, `m_paramListener`, `m_lastDataTimeNs` |
| **Constants** | `k` prefix + `PascalCase` | `kParent`, `kEmpty` |
| **Namespaces** | `snake_case` | `xsens_mvn_ros2`, `xsens_xme_ros2` |
| **ROS2 topics / services** | `snake_case` | `~/joint_states`, `~/perform_calibration` |
| **TF frame names** | `{model_name}_{segment}` | `skeleton_pelvis`, `skeleton_left_hand` |

### Header Files

- Use **`#pragma once`** for include guards.
- Prefer **angle-bracket includes** (`<package/header.hpp>`) for installed package headers.
- Group includes in order: standard library, third-party (Eigen, Boost), ROS2, project headers.

### Documentation

- Use **`///`** (Doxygen-style) doc comments on public APIs (classes, methods, structs).
- Add **`@param`** and **`@return`** annotations for non-trivial interfaces.
- Inline comments only where logic is not self-evident.

### ROS2-Specific Patterns

- **Use lifecycle nodes** (`rclcpp_lifecycle::LifecycleNode`) for all nodes. Perform resource allocation in `on_configure`, start publishing in `on_activate`, and release resources in `on_cleanup`. Use `LifecyclePublisher` instead of `rclcpp::Publisher`.
- **No raw `declare_parameter` calls.** Use `generate_parameter_library` with a YAML definition.
- **Inject `rclcpp::Logger`** into library classes — do not create nodes just for logging.
- **Publish diagnostics** via `diagnostic_updater::Updater` on every node.
- **Use `RCLCPP_WARN_THROTTLE`** (5 s) for transient/retrying conditions — never spam logs.
- **Guard services and actions** with a lifecycle state check (`get_current_state().id() == PRIMARY_STATE_ACTIVE`) to reject requests when the node is not active.

### CMake Patterns

- Every package uses `ament_cmake` as the build type.
- Shared libraries are named `${PROJECT_NAME}_ComponentName` (e.g. `xsens_mvn_ros2_stream_HumanDataHandler`).
- Use `BUILD_INTERFACE` / `INSTALL_INTERFACE` generator expressions for include directories.
- Export targets via `ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)`.
- Generated parameter libraries are declared with `generate_parameter_library()` and linked as a regular CMake target.

### Linting

All packages declare `ament_lint_auto` + `ament_lint_common` as test dependencies. This enables the standard ROS2 linting suite automatically:

- **ament_copyright** — license header check (**currently disabled** — `set(ament_cmake_copyright_FOUND TRUE)` in each `CMakeLists.txt` until the project license is finalized; re-enable by removing that line and adding the appropriate copyright/license headers to all source files)
- **ament_cppcheck** — static analysis
- **ament_cpplint** — Google C++ style subset
- **ament_flake8** — Python linting (launch files)
- **ament_lint_cmake** — CMakeLists.txt linting
- **ament_xmllint** — XML validation (package.xml, launch files)
- **ament_uncrustify** — C++ code formatting (the default ROS2 formatter; no custom `.clang-format` overrides)

---

## Testing

### Test Framework

Unit tests use **Google Test** (`ament_cmake_gtest`). The `colcon_defaults.yaml` configures sequential test execution with up to 3 retries on failure.

### Existing Tests

| Package | Test | What it covers |
|---------|------|---------------|
| `xsens_mvn_ros2_common` | `test_xsens_model` | `xmeSegmentToCanonical`, `kineticParent` lookups |
| `xsens_mvn_ros2_common` | `test_skeleton_publisher` | POD types, arm correction functions, frame naming |
| `xsens_mvn_ros2_common` | `test_skeleton_publisher_node` | TF publishing, relative transforms via lifecycle node (configure + activate) |
| `xsens_mvn_ros2_stream` | `test_human_data_handler` | `setLink`/`getLink` roundtrip, COM, thread safety |
| `xsens_mvn_ros2_stream` | `test_stream_client_mock` | `IMotionCaptureSource` contract, including the single-avatar defaults of the multi-avatar API |
| `xsens_mvn_ros2_stream` | `test_avatar_demux` | Multi-avatar demultiplexing, replayed from a recorded three-avatar MVN scene (see below) |
| `xsens_mvn_ros2_description` | `test_segment_defs` | Body and finger tables: no null parents, no duplicates, positive reference lengths, finger chain and per-hand mesh correctness |
| `xsens_mvn_ros2_description` | `test_scale_utils` | Scale-factor computation and the [0.5, 2.0] clamp |
| `xsens_mvn_ros2_xme` | Stub only | Hardware-in-the-loop not available without suit |

### The MVN capture fixture

`src/pkgs/xsens_mvn_ros2_stream/test/data/mvn_three_avatars.bin` is a trimmed
recording of a real MVN stream: two body avatars (63 segments, 28 joints, with
MANUS finger data) and one tracked object (1 segment), two frames of every
datagram type each. It is length-prefixed records — a 4-byte big-endian length
followed by that many bytes of datagram.

`test_avatar_demux` replays it into a real UDP socket, so the demultiplexing is
exercised against genuine MVN bytes rather than synthesised ones and the tests
also catch the wire format drifting. It covers avatars staying separated,
objects building a jointless model, single-avatar mode ignoring the rest, an
object as the primary avatar, a model rebuilding when its segment count
changes, an avatar going quiet, and a stream that only starts a few seconds
after the client (receive timeouts must not be mistaken for an empty datagram).

Re-recording it needs the node stopped (it owns the UDP port) and the scene
still has to be two bodies plus an object, which `FixtureHoldsThreeAvatars`
asserts.

### Running Tests

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run all tests
colcon test

# Run tests for a specific package
colcon test --packages-select xsens_mvn_ros2_stream

# View results
colcon test-result --verbose
```

---

## Contributing

### Branching

Feature branches follow the pattern `feature/<ticket>_<description>` (e.g. `feature/2588_create_ros_complient_error_handling`).

### Adding a New Package

1. Create the package under `src/pkgs/`.
2. Add it to the `packages-up-to` list in `colcon_defaults.yaml`.
3. Add `ament_lint_auto` and `ament_lint_common` as test dependencies in `package.xml`.
4. Add `ament_lint_auto_find_test_dependencies()` in your `CMakeLists.txt` testing block.
5. If the meta-package should depend on it, add an `<exec_depend>` to `src/pkgs/xsens_mvn_ros2/package.xml`.

---
