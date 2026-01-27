# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **Path_and_Safety** module for the Buckeye AutoDrive Year 5 autonomous vehicle project. It handles communication between the Intel compute unit and the HMI (Human-Machine Interface) running on a Jetson, using ROS 2 and TCP sockets with Protocol Buffers.

## Architecture

### Communication Flow

```
Intel (ROS 2 Nodes) <--TCP/Protobuf--> HMI Backend (Jetson)
```

**Two-way communication:**
1. **Intel → HMI (TX)**: Navigation data (GPS position, heading, waypoints) and camera streams
2. **HMI → Intel (RX)**: Control commands (engage/disengage, target destination)

### Main Nodes

- **`safety.py`** - Safety state machine (publishes vehicle state):
  - Uses `transitions` library for FSM with states: STARTUP, PASSIVE_MODE, ACTIVATION_CONDITION, BRAKE_ACTIVATION, WAIT_BRAKE_RELEASE, STEER_ACTIVATION, PROPULSION_ACTIVATION, AV, DEACTIVATION, ACTIVATION_FAILURE
  - Publishes FSM state to ROS topic for other nodes to consume
  - CAN bus communication for vehicle control (steering, braking, propulsion)
  - Multiprocessing architecture with shared memory between CAN RX/TX processes

- **`waipoint_node.py`** - Main ROS 2 node (subscribes first, then publishes):
  - **Subscribes to:**
    - `/raw_points_remain` (Float64MultiArray) - waypoints from Controls
    - `/inspvax` (Inspvax) - NovAtel GPS position and heading
    - Safety FSM state topic from safety.py
  - **Publishes to:**
    - `/safety/engage_state` (Int32) - 0=disengage, 1=engage, 3=disabled
    - `/controls/target_destination` (String) - destination letter A-Z
  - **TCP TX** on port 5001: Navigation protobuf to HMI (includes safety_states field)
  - **TCP RX** on port 6001: HMITxMessage commands from HMI

- **`video_node.py`** - Camera streaming node:
  - Subscribes to `/blackfly_0/image_raw`, `/blackfly_1/image_raw`, `/blackfly_2/image_raw`
  - JPEG-compresses frames (quality=50) and sends via RTSP on port 8554 at 24 Hz

### Protobuf Messages

Defined in `testing/protobuf_structure/`:

- **`HMI_RX_CONTROLS.proto`** (Intel → HMI):
  - `Navigation`: current_lat, current_lon, heading_deg, waypoints[], safety_states
  - `Waypoint`: lat, lon

- **`HMI_TX_CONTROLS.proto`** (HMI → Intel):
  - `HMITxMessage`: engage_status (0=DISENGAGE, 1=ENGAGE, 2=DISABLED), target_destination (A-Z)

- **`CAMERA.proto`**:
  - `CameraBatch`: frames[], timestamp
  - `CameraFrame`: camera_id, jpeg_data

### Safety FSM States

Used in `safety_states` field of Navigation protobuf:

| Value | State | Description |
|-------|-------|-------------|
| 0 | STATE_0 | Initial state |
| 1 | STARTUP | System starting, CAN initialization |
| 2 | PASSIVE_MODE | Waiting for HMI engage command |
| 3 | ACTIVATION_CONDITION | Checking activation conditions |
| 4 | BRAKE_ACTIVATION | Activating brakes |
| 5 | WAIT_BRAKE_RELEASE | Waiting for brake release |
| 6 | STEER_ACTIVATION | Activating steering control |
| 7 | PROPULSION_ACTIVATION | Activating propulsion |
| 8 | AV | Autonomous mode active |
| 9 | DEACTIVATION | Deactivating autonomous mode |
| 10 | ACTIVATION_FAILURE | Activation failed, resetting |

### TCP Protocol

All TCP messages use a 4-byte big-endian length prefix followed by the protobuf payload.

## Running the Nodes

```bash
# Run waypoint node (requires ROS 2 environment)
ros2 run <package_name> waipoint_node

# With custom TCP endpoints
ros2 run <package_name> waipoint_node --ros-args -p hmi_tx_host:=192.168.1.100 -p hmi_tx_port:=5001

# Run video node
ros2 run <package_name> video_node --ros-args -p hmi_tx_host:=192.168.1.100 -p hmi_tx_port:=8554
```

## Testing

Test scripts simulate HMI endpoints:

```bash
# Test receiving navigation data from Intel
python intel2hmi_test.py   # Listens on port 5001

# Test sending commands to Intel
python hmi2intel_test.py   # Listens on port 6001, interactive CLI

# Test receiving video streams
python video_test.py       # Listens on port 8554, displays with OpenCV
```

## Regenerating Protobuf Files

```bash
cd testing/protobuf_structure
protoc --python_out=../.. HMI_RX_CONTROLS.proto
protoc --python_out=../.. HMI_TX_CONTROLS.proto
protoc --python_out=../.. CAMERA.proto
```

## Key Dependencies

- ROS 2 (rclpy)
- `novatel_gps_msgs` - NovAtel GPS message types
- `cv_bridge` - ROS/OpenCV image conversion
- `protobuf` - Protocol Buffers
- `opencv-python` (cv2) - Image processing
- `transitions` - State machine library (safety.py)
- `cantools`, `python-can` - CAN bus communication (safety.py)
