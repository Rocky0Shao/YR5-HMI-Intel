# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **Path_and_Safety** module for the Buckeye AutoDrive Year 5 autonomous vehicle project. It handles communication between the Intel compute unit and the HMI (Human-Machine Interface) running on a Jetson, using ROS 2 and TCP sockets with Protocol Buffers.

## Architecture

### Communication Flow

```
Intel (ROS 2 Nodes) <--TCP/Protobuf--> HMI Backend (Jetson)
```

### Port Assignments

| Port | Direction | Node | Content | Protocol |
|------|-----------|------|---------|----------|
| 5001 | Intel → HMI | data_stream_node | Navigation + CameraBatch | TCP/Protobuf (type-prefixed) |
| 5003 | Intel → HMI | safety_comms_node | SafetyStatus | TCP/Protobuf |
| 6001 | HMI → Intel | safety_comms_node | HMITxMessage | TCP/Protobuf |

### Main Nodes

#### `safety_comms_node.py` - Safety Communications (Bidirectional)

Handles safety FSM state transmission and HMI command reception.

**RX (HMI → Intel) on port 6001:**
- Intel listens as server, HMI connects as client
- Receives `HMITxMessage` (engage_status, target_destination)

**TX (Intel → HMI) on port 5003:**
- Intel connects as client, HMI listens as server
- Sends `SafetyStatus` on every state change

**ROS Subscriptions:**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/fsm_state` | `std_msgs/String` | FSM state number as string |
| `/fsm_description` | `std_msgs/String` | FSM state description (e.g., "close door") |

**ROS Publications:**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/safety/engage_state` | `std_msgs/Int32` | 0=disengage, 1=engage, 3=disabled |
| `/controls/target_destination` | `std_msgs/String` | Destination letter A-Z |

#### `data_stream_node.py` - Data Streaming (TX Only)

Streams navigation and video data to HMI on a single port with message type discrimination.

**TX (Intel → HMI) on port 5001:**
- Intel connects as client, HMI listens as server
- Uses 1-byte message type discriminator:
  - `0x01` = Navigation (sent at 5 Hz)
  - `0x02` = CameraBatch (sent at 24 Hz)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][1 byte: message type][N bytes: protobuf payload]
```
Note: The length field includes the type byte.

**ROS Subscriptions:**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/raw_points_remain` | `std_msgs/Float64MultiArray` | Waypoints [lat1,lon1,lat2,lon2,...] |
| `/inspvax` | `novatel_gps_msgs/Inspvax` | GPS position and heading |
| `/blackfly_0/image_raw` | `sensor_msgs/Image` | Camera 0 |
| `/blackfly_1/image_raw` | `sensor_msgs/Image` | Camera 1 |
| `/blackfly_2/image_raw` | `sensor_msgs/Image` | Camera 2 |

### Legacy Nodes (Deprecated)

- **`waipoint_node.py`** - Original combined node (replaced by safety_comms_node + data_stream_node)
- **`video_node.py`** - Original video-only node (replaced by data_stream_node)

### Protobuf Messages

Defined in `testing/protobuf_structure/`:

- **`HMI_RX_CONTROLS.proto`** (Intel → HMI via port 5001):
  - `Navigation`: current_lat, current_lon, heading_deg, waypoints[], safety_states
  - `Waypoint`: lat, lon

- **`SAFETY_STATUS.proto`** (Intel → HMI via port 5003):
  - `SafetyStatus`: state (int32), description (string)

- **`HMI_TX_CONTROLS.proto`** (HMI → Intel via port 6001):
  - `HMITxMessage`: engage_status (0=DISENGAGE, 1=ENGAGE, 2=DISABLED), target_destination (A-Z)

- **`CAMERA.proto`** (Intel → HMI via port 5001):
  - `CameraBatch`: frames[], timestamp
  - `CameraFrame`: camera_id, jpeg_data

### Safety FSM States

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

**Standard framing (ports 5003, 6001):**
```
[4 bytes: big-endian uint32 length][N bytes: protobuf payload]
```

**Type-prefixed framing (port 5001):**
```
[4 bytes: big-endian uint32 length][1 byte: message type][N bytes: protobuf payload]
```

## File Inventory

### Main Nodes
| File | Description |
|------|-------------|
| `safety_comms_node.py` | Bidirectional safety/commands node (ports 5003, 6001) |
| `data_stream_node.py` | Data streaming node for Nav + Video (port 5001) |

### Legacy Nodes (Deprecated)
| File | Description |
|------|-------------|
| `waipoint_node.py` | Original combined node - use safety_comms_node + data_stream_node instead |
| `video_node.py` | Original video node - use data_stream_node instead |

### Test Scripts
| File | Port | Description |
|------|------|-------------|
| `intel2hmi_test.py` | 5001 | Simulates HMI receiving Nav + CameraBatch (type-prefixed) |
| `safety_test.py` | 5003 | Simulates HMI receiving SafetyStatus |
| `hmi2intel_test.py` | 6001 | Simulates HMI sending commands (connects as client) |
| `video_test.py` | 8554 | **Deprecated** - legacy video test for old video_node.py |

### Protobuf Files
| File | Location |
|------|----------|
| `HMI_RX_CONTROLS.proto` | `testing/protobuf_structure/` |
| `HMI_TX_CONTROLS.proto` | `testing/protobuf_structure/` |
| `CAMERA.proto` | `testing/protobuf_structure/` |
| `SAFETY_STATUS.proto` | `testing/protobuf_structure/` |

### Generated Python Protobuf
| File | Location |
|------|----------|
| `HMI_RX_CONTROLS_pb2.py` | Root directory |
| `HMI_TX_CONTROLS_pb2.py` | Root directory |
| `CAMERA_pb2.py` | Root directory |
| `SAFETY_STATUS_pb2.py` | Root directory |

### Documentation
| File | Description |
|------|-------------|
| `CLAUDE.md` | This file - project guidance for Claude Code |
| `HMI_CHANGES_REQUIRED.md` | Documentation of changes needed on HMI side |

## Running the Nodes

```bash
# Run safety communications node
ros2 run <package_name> safety_comms_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=5003 \
    -p hmi_rx_port:=6001

# Run data stream node
ros2 run <package_name> data_stream_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=5001
```

## Testing

### Test Scripts

Test scripts simulate HMI endpoints for development testing:

```bash
# Terminal 1: Receive Navigation + Video data (port 5001)
python intel2hmi_test.py

# Terminal 2: Receive SafetyStatus (port 5003)
python safety_test.py

# Terminal 3: Send HMI commands (port 6001)
# Note: Requires safety_comms_node to be running first
python hmi2intel_test.py
```

### Full Integration Test

1. **Start test receivers (simulate HMI):**
   ```bash
   python intel2hmi_test.py  # Terminal 1
   python safety_test.py     # Terminal 2
   ```

2. **Start Intel nodes:**
   ```bash
   python data_stream_node.py    # Terminal 3
   python safety_comms_node.py   # Terminal 4
   ```

3. **Send commands from HMI:**
   ```bash
   python hmi2intel_test.py  # Terminal 5
   ```

## Regenerating Protobuf Files

```bash
cd testing/protobuf_structure
protoc --python_out=../.. HMI_RX_CONTROLS.proto
protoc --python_out=../.. HMI_TX_CONTROLS.proto
protoc --python_out=../.. CAMERA.proto
protoc --python_out=../.. SAFETY_STATUS.proto
```

## Key Dependencies

- ROS 2 (rclpy)
- `novatel_gps_msgs` - NovAtel GPS message types
- `cv_bridge` - ROS/OpenCV image conversion
- `protobuf` - Protocol Buffers
- `opencv-python` (cv2) - Image processing
- `numpy` - Array processing
- `transitions` - State machine library (safety.py)
- `cantools`, `python-can` - CAN bus communication (safety.py)

## References

- `references/YR5-HMI/` - HMI codebase (Qt/QML) that receives data from Intel
- `references/autodrive_integration/` - Safety FSM code with state definitions
