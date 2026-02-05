# ROS 2 Node Interface Documentation

This document describes the inputs, outputs, and communication interfaces for the Intel-side ROS 2 nodes that communicate with the HMI (Jetson).

---

## Port Summary

### INPUT Ports (Intel → HMI)

```
┌──────┬────────────────────┬─────────────────────────────────────────────────┐
│ Port │     Protocol       │                    Purpose                      │
├──────┼────────────────────┼─────────────────────────────────────────────────┤
│ 5001 │ TCP/Protobuf       │ Navigation + CameraBatch (type-prefixed)        │
│      │ (type-prefixed)    │   0x01 = Navigation (GPS, heading, waypoints)   │
│      │                    │   0x02 = CameraBatch (JPEG camera frames)       │
├──────┼────────────────────┼─────────────────────────────────────────────────┤
│ 5002 │ TCP/Protobuf       │ Perception/CAN data (stubbed, not active)       │
├──────┼────────────────────┼─────────────────────────────────────────────────┤
│ 5003 │ TCP/Protobuf       │ SafetyStatus (FSM state + description)          │
└──────┴────────────────────┴─────────────────────────────────────────────────┘
```

### OUTPUT Ports (HMI → Intel)

```
┌──────┬──────────────┬───────────────────────────────────────────┐
│ Port │   Protocol   │                  Purpose                  │
├──────┼──────────────┼───────────────────────────────────────────┤
│ 6001 │ TCP/Protobuf │ HMITxMessage (engage + destination)       │
└──────┴──────────────┴───────────────────────────────────────────┘
```

---

## safety_comms_node.py (SafetyCommsNode)

**Node Name:** `safety_comms_node`

Bidirectional communication node for safety FSM state and HMI commands.

### ROS 2 Subscriptions (Inputs)

| Topic | Message Type | QoS Depth | Description |
|-------|--------------|-----------|-------------|
| `/fsm_state` | `std_msgs/String` | 10 | FSM state number as string (e.g., "2") |
| `/fsm_description` | `std_msgs/String` | 10 | FSM state description (e.g., "close door") |

### ROS 2 Publishers (Outputs)

| Topic | Message Type | QoS Depth | Description |
|-------|--------------|-----------|-------------|
| `/safety/engage_state` | `std_msgs/Int32` | 10 | Engage state: `0`=DISENGAGE, `1`=ENGAGE, `3`=DISABLED |
| `/controls/target_destination` | `std_msgs/String` | 10 | Target destination letter (A-Z) |

### TCP Interfaces

#### TX: Intel → HMI (SafetyStatus on port 5003)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_tx_host` | `127.0.0.1` | HMI backend IP address |
| `hmi_tx_port` | `5003` | HMI SafetyStatus port |

**Protocol:** Length-prefixed Protobuf
**Send Rate:** On state change (event-driven)
**Direction:** Client (Intel connects to HMI server)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][N bytes: protobuf payload]
```

**Protobuf Message (`SafetyStatus`):**
```protobuf
message SafetyStatus {
    int32 state = 1;        // FSM state number (0-10)
    string description = 2; // Human-readable description
}
```

#### RX: HMI → Intel (HMITxMessage on port 6001)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_rx_port` | `6001` | Port to listen for HMI commands |

**Protocol:** Length-prefixed Protobuf
**Poll Rate:** 20 Hz (50ms interval)
**Direction:** Server (Intel listens, HMI connects)

**Protobuf Message (`HMITxMessage`):**
```protobuf
message HMITxMessage {
    int32 engage_status = 1;       // 0=DISENGAGE, 1=ENGAGE, 2=DISABLED
    string target_destination = 2; // "A" through "Z"
}
```

**Engage Status Mapping:**
| HMI Value | Meaning | ROS Published Value |
|-----------|---------|---------------------|
| 0 | DISENGAGE | 0 |
| 1 | ENGAGE | 1 |
| 2 | DISABLED | 3 |

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hmi_tx_host` | string | `127.0.0.1` | TCP host for SafetyStatus TX |
| `hmi_tx_port` | int | `5003` | TCP port for SafetyStatus TX |
| `hmi_rx_port` | int | `6001` | TCP port for Command RX (server) |

### Example Launch

```bash
ros2 run <package> safety_comms_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=5003 \
    -p hmi_rx_port:=6001
```

### Data Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │       safety_comms_node.py              │
                    │         (safety_comms_node)             │
                    │                                         │
    /fsm_state      │  ┌──────────┐                           │
    (String)        ├─►│ cb_fsm_  │                           │
                    │  │ state    │     ┌──────────────┐      │  TCP:5003
                    │  └──────────┘     │ SafetyStatus │      │  SafetyStatus
                    │                   │ (on change)  ├──────┼─────────────►
 /fsm_description   │  ┌──────────┐     │              │      │     HMI
    (String)        ├─►│ cb_fsm_  │     └──────────────┘      │
                    │  │ descr    │                           │
                    │  └──────────┘                           │
                    │                                         │
                    │                   ┌──────────────┐      │  TCP:6001
                    │                   │ rx_step()    │◄─────┼─────────────
                    │                   │ @ 20 Hz      │      │  HMITxMessage
                    │                   │ (server)     │      │     HMI
                    │                   └──────┬───────┘      │
                    │                          │              │
                    │     ┌────────────────────┼──────────┐   │
                    │     │                    │          │   │
                    │     ▼                    ▼          │   │
                    │ /safety/         /controls/         │   │
                    │ engage_state     target_destination │   │
                    │ (Int32)          (String)           │   │
                    └─────────────────────────────────────────┘
```

---

## data_stream_node.py (DataStreamNode)

**Node Name:** `data_stream_node`

TX-only node for streaming navigation and video data to HMI on a single port.

### ROS 2 Subscriptions (Inputs)

| Topic | Message Type | QoS | Description |
|-------|--------------|-----|-------------|
| `/raw_points_remain` | `std_msgs/Float64MultiArray` | depth=10 | Waypoints `[lat1, lon1, lat2, lon2, ...]` |
| `/inspvax` | `novatel_gps_msgs/Inspvax` | depth=10 | GPS position and heading |
| `/blackfly_0/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Camera 0 |
| `/blackfly_1/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Camera 1 |
| `/blackfly_2/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Camera 2 |

### ROS 2 Publishers (Outputs)

*This node has no ROS publishers.*

### TCP Interface

#### TX: Intel → HMI (Navigation + CameraBatch on port 5001)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_tx_host` | `127.0.0.1` | HMI backend IP address |
| `hmi_tx_port` | `5001` | HMI data stream port |

**Protocol:** Type-prefixed Protobuf
**Direction:** Client (Intel connects to HMI server)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][1 byte: message type][N bytes: protobuf payload]
```
Note: The length field includes the type byte (length = 1 + payload_size).

**Message Types:**
| Type ID | Message | Send Rate |
|---------|---------|-----------|
| `0x01` | Navigation | 5 Hz |
| `0x02` | CameraBatch | 24 Hz |

**Protobuf Message (`Navigation`):**
```protobuf
message Navigation {
    float current_lat  = 1;   // Current GPS latitude (degrees)
    float current_lon  = 2;   // Current GPS longitude (degrees)
    float heading_deg  = 3;   // Vehicle heading/azimuth (degrees)
    repeated Waypoint waypoints = 4;  // Downsampled remaining waypoints
    int32 safety_states = 5;  // (deprecated, use SafetyStatus on port 5003)
}

message Waypoint {
    float lat = 1;
    float lon = 2;
}
```

**Protobuf Message (`CameraBatch`):**
```protobuf
message CameraBatch {
    repeated CameraFrame frames = 1;  // List of camera frames
    int64 timestamp = 2;              // Unix timestamp in milliseconds
}

message CameraFrame {
    string camera_id = 1;  // "cam0", "cam1", or "cam2"
    bytes jpeg_data = 2;   // JPEG-compressed image data
}
```

**Image Processing:**
- Images resized to **400x300** pixels
- JPEG compression quality: **50**
- Frame order in batch: `["cam0", "cam1", "cam2"]`

### Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hmi_tx_host` | string | `127.0.0.1` | TCP host for data stream |
| `hmi_tx_port` | int | `5001` | TCP port for data stream |

### Example Launch

```bash
ros2 run <package> data_stream_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=5001
```

### Data Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │        data_stream_node.py              │
                    │          (data_stream_node)             │
                    │                                         │
 /raw_points_remain │  ┌──────────┐     ┌──────────────┐      │
 (Float64MultiArray)├─►│ cb_raw_  │     │ Navigation   │      │
                    │  │ points   │     │ @ 5 Hz       │      │
        /inspvax    │  └──────────┘     │ type=0x01    │      │
       (Inspvax)    ├─►┌──────────┐     └──────┬───────┘      │
                    │  │ cb_      │            │              │
                    │  │ inspvax  │            │              │
                    │  └──────────┘            │              │  TCP:5001
                    │                          ├──────────────┼───────────►
/blackfly_0/image   │  ┌──────────┐            │              │    HMI
   (Image)          ├─►│ cb_cam_0 │     ┌──────┴───────┐      │
                    │  └──────────┘     │ CameraBatch  │      │
/blackfly_1/image   │  ┌──────────┐     │ @ 24 Hz      │      │
   (Image)          ├─►│ cb_cam_1 ├────►│ type=0x02    │      │
                    │  └──────────┘     │              │      │
/blackfly_2/image   │  ┌──────────┐     │ - resize     │      │
   (Image)          ├─►│ cb_cam_2 │     │ - JPEG q=50  │      │
                    │  └──────────┘     └──────────────┘      │
                    │                                         │
                    └─────────────────────────────────────────┘
```

---

## Test Scripts

Test scripts simulate HMI endpoints for development and debugging:

| Script | Port | Role | Description |
|--------|------|------|-------------|
| `intel2hmi_test.py` | 5001 | Server | Receives Navigation + CameraBatch (type-prefixed), displays camera feeds |
| `safety_test.py` | 5003 | Server | Receives SafetyStatus, displays FSM state and description |
| `hmi2intel_test.py` | 6001 | Client | Sends HMITxMessage commands interactively |
| `video_test.py` | 8554 | Server | **Deprecated** - for legacy video_node.py only |

### Quick Test (without ROS)

```bash
# Terminal 1: Start data stream receiver
cd /home/rocky/autodrive/YR5-HMI-Intel/Path_and_Safety
python intel2hmi_test.py

# Terminal 2: Start safety status receiver
python safety_test.py

# Terminal 3: Run data stream node
python data_stream_node.py

# Terminal 4: Run safety comms node
python safety_comms_node.py

# Terminal 5: Send HMI commands
python hmi2intel_test.py
```

### Test Script Details

**intel2hmi_test.py** (port 5001):
- Listens for connections from `data_stream_node.py`
- Parses type-prefixed messages (0x01=Navigation, 0x02=CameraBatch)
- Displays camera frames in OpenCV windows
- Shows bandwidth statistics
- Press 'q' in camera window to quit

**safety_test.py** (port 5003):
- Listens for connections from `safety_comms_node.py`
- Displays FSM state number and name
- Displays state description

**hmi2intel_test.py** (port 6001):
- Connects as client to `safety_comms_node.py`
- Interactive CLI for sending engage/disengage commands
- Supports destination selection (A-Z)

---

## Legacy Nodes (Deprecated)

The following nodes have been replaced by the new architecture:

- **`waipoint_node.py`** → Replaced by `safety_comms_node.py` + `data_stream_node.py`
- **`video_node.py`** → Replaced by `data_stream_node.py`

---

## HMI Integration Checklist

### For HMI Backend (Receiver Side)

1. **Data Stream (port 5001) - NEW FORMAT**
   - Listen for TCP connections from Intel
   - Read 4-byte length prefix (big-endian)
   - Read 1-byte message type
   - Read remaining payload bytes
   - Deserialize based on type:
     - `0x01` → `Navigation` protobuf
     - `0x02` → `CameraBatch` protobuf
   - Handle reconnection (node will retry on connection loss)

2. **SafetyStatus (port 5003) - NEW**
   - Listen for TCP connections from Intel
   - Read 4-byte length prefix (big-endian)
   - Read payload and deserialize as `SafetyStatus` protobuf
   - Display state and description on HMI

3. **Control Commands (port 6001)**
   - Connect to Intel as client
   - Send length-prefixed `HMITxMessage` protobuf
   - Intel polls at 20 Hz

### For ROS 2 Integration (Publisher Side)

1. **Waypoints Publisher**
   - Publish to `/raw_points_remain` as `Float64MultiArray`
   - Data format: `[lat1, lon1, lat2, lon2, ...]`
   - Points will be downsampled to 1m spacing

2. **GPS Publisher**
   - Publish to `/inspvax` using `novatel_gps_msgs/Inspvax`
   - Required fields: `latitude`, `longitude`, `azimuth`

3. **Camera Publishers**
   - Publish to `/blackfly_0/image_raw`, `/blackfly_1/image_raw`, `/blackfly_2/image_raw`
   - Use `sensor_msgs/Image` with QoS BEST_EFFORT

4. **Safety FSM Publishers**
   - Publish state to `/fsm_state` as `String` (e.g., "2")
   - Publish description to `/fsm_description` as `String` (e.g., "close door")

5. **Engage State Subscriber**
   - Subscribe to `/safety/engage_state` (`Int32`)
   - Values: 0=disengage, 1=engage, 3=disabled

6. **Destination Subscriber**
   - Subscribe to `/controls/target_destination` (`String`)
   - Values: single letter A-Z

---

## TCP Message Parsing Examples (Python)

### Standard Length-Prefixed (ports 5003, 6001)

```python
import struct

def recv_protobuf_message(sock, proto_class):
    """Receive a length-prefixed protobuf message."""
    header = recv_exactly(sock, 4)
    msg_len = struct.unpack(">I", header)[0]
    payload = recv_exactly(sock, msg_len)
    msg = proto_class()
    msg.ParseFromString(payload)
    return msg

def send_protobuf_message(sock, msg):
    """Send a length-prefixed protobuf message."""
    payload = msg.SerializeToString()
    header = struct.pack(">I", len(payload))
    sock.sendall(header + payload)
```

### Type-Prefixed (port 5001)

```python
import struct

MSG_TYPE_NAVIGATION = 0x01
MSG_TYPE_CAMERA = 0x02

def recv_typed_message(sock, nav_class, camera_class):
    """Receive a type-prefixed protobuf message."""
    header = recv_exactly(sock, 4)
    total_len = struct.unpack(">I", header)[0]

    msg_type = struct.unpack("B", recv_exactly(sock, 1))[0]
    payload = recv_exactly(sock, total_len - 1)

    if msg_type == MSG_TYPE_NAVIGATION:
        msg = nav_class()
    elif msg_type == MSG_TYPE_CAMERA:
        msg = camera_class()
    else:
        raise ValueError(f"Unknown message type: {msg_type}")

    msg.ParseFromString(payload)
    return msg_type, msg

def recv_exactly(sock, n):
    """Read exactly n bytes from socket."""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed")
        data += chunk
    return data
```
