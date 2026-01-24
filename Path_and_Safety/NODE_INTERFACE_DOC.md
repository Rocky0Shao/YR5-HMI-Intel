# ROS 2 Node Interface Documentation

This document describes the inputs, outputs, and communication interfaces for the `waipoint_node.py` and `video_node.py` ROS 2 nodes.

---

## waipoint_node.py (WaypointSubscriber)

**Node Name:** `waypoint_subscriber`

This node bridges ROS 2 and the HMI backend. It subscribes to navigation data from the vehicle systems, transmits it to the HMI over TCP, and receives control commands from the HMI which it publishes to ROS topics.

### ROS 2 Subscriptions (Inputs)

| Topic | Message Type | QoS Depth | Description |
|-------|--------------|-----------|-------------|
| `/raw_points_remain` | `std_msgs/Float64MultiArray` | 10 | Waypoints from Controls. Flat array `[lat1, lon1, lat2, lon2, ...]`. Downsampled to 1-meter spacing before transmission. |
| `/inspvax` | `novatel_gps_msgs/Inspvax` | 10 | NovAtel GPS position. Uses `latitude`, `longitude`, and `azimuth` fields (degrees). |

### ROS 2 Publishers (Outputs)

| Topic | Message Type | QoS Depth | Description |
|-------|--------------|-----------|-------------|
| `/safety/engage_state` | `std_msgs/Int32` | 10 | Engage state for the Safety node. Values: `0`=DISENGAGE, `1`=ENGAGE, `3`=DISABLED |
| `/controls/target_destination` | `std_msgs/String` | 10 | Target destination letter (A-Z) for Controls. Only published when non-empty. |

### TCP Interfaces

#### TX: Intel → HMI (Navigation Data)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_tx_host` | `127.0.0.1` | HMI backend IP address |
| `hmi_tx_port` | `65432` | HMI backend port |

**Protocol:** Length-prefixed Protobuf
**Send Rate:** 5 Hz (200ms interval)
**Direction:** Client (this node connects to HMI server)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][N bytes: protobuf payload]
```

**Protobuf Message (`Navigation`):**
```protobuf
message Navigation {
    float current_lat  = 1;   // Current GPS latitude (degrees)
    float current_lon  = 2;   // Current GPS longitude (degrees)
    float heading_deg  = 3;   // Vehicle heading/azimuth (degrees)
    repeated Waypoint waypoints = 4;  // Downsampled remaining waypoints
    int32 safety_states = 5;  // FSM state (currently not populated)
}

message Waypoint {
    float lat = 1;
    float lon = 2;
}
```

**Notes:**
- Waypoints are downsampled: only points >= 1 meter apart are included
- Non-finite (NaN/Inf) values are filtered out
- Empty messages (0 bytes) are not sent

#### RX: HMI → Intel (Control Commands)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_rx_host` | `127.0.0.2` | HMI command server IP |
| `hmi_rx_port` | `65431` | HMI command server port |

**Protocol:** Length-prefixed Protobuf
**Poll Rate:** 20 Hz (50ms interval)
**Direction:** Client (this node connects to HMI server)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][N bytes: protobuf payload]
```

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
| `hmi_tx_host` | string | `127.0.0.1` | TCP host for Navigation TX |
| `hmi_tx_port` | int | `65432` | TCP port for Navigation TX |
| `hmi_rx_host` | string | `127.0.0.2` | TCP host for Command RX |
| `hmi_rx_port` | int | `65431` | TCP port for Command RX |

### Example Launch

```bash
ros2 run <package> waipoint_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=65432 \
    -p hmi_rx_host:=192.168.1.100 \
    -p hmi_rx_port:=65431
```

### Data Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │         waipoint_node.py                │
                    │         (waypoint_subscriber)           │
                    │                                         │
 /raw_points_remain │  ┌──────────┐                           │
 (Float64MultiArray)├─►│ cb_raw_  │                           │
                    │  │ points_  │                           │
                    │  │ remain   │     ┌──────────────┐      │  TCP:65432
                    │  └──────────┘     │ build_nav_   │      │  Navigation
                    │                   │ proto()      ├──────┼─────────────►
        /inspvax    │  ┌──────────┐     │              │      │     HMI
       (Inspvax)    ├─►│ cb_      │     │ tx_nav()     │      │
                    │  │ inspvax  │     │ @ 5 Hz       │      │
                    │  └──────────┘     └──────────────┘      │
                    │                                         │
                    │                   ┌──────────────┐      │  TCP:65431
                    │                   │ rx_step()    │◄─────┼─────────────
                    │                   │ @ 20 Hz      │      │  HMITxMessage
                    │                   │              │      │     HMI
                    │                   │ _handle_hmi_ │      │
                    │                   │ command()    │      │
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

## video_node.py (MultiCameraNode)

**Node Name:** `multi_camera_subscriber`

This node subscribes to camera image topics, compresses frames to JPEG, batches them into a Protobuf message, and streams them to the HMI over TCP.

### ROS 2 Subscriptions (Inputs)

| Topic | Message Type | QoS | Description |
|-------|--------------|-----|-------------|
| `/blackfly_0/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Front/primary camera raw image |
| `/blackfly_1/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Secondary camera raw image |
| `/blackfly_2/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth=10 | Tertiary camera raw image |

**Expected Image Encoding:** Any encoding convertible to `bgr8` via cv_bridge

### ROS 2 Publishers (Outputs)

*This node has no ROS publishers.*

### TCP Interface

#### TX: Intel → HMI (Camera Stream)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hmi_tx_host` | `127.0.0.1` | HMI backend IP address |
| `hmi_tx_port` | `65433` | HMI video stream port |

**Protocol:** Length-prefixed Protobuf
**Send Rate:** 24 Hz (~41.7ms interval)
**Direction:** Client (this node connects to HMI server)

**Wire Format:**
```
[4 bytes: big-endian uint32 length][N bytes: protobuf payload]
```

**Protobuf Message (`CameraBatch`):**
```protobuf
package camera_msgs;

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
| `hmi_tx_host` | string | `127.0.0.1` | TCP host for video stream |
| `hmi_tx_port` | int | `65433` | TCP port for video stream |

### Example Launch

```bash
ros2 run <package> video_node --ros-args \
    -p hmi_tx_host:=192.168.1.100 \
    -p hmi_tx_port:=65433
```

### Data Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │          video_node.py                  │
                    │       (multi_camera_subscriber)         │
                    │                                         │
/blackfly_0/image_raw│  ┌──────────┐     ┌──────────────┐     │
   (Image)          ├─►│callback_ │     │              │     │
                    │  │cam_0     ├────►│ frames dict  │     │
                    │  └──────────┘     │              │     │
/blackfly_1/image_raw│  ┌──────────┐     │ cam0: [img]  │     │
   (Image)          ├─►│callback_ │     │ cam1: [img]  │     │
                    │  │cam_1     ├────►│ cam2: [img]  │     │
                    │  └──────────┘     │              │     │
/blackfly_2/image_raw│  ┌──────────┐     └──────┬───────┘     │
   (Image)          ├─►│callback_ │            │             │
                    │  │cam_2     ├────────────┘             │
                    │  └──────────┘                          │
                    │                                         │
                    │              ┌──────────────────┐       │  TCP:65433
                    │              │ tx_video()       │       │  CameraBatch
                    │              │ @ 24 Hz          ├───────┼───────────►
                    │              │                  │       │    HMI
                    │              │ - resize 400x300 │       │
                    │              │ - JPEG q=50      │       │
                    │              │ - Protobuf batch │       │
                    │              └──────────────────┘       │
                    │                                         │
                    └─────────────────────────────────────────┘
```

---

## Integration Checklist

### For HMI Backend (Receiver Side)

1. **Navigation Data (port 65432)**
   - Listen for TCP connections
   - Read 4-byte length prefix (big-endian)
   - Read payload bytes and deserialize as `Navigation` protobuf
   - Handle reconnection (node will retry on connection loss)

2. **Control Commands (port 65431)**
   - Listen for TCP connections
   - Send length-prefixed `HMITxMessage` protobuf
   - Expect node to poll at 20 Hz

3. **Video Stream (port 65433)**
   - Listen for TCP connections
   - Read 4-byte length prefix (big-endian)
   - Deserialize as `CameraBatch` protobuf
   - Decode JPEG bytes for each `CameraFrame`

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

4. **Engage State Subscriber**
   - Subscribe to `/safety/engage_state` (`Int32`)
   - Values: 0=disengage, 1=engage, 3=disabled

5. **Destination Subscriber**
   - Subscribe to `/controls/target_destination` (`String`)
   - Values: single letter A-Z

---

## TCP Message Parsing Example (Python)

```python
import struct
import socket

def recv_protobuf_message(sock, proto_class):
    """Receive a length-prefixed protobuf message."""
    # Read 4-byte length header
    header = b''
    while len(header) < 4:
        chunk = sock.recv(4 - len(header))
        if not chunk:
            raise ConnectionError("Connection closed")
        header += chunk

    msg_len = struct.unpack(">I", header)[0]

    # Read payload
    payload = b''
    while len(payload) < msg_len:
        chunk = sock.recv(msg_len - len(payload))
        if not chunk:
            raise ConnectionError("Connection closed")
        payload += chunk

    # Deserialize
    msg = proto_class()
    msg.ParseFromString(payload)
    return msg

def send_protobuf_message(sock, msg):
    """Send a length-prefixed protobuf message."""
    payload = msg.SerializeToString()
    header = struct.pack(">I", len(payload))
    sock.sendall(header + payload)
```
