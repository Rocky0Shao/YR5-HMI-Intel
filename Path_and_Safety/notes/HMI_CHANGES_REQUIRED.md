# HMI Changes Required

This document lists the changes required on the HMI (Jetson) side to support the refactored Intel node architecture.

---

## Summary of Changes

| Change | Priority | Complexity |
|--------|----------|------------|
| Port 5001: Add message type discriminator | **High** | Medium |
| Port 5003: Add SafetyStatus receiver | **High** | Low |
| Add SAFETY_STATUS.proto | **High** | Low |
| Port 6001: Update connection role | Medium | Low |

---

## 1. Port 5001: Message Type Discriminator (HIGH PRIORITY)

### Current Behavior
- HMI listens on port 5001
- Receives length-prefixed `Navigation` protobuf only

### New Behavior
- HMI listens on port 5001
- Receives **type-prefixed** messages (Navigation OR CameraBatch)

### Wire Format Change

**Old format:**
```
[4 bytes: length][N bytes: Navigation protobuf]
```

**New format:**
```
[4 bytes: length][1 byte: type][N bytes: protobuf payload]
```

Where:
- `length` = 1 + payload_size (includes the type byte)
- `type` = `0x01` for Navigation, `0x02` for CameraBatch

### Code Changes Required

In `GlobalReceiver.cpp` (or equivalent), update the message parsing:

```cpp
// Pseudocode for new parsing logic
uint32_t total_len = readBigEndianUint32(socket);
uint8_t msg_type = readByte(socket);
std::vector<uint8_t> payload = readBytes(socket, total_len - 1);

switch (msg_type) {
    case 0x01:  // Navigation
        Navigation nav;
        nav.ParseFromArray(payload.data(), payload.size());
        emit controlsMessage(nav);
        break;
    case 0x02:  // CameraBatch
        CameraBatch batch;
        batch.ParseFromArray(payload.data(), payload.size());
        emit cameraMessage(batch);
        break;
    default:
        qWarning() << "Unknown message type:" << msg_type;
}
```

### Files to Modify
- `src/backend/GlobalReceiver.h`
- `src/backend/GlobalReceiver.cpp`
- Add signal/slot for camera messages if not already present

---

## 2. Port 5003: SafetyStatus Receiver (HIGH PRIORITY)

### New Functionality
- HMI listens on a **new port 5003**
- Receives `SafetyStatus` protobuf (state + description)
- Display safety state and description on HMI UI

### Wire Format
```
[4 bytes: big-endian length][N bytes: SafetyStatus protobuf]
```

### Protobuf Definition

Add `SAFETY_STATUS.proto`:
```protobuf
syntax = "proto3";

message SafetyStatus {
    int32 state = 1;        // FSM state number (0-10)
    string description = 2; // Human-readable description (e.g., "close door")
}
```

### FSM State Values
| Value | State | Description |
|-------|-------|-------------|
| 0 | STATE_0 | Initial state |
| 1 | STARTUP | System starting |
| 2 | PASSIVE_MODE | Waiting for engage |
| 3 | ACTIVATION_CONDITION | Checking conditions |
| 4 | BRAKE_ACTIVATION | Activating brakes |
| 5 | WAIT_BRAKE_RELEASE | Waiting for brake release |
| 6 | STEER_ACTIVATION | Activating steering |
| 7 | PROPULSION_ACTIVATION | Activating propulsion |
| 8 | AV | Autonomous mode active |
| 9 | DEACTIVATION | Deactivating |
| 10 | ACTIVATION_FAILURE | Activation failed |

### Code Changes Required

1. **Add protobuf file:**
   - Create `src/proto/SAFETY_STATUS.proto`
   - Regenerate protobuf files

2. **Add SafetyBackend class** (similar to NavigationBackend):
   ```cpp
   class SafetyBackend : public QObject {
       Q_OBJECT
       Q_PROPERTY(int state READ state NOTIFY stateChanged)
       Q_PROPERTY(QString description READ description NOTIFY descriptionChanged)
   public:
       int state() const { return m_state; }
       QString description() const { return m_description; }
   public slots:
       void onSafetyMessage(const SafetyStatus& msg);
   signals:
       void stateChanged();
       void descriptionChanged();
   private:
       int m_state = 0;
       QString m_description;
   };
   ```

3. **Update GlobalReceiver:**
   - Add TCP server on port 5003
   - Parse `SafetyStatus` messages
   - Emit signal to SafetyBackend

4. **Update QML:**
   - Display safety state and description on relevant pages

### Files to Create/Modify
- Create `src/proto/SAFETY_STATUS.proto`
- Create `src/backend/SafetyBackend.h`
- Create `src/backend/SafetyBackend.cpp`
- Modify `src/backend/GlobalReceiver.h`
- Modify `src/backend/GlobalReceiver.cpp`
- Modify `main.cpp` (register SafetyBackend)
- Modify QML files to display safety info

---

## 3. Port 6001: Connection Role Change (MEDIUM PRIORITY)

### Current Behavior
- HMI connects as **client** to Intel at port 6001

### New Behavior
- HMI connects as **client** to Intel at port 6001 (same as before)
- **Intel now listens as server** on port 6001

### Impact
- This should be **transparent** to HMI if it was already connecting as client
- Verify `GlobalTransmitter` connects to Intel's IP on port 6001

### Files to Verify
- `src/backend/GlobalTransmitter.cpp` - ensure connecting as client

---

## 4. Update TCP_PORTS.md

Update the ports reference document:

```markdown
| Port | Direction | Protocol | Purpose | Status |
|------|-----------|----------|---------|--------|
| 5001 | INPUT (RX) | TCP/Protobuf (type-prefixed) | Navigation + CameraBatch | Active |
| 5002 | INPUT (RX) | TCP/Protobuf | Perception data | Stubbed |
| 5003 | INPUT (RX) | TCP/Protobuf | SafetyStatus | **NEW** |
| 6001 | OUTPUT (TX) | TCP/Protobuf | Engage commands | Active |
```

---

## Implementation Order

1. **Phase 1: SafetyStatus (port 5003)**
   - Add protobuf definition
   - Add receiver and backend
   - Test with Intel node

2. **Phase 2: Type-prefixed messages (port 5001)**
   - Update parsing logic
   - Add camera message handling
   - Test Navigation + CameraBatch

3. **Phase 3: UI Integration**
   - Display safety state/description
   - Display camera frames (if not already)

---

## Testing

### Test SafetyStatus (port 5003)
```python
import socket
import struct
import SAFETY_STATUS_pb2

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('HMI_IP', 5003))

status = SAFETY_STATUS_pb2.SafetyStatus()
status.state = 2
status.description = "Waiting for HMI engage"
payload = status.SerializeToString()
sock.sendall(struct.pack(">I", len(payload)) + payload)
```

### Test Type-Prefixed Messages (port 5001)
```python
import socket
import struct
import HMI_RX_CONTROLS_pb2
import CAMERA_pb2

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('HMI_IP', 5001))

# Send Navigation (type 0x01)
nav = HMI_RX_CONTROLS_pb2.Navigation()
nav.current_lat = 40.0
nav.current_lon = -83.0
payload = nav.SerializeToString()
frame = struct.pack(">I", 1 + len(payload)) + struct.pack("B", 0x01) + payload
sock.sendall(frame)

# Send CameraBatch (type 0x02)
batch = CAMERA_pb2.CameraBatch()
batch.timestamp = 1234567890
payload = batch.SerializeToString()
frame = struct.pack(">I", 1 + len(payload)) + struct.pack("B", 0x02) + payload
sock.sendall(frame)
```

---

## Questions for HMI Team

1. Is there an existing mechanism for displaying camera frames, or does this need to be added?
2. Where should the safety state/description be displayed in the UI?
3. Are there any additional safety states or descriptions that need special handling?
