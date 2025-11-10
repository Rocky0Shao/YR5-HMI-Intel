# Goals

## 1. Send Data to HMI Backend

### 1.1 Subscribe to ROS Topic from Controls Subteam
- Topic provides **waypoints defining the vehicle’s planned path**.  
- Waypoints are **vehicle-relative coordinates**.

### 1.2 Subscribe to ROS Topic from Sensors Subteam
- Topic provides **current global vehicle position**.

### 1.3 Convert Vehicle-Relative Path to Global Coordinates
- Combine **vehicle-relative waypoints** with **current global position**.  
- Output: **global-relative path waypoints** (primary_output).

### 1.4 Encode Output to Protobuf
- Convert `primary_output` into a **protobuf message**.

### 1.5 Send to HMI Backend via WebSocket
- Transmit protobuf message to backend through **WebSocket**.  
- Enables **visualization of vehicle path tracking** in HMI.

---

## 2. Receive Data from HMI Backend

### 2.1 Receive Vital Data
- Receive **protobuf message** via **WebSocket**.  
- Contains **E-stop (emergency stop)** command from HMI.

### 2.2 Publish ROS Topic to Controls Subteam
- Publish the received **E-stop message** to the Controls team’s ROS topic.  
- Allows vehicle to **stop immediately** when triggered by HMI.


# Daily Log — 11/07

## Summary
Acquired ROS bag from Controls team. Initially assumed a second ROS bag from Sensors was needed to access GPS data. After reviewing ROS topics in the Controls ROS bag, confirmed GPS data was already included. Received a tip to use `/inspvax` for lower-latency GPS information.

## Progress
- Installed `gps_msgs` type in ROS to correctly read the ROS bag.  
- Working on decoding ROS bag data.



# Daily Log — 11/09

## Summary
Continued work on decoding the ROS bag from the Controls team to extract waypoints for path planning.  
Initially thought manual conversion from car-relative to global coordinates (accounting for heading) was needed.  
After discussing with the Controls team lead, confirmed that **global coordinate waypoints are already included** in the ROS bag.

The global coordinates are only a few centimeters apart, resulting in excessive waypoint density. To reduce redundancy, a downsampling pipeline was implemented to include only points that exceed a specified distance threshold from the previous waypoint. This ensures an optimized, evenly spaced set of waypoints for transmission and processing.

## Progress
- Decoded global waypoint data from 1D array into list format: `[[x1, y1], [x2, y2], ...]`
- Downsample pipeline implemented
- Designed output data structure for sending coordinates to a **protobuf message**
- Researched how to **transmit data via WebSocket** to the HMI Jetson processor
