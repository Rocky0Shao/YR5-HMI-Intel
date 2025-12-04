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
Initially assumed manual conversion from car-relative to global coordinates (accounting for heading) was required.  
After clarification with the Controls team lead, confirmed that **global coordinate waypoints are already included** in the ROS bag.  

Because consecutive global coordinates were only a few centimeters apart, the data was overly dense. A downsampling pipeline was implemented to retain only points separated by a specified distance threshold, producing an optimized and evenly spaced set of waypoints for transmission and visualization.  

Successfully implemented and tested the downsampling pipeline, comparing filtered and unfiltered waypoints using a scatter plot for validation.  

## Visualization

A quick visualization video was created to show the CAR driving during ROS bag replay, using the downsampled waypoints.  
🎥 [Watch on YouTube](https://youtu.be/T3FgTb362sk)

## Progress
- Decoded global waypoint data from a flattened 1D array into coordinate pairs: `[(x1, y1), (x2, y2), ...]`
- Implemented distance-based downsampling logic
- Implemented Visual Feedback
- Defined output structure for **protobuf message** packaging
- Researched **WebSocket transmission** for integration with the HMI Jetson processor



# Daily Log — 11/11

## Summary
Set up the **protobuf message structure** and generated the compiled output from the defined `.proto` schema.  
Researched **TCP socket communication** and verified the custom WebSocket client through local testing.  
Experimented with establishing reliable TCP communication between modules in the system.  

Successfully implemented a **TCP sender** inside the ROS node that transmits protobuf messages.  
Built a **TCP receiver** that accepts these messages, decodes the protobuf data, and prints the results to the terminal in an organized format.

## Progress
- Defined and tested protobuf schema  
- Generated protobuf compiler output  
- Implemented WebSocket client for early communication tests  
- Researched and tested TCP socket communication  
- Added TCP sender to ROS node  
- Created and verified TCP receiver that decodes and displays protobuf messages  



# Daily Log — 11/13

## Summary
More tasks to do  
- Send Target Destination to Controls (Ros Topic String, Char for destination)  
- Send Engage/Disengage to Safety (Ros Topic boolean)

## Progress
- Set up ROS publishers  
- Set up protobuf decoder in main node  


# Daily Log — 11/16

## Summary
Need a test script (fake HMI TX request) that sends protobuf via WebSocket to the Main Node so I can verify the decoding + ROS publishing flow.

## Progress
- Planned structure for the mock sender  
- Identified message types and framing requirements  


# Daily Log — 11/20

## Summary
Added full RX support so the main node can receive `HMITxMessage` from the HMI backend over TCP and convert those commands into ROS topics.

## Progress
- Added RX socket connection and periodic polling timer  
- Implemented protobuf parsing for engage status + destination  
- Published decoded values to `/safety/engage_state` and `/controls/target_destination`


# Daily Log — 11/23

## Summary
hmi_rx is me sending to hmi
hmi_tx is me recieving from hmi

## Progress
- Made hmi2intel node with keyboard input
- display engage status & destination via terminal output


# Daily Log — 12/04

## Summary

### The Core Objective
I need to display live video feeds from our three Blackfly cameras onto the HMI’s Cameras Page in real-time.

### The Problem: 
Currently, the HMI tries to pull an RTSP stream directly. This works fine on Windows Qt Creator, but it fails completely when running inside our HMI Docker container on the vehicle hardware, even with host networking enabled.

### The Solution: 
Instead of relying on the RTSP feed, I need to build a custom pipeline. I need to create a mechanism on the Intel (compute) side that ingests the raw ROS camera data and pushes it to the HMI (Jetson/Display) via a robust protocol (TCP or WebSockets) that the Qt application can render. I will make a separate ROS node to avoid single bloated ROS node crashing 1 thread.

## Technical Requirements
**Scope:** Responsible for the entire pipeline, including backend data handling and frontend visualization.

### 1. Backend (The "Bridge" Node)
**Goal:** Implement a new instance (or update the existing RX node) on the **Intel** side.

* **Input:** Subscribe to the following ROS topics from the bag/live stream:
    * `/blackfly_0/image_raw`
    * `/blackfly_1/image_raw`
    * `/blackfly_2/image_raw`
* **Processing:** Compress frames (JPEG/H.264) to ensure bandwidth efficiency before transmission.
* **Output:** Transmit these frames to the HMI via **TCP** or **WebSockets**.

> **Note on Synchronization:** This is critical. The video feed must not lag behind the map annotations. **Low latency is the priority.**

### 2. Frontend (The HMI)
**Goal:** Update the Qt/QML code to receive and display these custom frames.

* **Target File:** `YR5-HMI/pages/CamerasPage.qml`
* **Task:** Replace the broken RTSP player logic with a client (TCP/Websocket) that accepts your incoming image stream and paints it to the UI.

### 3. Environment
* **Docker:** The HMI runs in a container. I must use the provided image which already contains `Qt multimedia`, `ffmpeg`, and `gstreamer`.
* **Hardware:** * **Production:** Jetson.
    * **Development:** PC. 
    * *Constraint:* Keep cross-platform compatibility in mind.
## Progress
- 