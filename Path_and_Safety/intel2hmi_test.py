#!/usr/bin/env python3
"""
intel2hmi_test.py - Test receiver for Intel -> HMI messages

Simulates HMI backend receiving all Intel transmissions:
  Port 5001 (data_stream_node):
    - 0x01 = Navigation
    - 0x02 = CameraBatch
    Wire format: [4-byte length][1-byte type][payload]

  Port 5003 (safety_comms_node):
    - SafetyStatus (state, description)
    Wire format: [4-byte length][payload]
"""
import socket
import struct
import time
import threading
from typing import Optional

import cv2
import numpy as np

import HMI_RX_CONTROLS_pb2 as hmi
import CAMERA_pb2
import SAFETY_STATUS_pb2 as safety_pb

# Message type constants
MSG_TYPE_NAVIGATION = 0x01
MSG_TYPE_CAMERA = 0x02


def recv_exact(conn: socket.socket, n: int) -> Optional[bytes]:
    """Read exactly n bytes from socket."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def handle_client(conn: socket.socket, addr):
    print(f"[connected] {addr[0]}:{addr[1]}")

    # Bandwidth tracking
    byte_counter = 0
    last_time = time.time()
    update_interval = 1.0

    # Message counters
    nav_count = 0
    camera_count = 0

    try:
        while True:
            # Read 4-byte big-endian length prefix
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("\n[disconnect]")
                return
            (total_length,) = struct.unpack(">I", hdr)
            if total_length == 0:
                continue

            # Read 1-byte message type
            type_byte = recv_exact(conn, 1)
            if type_byte is None:
                print("\n[disconnect mid-frame]")
                return
            msg_type = struct.unpack("B", type_byte)[0]

            # Read payload (total_length - 1 for the type byte)
            payload_len = total_length - 1
            payload = recv_exact(conn, payload_len)
            if payload is None:
                print("\n[disconnect mid-payload]")
                return

            # Bandwidth tracking
            byte_counter += (4 + total_length)
            current_time = time.time()
            elapsed = current_time - last_time

            if elapsed >= update_interval:
                bytes_per_sec = byte_counter / elapsed
                mb_per_sec = bytes_per_sec / (1024 * 1024)
                mbps = (bytes_per_sec * 8) / (1000 * 1000)
                print(f"\r[ {mb_per_sec:.2f} MB/s | {mbps:.2f} Mbps | Nav: {nav_count} | Cam: {camera_count} ]", end="")
                byte_counter = 0
                last_time = current_time

            # Parse based on message type
            if msg_type == MSG_TYPE_NAVIGATION:
                nav_count += 1
                nav = hmi.Navigation()
                nav.ParseFromString(payload)

                # Print navigation info (less verbose)
                print(f"\n=== Navigation #{nav_count} ===")
                print(f"  lat: {nav.current_lat:.8f}, lon: {nav.current_lon:.8f}")
                print(f"  heading: {nav.heading_deg:.2f}°, waypoints: {len(nav.waypoints)}")

            elif msg_type == MSG_TYPE_CAMERA:
                camera_count += 1
                batch = CAMERA_pb2.CameraBatch()
                batch.ParseFromString(payload)

                # Display camera frames
                for frame in batch.frames:
                    np_arr = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        cv2.imshow(f"Stream: {frame.camera_id}", img)

                # Handle UI events (press 'q' to quit)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n[quit requested]")
                    return
            else:
                print(f"\n[unknown message type: 0x{msg_type:02X}]")

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        cv2.destroyAllWindows()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        conn.close()


def handle_safety_client(conn: socket.socket, addr):
    """Handle SafetyStatus messages from safety_comms_node (port 5003)."""
    print(f"[safety connected] {addr[0]}:{addr[1]}")
    msg_count = 0

    try:
        while True:
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("\n[safety disconnect]")
                return
            (msg_len,) = struct.unpack(">I", hdr)
            if msg_len == 0:
                continue

            payload = recv_exact(conn, msg_len)
            if payload is None:
                print("\n[safety disconnect mid-payload]")
                return

            msg_count += 1
            try:
                status = safety_pb.SafetyStatus()
                status.ParseFromString(payload)
                print(f"\n=== SafetyStatus #{msg_count} ===")
                print(f"  state:       {status.state}")
                print(f"  description: \"{status.description}\"")
            except Exception as e:
                print(f"\n[safety parse error] {e}")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        conn.close()


def run_safety_listener():
    """Listen for SafetyStatus on port 5003 in a background thread."""
    host = "0.0.0.0"
    port = 5003
    print(f"[safety listening] {host}:{port}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(5)
        while True:
            try:
                conn, addr = s.accept()
                handle_safety_client(conn, addr)
            except (KeyboardInterrupt, OSError):
                break


def main():
    host = "0.0.0.0"
    print("=== Intel→HMI Test Receiver ===")
    print("  Port 5001: Navigation + CameraBatch (data_stream_node)")
    print("  Port 5003: SafetyStatus (safety_comms_node)")
    print("Press 'q' in camera window or Ctrl+C to quit\n")

    # Start safety listener in background thread
    safety_thread = threading.Thread(target=run_safety_listener, daemon=True)
    safety_thread.start()

    # Data stream listener on main thread
    print(f"[data listening] {host}:5001")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, 5001))
        s.listen(5)
        while True:
            try:
                conn, addr = s.accept()
                handle_client(conn, addr)
            except KeyboardInterrupt:
                print("\n[shutdown]")
                break


if __name__ == "__main__":
    main()
