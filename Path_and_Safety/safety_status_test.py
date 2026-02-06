#!/usr/bin/env python3
"""
safety_status_test.py - Test receiver for safety_comms_node.py TX

Simulates HMI backend receiving SafetyStatus on port 5003.

Wire format: [4-byte big-endian length][protobuf payload]

SafetyStatus fields:
    - state (int32) - FSM state number
    - description (string) - FSM state description
"""
import socket
import struct
from typing import Optional

import SAFETY_STATUS_pb2 as safety_pb


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
    msg_count = 0

    try:
        while True:
            # Read 4-byte big-endian length prefix
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("\n[disconnect]")
                return
            (msg_len,) = struct.unpack(">I", hdr)
            if msg_len == 0:
                continue

            # Read payload
            payload = recv_exact(conn, msg_len)
            if payload is None:
                print("\n[disconnect mid-payload]")
                return

            # Parse SafetyStatus
            msg_count += 1
            try:
                status = safety_pb.SafetyStatus()
                status.ParseFromString(payload)
                print(f"\n=== SafetyStatus #{msg_count} ===")
                print(f"  state:       {status.state}")
                print(f"  description: \"{status.description}\"")
            except Exception as e:
                print(f"\n[parse error] {e}")

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        conn.close()


def main():
    host = "0.0.0.0"
    port = 5003
    print(f"=== SafetyStatus Test Receiver (port {port}) ===")
    print(f"[listening] {host}:{port}")
    print("Press Ctrl+C to quit\n")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
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
