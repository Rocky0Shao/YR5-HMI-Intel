#!/usr/bin/env python3
"""
hmi2intel_test.py - Test sender for safety_comms_node.py

Simulates HMI sending HMITxMessage commands to Intel on port 6001.
Connects as client to Intel server.

Wire format: [4-byte length][payload]
"""
import socket
import struct
from typing import Optional

import HMI_TX_CONTROLS_pb2 as hmi_tx


def build_frame(engage_status: int, destination: str) -> bytes:
    """Serialize HMITxMessage and prefix with length."""
    msg = hmi_tx.HMITxMessage()
    msg.engage_status = int(engage_status)
    if destination:
        msg.target_destination = destination
    payload = msg.SerializeToString()
    return struct.pack(">I", len(payload)) + payload


def prompt_int(prompt: str, valid: set) -> Optional[int]:
    try:
        val = int(input(prompt).strip())
    except ValueError:
        print("Please enter a number.")
        return None
    if val not in valid:
        print(f"Enter one of: {sorted(valid)}")
        return None
    return val


def prompt_destination(prompt: str) -> Optional[str]:
    val = input(prompt).strip()
    if val == "":
        return ""  # keep current
    if len(val) != 1 or not val.isalpha():
        print("Enter a single letter (a-z) or leave blank to keep current.")
        return None
    return val.upper()


def interactive_session(sock: socket.socket) -> None:
    """Run interactive command session."""
    print("\nCommands:")
    print("  0 = DISENGAGE, 1 = ENGAGE, 2 = DISABLED")
    print("  Destination: single letter a-z (blank keeps previous)")
    print("  'q' to quit\n")

    current_dest = ""
    current_status = 0

    try:
        while True:
            raw = input("\nPress Enter to send command, or 'q' to quit: ").strip()
            if raw.lower() == "q":
                print("[quitting]")
                break

            status = prompt_int("Engage status (0/1/2): ", {0, 1, 2})
            if status is None:
                continue
            dest = prompt_destination("Destination (a-z, blank=keep): ")
            if dest is None:
                continue

            current_status = status
            if dest != "":
                current_dest = dest

            frame = build_frame(current_status, current_dest)
            try:
                sock.sendall(frame)
                dest_label = current_dest or "(empty)"
                print(f"Sent: engage_status={current_status}, destination='{dest_label}'")
            except Exception as e:
                print(f"[send failed] {e}")
                break

    except KeyboardInterrupt:
        print("\n[interrupted]")


def main() -> None:
    host = '192.168.69.10'  # Intel's address
    port = 6001

    print(f"=== HMI→Intel Test Sender (port {port}) ===")
    print(f"Connecting to Intel at {host}:{port}...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(5.0)
        sock.connect((host, port))
        sock.settimeout(None)
        print(f"[connected] to {host}:{port}")

        interactive_session(sock)

    except ConnectionRefusedError:
        print(f"[error] Connection refused. Is safety_comms_node running on {host}:{port}?")
    except socket.timeout:
        print(f"[error] Connection timed out. Is safety_comms_node running on {host}:{port}?")
    except Exception as e:
        print(f"[error] {e}")
    finally:
        try:
            sock.shutdown(socket.SHUT_WR)
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass
        print("[connection closed]")


if __name__ == "__main__":
    main()
