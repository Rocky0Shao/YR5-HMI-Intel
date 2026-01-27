#!/usr/bin/env python3
"""
Interactive HMI TX test server.
Listens for main_node.py (client) to connect, then sends HMITxMessage protobufs over TCP with a 4-byte big-endian length prefix.
"""
import socket
import struct
from dataclasses import dataclass
from typing import Optional

import HMI_TX_CONTROLS_pb2 as hmi_tx


@dataclass
class TxConfig:
    host: str = '127.0.0.2'
    port: int = 6001  # matches waipoint_node.py default hmi_rx_port
    backlog: int = 1


def build_frame(engage_status: int, destination: str) -> bytes:
    """Serialize HMITxMessage and prefix with length."""
    msg = hmi_tx.HMITxMessage()
    msg.engage_status = int(engage_status)
    if destination:
        msg.target_destination = destination
    payload = msg.SerializeToString()  # no explicit length check for simplicity
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


def main() -> None:
    cfg = TxConfig()
    print(f"[listening] {cfg.host}:{cfg.port} (waiting for main_node.py)")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((cfg.host, cfg.port))
        server.listen(cfg.backlog)

        while True:
            try:
                conn, addr = server.accept()
            except KeyboardInterrupt:
                print("\nShutting down.")
                break

            with conn:
                print(f"[connected] {addr[0]}:{addr[1]}")
                print("Commands:")
                print("  0 = DISENGAGE, 1 = ENGAGE, 2 = DISABLED")
                print("  Destination: single letter a-z (blank keeps previous)")
                print("  Ctrl+C or 'q' to quit, Enter to send\n")

                current_dest = ""
                current_status = 0

                try:
                    while True:
                        raw = input("Enter 'q' to quit connection or press Enter to send: ").strip()
                        if raw.lower() == "q":
                            print("[disconnecting client]")
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
                            conn.sendall(frame)
                            dest_label = current_dest or "(empty)"
                            print(f"Sent engage_status={current_status}, target_destination='{dest_label}'")
                        except Exception as e:
                            print(f"[send failed] {e}")
                            break
                except KeyboardInterrupt:
                    print("\n[stop]")
                    break
                finally:
                    try:
                        conn.shutdown(socket.SHUT_WR)
                    except Exception:
                        pass
                    print("[client closed]")


if __name__ == "__main__":
    main()
