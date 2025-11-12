#!/usr/bin/env python3
import argparse, socket, struct
from typing import Optional
from proto_out import HMI_RX_CONTROLS_pb2 as hmi

def recv_exact(conn: socket.socket, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)

def handle_client(conn: socket.socket, addr):
    print(f"[connected] {addr[0]}:{addr[1]}")
    try:
        while True:
            # Read 4-byte length prefix
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("[disconnect]")
                return
            (length,) = struct.unpack(">I", hdr)
            if length == 0:
                # Skip empty frames
                continue

            payload = recv_exact(conn, length)
            if payload is None:
                print("[disconnect mid-frame]")
                return

            nav = hmi.Navigation()
            nav.ParseFromString(payload)

            # Pretty print
            print("\n=== Navigation ===")
            if nav.HasField("current_lat"):
                print(f"lat: {nav.current_lat:.8f}")
            if nav.HasField("current_lon"):
                print(f"lon: {nav.current_lon:.8f}")
            if nav.HasField("heading_deg"):
                print(f"heading_deg: {nav.heading_deg:.2f}")
            print(f"waypoints: {len(nav.waypoints)}")
            # Print first few waypoints
            for i, wp in enumerate(nav.waypoints[:5]):
                print(f"  [{i}] lat={wp.lat:.8f}, lon={wp.lon:.8f}")
            if len(nav.waypoints) > 5:
                print(f"  ... ({len(nav.waypoints)-5} more)")
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        conn.close()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=65432)
    args = ap.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((args.host, args.port))
        s.listen(5)
        print(f"[listening] {args.host}:{args.port}")
        while True:
            conn, addr = s.accept()
            handle_client(conn, addr)

if __name__ == "__main__":
    main()
