#!/usr/bin/env python3

import argparse
import socket
import struct
import time
import sys

import message_pb2


def send_messages(host: str, port: int, count: int, interval: float, text: str):
    with socket.create_connection((host, port)) as sock:
        print(f"Connected to {host}:{port}")
        for i in range(1, count + 1):
            msg = message_pb2.Test()
            msg.message = f"{text} #{i}"
            msg.id = i
            data = msg.SerializeToString()
            # length-prefixed (4-byte big-endian)
            prefix = struct.pack('>I', len(data))
            sock.sendall(prefix + data)
            print(f"Sent message id={msg.id} len={len(data)}")
            time.sleep(interval)
        print("Done sending")


def main():
    parser = argparse.ArgumentParser(description="Protobuf sender")
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=50051)
    parser.add_argument('--count', type=int, default=5)
    parser.add_argument('--interval', type=float, default=0.5)
    parser.add_argument('--text', default='Test message')
    args = parser.parse_args()

    try:
        send_messages(args.host, args.port, args.count, args.interval, args.text)
    except ConnectionRefusedError:
        print(f"Could not connect to {args.host}:{args.port}", file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
