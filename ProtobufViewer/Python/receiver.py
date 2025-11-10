#!/usr/bin/env python3

import argparse
import socket
import struct
import threading
import sys

from google.protobuf import json_format
import message_pb2


def recvall(sock, n):
    """Receive exactly n bytes or return None if EOF."""
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return bytes(data)


def handle_client(conn, addr):
    print(f"Client connected: {addr}")
    try:
        while True:
            prefix = recvall(conn, 4)
            if prefix is None:
                break
            (length,) = struct.unpack('>I', prefix)
            body = recvall(conn, length)
            if body is None:
                break
            msg = message_pb2.Test()
            msg.ParseFromString(body)
            
            obj = json_format.MessageToDict(msg, preserving_proto_field_name=True)
            print(f"Received message: id={msg.id} message=\"{msg.message}\"")
            print(obj)
    except Exception as e:
        print(f"Client handler error: {e}")
    finally:
        conn.close()
        print(f"Client disconnected: {addr}")


def serve(host: str, port: int):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen()
        print(f"Listening on {host}:{port}")
        try:
            while True:
                conn, addr = srv.accept()
                t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print('\nShutting down')


def main():
    parser = argparse.ArgumentParser(description='Protobuf receiver')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=50051)
    args = parser.parse_args()
    serve(args.host, args.port)


if __name__ == '__main__':
    main()
