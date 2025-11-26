import socket, struct
from typing import Optional
import HMI_RX_CAN_pb2 as can_pb2
import cantools

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
            # Read 4-byte big-endian length prefix
            hdr = recv_exact(conn, 4)
            if hdr is None:
                print("[disconnect]")
                return
            (length,) = struct.unpack(">I", hdr)
            if length == 0:
                continue

            payload = recv_exact(conn, length)
            if payload is None:
                print("[disconnect mid-frame]")
                return


            
            message_wrapper = can_pb2.MessageWrapper()

            message_name = message_wrapper.WhichOneof("messages")

            msg = None

            """
            if message_name == "av_light":
                msg = can_pb2.AVLight()
            elif message_name == "av_state":
                msg = can_pb2.AVState()"""
            
            message_wrapper.ParseFromString(payload)

            
       
            #av_light = can_pb2.AVLight()

            #av_light.ParseFromString(payload)
    

            # Pretty print without HasField on proto3 scalars
            #print("received" + str(av_light))
            print("Name: " + str(message_name))
            print(message_wrapper)

    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        conn.close()

def main():
    host = "127.0.0.1"
    port = 5002
    print(f"[listening] {host}:{port}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(5)
        while True:
            conn, addr = s.accept()
            handle_client(conn, addr)

if __name__ == "__main__":
    main()
