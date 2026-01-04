import socket
import struct
import cv2
import numpy as np
import CAMERA_pb2  # Imports classes from CAMERA_pb2.py

def recvall(sock, n):
    """
    Helper function to ensure we read exactly n bytes from the socket.
    TCP streams can fragment packets, so we must loop until we get everything.
    """
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def main():
    HOST = '127.0.0.1'  
    PORT = 65433        

    print(f"Starting HMI Receiver on {HOST}:{PORT}...")
    
    # 1. Setup the Server Socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()

        print("Waiting for connection from ROS node...")
        conn, addr = server_sock.accept()
        
        with conn:
            print(f"Connected by {addr}")
            
            try:
                while True:
                    # 2. Read the 4-byte Length Header
                    raw_msglen = recvall(conn, 4)
                    if not raw_msglen:
                        break # Connection closed
                    
                    msglen = struct.unpack('>I', raw_msglen)[0]

                    # 3. Read the Protobuf Payload
                    proto_data = recvall(conn, msglen)
                    if not proto_data:
                        break

                    # 4. Deserialize Protobuf
                    # We use the class directly from the imported module
                    batch = CAMERA_pb2.CameraBatch()
                    batch.ParseFromString(proto_data)

                    # 5. Decode and Display Images
                    for frame in batch.frames:
                        cam_id = frame.camera_id
                        
                        np_arr = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
                        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if img is not None:
                            cv2.imshow(f"Stream: {cam_id}", img)
                        else:
                            print(f"Failed to decode image for {cam_id}")

                    # 6. Handle UI Events (press 'q' to quit)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Quitting...")
                        break

            except Exception as e:
                print(f"Error: {e}")
            finally:
                print("Closing connection.")
                cv2.destroyAllWindows()

if __name__ == "__main__":
    main()