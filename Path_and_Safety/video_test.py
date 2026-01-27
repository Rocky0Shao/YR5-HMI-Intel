import socket
import struct
import cv2
import numpy as np
import time  # Added for bandwidth calculation
import CAMERA_pb2

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
    PORT = 8554        

    print(f"Starting HMI Receiver on {HOST}:{PORT}...")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()

        print("Waiting for connection from ROS node...")
        conn, addr = server_sock.accept()
        
        with conn:
            print(f"Connected by {addr}")
            
            # --- Bandwidth Tracking Variables ---
            byte_counter = 0
            last_time = time.time()
            update_interval = 1.0  # Update display every 1 second
            # ------------------------------------

            try:
                while True:
                    # 1. Read the 4-byte Length Header
                    raw_msglen = recvall(conn, 4)
                    if not raw_msglen:
                        break 
                    
                    msglen = struct.unpack('>I', raw_msglen)[0]

                    # 2. Read the Protobuf Payload
                    proto_data = recvall(conn, msglen)
                    if not proto_data:
                        break

                    # --- Bandwidth Calculation ---
                    # Add header size (4 bytes) + payload size
                    byte_counter += (4 + msglen)
                    
                    current_time = time.time()
                    elapsed = current_time - last_time

                    if elapsed >= update_interval:
                        # Calculate speeds
                        bytes_per_sec = byte_counter / elapsed
                        mb_per_sec = bytes_per_sec / (1024 * 1024)   # Megabytes per second
                        mbps = (bytes_per_sec * 8) / (1000 * 1000)   # Megabits per second (Network standard)

                        # Print status with \r to overwrite the line (keeping terminal clean)
                        print(f"\r[ Bandwidth: {mb_per_sec:.2f} MB/s | {mbps:.2f} Mbps ]", end="")
                        
                        # Reset counters
                        byte_counter = 0
                        last_time = current_time
                    # -----------------------------

                    # 3. Deserialize Protobuf
                    batch = CAMERA_pb2.CameraBatch()
                    batch.ParseFromString(proto_data)

                    # 4. Decode and Display Images
                    for frame in batch.frames:
                        cam_id = frame.camera_id
                        
                        np_arr = np.frombuffer(frame.jpeg_data, dtype=np.uint8)
                        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if img is not None:
                            cv2.imshow(f"Stream: {cam_id}", img)
                        else:
                            print(f"\nFailed to decode image for {cam_id}") # \n added to clear bandwidth line

                    # 5. Handle UI Events (press 'q' to quit)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("\nQuitting...")
                        break

            except Exception as e:
                print(f"\nError: {e}")
            finally:
                print("\nClosing connection.")
                cv2.destroyAllWindows()

if __name__ == "__main__":
    main()