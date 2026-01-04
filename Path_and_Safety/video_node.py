import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- Essential Imports ---
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 

# --- Other Imports ---
from typing import List, Tuple, Sequence, Optional
import math
import socket   
import struct
import cv2
import numpy as np
import time

# --- IMPORT PROTOBUF DEFINITION ---
import CAMERA_pb2

class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')

        self.bridge = CvBridge()

        # --- TCP Configuration ---
        self.declare_parameter('hmi_tx_host', '127.0.0.1')
        self.declare_parameter('hmi_tx_port', 65433)
        self.hmi_tx_host = self.get_parameter('hmi_tx_host').value
        self.hmi_tx_port = int(self.get_parameter('hmi_tx_port').value)
        self.hmi_tx_sock: Optional[socket.socket] = None

        self._connect_hmi_tx()

        # --- Display Config ---
        self.display_width = 400
        self.display_height = 300
        
        # Buffer to hold latest frames
        self.frames = {
            "cam0": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8),
            "cam1": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8),
            "cam2": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8)
        }

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers ---
        self.create_subscription(Image, '/blackfly_0/image_raw', self.callback_cam_0, qos_profile)
        self.create_subscription(Image, '/blackfly_1/image_raw', self.callback_cam_1, qos_profile)
        self.create_subscription(Image, '/blackfly_2/image_raw', self.callback_cam_2, qos_profile)

        # --- Timer for TCP Sending (24 Hz) ---
        self.create_timer(1.0 / 24.0, self.tx_video)

        self.get_logger().info(f'Camera Node Started. Streaming serialized Protobuf to {self.hmi_tx_host}:{self.hmi_tx_port}')


    def _connect_hmi_tx(self) -> None:
        if self.hmi_tx_sock:
            try:
                self.hmi_tx_sock.close()
            except Exception:
                pass
            self.hmi_tx_sock = None

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(2.0)
            s.connect((self.hmi_tx_host, self.hmi_tx_port))
            s.settimeout(None) 
            self.hmi_tx_sock = s
            self.get_logger().info(f'Connected TCP to {self.hmi_tx_host}:{self.hmi_tx_port}')
        except Exception as e:
            self.hmi_tx_sock = None
            self.get_logger().warning(f'TCP connect failed: {e}')

    def tx_video(self):
        """Bundles all frames into a Protobuf CameraBatch and sends it."""
        
        # Check connection before processing
        if self.hmi_tx_sock is None:
            self._connect_hmi_tx()
            if self.hmi_tx_sock is None:
                return

        # 1. Create the Protobuf Message Object
        batch_msg = CAMERA_pb2.CameraBatch()
        batch_msg.timestamp = int(time.time() * 1000) # milliseconds

        camera_order = ["cam0", "cam1", "cam2"]
        
        # 2. Loop through cameras and populate the message
        for key in camera_order:
            img = self.frames[key]

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50] 
            success, jpeg_data = cv2.imencode('.jpg', img, encode_param)
            
            if success:
                # Add a new frame to the repeated 'frames' field in the batch
                frame_msg = batch_msg.frames.add()
                frame_msg.camera_id = key
                frame_msg.jpeg_data = jpeg_data.tobytes()
            else:
                self.get_logger().warning(f"Failed to encode {key}")

        # 3. Serialize the Protobuf object to bytes
        try:
            serialized_payload = batch_msg.SerializeToString()
            
            # 4. Create Header [4-byte length]
            header = struct.pack(">I", len(serialized_payload))
            
            # 5. Send Header + Payload
            self.hmi_tx_sock.sendall(header + serialized_payload)

        except (socket.timeout, ConnectionRefusedError, ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warning(f'TCP send failed, will retry later: {e}')
            self.hmi_tx_sock = None
        except Exception as e:
            self.get_logger().error(f'Unexpected TCP error: {e}')
            self.hmi_tx_sock = None

    def process_image(self, msg, key):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (self.display_width, self.display_height))
            self.frames[key] = cv_image
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    # --- Callbacks ---
    def callback_cam_0(self, msg):
        self.process_image(msg, "cam0")

    def callback_cam_1(self, msg):
        self.process_image(msg, "cam1")

    def callback_cam_2(self, msg):
        self.process_image(msg, "cam2")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()