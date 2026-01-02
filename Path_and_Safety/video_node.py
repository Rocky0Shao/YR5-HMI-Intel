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

        # --- Display/Stitching Config ---
        self.display_width = 400
        self.display_height = 300
        
        # Buffer to hold latest frames
        # FIX: We use these keys ("cam0", etc) consistently now
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
        # FIX: Changed 'self.cb_cam_0' to 'self.callback_cam_0' to match the functions below
        self.create_subscription(Image, '/blackfly_0/image_raw', self.callback_cam_0, qos_profile)
        self.create_subscription(Image, '/blackfly_1/image_raw', self.callback_cam_1, qos_profile)
        self.create_subscription(Image, '/blackfly_2/image_raw', self.callback_cam_2, qos_profile)

        # --- Timer for TCP Sending (15 Hz) ---
        self.create_timer(1.0 / 15.0, self.tx_video)

        self.get_logger().info(f'Camera Node Started. Streaming to {self.hmi_tx_host}:{self.hmi_tx_port}')


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
        """Stitches frames, compresses to JPEG, and sends over TCP."""
        try:
            # FIX: These keys must match what is in self.frames
            combined_img = np.hstack((
                self.frames["cam0"], 
                self.frames["cam1"], 
                self.frames["cam2"]
            ))
        except Exception as e:
            self.get_logger().error(f"Stitching failed: {e}")
            return

        # Compress to JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50] 
        success, jpeg_data = cv2.imencode('.jpg', combined_img, encode_param)
        
        if not success:
            return

        payload = jpeg_data.tobytes()

        try:
            if self.hmi_tx_sock is None:
                self._connect_hmi_tx()
            
            if self.hmi_tx_sock is None:
                return

            # Framing: [4-byte Length][Payload]
            frame = struct.pack(">I", len(payload)) + payload
            self.hmi_tx_sock.sendall(frame)

        except (socket.timeout, ConnectionRefusedError, ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warning(f'TCP send failed, will retry: {e}')
            self._connect_hmi_tx()
        except Exception as e:
            self.get_logger().error(f'Unexpected TCP error: {e}')

    def process_image(self, msg, key):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (self.display_width, self.display_height))
            
            # This updates the specific camera frame in the dictionary
            self.frames[key] = cv_image
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    # --- Callbacks ---
    # FIX: Logic updated to use "cam0", "cam1", "cam2"
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