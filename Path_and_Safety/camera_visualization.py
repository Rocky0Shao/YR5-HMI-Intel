import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2  # <--- IMPORT OPENCV HERE

# --- Essential Imports ---
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

# --- Other Imports ---
from typing import List, Tuple, Sequence, Optional
import math
import socket   
import struct

class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')

        # Create the CV Bridge object
        self.bridge = CvBridge()

        # Define QoS Profile (Best Effort matches most camera drivers/bags)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber 1 ---
        self.sub_cam_0 = self.create_subscription(
            Image, '/blackfly_0/image_raw',
            self.callback_cam_0, qos_profile)

        # --- Subscriber 2 ---
        self.sub_cam_1 = self.create_subscription(
            Image, '/blackfly_1/image_raw',
            self.callback_cam_1, qos_profile)

        # --- Subscriber 3 ---
        self.sub_cam_2 = self.create_subscription(
            Image, '/blackfly_2/image_raw',
            self.callback_cam_2, qos_profile)

        self.get_logger().info('MultiCameraNode waiting for images...')

    def process_image(self, msg, camera_id):
        """Helper function to handle image data from any camera."""
        try:
            # 1. Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Resize image if it's too big (Optional, good for 4K cameras)
            # cv_image = cv2.resize(cv_image, (640, 480)) 

            # 3. Open a window and show the image
            # 'camera_id' acts as the Window Name. 
            # Since you pass different IDs, you will get 3 separate windows.
            cv2.imshow(camera_id, cv_image)
            
            # 4. Process GUI events (Essential for the window to update)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert/display image from {camera_id}: {e}')

    # --- Callbacks ---
    def callback_cam_0(self, msg):
        self.process_image(msg, "Blackfly_0")

    def callback_cam_1(self, msg):
        self.process_image(msg, "Blackfly_1")

    def callback_cam_2(self, msg):
        self.process_image(msg, "Blackfly_2")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cleanup
        node.destroy_node()
        rclpy.shutdown()
        # Close all OpenCV windows when the script stops
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()