import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- Essential Imports for this task ---
from sensor_msgs.msg import Image  # Added for the camera topics
from cv_bridge import CvBridge     # Added to convert ROS images to OpenCV

# --- Your Provided Imports ---

class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')

        # Create the CV Bridge object for image conversion
        self.bridge = CvBridge()

        # Define QoS Profile
        # 'Best Effort' is often required for high-bandwidth sensor data like cameras.
        # If the camera publishes 'Best Effort' and you subscribe 'Reliable', you will get NO data.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber 1: Blackfly 0 ---
        self.sub_cam_0 = self.create_subscription(
            Image,
            '/blackfly_0/image_raw',
            self.callback_cam_0,
            qos_profile
        )

        # --- Subscriber 2: Blackfly 1 ---
        self.sub_cam_1 = self.create_subscription(
            Image,
            '/blackfly_1/image_raw',
            self.callback_cam_1,
            qos_profile
        )

        # --- Subscriber 3: Blackfly 2 ---
        self.sub_cam_2 = self.create_subscription(
            Image,
            '/blackfly_2/image_raw',
            self.callback_cam_2,
            qos_profile
        )

        self.get_logger().info('MultiCameraNode has started subscribing to blackfly topics.')

    def process_image(self, msg, camera_id):
        """Helper function to handle image data from any camera."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # TODO: Add your logic here (using HMI_TX, socket, etc.)
            
            # Just printing for demonstration
            self.get_logger().info(f'Received frame from {camera_id}: {cv_image.shape}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from {camera_id}: {e}')

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()