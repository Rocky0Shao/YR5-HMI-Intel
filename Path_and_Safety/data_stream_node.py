"""
data_stream_node.py - Data Stream Node

TX-only node for streaming navigation and video data to HMI.

TX (Intel -> HMI) on port 5001:
    - Navigation (message type 0x01): GPS position, heading, waypoints at 5 Hz
    - CameraBatch (message type 0x02): JPEG-compressed camera frames at 24 Hz
    - Intel connects as client, HMI listens as server

Wire Format:
    [4 bytes: big-endian uint32 length][1 byte: message type][N bytes: protobuf payload]

Message Types:
    0x01 = Navigation
    0x02 = CameraBatch

ROS Subscriptions:
    - /raw_points_remain (std_msgs/Float64MultiArray) - waypoints from Controls
    - /inspvax (novatel_gps_msgs/Inspvax) - GPS position and heading
    - /blackfly_0/image_raw (sensor_msgs/Image) - camera 0
    - /blackfly_1/image_raw (sensor_msgs/Image) - camera 1
    - /blackfly_2/image_raw (sensor_msgs/Image) - camera 2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import List, Tuple, Sequence, Optional
import math
import socket
import struct
import cv2
import numpy as np
import time

import HMI_RX_CONTROLS_pb2 as hmi_rx
import CAMERA_pb2

# Message type identifiers
MSG_TYPE_NAVIGATION = 0x01
MSG_TYPE_CAMERA = 0x02


def parse_xy_flat(data: Sequence[float]) -> List[Tuple[float, float]]:
    """Convert [x1,y1,x2,y2,...] -> [(x1,y1),(x2,y2),...]. Drops trailing odd value."""
    n = len(data)
    if n % 2:
        n -= 1
    xs = data[:n:2]
    ys = data[1:n:2]
    return list(zip(xs, ys))


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters. Inputs in degrees."""
    R = 6371000.0
    p1, l1, p2, l2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dp = p2 - p1
    dl = l2 - l1
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def parse_xy_pairs(data: List[Tuple[float, float]], threshold: float) -> List[Tuple[float, float]]:
    """Downsample successive (lat, lon) points by distance (threshold in meters)."""
    if not data:
        return []
    if threshold <= 0:
        return list(data)

    # Find first valid point
    first_valid_idx = None
    for idx, (lat, lon) in enumerate(data):
        if math.isfinite(lat) and math.isfinite(lon):
            first_valid_idx = idx
            break

    if first_valid_idx is None:
        return []

    out: List[Tuple[float, float]] = [data[first_valid_idx]]
    prev_lat, prev_lon = data[first_valid_idx]

    for lat, lon in data[first_valid_idx + 1:]:
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if _haversine_m(prev_lat, prev_lon, lat, lon) >= threshold:
            out.append((lat, lon))
            prev_lat, prev_lon = lat, lon

    return out


class DataStreamNode(Node):
    def __init__(self):
        super().__init__('data_stream_node')

        self.bridge = CvBridge()

        # --- TCP Configuration ---
        self.declare_parameter('hmi_tx_host', '127.0.0.1')
        self.declare_parameter('hmi_tx_port', 5001)
        self.hmi_tx_host: str = self.get_parameter('hmi_tx_host').value
        self.hmi_tx_port: int = int(self.get_parameter('hmi_tx_port').value)
        self.hmi_tx_sock: Optional[socket.socket] = None

        self._connect_hmi_tx()

        # --- Navigation data holders ---
        self.latest_azimuth: float = float('nan')
        self.current_lat: float = float('nan')
        self.current_lon: float = float('nan')
        self.filtered_pts: List[Tuple[float, float]] = []

        # --- Camera data holders ---
        self.display_width = 400
        self.display_height = 300
        self.frames = {
            "cam0": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8),
            "cam1": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8),
            "cam2": np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8)
        }

        # --- ROS Subscribers: Navigation ---
        self.create_subscription(
            Float64MultiArray,
            '/raw_points_remain',
            self.cb_raw_points_remain,
            10
        )
        self.create_subscription(
            Inspvax,
            '/inspvax',
            self.cb_inspvax,
            10
        )

        # --- ROS Subscribers: Camera (BEST_EFFORT for image streams) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Image, '/blackfly_0/image_raw', self.cb_cam_0, qos_profile)
        self.create_subscription(Image, '/blackfly_1/image_raw', self.cb_cam_1, qos_profile)
        self.create_subscription(Image, '/blackfly_2/image_raw', self.cb_cam_2, qos_profile)

        # --- Timers ---
        # Navigation at 5 Hz
        self.nav_timer = self.create_timer(0.2, self.tx_navigation)
        # Video at 24 Hz
        self.video_timer = self.create_timer(1.0 / 24.0, self.tx_video)

        self.get_logger().info(
            f'DataStreamNode started - streaming to {self.hmi_tx_host}:{self.hmi_tx_port}'
        )

    # --------------------------------------------------------------------------
    # TCP Connection
    # --------------------------------------------------------------------------

    def _connect_hmi_tx(self) -> None:
        """Connect to HMI for data transmission."""
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
            self.get_logger().info(f'Connected to HMI at {self.hmi_tx_host}:{self.hmi_tx_port}')
        except Exception as e:
            self.hmi_tx_sock = None
            self.get_logger().warning(f'HMI TX connect failed: {e}')

    def _send_frame(self, msg_type: int, payload: bytes) -> bool:
        """
        Send a message with type discriminator.

        Wire format: [4-byte length][1-byte type][payload]
        The length field includes the type byte.
        """
        if not payload:
            return False

        if self.hmi_tx_sock is None:
            self._connect_hmi_tx()
        if self.hmi_tx_sock is None:
            return False

        try:
            # Length = 1 (type byte) + len(payload)
            total_len = 1 + len(payload)
            frame = struct.pack(">I", total_len) + struct.pack("B", msg_type) + payload
            self.hmi_tx_sock.sendall(frame)
            return True
        except (socket.timeout, ConnectionRefusedError,
                ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warning(f'TCP send failed: {e}')
            self._connect_hmi_tx()
            return False
        except Exception as e:
            self.get_logger().error(f'TCP error: {e}')
            self._connect_hmi_tx()
            return False

    # --------------------------------------------------------------------------
    # Navigation TX (5 Hz)
    # --------------------------------------------------------------------------

    def _build_nav_proto(self) -> bytes:
        """Build Navigation protobuf message."""
        nav = hmi_rx.Navigation()
        if math.isfinite(self.current_lat):
            nav.current_lat = self.current_lat
        if math.isfinite(self.current_lon):
            nav.current_lon = self.current_lon
        if math.isfinite(self.latest_azimuth):
            nav.heading_deg = self.latest_azimuth

        for lat, lon in self.filtered_pts:
            wp = nav.waypoints.add()
            wp.lat = float(lat)
            wp.lon = float(lon)

        if nav.ByteSize() == 0:
            return b""
        return nav.SerializeToString()

    def tx_navigation(self) -> None:
        """Send Navigation protobuf to HMI."""
        try:
            payload = self._build_nav_proto()
            if payload:
                self._send_frame(MSG_TYPE_NAVIGATION, payload)
        except Exception as e:
            self.get_logger().error(f'Navigation TX error: {e}')

    # --------------------------------------------------------------------------
    # Video TX (24 Hz)
    # --------------------------------------------------------------------------

    def tx_video(self) -> None:
        """Send CameraBatch protobuf to HMI."""
        try:
            batch_msg = CAMERA_pb2.CameraBatch()
            batch_msg.timestamp = int(time.time() * 1000)

            camera_order = ["cam0", "cam1", "cam2"]

            for key in camera_order:
                img = self.frames[key]
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                success, jpeg_data = cv2.imencode('.jpg', img, encode_param)

                if success:
                    frame_msg = batch_msg.frames.add()
                    frame_msg.camera_id = key
                    frame_msg.jpeg_data = jpeg_data.tobytes()

            payload = batch_msg.SerializeToString()
            if payload:
                self._send_frame(MSG_TYPE_CAMERA, payload)

        except Exception as e:
            self.get_logger().error(f'Video TX error: {e}')

    # --------------------------------------------------------------------------
    # ROS Callbacks: Navigation
    # --------------------------------------------------------------------------

    def cb_raw_points_remain(self, msg: Float64MultiArray) -> None:
        """Handle waypoints from Controls."""
        unfiltered_pts = parse_xy_flat(msg.data)
        self.filtered_pts = parse_xy_pairs(unfiltered_pts, 1.0)
        self.get_logger().debug(f'/raw_points_remain: {len(self.filtered_pts)} points')

    def cb_inspvax(self, msg: Inspvax) -> None:
        """Handle GPS data from NovAtel."""
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.latest_azimuth = float(msg.azimuth)

    # --------------------------------------------------------------------------
    # ROS Callbacks: Camera
    # --------------------------------------------------------------------------

    def _process_image(self, msg: Image, key: str) -> None:
        """Convert ROS Image to resized BGR frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (self.display_width, self.display_height))
            self.frames[key] = cv_image
        except Exception as e:
            self.get_logger().error(f'Image conversion failed for {key}: {e}')

    def cb_cam_0(self, msg: Image) -> None:
        self._process_image(msg, "cam0")

    def cb_cam_1(self, msg: Image) -> None:
        self._process_image(msg, "cam1")

    def cb_cam_2(self, msg: Image) -> None:
        self._process_image(msg, "cam2")

    # --------------------------------------------------------------------------
    # Cleanup
    # --------------------------------------------------------------------------

    def destroy_node(self):
        """Clean up socket on shutdown."""
        if self.hmi_tx_sock:
            try:
                self.hmi_tx_sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
