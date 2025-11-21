import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax
from std_msgs.msg import String, Int32  # NEW

from typing import List, Tuple, Sequence, Optional

import math
import socket  
import struct
import HMI_RX_CONTROLS_pb2 as hmi_rx
import HMI_TX_CONTROLS_pb2 as hmi_tx

def parse_xy_flat(data: Sequence[float]) -> List[Tuple[float, float]]:
    """Convert [x1,y1,x2,y2,...] -> [(x1,y1),(x2,y2),...]. Drops a trailing odd value."""
    n = len(data)
    if n % 2:  # odd length
        n -= 1
    xs = data[:n:2]
    ys = data[1:n:2]
    return list(zip(xs, ys))

def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters. Inputs in degrees."""
    R = 6371000.0
    φ1, λ1, φ2, λ2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dφ = φ2 - φ1
    dλ = λ2 - λ1
    a = math.sin(dφ/2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(dλ/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def parse_xy_pairs(data: List[Tuple[float, float]], threshold: float) -> List[Tuple[float, float]]:
    """
    Downsample successive (lat, lon) points by distance.   
    Keeps the first point, then keeps a point only if distance from last kept >= threshold meters.
    """
    if not data:
        return []
    if threshold <= 0:
        return list(data)

    out: List[Tuple[float, float]] = [data[0]]
    prev_lat, prev_lon = data[0]

    for lat, lon in data[1:]:
        # skip non-finite values
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if _haversine_m(prev_lat, prev_lon, lat, lon) >= threshold:
            out.append((lat, lon))
            prev_lat, prev_lon = lat, lon

    return out

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Subscribe to raw points
        self.create_subscription(Float64MultiArray,
                                 '/raw_points_remain',
                                 self.cb_raw_points_remain,
                                 10)

        # Subscribe to inspvax (NovAtel GPS message)
        self.create_subscription(Inspvax,
                                 '/inspvax',
                                 self.cb_inspvax,
                                 10)


        # Publishers
        # Send Target Destination to Controls (ROS Topic String)
        self.dest_pub = self.create_publisher(
            String,
            '/controls/target_destination',
            10
        )
        # Send Engage/Disengage/Disabled to Safety (ROS Topic int: 0,1,3)
        self.engage_pub = self.create_publisher(
            Int32,
            '/safety/engage_state',
            10
        )

        # Holders
        self.latest_azimuth: float = float('nan')
        self.current_lat: float = float('nan')
        self.current_lon: float = float('nan')
        self.filtered_pts: List[Tuple[float, float]] = []

        # State for HMI topics
        self.current_destination: str = ""   # set this from your HMI input
        self.current_engage_state: int = 0   # 0=disengage, 1=engage, 3=disabled


        # --- TCP: TX (ROS → HMI, Navigation protobuf) ---

        self.declare_parameter('hmi_tx_host', '127.0.0.1')
        self.declare_parameter('hmi_tx_port', 65432)   # Navigation out
        self.hmi_tx_host: str = self.get_parameter('hmi_tx_host').value
        self.hmi_tx_port: int = int(self.get_parameter('hmi_tx_port').value)
        self.hmi_tx_sock: Optional[socket.socket] = None

        self._connect_hmi_tx()

        # --- TCP: RX (HMI → ROS, HMITxMessage commands) ---

        self.declare_parameter('hmi_rx_host', '127.0.0.1')
        self.declare_parameter('hmi_rx_port', 5001)    # Commands in
        self.hmi_rx_host: str = self.get_parameter('hmi_rx_host').value
        self.hmi_rx_port: int = int(self.get_parameter('hmi_rx_port').value)
        self.hmi_rx_sock: Optional[socket.socket] = None
        self.hmi_rx_buf: bytes = b''


        # Timers
        # TX Navigation at 5 Hz
        self.tx_timer = self.create_timer(0.2, self.tx_nav)
        # RX Commands poll at 20 Hz
        self.rx_timer = self.create_timer(0.05, self.rx_step)


    def _connect_hmi_tx(self) -> None:
        """(Re)connect HMI TX socket (ROS → HMI backend)."""
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
            s.settimeout(0.5)
            self.hmi_tx_sock = s
            self.get_logger().info(f'Connected HMI TX TCP to {self.hmi_tx_host}:{self.hmi_tx_port}')
        except Exception as e:
            self.hmi_tx_sock = None
            self.get_logger().warning(f'HMI TX TCP connect failed: {e}')

    def build_nav_proto(self) -> bytes:
        nav = hmi_rx.Navigation()
        if math.isfinite(self.current_lat):
            nav.current_lat = self.current_lat
        if math.isfinite(self.current_lon):
            nav.current_lon = self.current_lon
        if math.isfinite(self.latest_azimuth):
            nav.heading_deg = self.latest_azimuth

        # filtered_pts is [(lat, lon)]
        for lat, lon in self.filtered_pts:
            wp = nav.waypoints.add()
            wp.lat = float(lat)
            wp.lon = float(lon)

        # Skip if message would be empty
        if nav.ByteSize() == 0:
            return b""
        return nav.SerializeToString()

    def tx_nav(self) -> None:
        """Send Navigation protobuf to HMI over TCP (length-prefixed)."""
        try:
            payload = self.build_nav_proto()
            if not payload:
                return

            if self.hmi_tx_sock is None:
                self._connect_hmi_tx()
            if self.hmi_tx_sock is None:
                return

            frame = struct.pack(">I", len(payload)) + payload
            self.hmi_tx_sock.sendall(frame)

        except (socket.timeout, ConnectionRefusedError,
                ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warning(f'TCP send failed, will retry: {e}')
            self._connect_hmi_tx()
        except Exception as e:
            self.get_logger().error(f'Failed to build/send Navigation proto: {e}')


    # ----------------------------------------------------------------------
    # HMI RX: HMI → ROS (HMITxMessage command protobuf)
    # ----------------------------------------------------------------------

    def _connect_hmi_rx(self) -> None:
        """(Re)connect HMI RX socket (HMI backend → ROS commands)."""
        if self.hmi_rx_sock:
            try:
                self.hmi_rx_sock.close()
            except Exception:
                pass
            self.hmi_rx_sock = None

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self.hmi_rx_host, self.hmi_rx_port))
            # short read timeout so recv() doesn't block spin
            s.settimeout(0.01)
            self.hmi_rx_sock = s
            self.get_logger().info(f'Connected HMI RX TCP to {self.hmi_rx_host}:{self.hmi_rx_port}')
        except Exception as e:
            self.hmi_rx_sock = None
            self.get_logger().warning(f'HMI RX TCP connect failed: {e}')

    def _handle_hmi_command(self, cmd: hmi_tx.HMITxMessage) -> None:
        """
        Update internal state from HMITxMessage and publish to ROS topics.

        HMITxMessage:
            int32 engage_status = 1;  // 0=DISENGAGE, 1=ENGAGE, 2=DISABLED
            string target_destination = 2; // "A".."Z"
        """
        # Map engage_status to safety node convention (0,1,3)
        status = int(cmd.engage_status)
        if status == 2:
            status = 3  # map DISABLED=2 → 3 for /safety/engage_state

        self.current_engage_state = status

        if cmd.target_destination:
            self.current_destination = cmd.target_destination

        # Publish to ROS
        self.publish_engage_state()
        self.publish_destination()

    def rx_step(self) -> None:
        """
        Poll RX socket, read length-prefixed HMITxMessage frames, decode,
        and publish engage/destination topics.
        """
        # Ensure connection
        if self.hmi_rx_sock is None:
            self._connect_hmi_rx()
            if self.hmi_rx_sock is None:
                return

        try:
            chunk = self.hmi_rx_sock.recv(4096)
            if not chunk:
                # Peer closed connection
                self.get_logger().warning('HMI RX connection closed by peer; reconnecting')
                try:
                    self.hmi_rx_sock.close()
                except Exception:
                    pass
                self.hmi_rx_sock = None
                return

            self.hmi_rx_buf += chunk

            # Parse one or more frames from buffer
            while True:
                if len(self.hmi_rx_buf) < 4:
                    break
                msg_len = struct.unpack(">I", self.hmi_rx_buf[:4])[0]
                if len(self.hmi_rx_buf) < 4 + msg_len:
                    break

                frame = self.hmi_rx_buf[4:4 + msg_len]
                self.hmi_rx_buf = self.hmi_rx_buf[4 + msg_len:]

                cmd = hmi_tx.HMITxMessage()
                try:
                    cmd.ParseFromString(frame)
                    self._handle_hmi_command(cmd)
                except Exception as e:
                    self.get_logger().error(f'Failed to parse HMITxMessage: {e}')

        except socket.timeout:
            # normal, just no data this cycle
            pass
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError) as e:
            self.get_logger().warning(f'HMI RX socket error, will reconnect: {e}')
            try:
                if self.hmi_rx_sock:
                    self.hmi_rx_sock.close()
            except Exception:
                pass
            self.hmi_rx_sock = None
        except Exception as e:
            self.get_logger().error(f'HMI RX unexpected error: {e}')

    # ----------------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------------
    def cb_raw_points_remain(self, msg: Float64MultiArray):
        unfiltered_pts = parse_xy_flat(msg.data)               # [(lat, lon), ...]
        self.filtered_pts = parse_xy_pairs(unfiltered_pts, 1.0)
        self.get_logger().info(f'/raw_points_remain: {len(self.filtered_pts)} points')
            
    def cb_inspvax(self, msg: Inspvax):
        # NovAtel Inspvax typically provides latitude, longitude, azimuth (deg)
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.latest_azimuth = float(msg.azimuth)

    # ----------------------------------------------------------------------
    # ROS publication helpers
    # ----------------------------------------------------------------------

    def publish_destination(self) -> None:
        """Publish current destination string to Controls."""
        if self.current_destination:
            msg = String()
            msg.data = self.current_destination
            self.dest_pub.publish(msg)

    def publish_engage_state(self) -> None:
        """Publish current engage state int to Safety."""
        msg = Int32()
        msg.data = int(self.current_engage_state)
        self.engage_pub.publish(msg)



    # --- Cleanup ---

    def destroy_node(self):
        try:
            if self.hmi_tx_sock:
                try:
                    self.hmi_tx_sock.shutdown(socket.SHUT_WR)
                except Exception:
                    pass
                self.hmi_tx_sock.close()
        except Exception:
            pass

        try:
            if self.hmi_rx_sock:
                try:
                    self.hmi_rx_sock.shutdown(socket.SHUT_WR)
                except Exception:
                    pass
                self.hmi_rx_sock.close()
        except Exception:
            pass

        super().destroy_node()


        
def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()