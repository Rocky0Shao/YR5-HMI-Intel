import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax

from typing import List, Tuple, Sequence

import math
import socket  
import struct
from proto_out import HMI_RX_CONTROLS_pb2 as hmi

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
        self.create_subscription(Float64MultiArray, '/raw_points_remain', self.cb_raw_points_remain, 10)

        # Subscribe to inspvax (NovAtel GPS message)
        self.create_subscription(Inspvax, '/inspvax', self.cb_inspvax, 10)

        # Holders
        self.latest_azimuth: float = float('nan')
        self.current_lat: float = float('nan')
        self.current_lon: float = float('nan')
        self.filtered_pts: List[Tuple[float, float]] = []

        # --- Initialize TCP Socket here ---
        self.declare_parameter('tcp_host', '127.0.0.1')
        self.declare_parameter('tcp_port', 65432)
        self.tcp_host: str = self.get_parameter('tcp_host').value
        self.tcp_port: int = int(self.get_parameter('tcp_port').value)
        self.sock: socket.socket | None = None
        self._connect_tcp()

        # send at 5 Hz
        self.timer = self.create_timer(0.2, self.tx_nav)

    def _connect_tcp(self) -> None:
        # Close existing if any
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(2.0)
            s.connect((self.tcp_host, self.tcp_port))
            s.settimeout(0.5)  # short ops timeout
            self.sock = s
            self.get_logger().info(f'Connected TCP to {self.tcp_host}:{self.tcp_port}')
        except Exception as e:
            self.sock = None
            self.get_logger().warn(f'TCP connect failed: {e}')


    def cb_raw_points_remain(self, msg: Float64MultiArray):
        unfiltered_pts = parse_xy_flat(msg.data)               # [(lat, lon), ...]
        self.filtered_pts = parse_xy_pairs(unfiltered_pts, 1.0)
        self.get_logger().info(f'/raw_points_remain: {len(self.filtered_pts)} points')
            
    def cb_inspvax(self, msg: Inspvax):
        # NovAtel Inspvax typically provides latitude, longitude, azimuth (deg)
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.latest_azimuth = float(msg.azimuth)

    def build_nav_proto(self) -> bytes:
        nav = hmi.Navigation()
        if math.isfinite(self.current_lat): nav.current_lat = self.current_lat
        if math.isfinite(self.current_lon): nav.current_lon = self.current_lon
        if math.isfinite(self.latest_azimuth): nav.heading_deg = self.latest_azimuth

        # filtered_pts is [(lat, lon)]
        for lat, lon in self.filtered_pts:
            wp = nav.waypoints.add()
            wp.lat = float(lat)
            wp.lon = float(lon)
    
        # Skip if message would be empty
        if nav.ByteSize() == 0:
            return b""
        return nav.SerializeToString()

    def tx_nav(self):
        try:
            payload = self.build_nav_proto()
            if not payload:
                return

            # --- TODO 2 fixed: Send payload over TCP socket ---
            if self.sock is None:
                self._connect_tcp()
            if self.sock is None:
                return

            frame = struct.pack(">I", len(payload)) + payload
            self.sock.sendall(frame)
            self.sock.sendall(payload)

        except (socket.timeout, ConnectionRefusedError, ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warn(f'TCP send failed, will retry: {e}')
            self._connect_tcp()
        except Exception as e:
            self.get_logger().error(f'Failed to build/send Navigation proto: {e}')

    # --- Cleanup ---
    def destroy_node(self):
        try:
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_WR)
                except Exception:
                    pass
                self.sock.close()
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