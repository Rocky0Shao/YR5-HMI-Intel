import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax

from typing import List, Tuple, Sequence

import math

from proto_out import HMI_RX_CONTROLS_pb2 as hmi
from ws_client import WSClient

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

        # Start WS client
        uri = "ws://hmi-jetson.local:8765"   # set your server URI
        self.ws = WSClient(uri, on_error=lambda m: self.get_logger().warn(m))
        self.ws.start()
    
        # send at 5 Hz
        self.timer = self.create_timer(0.2, self.tx_nav)


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

        return nav.SerializeToString()

    def tx_nav(self):
        try:
            payload = self.build_nav_proto()
            if payload:  # avoid empty sends
                self.ws.send(payload)  # enqueue for async send

            # Example A: write to a debug file
            # with open('/tmp/nav.bin', 'wb') as f:
            #     f.write(payload)

            # Example B: send over a WebSocket to HMI (binary frame)
            import asyncio, websockets
            asyncio.get_running_loop().create_task(self._ws_send(payload))

            # or publish on a ROS topic as bytes if you prefer (std_msgs/ByteMultiArray)
            # (define a publisher and publish payload)

        except Exception as e:
            self.get_logger().error(f'Failed to build/send Navigation proto: {e}')

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
