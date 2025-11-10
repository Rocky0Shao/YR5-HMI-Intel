import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax

from typing import List, Tuple, Sequence

import math

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
        self.raw_points_remain: List[Tuple[float, float]] = []


    def cb_raw_points_remain(self, msg: Float64MultiArray):
        unfiltered_pts = parse_xy_flat(msg.data)


        filtered_pts = parse_xy_pairs(unfiltered_pts, threshold=1.0)  # 1 meter threshold
        self.raw_points_all = filtered_pts
        if filtered_pts:
            self.get_logger().info(f'/raw_points_remain: {len(filtered_pts)} points; first={filtered_pts[0]} \n')
        else:
            self.get_logger().info('/raw_points_remain: 0 points')
            
    def cb_inspvax(self, msg: Inspvax):
        """Extract azimuth from /inspvax and store it."""
        self.latest_azimuth = msg.azimuth
        self.get_logger().info(f'/inspvax: azimuth = {self.latest_azimuth:.3f}°')

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
