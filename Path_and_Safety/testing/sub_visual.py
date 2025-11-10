import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from novatel_gps_msgs.msg import Inspvax

from typing import List, Tuple, Sequence
import math

# NEW
import numpy as np
import cv2

def parse_xy_flat(data: Sequence[float]) -> List[Tuple[float, float]]:
    """Convert [x1,y1,x2,y2,...] -> [(x1,y1),(x2,y2),...]. Drops a trailing odd value."""
    n = len(data)
    if n % 2:
        n -= 1
    xs = data[:n:2]
    ys = data[1:n:2]
    return list(zip(xs, ys))

def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    φ1, λ1, φ2, λ2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dφ = φ2 - φ1
    dλ = λ2 - λ1
    a = math.sin(dφ/2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(dλ/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def parse_xy_pairs(data: List[Tuple[float, float]], threshold: float) -> List[Tuple[float, float]]:
    """Downsample successive (lat, lon) by distance threshold in meters."""
    if not data:
        return []
    if threshold <= 0:
        return list(data)
    out: List[Tuple[float, float]] = [data[0]]
    prev_lat, prev_lon = data[0]
    for lat, lon in data[1:]:
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if _haversine_m(prev_lat, prev_lon, lat, lon) >= threshold:
            out.append((lat, lon))
            prev_lat, prev_lon = lat, lon
    return out

# NEW: local meter projection for quick drawing
def _meters_per_deg(lat_deg: float) -> Tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    m_per_deg_lat = 111_132.92 - 559.82*math.cos(2*lat_rad) + 1.175*math.cos(4*lat_rad)
    m_per_deg_lon = 111_412.84*math.cos(lat_rad) - 93.5*math.cos(3*lat_rad)
    return m_per_deg_lat, m_per_deg_lon

def latlon_to_local_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Return (x_east_m, y_north_m) relative to (lat0, lon0)."""
    m_per_deg_lat, m_per_deg_lon = _meters_per_deg(lat0)
    dy = (lat - lat0) * m_per_deg_lat
    dx = (lon - lon0) * m_per_deg_lon
    return dx, dy

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Subscribers
        self.create_subscription(Float64MultiArray, '/raw_points_remain', self.cb_raw_points_remain, 10)
        self.create_subscription(Inspvax, '/inspvax', self.cb_inspvax, 10)

        # State
        self.latest_azimuth: float = float('nan')  # degrees, 0 = north, 90 = east per NovAtel
        self.raw_points_all: List[Tuple[float, float]] = []  # filtered lat,lon

        # NEW: render timer ~20 FPS
        self._win_name = "Waypoints stream"
        cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        self._canvas_px = (720, 720)  # H, W
        self._meters_pad = 5.0        # border pad in meters
        self.create_timer(0.05, self._render_frame)  # 20 fps

    def cb_raw_points_remain(self, msg: Float64MultiArray):
        unfiltered_pts = parse_xy_flat(msg.data)
        filtered_pts = parse_xy_pairs(unfiltered_pts, threshold=1.0)  # 1 meter
        self.raw_points_all = filtered_pts
        if filtered_pts:
            self.get_logger().info(f'/raw_points_remain: {len(filtered_pts)} points; first={filtered_pts[0]}')
        else:
            self.get_logger().info('/raw_points_remain: 0 points')

    def cb_inspvax(self, msg: Inspvax):
        self.latest_azimuth = float(msg.azimuth)
        # keep logs sparse
        # self.get_logger().info(f'/inspvax: azimuth = {self.latest_azimuth:.3f}°')

    # NEW: single-window streaming renderer
    def _render_frame(self):
        H, W = self._canvas_px
        img = np.full((H, W, 3), 255, np.uint8)

        pts = self.raw_points_all
        if not pts:
            cv2.putText(img, "Waiting for /raw_points_remain ...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.imshow(self._win_name, img)
            cv2.waitKey(1)
            return

        # Anchor
        lat0, lon0 = pts[0]
        # Map lat/lon -> local meters
        xy_m = [latlon_to_local_m(lat0, lon0, lat, lon) for (lat, lon) in pts]
        xs = [x for x, y in xy_m]
        ys = [y for x, y in xy_m]

        # Bounds with padding
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        dx = max_x - min_x
        dy = max_y - min_y
        dx = dx if dx > 0 else 1.0
        dy = dy if dy > 0 else 1.0

        # Add small meter padding to avoid edge clipping
        min_x -= self._meters_pad
        max_x += self._meters_pad
        min_y -= self._meters_pad
        max_y += self._meters_pad

        # Fit to canvas with y up
        def to_px(xm: float, ym: float) -> Tuple[int, int]:
            u = (xm - min_x) / (max_x - min_x)
            v = (ym - min_y) / (max_y - min_y)
            px = int(u * (W - 1))
            py = int((1.0 - v) * (H - 1))
            return px, py

        # Draw path
        for i in range(1, len(xy_m)):
            p0 = to_px(*xy_m[i - 1])
            p1 = to_px(*xy_m[i])
            cv2.line(img, p0, p1, (180, 180, 180), 2)

        # Draw points
        for (xm, ym) in xy_m:
            cv2.circle(img, to_px(xm, ym), 3, (50, 50, 200), -1)

        # Draw current pose as a triangle arrow using latest azimuth if available
        if math.isfinite(self.latest_azimuth):
            # NovAtel azimuth: 0 = north, clockwise. Convert to radians with x=east, y=north.
            az_deg = self.latest_azimuth
            az_rad = math.radians(az_deg)
            # Use last path point as current position
            cur_xm, cur_ym = xy_m[0]
            p = np.array(to_px(cur_xm, cur_ym))

            # Arrow geometry in pixels
            length_px = 40
            width_px = 16
            # Heading vector (x right, y down in image coords)
            hx = math.sin(az_rad)
            hy = -math.cos(az_rad)
            h = np.array([hx, hy], dtype=float)
            ortho = np.array([-h[1], h[0]])

            tip = (p + h * length_px).astype(int)
            left = (p - h * 10 + ortho * width_px/2).astype(int)
            right = (p - h * 10 - ortho * width_px/2).astype(int)

            cv2.fillConvexPoly(img, np.array([tip, left, right]), (0, 140, 0))
            cv2.circle(img, p.astype(int), 5, (0, 0, 0), -1)
            cv2.putText(img, f"Azimuth {az_deg:.1f} deg", (20, H-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)

        cv2.imshow(self._win_name, img)
        cv2.waitKey(1)  # keeps window responsive, no blocking

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # NEW
        rclpy.shutdown()

if __name__ == '__main__':
    main()
