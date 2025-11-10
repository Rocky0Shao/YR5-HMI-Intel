import math
from typing import List, Tuple, Sequence





import ast
with open("/home/rocky/autodrive/YR5-CANtainer/Path_and_Safety/testing/waypoints_CAR_test.txt") as f:
    raw_points = ast.literal_eval(f.read())
    print(f"Loaded {len(raw_points)} raw points from unfilterd_pts.txt")


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

    for lat, lon  in data[1:]:
        #unpack tuple values
 
        # skip non-finite values
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if _haversine_m(prev_lat, prev_lon, lat, lon) >= threshold:
            out.append((lat, lon))
            prev_lat, prev_lon = lat, lon

    return out

printed_points = parse_xy_pairs(raw_points, threshold=0.3)
print(f"\nFiltered down to {len(printed_points)} points with 1 meter threshold:\n")

with open('waypoints_CAR_test.txt_30cm.txt', 'w') as file:
    file.write(printed_points.__str__())
    print(f'Wrote {len(printed_points)} filtered points to filtered_pts.txt')