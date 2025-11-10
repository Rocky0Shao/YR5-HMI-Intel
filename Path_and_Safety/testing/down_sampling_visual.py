# To doimport ast
from pathlib import Path
import matplotlib.pyplot as plt
import ast
def load_tuples(path: str):
    with open(path) as f:
        return [(float(a), float(b)) for (a, b) in ast.literal_eval(f.read())]

def plot_points(name: str, pts, outfile: str):
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    lat_pad = max(1e-6, 0.02 * max(1e-8, max_lat - min_lat))
    lon_pad = max(1e-6, 0.02 * max(1e-8, max_lon - min_lon))

    plt.figure(figsize=(6, 6))
    plt.scatter(lons, lats, s=6)  # x=lon, y=lat
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(min_lon - lon_pad, max_lon + lon_pad)
    plt.ylim(min_lat - lat_pad, max_lat + lat_pad)
    plt.xlabel("Longitude (deg)")
    plt.ylabel("Latitude (deg)")
    plt.title(f"{name.capitalize()} waypoints (n={len(pts)})")
    plt.grid(True, linewidth=0.3)
    plt.tight_layout()
    plt.savefig(outfile, dpi=160, bbox_inches='tight')
    plt.show()

unfiltered = load_tuples("/home/rocky/autodrive/YR5-CANtainer/Path_and_Safety/testing/waypoints_CAR_test.txt")
filtered = load_tuples("/home/rocky/autodrive/YR5-CANtainer/Path_and_Safety/testing/waypoints_CAR_test.txt_30cm.txt")

plot_points("unfiltered", unfiltered, "waypoints_CAR_test_unfiltered_waypoints.png")
plot_points("filtered", filtered, "waypoints_CAR_test_filtered_waypoints.png")
