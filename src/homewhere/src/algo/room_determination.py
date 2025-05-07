#!/usr/bin/env python3
"""
room_determination.py
Utility for determining which room a global coordinate belongs to, using
polygonal room bounds stored in a YAML file.

The YAML file must contain for each room:
  center : [cx, cy]              # room center in global coordinates
  polygon: [[x1, y1], ...]       # vertex list in *local* room frame  (meters)

It can be structured either as:

map_table:
  "1":
    center: ...
    polygon: ...
  "2": ...

or directly:

"1":
  center: ...
  polygon: ...

Example:
    room_id = determine_room("/path/to/map_table.yaml", 3.2, 7.1)
"""

import yaml
from typing import List, Tuple, Dict

EPS = 1e-9


def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """
    Ray‑casting algorithm to test if point (x,y) is inside the polygon.

    Parameters
    ----------
    x, y : float
        Global coordinates of the query point.
    polygon : list[(float,float)]
        List of vertices [(x1,y1), (x2,y2), ...] in *global* coordinates.

    Returns
    -------
    bool : True  -> inside / on edge
           False -> outside
    """
    inside = False
    n = len(polygon)
    if n < 3:
        return False  # not a polygon

    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]

        # Check if the horizontal ray from (x,y) intersects the edge (p1,p2)
        if min(p1y, p2y) < y <= max(p1y, p2y) and x <= max(p1x, p2x):
            # Calculate intersection point of the edge with the horizontal line at y
            if abs(p2y - p1y) > EPS:
                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
            else:
                xinters = p1x  # Edge is horizontal; treat intersection at p1x

            # If the point is to the left of the intersection, toggle inside flag
            if x <= xinters + EPS:
                inside = not inside

        p1x, p1y = p2x, p2y

    return inside


def _load_map_table(path: str) -> Dict[str, Dict]:
    """Load YAML and return the map_table dict."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    # Allow two layouts: keyed directly or under "map_table"
    if "map_table" in data:
        return data["map_table"]
    return data


def determine_room(yaml_path: str, x_global: float, y_global: float) -> str:
    """
    Determine which room contains the global point (x_global, y_global).

    Parameters
    ----------
    yaml_path : str
        Path to map_table YAML file.
    x_global, y_global : float
        Global coordinates of the point (meters).

    Returns
    -------
    str
        room_id if found, otherwise "0".
    """
    map_table = _load_map_table(yaml_path)

    for room_id, info in map_table.items():
        try:
            cx, cy = info["center"]
            vertices_local = info["polygon"]
        except KeyError:
            # Skip rooms without polygon definition
            continue

        # Convert local polygon vertices to global frame
        poly_global = [(vx + cx, vy + cy) for vx, vy in vertices_local]

        if point_in_polygon(x_global, y_global, poly_global):
            return str(room_id)

    return "0"


# Optional CLI usage
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Determine which room a global (x, y) coordinate belongs to."
    )
    parser.add_argument("yaml", help="Path to map_table YAML file")
    parser.add_argument("x", type=float, help="Global X coordinate (meters)")
    parser.add_argument("y", type=float, help="Global Y coordinate (meters)")
    args = parser.parse_args()

    room = determine_room(args.yaml, args.x, args.y)
    print(room)
