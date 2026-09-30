"""Step 6: grid map built from ultrasonic readings.

Coordinates: x = cm to the right of the map origin, y = cm forward.
Heading: 0 = forward (+y), 90 = right (+x), 180 = back, 270 = left.
The array is indexed [y, x]. 1 = obstacle, 0 = free.
"""
import math
import numpy as np

CELL_CM = 10            # size of one grid cell
GRID_W, GRID_H = 40, 40 # 40 x 40 cells = 4 m x 4 m
MAX_RANGE_CM = 100      # ignore readings farther than this (they get noisy)
CLEARANCE = 1           # cells of padding added around each obstacle
INTERP_MAX_GAP_CM = 15  # connect two consecutive hits if closer than this


class GridMap:
    def __init__(self):
        self.raw = np.zeros((GRID_H, GRID_W), dtype=np.uint8)

    def to_cell(self, x_cm, y_cm):
        return int(x_cm // CELL_CM), int(y_cm // CELL_CM)

    def _mark(self, x_cm, y_cm):
        cx, cy = self.to_cell(x_cm, y_cm)
        if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
            self.raw[cy, cx] = 1

    def _line(self, p1, p2):
        """Fill the cells between two hit points (interpolation)."""
        steps = max(1, int(math.dist(p1, p2) // (CELL_CM / 2)))
        for i in range(steps + 1):
            t = i / steps
            self._mark(p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))

    def add_scan(self, pose, readings):
        """pose = (x_cm, y_cm, heading_deg); readings = [(sensor_angle_deg, dist_cm), ...]"""
        x, y, heading = pose
        prev = None
        for angle, dist in readings:
            if dist is None or dist <= 0 or dist > MAX_RANGE_CM:
                prev = None
                continue
            theta = math.radians(heading + angle)
            hit = (x + dist * math.sin(theta), y + dist * math.cos(theta))
            self._mark(*hit)
            if prev is not None and math.dist(prev, hit) < INTERP_MAX_GAP_CM:
                self._line(prev, hit)
            prev = hit

    def padded(self):
        """Map with CLEARANCE cells added around every obstacle (used for planning)."""
        out = self.raw.copy()
        ys, xs = np.nonzero(self.raw)
        for y, x in zip(ys, xs):
            out[max(0, y - CLEARANCE):y + CLEARANCE + 1,
                max(0, x - CLEARANCE):x + CLEARANCE + 1] = 1
        return out

    def show(self, car=None, goal=None, path=None):
        """Print the map with forward at the top. # obstacle, + clearance, * path, C car, G goal."""
        pad = self.padded()
        path = set(path or [])
        for y in range(GRID_H - 1, -1, -1):
            row = ""
            for x in range(GRID_W):
                c = (x, y)
                if c == car: row += "C"
                elif c == goal: row += "G"
                elif self.raw[y, x]: row += "#"
                elif pad[y, x]: row += "+"
                elif c in path: row += "*"
                else: row += "."
            print(row)
        print()
