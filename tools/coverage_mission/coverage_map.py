"""Accumulating 2D coverage map built from Trimble X7 LAS/LAZ scans.

Pure numpy; no ROS. Every scan is merged into one persistent occupancy
grid so frontiers (free cells that border unscanned cells) only remain
at genuinely unscanned terrain.
"""
import math

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def read_las_xyz(path, max_points=500000):
    """Read x/y/z from a LAS/LAZ file, downsampled to max_points."""
    try:
        import laspy
    except ImportError as error:
        raise RuntimeError(
            'Install laspy to read Trimble LAS/LAZ scans: '
            'python3 -m pip install laspy lazrs'
        ) from error

    las = laspy.read(path)
    point_count = len(las.x)
    if point_count == 0:
        return np.empty((0, 3), dtype=np.float64)

    stride = max(1, int(np.ceil(point_count / max_points)))
    x = np.asarray(las.x[::stride], dtype=np.float64)
    y = np.asarray(las.y[::stride], dtype=np.float64)
    z = np.asarray(las.z[::stride], dtype=np.float64)
    return np.column_stack((x, y, z))


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x = x0
    y = y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


class CoverageMap:

    def __init__(self, resolution=0.10, padding_m=2.0, min_z=-0.25,
                 max_z=1.20, max_range_m=80.0):
        if resolution <= 0.0:
            raise ValueError('resolution must be greater than zero')
        if max_z <= min_z:
            raise ValueError('max_z must be greater than min_z')
        self.resolution = resolution
        self.padding = padding_m
        self.min_z = min_z
        self.max_z = max_z
        self.max_range = max_range_m
        self.min_x = 0.0
        self.min_y = 0.0
        self.grid = None

    def snap_down(self, value):
        return math.floor(value / self.resolution) * self.resolution

    def snap_up(self, value):
        return math.ceil(value / self.resolution) * self.resolution

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.min_x) / self.resolution)),
            int(math.floor((y - self.min_y) / self.resolution)),
        )

    def cell_to_world(self, cell_x, cell_y):
        return (
            self.min_x + (cell_x + 0.5) * self.resolution,
            self.min_y + (cell_y + 0.5) * self.resolution,
        )

    def cell_in_bounds(self, cell):
        height, width = self.grid.shape
        return 0 <= cell[0] < width and 0 <= cell[1] < height

    def add_scan(self, points, origin_xy):
        """Merge one scan (Nx3 map-frame points) taken from origin_xy."""
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        origin_x, origin_y = float(origin_xy[0]), float(origin_xy[1])
        if points.size == 0:
            return 0

        z_mask = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)
        range_mask = np.hypot(
            points[:, 0] - origin_x,
            points[:, 1] - origin_y,
        ) <= self.max_range
        obstacle_points = points[z_mask & range_mask]
        if len(obstacle_points) == 0:
            return 0

        # Bounds snapped to the cell grid so cells stay aligned when the
        # map grows with later stations.
        min_x = self.snap_down(
            min(np.min(obstacle_points[:, 0]), origin_x) - self.padding
        )
        max_x = self.snap_up(
            max(np.max(obstacle_points[:, 0]), origin_x) + self.padding
        )
        min_y = self.snap_down(
            min(np.min(obstacle_points[:, 1]), origin_y) - self.padding
        )
        max_y = self.snap_up(
            max(np.max(obstacle_points[:, 1]), origin_y) + self.padding
        )

        if self.grid is not None:
            min_x = min(min_x, self.min_x)
            min_y = min(min_y, self.min_y)
            max_x = max(max_x, self.min_x + self.grid.shape[1] * self.resolution)
            max_y = max(max_y, self.min_y + self.grid.shape[0] * self.resolution)

        width = max(1, int(round((max_x - min_x) / self.resolution)))
        height = max(1, int(round((max_y - min_y) / self.resolution)))
        grid = np.full((height, width), UNKNOWN, dtype=np.int8)

        if self.grid is not None:
            offset_x = int(round((self.min_x - min_x) / self.resolution))
            offset_y = int(round((self.min_y - min_y) / self.resolution))
            grid[
                offset_y:offset_y + self.grid.shape[0],
                offset_x:offset_x + self.grid.shape[1],
            ] = self.grid

        self.min_x = min_x
        self.min_y = min_y
        self.grid = grid

        origin_cell = self.world_to_cell(origin_x, origin_y)
        occupied_cells = set()
        for point in obstacle_points:
            end_cell = self.world_to_cell(point[0], point[1])
            if not self.cell_in_bounds(end_cell):
                continue
            for cell in bresenham(
                origin_cell[0],
                origin_cell[1],
                end_cell[0],
                end_cell[1],
            ):
                if not self.cell_in_bounds(cell):
                    break
                # Never punch holes into walls seen from an earlier
                # station; the 2D projection of a ray may cross them.
                if grid[cell[1], cell[0]] != OCCUPIED:
                    grid[cell[1], cell[0]] = FREE
            occupied_cells.add(end_cell)

        for x, y in occupied_cells:
            grid[y, x] = OCCUPIED
        return len(obstacle_points)

    def frontiers(self):
        """All free cells bordering unknown, as world (x, y) points."""
        if self.grid is None:
            return []
        grid = self.grid
        free = grid == FREE
        unknown_neighbor = np.zeros_like(free)
        unknown = grid == UNKNOWN
        # Shift the unknown mask in the 8 neighbor directions.
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.zeros_like(unknown)
                src_y = slice(max(0, dy), grid.shape[0] + min(0, dy))
                dst_y = slice(max(0, -dy), grid.shape[0] + min(0, -dy))
                src_x = slice(max(0, dx), grid.shape[1] + min(0, dx))
                dst_x = slice(max(0, -dx), grid.shape[1] + min(0, -dx))
                shifted[dst_y, dst_x] = unknown[src_y, src_x]
                unknown_neighbor |= shifted
        cells = np.argwhere(free & unknown_neighbor)
        return [self.cell_to_world(cx, cy) for cy, cx in cells]

    def select_frontier(self, robot_xy, min_distance_m=1.0,
                        excluded=(), exclusion_radius_m=1.0):
        """Pick the farthest reachable frontier from the robot.

        Farthest-first maximizes new terrain per X7 station. `excluded`
        holds previously failed goals to skip.
        """
        best = None
        best_score = -math.inf
        for x, y in self.frontiers():
            distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if distance < min_distance_m:
                continue
            if any(
                math.hypot(x - ex, y - ey) < exclusion_radius_m
                for ex, ey in excluded
            ):
                continue
            if distance > best_score:
                best_score = distance
                best = (x, y)
        return best

    def coverage_stats(self):
        if self.grid is None:
            return {'free': 0, 'occupied': 0, 'unknown': 0, 'covered_m2': 0.0}
        free = int(np.count_nonzero(self.grid == FREE))
        occupied = int(np.count_nonzero(self.grid == OCCUPIED))
        unknown = int(np.count_nonzero(self.grid == UNKNOWN))
        cell_area = self.resolution * self.resolution
        return {
            'free': free,
            'occupied': occupied,
            'unknown': unknown,
            'covered_m2': (free + occupied) * cell_area,
        }

    def save_pgm(self, path):
        """Write the map as a PGM image (white=free, black=wall, gray=unknown)."""
        if self.grid is None:
            return
        image = np.full(self.grid.shape, 127, dtype=np.uint8)
        image[self.grid == FREE] = 255
        image[self.grid == OCCUPIED] = 0
        flipped = np.flipud(image)  # +y up
        with open(path, 'wb') as stream:
            header = f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'
            stream.write(header.encode('ascii'))
            stream.write(flipped.tobytes())
