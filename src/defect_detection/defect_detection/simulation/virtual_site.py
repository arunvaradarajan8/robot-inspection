"""The virtual site a demo mission navigates: geometry plus defects.

Demo mode replaces the depth camera with a site that only exists in the
robot's head, so the mission can run its full loop -
explore, walk to a defect, trigger the X7 - in an empty field with no
structure to see. The site is authored in a YAML file:

    frame_note: coordinates are metres, x forward and y left of the
                robot's pose when the demo launched.

    surfaces:
      - type: wall        # a vertical slab
        x: 8.0            # front face
        y: [-10.0, 10.0]  # lateral extent
        height: 4.0
        thickness: 0.3
      - type: box         # an axis-aligned block
        center: [4.0, -6.0, 0.75]
        size: [1.2, 1.2, 1.5]
      - type: cylinder    # a pillar
        center: [6.0, 5.0]
        radius: 0.35
        height: 3.0
      - type: ground      # a flat patch, mostly for a sane raytrace floor
        x: [-5.0, 12.0]
        y: [-12.0, 12.0]

    defects:
      - class_id: crack
        position: [7.9, -3.0, 1.6]
        size: 0.55
        confidence: 0.88

A site may instead - or additionally - carry a captured cloud:

    cloud_path: scans/pier_3.las    # .las/.laz/.pcd/.ply, relative to
                                    # the YAML file
    cloud_transform:                # optional, applied to that cloud
      translation: [0.0, 0.0, 0.0]
      yaw_deg: 0.0

so a real scan of a structure can drive the demo with the defect list
authored on top of it.
"""

import math
from pathlib import Path

import numpy as np
import yaml


class SiteError(ValueError):
    """The site file is missing something the demo needs."""


class Defect:
    """One planted defect: where it is and what the detector 'sees'."""

    def __init__(self, class_id, position, size, confidence):
        self.class_id = class_id
        self.position = np.asarray(position, dtype=np.float64)
        self.size = float(size)
        self.confidence = float(confidence)


class VirtualSite:
    """Sampled site geometry and its defects, in site coordinates."""

    def __init__(self, points, defects, name='virtual site'):
        self.points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        self.defects = list(defects)
        self.name = name

    def transformed(self, x, y, yaw):
        """Return a copy placed at (x, y, yaw) in another frame."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotation = np.array([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ])
        offset = np.array([x, y, 0.0])
        return VirtualSite(
            self.points @ rotation.T + offset,
            [
                Defect(
                    defect.class_id,
                    rotation @ defect.position + offset,
                    defect.size,
                    defect.confidence,
                )
                for defect in self.defects
            ],
            self.name,
        )


def sweep(
    points,
    origin_x,
    origin_y,
    sensor_range,
    bearing_bin,
    occlusion_depth,
    max_points,
    range_noise=0.0,
    rng=None,
):
    """Points a sensor standing at (origin_x, origin_y) would return.

    Range-limited, then occluded: within each bearing bin only the
    nearest surface and a shallow skin behind it survive. That is what
    makes the demo behave like a mission rather than a replay - the
    occupancy map's raytrace carves free space up to a wall and leaves
    what is behind it unknown, so the frontier planner keeps finding
    somewhere to go instead of declaring the site mapped on sweep one.
    """
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    delta_x = points[:, 0] - origin_x
    delta_y = points[:, 1] - origin_y
    ranges = np.hypot(delta_x, delta_y)
    in_range = (ranges <= sensor_range) & (ranges > 1e-6)
    if not np.any(in_range):
        return np.empty((0, 3))

    points = points[in_range]
    ranges = ranges[in_range]
    bearings = np.arctan2(delta_y[in_range], delta_x[in_range])

    bearing_bin = max(1e-3, bearing_bin)
    bins = np.floor((bearings + math.pi) / bearing_bin).astype(np.int64)
    nearest = np.full(int(np.max(bins)) + 1, np.inf)
    np.minimum.at(nearest, bins, ranges)
    visible = points[ranges <= nearest[bins] + occlusion_depth]

    if range_noise > 0.0 and len(visible):
        rng = rng if rng is not None else np.random.default_rng()
        visible = visible + rng.normal(0.0, range_noise, visible.shape)
    if max_points and len(visible) > max_points:
        # Uniform stride keeps the sweep's spatial spread; the first N
        # points would only be one slice of the site.
        visible = visible[::int(np.ceil(len(visible) / max_points))]
    return visible


def load_site(path, density=1.0, seed=7):
    """Build a VirtualSite from a YAML site description."""
    path = Path(path).expanduser()
    if not path.is_file():
        raise SiteError(f'Site file not found: {path}')
    document = yaml.safe_load(path.read_text()) or {}
    if not isinstance(document, dict):
        raise SiteError(f'{path} must contain a YAML mapping')

    rng = np.random.default_rng(seed)
    clouds = []
    for surface in document.get('surfaces') or []:
        clouds.append(sample_surface(surface, rng, density))

    cloud_path = document.get('cloud_path')
    if cloud_path:
        cloud = read_cloud_file(
            (path.parent / str(cloud_path)).resolve(),
            document.get('cloud_max_points', 400000),
            rng,
        )
        clouds.append(place_cloud(cloud, document.get('cloud_transform') or {}))

    if not clouds:
        raise SiteError(
            f'{path} defines no geometry: add "surfaces" or "cloud_path"'
        )

    points = np.vstack(clouds)
    noise = float(document.get('surface_noise_m', 0.01))
    if noise > 0.0:
        points = points + rng.normal(0.0, noise, points.shape)

    return VirtualSite(
        points,
        [parse_defect(entry) for entry in document.get('defects') or []],
        document.get('name') or path.stem,
    )


def parse_defect(entry):
    if not isinstance(entry, dict) or 'position' not in entry:
        raise SiteError(f'Defect entry needs a "position": {entry!r}')
    position = entry['position']
    if len(position) != 3:
        raise SiteError(f'Defect position must be [x, y, z]: {position!r}')
    return Defect(
        class_id=str(entry.get('class_id', 'defect')),
        position=position,
        size=float(entry.get('size', 0.4)),
        confidence=float(entry.get('confidence', 0.85)),
    )


# ---- surface sampling ---------------------------------------------------
#
# Every surface is sampled at roughly POINTS_PER_SQUARE_METRE so a site
# built from a handful of primitives lands in the same density range the
# occupancy map is tuned for.

POINTS_PER_SQUARE_METRE = 400.0


def sample_count(area, density):
    return max(64, int(area * POINTS_PER_SQUARE_METRE * density))


def sample_surface(surface, rng, density):
    if not isinstance(surface, dict):
        raise SiteError(f'Each surface must be a mapping: {surface!r}')
    kind = str(surface.get('type', '')).lower()
    if kind == 'wall':
        return sample_wall(surface, rng, density)
    if kind == 'box':
        return sample_box(surface, rng, density)
    if kind == 'cylinder':
        return sample_cylinder(surface, rng, density)
    if kind == 'ground':
        return sample_ground(surface, rng, density)
    raise SiteError(
        f'Unknown surface type {kind!r}; expected wall, box, cylinder, ground'
    )


def span(surface, key):
    values = surface.get(key)
    if values is None or len(values) != 2:
        raise SiteError(f'Surface needs {key}: [min, max]: {surface!r}')
    low, high = float(values[0]), float(values[1])
    if high <= low:
        raise SiteError(f'Surface {key} must be increasing: {surface!r}')
    return low, high


def sample_wall(surface, rng, density):
    """A vertical slab. Only the face toward the site origin is sampled;
    the far side never becomes visible and would only cost points."""
    y_min, y_max = span(surface, 'y')
    height = float(surface.get('height', 3.0))
    thickness = float(surface.get('thickness', 0.3))
    x = float(surface['x']) if 'x' in surface else None
    if x is None:
        raise SiteError(f'A wall needs its face position x: {surface!r}')
    count = sample_count((y_max - y_min) * height, density)
    return np.column_stack((
        rng.uniform(x, x + thickness, count),
        rng.uniform(y_min, y_max, count),
        rng.uniform(0.0, height, count),
    ))


def sample_box(surface, rng, density):
    center = np.asarray(surface.get('center', (0.0, 0.0, 0.0)), dtype=float)
    if len(center) == 2:
        center = np.append(center, 0.0)
    size = np.asarray(surface.get('size', (1.0, 1.0, 1.0)), dtype=float)
    half = size / 2.0
    # Sample the four vertical faces and the top; the underside is never
    # seen by a sensor standing on the same ground.
    faces = []
    for axis, extent in ((0, half[0]), (1, half[1])):
        other = 1 - axis
        count = sample_count(size[other] * size[2], density)
        for sign in (-1.0, 1.0):
            face = np.empty((count, 3))
            face[:, axis] = center[axis] + sign * extent
            face[:, other] = rng.uniform(
                center[other] - half[other], center[other] + half[other], count
            )
            face[:, 2] = rng.uniform(center[2] - half[2], center[2] + half[2], count)
            faces.append(face)
    top_count = sample_count(size[0] * size[1], density)
    faces.append(np.column_stack((
        rng.uniform(center[0] - half[0], center[0] + half[0], top_count),
        rng.uniform(center[1] - half[1], center[1] + half[1], top_count),
        np.full(top_count, center[2] + half[2]),
    )))
    return np.vstack(faces)


def sample_cylinder(surface, rng, density):
    center = np.asarray(surface.get('center', (0.0, 0.0)), dtype=float)
    radius = float(surface.get('radius', 0.35))
    height = float(surface.get('height', 3.0))
    base = float(surface.get('base_z', 0.0))
    count = sample_count(2.0 * math.pi * radius * height, density)
    angle = rng.uniform(0.0, 2.0 * math.pi, count)
    return np.column_stack((
        center[0] + radius * np.cos(angle),
        center[1] + radius * np.sin(angle),
        rng.uniform(base, base + height, count),
    ))


def sample_ground(surface, rng, density):
    x_min, x_max = span(surface, 'x')
    y_min, y_max = span(surface, 'y')
    z = float(surface.get('z', 0.0))
    # Ground is sampled sparsely: it carries no obstacle information and
    # the occupancy map filters it out by height anyway. It exists so the
    # site reads correctly in RViz.
    count = sample_count((x_max - x_min) * (y_max - y_min), density * 0.02)
    return np.column_stack((
        rng.uniform(x_min, x_max, count),
        rng.uniform(y_min, y_max, count),
        np.full(count, z),
    ))


# ---- captured clouds ----------------------------------------------------


def read_cloud_file(path, max_points, rng):
    """Read an Nx3 array from a LAS/LAZ, PCD, or PLY file."""
    if not path.is_file():
        raise SiteError(f'cloud_path does not exist: {path}')
    suffix = path.suffix.lower()
    if suffix in ('.las', '.laz'):
        points = read_las(path)
    elif suffix == '.pcd':
        points = read_pcd(path)
    elif suffix == '.ply':
        points = read_ply(path)
    else:
        raise SiteError(
            f'Unsupported cloud format {suffix!r}; use .las, .laz, .pcd or .ply'
        )
    if max_points and len(points) > max_points:
        keep = rng.choice(len(points), size=int(max_points), replace=False)
        points = points[np.sort(keep)]
    return points


def read_las(path):
    try:
        import laspy
    except ImportError as error:
        raise SiteError(
            'laspy is required to load a LAS/LAZ site cloud: pip install laspy'
        ) from error
    with laspy.open(str(path)) as reader:
        las = reader.read()
    return np.column_stack((
        np.asarray(las.x, dtype=np.float64),
        np.asarray(las.y, dtype=np.float64),
        np.asarray(las.z, dtype=np.float64),
    ))


def read_pcd(path):
    """Read an ASCII or binary (uncompressed) PCD, xyz fields only."""
    with path.open('rb') as handle:
        fields = []
        sizes = []
        types = []
        counts = []
        point_count = 0
        data_format = 'ascii'
        while True:
            line = handle.readline()
            if not line:
                raise SiteError(f'{path} ended before its DATA section')
            tokens = line.decode('ascii', 'replace').split()
            if not tokens or tokens[0].startswith('#'):
                continue
            key = tokens[0].upper()
            if key == 'FIELDS':
                fields = tokens[1:]
            elif key == 'SIZE':
                sizes = [int(value) for value in tokens[1:]]
            elif key == 'TYPE':
                types = tokens[1:]
            elif key == 'COUNT':
                counts = [int(value) for value in tokens[1:]]
            elif key == 'POINTS':
                point_count = int(tokens[1])
            elif key == 'DATA':
                data_format = tokens[1].lower()
                break

        missing = [name for name in ('x', 'y', 'z') if name not in fields]
        if missing:
            raise SiteError(f'{path} has no {missing} field')
        if not counts:
            counts = [1] * len(fields)

        if data_format == 'ascii':
            rows = np.loadtxt(handle, dtype=np.float64, ndmin=2)
            index = [fields.index(name) for name in ('x', 'y', 'z')]
            return rows[:, index]
        if data_format != 'binary':
            raise SiteError(
                f'{path} uses DATA {data_format}; only ascii and binary are '
                'supported (convert compressed PCDs first)'
            )
        kind = {'F': 'f', 'U': 'u', 'I': 'i'}
        dtype = np.dtype([
            (name, f'{kind.get(type_code.upper(), "f")}{size}', count)
            if count > 1
            else (name, f'{kind.get(type_code.upper(), "f")}{size}')
            for name, size, type_code, count in zip(fields, sizes, types, counts)
        ])
        raw = np.frombuffer(handle.read(dtype.itemsize * point_count), dtype=dtype)
        return np.column_stack((
            raw['x'].astype(np.float64),
            raw['y'].astype(np.float64),
            raw['z'].astype(np.float64),
        ))


def read_ply(path):
    """Read an ASCII or binary-little-endian PLY, vertex xyz only."""
    with path.open('rb') as handle:
        if handle.readline().strip() != b'ply':
            raise SiteError(f'{path} is not a PLY file')
        data_format = None
        properties = []
        vertex_count = 0
        in_vertex = False
        while True:
            line = handle.readline()
            if not line:
                raise SiteError(f'{path} has no end_header')
            tokens = line.decode('ascii', 'replace').split()
            if not tokens:
                continue
            if tokens[0] == 'format':
                data_format = tokens[1]
            elif tokens[0] == 'element':
                in_vertex = tokens[1] == 'vertex'
                if in_vertex:
                    vertex_count = int(tokens[2])
            elif tokens[0] == 'property' and in_vertex:
                properties.append((tokens[-1], tokens[1]))
            elif tokens[0] == 'end_header':
                break

        names = [name for name, _ in properties]
        missing = [name for name in ('x', 'y', 'z') if name not in names]
        if missing:
            raise SiteError(f'{path} vertices have no {missing} property')

        if data_format == 'ascii':
            rows = np.loadtxt(
                handle, dtype=np.float64, max_rows=vertex_count, ndmin=2
            )
            index = [names.index(name) for name in ('x', 'y', 'z')]
            return rows[:, index]
        if data_format != 'binary_little_endian':
            raise SiteError(
                f'{path} uses format {data_format}; only ascii and '
                'binary_little_endian are supported'
            )
        ply_types = {
            'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8',
            'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
            'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
            'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
        }
        dtype = np.dtype([
            (name, '<' + ply_types.get(type_name, 'f4'))
            for name, type_name in properties
        ])
        raw = np.frombuffer(handle.read(dtype.itemsize * vertex_count), dtype=dtype)
        return np.column_stack((
            raw['x'].astype(np.float64),
            raw['y'].astype(np.float64),
            raw['z'].astype(np.float64),
        ))


def place_cloud(points, transform):
    """Apply a site file's optional cloud_transform to a captured cloud."""
    yaw = math.radians(float(transform.get('yaw_deg', 0.0)))
    translation = np.asarray(
        transform.get('translation', (0.0, 0.0, 0.0)), dtype=float
    )
    if len(translation) != 3:
        raise SiteError('cloud_transform translation must be [x, y, z]')
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rotation = np.array([
        [cos_yaw, -sin_yaw, 0.0],
        [sin_yaw, cos_yaw, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return points @ rotation.T + translation
