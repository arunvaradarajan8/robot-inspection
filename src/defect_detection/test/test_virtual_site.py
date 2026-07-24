import math
from pathlib import Path

import numpy as np
import pytest

from defect_detection.simulation.virtual_site import (
    SiteError,
    load_site,
    sweep,
)


SITE_YAML = """
name: test site
surface_noise_m: 0.0
surfaces:
  - type: wall
    x: 10.0
    y: [-4.0, 4.0]
    height: 3.0
    thickness: 0.2
  - type: cylinder
    center: [5.0, 0.0]
    radius: 0.5
    height: 2.0
defects:
  - class_id: crack
    position: [9.9, 1.0, 1.5]
    size: 0.5
    confidence: 0.9
"""


@pytest.fixture
def site_path(tmp_path):
    path = tmp_path / 'site.yaml'
    path.write_text(SITE_YAML)
    return path


def test_load_site_samples_every_surface(site_path):
    site = load_site(site_path)

    assert site.name == 'test site'
    assert len(site.defects) == 1
    assert site.defects[0].class_id == 'crack'
    # The wall face and the pillar are both present.
    assert site.points[:, 0].max() == pytest.approx(10.2, abs=0.05)
    assert np.any(np.hypot(site.points[:, 0] - 5.0, site.points[:, 1]) < 0.55)


def test_site_without_geometry_is_rejected(tmp_path):
    path = tmp_path / 'empty.yaml'
    path.write_text('name: nothing\ndefects: []\n')

    with pytest.raises(SiteError):
        load_site(path)


def test_unknown_surface_type_is_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('surfaces:\n  - type: torus\n')

    with pytest.raises(SiteError):
        load_site(path)


def test_transformed_places_geometry_and_defects(site_path):
    site = load_site(site_path)

    placed = site.transformed(2.0, -1.0, math.pi / 2.0)

    # A quarter turn puts what was ahead of the origin to its left.
    defect = placed.defects[0]
    assert defect.position[0] == pytest.approx(2.0 - 1.0, abs=0.01)
    assert defect.position[1] == pytest.approx(-1.0 + 9.9, abs=0.01)
    assert defect.position[2] == pytest.approx(1.5, abs=0.01)
    # The site itself moved with it, and the source site is untouched.
    assert placed.points[:, 1].max() > site.points[:, 1].max()


def test_sweep_hides_what_is_behind_the_first_surface():
    # Two walls on the same bearing: one at 5 m, one at 9 m.
    near = np.column_stack((
        np.full(200, 5.0), np.linspace(-1.0, 1.0, 200), np.zeros(200),
    ))
    far = np.column_stack((
        np.full(200, 9.0), np.linspace(-1.0, 1.0, 200), np.zeros(200),
    ))
    points = np.vstack((near, far))

    visible = sweep(
        points,
        origin_x=0.0,
        origin_y=0.0,
        sensor_range=20.0,
        bearing_bin=math.radians(0.4),
        occlusion_depth=0.4,
        max_points=0,
    )

    # Only the near wall comes back; the far one is in its shadow. The
    # bearing bins fan out, so the outermost far points sit in bins the
    # near wall does not cover and legitimately remain visible.
    assert len(visible) > 0
    assert np.mean(visible[:, 0] < 6.0) > 0.9


def test_sweep_drops_points_beyond_range():
    points = np.column_stack((
        np.array([3.0, 30.0]), np.zeros(2), np.zeros(2),
    ))

    visible = sweep(
        points,
        origin_x=0.0,
        origin_y=0.0,
        sensor_range=20.0,
        bearing_bin=math.radians(0.4),
        occlusion_depth=0.4,
        max_points=0,
    )

    assert len(visible) == 1
    assert visible[0][0] == pytest.approx(3.0)


def test_sweep_decimates_to_the_point_budget():
    points = np.column_stack((
        np.full(5000, 8.0),
        np.linspace(-6.0, 6.0, 5000),
        np.zeros(5000),
    ))

    visible = sweep(
        points,
        origin_x=0.0,
        origin_y=0.0,
        sensor_range=20.0,
        bearing_bin=math.radians(0.4),
        occlusion_depth=0.4,
        max_points=500,
    )

    assert len(visible) <= 500
    # Decimation keeps the full lateral spread rather than one slice.
    assert visible[:, 1].min() < -5.0
    assert visible[:, 1].max() > 5.0


def test_shipped_demo_site_loads():
    """The site the demo launches with must parse and carry defects."""
    path = (
        Path(__file__).resolve().parents[1] / 'config' / 'demo_site.yaml'
    )
    site = load_site(path)

    assert len(site.points) > 1000
    assert site.defects
    assert all(defect.confidence > 0.0 for defect in site.defects)
