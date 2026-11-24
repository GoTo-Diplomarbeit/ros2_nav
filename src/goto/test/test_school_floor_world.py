"""Checks worlds/school_floor.sdf without starting Gazebo: it parses, keeps the world name, and a robot can
drive from the spawn point into every room."""

import io
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage

WORLD_FILE = Path(__file__).resolve().parents[3] / "worlds" / "school_floor.sdf"
RESOLUTION = 0.05
ROBOT_RADIUS = 0.35  # half the Go2's length, so a door it passes by this test is wide enough in any heading
LIDAR_SLICE = (0.1, 0.5)  # heights of the 2D scan: only shapes reaching into it become obstacles

# One point inside each room (x, y). The toilets and the porches test the inner wall and the entrances.
ROOMS = {
    "class A": (-8.0, 3.0),
    "class B": (0.0, 3.2),
    "library": (9.0, 3.0),
    "storage (0.8 m door)": (-10.0, -5.0),
    "toilets, behind the inner wall": (-6.0, -8.0),
    "lobby": (0.0, -5.0),
    "canteen": (9.0, -3.0),
    "south porch": (0.0, -10.5),
    "east porch": (15.0, 0.0),
}


def _numbers(text):
    return [float(v) for v in text.split()]


def read_shapes(world_file):
    """All collision shapes of the world as (kind, size, x, y, yaw, z_min, z_max). Models and links have no pose."""
    shapes = []
    for collision in ET.parse(world_file).getroot().iter("collision"):
        pose = _numbers(collision.findtext("pose") or "0 0 0 0 0 0")
        geometry = collision.find("geometry")
        if geometry.find("plane") is not None:
            continue  # the ground
        kind = geometry[0].tag
        if kind == "box":
            size = _numbers(geometry.findtext("box/size"))
            height = size[2]
        elif kind == "cylinder":
            radius, height = float(geometry.findtext("cylinder/radius")), float(geometry.findtext("cylinder/length"))
            size = [radius, radius, height]
        elif kind == "sphere":
            radius = float(geometry.findtext("sphere/radius"))
            size, height = [radius, radius, radius], 2 * radius
        else:  # ellipsoid
            radii = _numbers(geometry.findtext("ellipsoid/radii"))
            size, height = radii, 2 * radii[2]
        shapes.append((kind, size, pose[0], pose[1], pose[5], pose[2] - height / 2, pose[2] + height / 2))
    return shapes


def obstacle_grid(shapes, x_range=(-13.0, 17.5), y_range=(-12.5, 10.0)):
    """True where a shape that the lidar slice can see covers the cell centre. Returns grid and cell-centre arrays."""
    xs = np.arange(x_range[0], x_range[1], RESOLUTION)
    ys = np.arange(y_range[0], y_range[1], RESOLUTION)
    gx, gy = np.meshgrid(xs, ys)  # rows = y
    blocked = np.zeros(gx.shape, dtype=bool)
    for kind, size, x, y, yaw, z_min, z_max in shapes:
        if z_max < LIDAR_SLICE[0] or z_min > LIDAR_SLICE[1]:
            continue  # a lintel or the roof: above the scan
        # cell centres in the shape's own frame (rotated by -yaw)
        dx, dy = gx - x, gy - y
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if kind == "box":
            blocked |= (np.abs(u) <= size[0] / 2) & (np.abs(v) <= size[1] / 2)
        else:  # cylinder, sphere and ellipsoid are all ellipses seen from above
            blocked |= (u / size[0]) ** 2 + (v / size[1]) ** 2 <= 1
    return blocked, xs, ys


@pytest.fixture(scope="module")
def world():
    return ET.parse(WORLD_FILE).getroot()


@pytest.fixture(scope="module")
def reachable():
    """Boolean grid of the cells where the robot's centre can be, reachable from the spawn point."""
    blocked, xs, ys = obstacle_grid(read_shapes(WORLD_FILE))
    radius_cells = int(round(ROBOT_RADIUS / RESOLUTION))
    disk = np.hypot(*np.mgrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]) <= radius_cells
    free = ~ndimage.binary_dilation(blocked, structure=disk)
    labels, _ = ndimage.label(free)
    spawn_label = labels[cell(ys, 0.0), cell(xs, 0.0)]
    assert spawn_label != 0, "the spawn point (0, 0) is blocked"
    return labels == spawn_label, xs, ys


def cell(axis, value):
    return int(round((value - axis[0]) / RESOLUTION))


def test_world_keeps_the_name_default(world):
    # Gazebo's internal topics are /world/default/..., like in the simulator's own worlds
    assert world.find("world").get("name") == "default"


def test_roof_is_a_separate_model_above_the_scan(world):
    roof = [m for m in world.iter("model") if m.get("name") == "school_roof"]
    assert len(roof) == 1
    assert all(s[5] >= 3.0 for s in read_shapes_of(roof[0]))


def read_shapes_of(model):
    """read_shapes for one <model> element."""
    buffer = io.BytesIO()
    ET.ElementTree(model).write(buffer)
    buffer.seek(0)
    return read_shapes(buffer)


@pytest.mark.parametrize("room", ROOMS)
def test_robot_can_reach_every_room(reachable, room):
    grid, xs, ys = reachable
    x, y = ROOMS[room]
    assert grid[cell(ys, y), cell(xs, x)], f"{room} at ({x}, {y}) is not reachable from the spawn point"


def test_robot_cannot_leave_the_building(reachable):
    grid, xs, ys = reachable
    # The reachable area ends at the porch walls: nothing is reachable outside them
    assert not grid[cell(ys, 0.0), cell(xs, -13.0)]  # west of the building
    assert not grid[cell(ys, -12.2), cell(xs, 0.0)]  # behind the south porch
    assert not grid[cell(ys, 0.0), cell(xs, 17.0)]  # behind the east porch


def test_the_physics_step_and_update_rate_match_the_real_time_factor(world):
    # Gazebo wants step x rate = factor. 0.001 s steps = 1000 Hz /clock, the simulator's usual value: with 0.002 the
    # robot may rock and flip.
    physics = world.find("world/physics")
    step = float(physics.findtext("max_step_size"))
    rate = float(physics.findtext("real_time_update_rate"))
    factor = float(physics.findtext("real_time_factor"))
    assert step * rate == pytest.approx(factor)
    assert step == pytest.approx(0.001)
