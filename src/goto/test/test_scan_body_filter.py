import math

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from goto.mapping.scan_body_filter import ScanBodyFilterNode, remove_body_points

# A scan with one ray per degree, like the real one (360 rays, first at -180 degrees)
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = math.radians(1.0)
BOX = dict(half_length_m=0.5, half_width_m=0.3)


def scan_with(points):
    """Ranges (all inf) with the given {angle_deg: range_m} set."""
    ranges = np.full(360, math.inf, dtype=np.float32)
    for angle_deg, range_m in points.items():
        ranges[int(round(angle_deg)) + 180] = range_m
    return ranges


def filtered(points):
    return remove_body_points(scan_with(points), ANGLE_MIN, ANGLE_INCREMENT, **BOX)


def test_the_rear_leg_points_measured_during_the_stall_are_removed():
    # 0.41 m at -146 .. -158 degrees: the rear-right leg (x -0.34 .. -0.38, y -0.15 .. -0.23)
    leg = {angle: 0.41 for angle in range(-158, -145)}
    assert np.all(np.isinf(filtered(leg)))


def test_a_wall_beside_the_robot_is_kept():
    # 1 m to the left (90 degrees) and 0.8 m behind (-180 degrees = index 0): outside the box
    result = filtered({90: 1.0, -180: 0.8})
    assert result[90 + 180] == pytest.approx(1.0)
    assert result[0] == pytest.approx(0.8)


def test_the_box_is_checked_by_x_and_y_not_by_distance():
    # 0.55 m straight ahead is outside the box (x 0.55 > 0.5) although closer than the corner of the box
    assert filtered({0: 0.55})[180] == pytest.approx(0.55)
    # 0.45 m straight ahead is inside
    assert np.isinf(filtered({0: 0.45})[180])
    # 0.35 m to the side is outside (y 0.35 > 0.3), 0.25 m is inside
    assert filtered({90: 0.35})[270] == pytest.approx(0.35)
    assert np.isinf(filtered({90: 0.25})[270])


def test_rays_without_a_return_stay_as_they_are():
    ranges = np.array([math.inf, math.nan, 2.0], dtype=np.float32)
    result = remove_body_points(ranges, 0.0, 0.1, **BOX)
    assert math.isinf(result[0])
    assert math.isnan(result[1])
    assert result[2] == pytest.approx(2.0)


def test_the_input_is_not_changed():
    ranges = scan_with({-150: 0.41})
    remove_body_points(ranges, ANGLE_MIN, ANGLE_INCREMENT, **BOX)
    assert ranges[30] == pytest.approx(0.41)


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append(message)


def make_message(frame_id, points):
    message = LaserScan()
    message.header.frame_id = frame_id
    message.angle_min, message.angle_increment = ANGLE_MIN, ANGLE_INCREMENT
    message.ranges = scan_with(points).tolist()
    return message


@pytest.fixture
def filter_node(rclpy_context):
    node = ScanBodyFilterNode()
    node._publisher = FakePublisher()
    yield node
    node.node.destroy_node()


def test_the_node_publishes_the_scan_without_the_body(filter_node):
    filter_node._on_scan(make_message("base_footprint", {-150: 0.41, 90: 1.0}))

    ranges = filter_node._publisher.sent[0].ranges
    assert math.isinf(ranges[30])  # the leg
    assert ranges[270] == pytest.approx(1.0)  # the wall


def test_a_scan_in_another_frame_is_passed_on_unchanged(filter_node):
    filter_node._on_scan(make_message("velodyne", {-150: 0.41}))

    assert filter_node._publisher.sent[0].ranges[30] == pytest.approx(0.41)
