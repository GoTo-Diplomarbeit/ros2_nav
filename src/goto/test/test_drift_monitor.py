import math

import pytest
import rclpy

from goto.mapping.drift_monitor import DriftMonitorNode, align, align_and_error, summarize, tilt_of, wrap_angle, yaw_of


def test_wrap_angle_keeps_angles_between_minus_pi_and_pi():
    assert wrap_angle(math.radians(359)) == pytest.approx(math.radians(-1))
    assert wrap_angle(math.radians(-181)) == pytest.approx(math.radians(179))
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_yaw_of_a_quaternion():
    half = math.radians(90) / 2
    assert yaw_of(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(math.radians(90))
    assert yaw_of(0.0, 0.0, 0.0, 1.0) == 0.0


def test_an_estimate_that_follows_the_real_pose_has_no_error_whatever_its_frame():
    # The estimate lives in another frame: 5 m further on and turned by 90 degrees. Laid over the start, it matches.
    real_start, estimate_start = (2.0, 1.0, 0.0), (5.0, 5.0, math.radians(90))
    real = (4.0, 1.0, 0.0)  # drove 2 m ahead
    estimate = (5.0, 7.0, math.radians(90))  # the same trip as seen in its frame: 2 m along its x... turned: along +y

    position_error, heading_error = align_and_error(real, estimate, real_start, estimate_start)

    assert position_error == pytest.approx(0.0, abs=1e-9)
    assert heading_error == pytest.approx(0.0, abs=1e-9)


def test_align_puts_the_estimate_in_the_real_frame_starting_where_the_real_robot_started():
    real_start, estimate_start = (2.0, 1.0, 0.0), (5.0, 5.0, math.radians(90))

    assert align(estimate_start, real_start, estimate_start) == pytest.approx((2.0, 1.0, 0.0))
    # the estimate moved 2 m along its own x axis, turned by 90 degrees to the real frame: that is 2 m along y there
    moved = align((7.0, 5.0, math.radians(90)), real_start, estimate_start)
    assert moved == pytest.approx((2.0, -1.0, 0.0), abs=1e-9)


def test_a_position_error_is_the_distance_between_the_aligned_estimate_and_the_real_pose():
    start = (0.0, 0.0, 0.0)

    position_error, heading_error = align_and_error((3.0, 0.0, 0.0), (2.0, 0.0, 0.0), start, start)  # 1 m short

    assert position_error == pytest.approx(1.0)
    assert heading_error == pytest.approx(0.0)


def test_a_heading_error_is_signed_and_wraps():
    start = (0.0, 0.0, math.radians(170))

    # the estimate turned 20 degrees further left than the real robot, across the +-180 degree line
    _, heading_error = align_and_error((0.0, 0.0, math.radians(170)), (0.0, 0.0, math.radians(-170)), start, start)
    _, other_way = align_and_error((0.0, 0.0, math.radians(-170)), (0.0, 0.0, math.radians(170)), start, start)

    assert math.degrees(heading_error) == pytest.approx(20.0)
    assert math.degrees(other_way) == pytest.approx(-20.0)


def test_a_wrong_heading_makes_the_position_error_grow_with_the_distance():
    start = (0.0, 0.0, 0.0)
    # the real robot drove 10 m straight on, the estimate thinks it drove at 5 degrees to the left
    estimate = (10.0 * math.cos(math.radians(5)), 10.0 * math.sin(math.radians(5)), math.radians(5))

    position_error, heading_error = align_and_error((10.0, 0.0, 0.0), estimate, start, start)

    assert position_error == pytest.approx(2 * 10.0 * math.sin(math.radians(2.5)), rel=1e-6)  # about 0.87 m
    assert math.degrees(heading_error) == pytest.approx(5.0)


def test_summary_of_no_samples_and_of_a_short_run():
    assert "No samples" in summarize([])
    # time, travelled, slam position, slam heading, odom position, odom heading, tilt
    rows = [[1.0, 0.5, 0.01, 0.2, 0.02, 0.5, 3.0], [2.0, 0.9, 0.02, -0.4, 0.05, 1.0, 5.0]]
    assert "n/a" in summarize(rows)


def test_summary_gives_the_error_per_10_m_driven():
    rows = [[10.0, 5.0, 0.2, 3.0, 0.5, 6.0, 4.0], [20.0, 20.0, 1.0, -8.0, 4.0, 12.0, 6.0]]

    text = summarize(rows)

    assert "driven: 20.0 m" in text
    assert "slam: position error at the end 1.00 m (largest 1.00 m, 0.50 m per 10 m driven)" in text
    assert "heading error at the end -8.0 deg (largest 8.0 deg)" in text
    assert "odom: position error at the end 4.00 m (largest 4.00 m, 2.00 m per 10 m driven)" in text
    assert "tilt: largest 6 deg, tipped over (more than 30 deg) in 0 of 2 samples" in text
    assert "FELL OVER" not in text


def test_the_summary_says_when_the_robot_fell_over():
    rows = [
        [10.0, 5.0, 0.2, 3.0, 0.5, 6.0, 4.0],
        [20.0, 6.0, 0.4, 3.0, 0.9, 6.0, 85.0],
        [30.0, 6.0, 0.4, 3.0, 0.9, 6.0, 88.0],
    ]

    text = summarize(rows)

    assert "tilt: largest 88 deg, tipped over (more than 30 deg) in 2 of 3 samples - THE ROBOT FELL OVER" in text


def test_the_tilt_of_a_quaternion_is_its_largest_roll_or_pitch():
    half = math.radians(60) / 2  # 60 degrees around x (roll), around y (pitch)
    assert tilt_of(math.sin(half), 0.0, 0.0, math.cos(half)) == pytest.approx(60.0)
    assert tilt_of(0.0, math.sin(half), 0.0, math.cos(half)) == pytest.approx(60.0)
    assert tilt_of(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(0.0, abs=1e-9)  # turning is no tilt
    assert tilt_of(0.0, 0.0, 0.0, 1.0) == 0.0


def test_the_node_starts_and_writes_the_csv_header(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"csv_path:={tmp_path / 'run' / 'drift.csv'}"])
    try:
        node = DriftMonitorNode()
        node._sample()  # nothing received yet: waits, writes no row
        node.destroy_node()
    finally:
        rclpy.shutdown()

    lines = (tmp_path / "run" / "drift.csv").read_text().splitlines()
    assert lines == [
        "time_s,travelled_m,slam_pos_err_m,slam_heading_err_deg,odom_pos_err_m,odom_heading_err_deg,tilt_deg,"
        "real_x,real_y,slam_x,slam_y,odom_x,odom_y"
    ]
    assert "No samples" in (tmp_path / "run" / "drift_summary.txt").read_text()
