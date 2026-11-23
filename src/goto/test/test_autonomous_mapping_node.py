import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from goto.mapping.autonomous_mapping_keys import AutonomousMappingKeysNode
from goto.mapping.autonomous_mapping_node import AutonomousMappingNode
from goto.mapping.frontiers import GridInfo, wall_distance_m

INFO = GridInfo(resolution=0.1, origin_x=0.0, origin_y=0.0)


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeNav2:
    """Stands in for Nav2's action server: always ready, remembers the goals it was sent."""

    def __init__(self):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, request):
        self.sent.append(request)
        return SimpleNamespace(add_done_callback=lambda callback: None)


class FakeHandle:
    def __init__(self):
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


class FakeCancelClient:
    """Stands in for an action's cancel_goal service."""

    def __init__(self):
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1


def result_of(status):
    return SimpleNamespace(result=lambda: SimpleNamespace(status=status))


def room_grid(door: bool):
    """A closed room of 4 x 3 m; with door=True a door in the east wall leads to unknown space."""
    grid = np.full((36, 46), -1, dtype=np.int8)
    grid[2:34, 2:44] = 100
    grid[3:33, 3:43] = 0
    if door:
        grid[13:18, 43] = 0  # a 0.5 m door
    return grid


@pytest.fixture
def explorer(rclpy_context, monkeypatch):
    """The explorer with a map, a position and a fake Nav2, as if everything were up and running."""
    node = AutonomousMappingNode()
    node._navigator = FakeNav2()
    node._info_time = time.monotonic()
    monkeypatch.setattr(node, "_robot_xy", lambda: (1.0, 1.5))
    yield node
    node.destroy_node()


def give_map(node, grid):
    node._grid, node._grid_info, node._map_changed = grid, INFO, True
    node._map_time = time.monotonic()
    node._wall_distance = wall_distance_m(grid, INFO)


def test_the_nodes_start(rclpy_context):
    explorer = AutonomousMappingNode()
    keys = AutonomousMappingKeysNode()

    assert explorer.node.get_name() == "autonomous_mapping"
    assert keys.node.get_name() == "autonomous_mapping_keys"
    assert explorer.status_text() == "waiting"

    explorer.destroy_node()
    keys.destroy_node()


def test_pause_stops_first_and_reports_paused_when_the_robot_stands(explorer):
    explorer._on_command(String(data="pause"))
    assert explorer.status_text() == "pausing"

    explorer._continue_pausing(time.monotonic())  # no odometry motion was ever seen
    assert explorer.status_text() == "paused"


def test_pausing_waits_while_the_robot_still_moves(explorer):
    explorer._on_command(String(data="pause"))
    explorer._last_motion_time = time.monotonic()

    explorer._continue_pausing(time.monotonic())

    assert explorer.status_text() == "pausing"


def test_resume_and_toggle(explorer):
    explorer._on_command(String(data="pause"))
    explorer._continue_pausing(time.monotonic())

    explorer._on_command(String(data="resume"))
    assert explorer.status_text() == "waiting"

    explorer._on_command(String(data="toggle"))
    assert explorer.status_text() == "pausing"


def test_the_explorer_drives_to_a_frontier(explorer):
    give_map(explorer, room_grid(door=True))

    explorer._explore(time.monotonic())

    assert explorer.status_text() == "exploring"
    assert explorer._goal is not None
    assert len(explorer._navigator.sent) == 1
    assert explorer._goal.x > 3.0  # towards the door in the east wall


def test_no_frontier_for_three_checks_means_done(explorer):
    give_map(explorer, room_grid(door=False))

    for _ in range(3):
        explorer._explore(time.monotonic())

    explorer._continue_pausing(time.monotonic())
    assert explorer.status_text() == "done"
    assert explorer._navigator.sent == []


def test_a_map_that_is_just_starting_is_not_done(explorer):
    give_map(explorer, np.full((20, 20), -1, dtype=np.int8))

    for _ in range(5):
        explorer._explore(time.monotonic())

    assert explorer.status_text() == "waiting"
    assert explorer._waiting_for == "map"


def test_a_map_with_a_small_free_patch_and_no_frontier_is_not_done(explorer):
    # Like slam_toolbox's first scan with its default settings: 1.4 m2 of free space in the middle of unknown space,
    # far from any wall, so the explorer finds no frontier. That is a map that has not grown yet, not a finished one.
    grid = np.full((40, 40), -1, dtype=np.int8)
    grid[10:22, 10:22] = 0
    give_map(explorer, grid)

    for _ in range(5):
        explorer._explore(time.monotonic())

    assert explorer.status_text() == "waiting"
    assert explorer._waiting_for == "map"


def test_a_silent_rtabmap_stops_new_goals(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._info_time = time.monotonic() - 60

    explorer._explore(time.monotonic())

    assert explorer._waiting_for == "rtabmap"
    assert explorer._navigator.sent == []


def test_without_rtabmap_a_silent_info_topic_does_not_stop_the_explorer(explorer):
    # slam_toolbox version: wait_for_rtabmap is false, so the missing /info is no reason to wait
    explorer._p.wait_for_rtabmap = False
    give_map(explorer, room_grid(door=True))
    explorer._info_time = time.monotonic() - 60

    explorer._explore(time.monotonic())

    assert explorer._waiting_for == ""
    assert len(explorer._navigator.sent) == 1


def test_the_time_limit_pauses_the_robot(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._p.time_limit_s, explorer._active_s = 10.0, 11.0

    explorer._explore(time.monotonic())
    explorer._continue_pausing(time.monotonic())

    assert explorer.status_text() == "time_limit"


def test_an_abort_right_after_accepting_is_retried_without_counting(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal, handle = explorer._goal, FakeHandle()
    explorer._goal_handle, explorer._accepted_at = handle, time.monotonic()
    cancel_all = FakeCancelClient()
    explorer._cancel_clients = [cancel_all]

    explorer._on_result(result_of(GoalStatus.STATUS_ABORTED), handle)

    assert explorer._goal is None
    assert explorer._failures == []
    assert cancel_all.calls == 1  # the controller may still drive its path: stop it
    explorer._explore(time.monotonic())  # the next tick sends the same frontier again
    assert explorer._goal is not None and explorer._goal.target == goal.target


def test_a_late_abort_far_from_the_goal_counts_and_two_give_the_frontier_up(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal = explorer._goal

    for _ in range(2):
        explorer._goal, handle = goal, FakeHandle()
        explorer._goal_handle, explorer._accepted_at = handle, time.monotonic() - 30
        explorer._on_result(result_of(GoalStatus.STATUS_ABORTED), handle)

    assert explorer._failed == [goal.target]


def test_arriving_twice_without_the_frontier_going_away_marks_it_explored(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal = explorer._goal

    for _ in range(2):
        explorer._goal, handle = goal, FakeHandle()
        explorer._goal_handle, explorer._accepted_at = handle, time.monotonic() - 30
        explorer._on_result(result_of(GoalStatus.STATUS_SUCCEEDED), handle)

    assert explorer._explored == [goal.target]
    assert explorer._failed == []


def test_resume_resets_the_counters_but_keeps_ignoring_given_up_frontiers(explorer):
    explorer._failed = [(1.0, 1.0)]
    explorer._explored = [(2.0, 2.0)]
    explorer._failures = [[1.0, 1.0, 1]]
    explorer._failure_places = [(0.0, 0.0)]
    explorer._on_command(String(data="pause"))
    explorer._continue_pausing(time.monotonic())

    explorer._on_command(String(data="resume"))

    assert explorer._failures == [] and explorer._failure_places == []
    assert explorer._failed == [(1.0, 1.0)] and explorer._explored == [(2.0, 2.0)]  # unreachable ones stay out


def test_a_goal_without_robot_motion_fails_after_a_short_wait(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal, now = explorer._goal, time.monotonic()
    explorer._goal_handle, explorer._goal_started = FakeHandle(), now - 25
    explorer._accepted_at = explorer._last_motion_time = now - 25

    explorer._supervise_goal(now)

    assert explorer._goal is None
    assert explorer._failures == [[goal.target[0], goal.target[1], 1]]


def test_a_goal_that_is_still_moving_is_left_alone(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    now = time.monotonic()
    explorer._goal_handle, explorer._goal_started = FakeHandle(), now - 25
    explorer._accepted_at, explorer._last_motion_time = now - 25, now - 1

    explorer._supervise_goal(now)

    assert explorer._goal is not None


def test_three_failures_at_the_same_spot_mean_the_robot_is_stuck(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal = explorer._goal  # the robot stays at (1.0, 1.5) the whole time

    for _ in range(3):
        explorer._count_failure(goal)

    explorer._continue_pausing(time.monotonic())
    assert explorer.status_text() == "stuck"


def test_failures_in_different_places_are_not_stuck(explorer, monkeypatch):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal = explorer._goal

    for x in (1.0, 2.0, 3.0):
        monkeypatch.setattr(explorer, "_robot_xy", lambda x=x: (x, 1.5))
        explorer._count_failure(goal)

    assert explorer.status_text() == "exploring"


def test_the_result_of_a_replaced_goal_is_ignored(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    explorer._goal_handle = FakeHandle()

    explorer._on_result(result_of(GoalStatus.STATUS_ABORTED), FakeHandle())  # another handle

    assert explorer._goal is not None


def test_keys_pause_before_driving_and_drive_once_paused(rclpy_context):
    keys = AutonomousMappingKeysNode()
    sent = []
    keys._command_publisher = SimpleNamespace(publish=lambda message: sent.append(message.data))
    keys._velocity_publisher = SimpleNamespace(publish=lambda message: sent.append(message.linear.x))

    keys._status = "exploring"
    keys._on_key("i")
    assert sent == ["pause"]
    assert keys._drive is None  # not driving while Nav2 may still drive

    keys._status = "paused"
    keys._on_key("i")
    keys._send_velocity()
    assert keys._drive is not None
    assert sent[-1] == pytest.approx(0.3)

    keys._on_key("p")
    assert sent[-1] == "toggle"

    keys.destroy_node()


def test_x_ends_the_keys_node_like_ctrl_c(rclpy_context):
    keys = AutonomousMappingKeysNode()

    with pytest.raises(KeyboardInterrupt):
        keys._on_key("x")

    keys.destroy_node()
