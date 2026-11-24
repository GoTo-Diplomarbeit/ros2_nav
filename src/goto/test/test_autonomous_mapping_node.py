import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

from goto.mapping.autonomous_mapping_keys import AutonomousMappingKeysNode
from goto.mapping.autonomous_mapping_node import AutonomousMappingNode, Mode
from goto.mapping.frontiers import GridInfo

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

    def send_goal_async(self, request, feedback_callback=None):
        self.sent.append(request)
        return SimpleNamespace(add_done_callback=lambda callback: None)


class FakePlanner:
    """
    Stands in for Nav2's planner: answers at once with a path from the robot (1.0, 1.5) to the goal, or with
    "no path" for the goals chosen by no_path_for. detour_for(x, y) makes a path longer. silent = never answers.
    """

    def __init__(self):
        self.requests = []
        self.silent = False
        self.error_code = 0
        self.no_path_for = lambda x, y: False
        self.detour_for = lambda x, y: 0.0

    def server_is_ready(self):
        return True

    def send_goal_async(self, request):
        self.requests.append(request)
        if self.silent:
            return SimpleNamespace(add_done_callback=lambda callback: None)
        x, y = request.goal.pose.position.x, request.goal.pose.position.y
        handle = SimpleNamespace(
            accepted=True,
            cancel_goal_async=lambda: None,
            get_result_async=lambda: SimpleNamespace(
                add_done_callback=lambda callback: callback(SimpleNamespace(result=lambda: self._answer(x, y)))
            ),
        )
        return SimpleNamespace(add_done_callback=lambda callback: callback(SimpleNamespace(result=lambda: handle)))

    def _answer(self, x, y):
        def pose(px, py):
            return PoseStamped(pose=SimpleNamespace(position=SimpleNamespace(x=px, y=py)))

        if self.error_code or self.no_path_for(x, y):
            error = self.error_code or 208
            return SimpleNamespace(
                status=GoalStatus.STATUS_ABORTED,
                result=SimpleNamespace(path=Path(), error_code=error, error_msg="fake"),
            )
        path = Path()
        path.poses = [
            SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=px, y=py)))
            for px, py in [(1.0, 1.5), (1.0, 1.5 + self.detour_for(x, y)), (x, y)]
        ]
        return SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(path=path, error_code=0, error_msg="")
        )


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
    node._path_planner = FakePlanner()
    node._info_time = time.monotonic()
    monkeypatch.setattr(node, "_robot_xy", lambda: (1.0, 1.5))
    yield node
    node.destroy_node()


def give_map(node, grid):
    """Sends the node a costmap the way Nav2 does: -1 unknown, 0 free, 99 blocked for the robot, 100 obstacle."""
    message = OccupancyGrid()
    message.info.width, message.info.height = grid.shape[1], grid.shape[0]
    message.info.resolution = INFO.resolution
    message.info.origin.position.x, message.info.origin.position.y = INFO.origin_x, INFO.origin_y
    message.data = grid.flatten().tolist()
    node._on_costmap(message)


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


def test_a_door_the_planner_keeps_closed_is_no_frontier(explorer):
    # Nav2 marks cells where the robot's centre can't be with 99 (inscribed). A gap that narrow is closed for the
    # planner, so the explorer must not look for a frontier in it - that is what the costmap is for.
    costmap = room_grid(door=True)
    costmap[13:18, 43] = 99
    give_map(explorer, costmap)

    for _ in range(3):
        explorer._explore(time.monotonic())

    explorer._continue_pausing(time.monotonic())
    assert explorer.status_text() == "done"
    assert explorer._navigator.sent == []


def test_a_costmap_that_did_not_change_does_not_start_a_new_frontier_search(explorer):
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())  # looks at the first costmap
    assert not explorer._map_changed
    first_arrival = explorer._map_time

    time.sleep(0.01)
    give_map(explorer, room_grid(door=True))  # Nav2 sends the costmap again every second, unchanged

    assert not explorer._map_changed
    assert explorer._map_time == first_arrival  # "the map last changed" stays

    costmap = room_grid(door=True)
    costmap[20, 20] = 100
    give_map(explorer, costmap)
    assert explorer._map_changed
    assert explorer._map_time > first_arrival


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
    assert explorer._waiting_for == "costmap"


def test_a_map_with_a_small_free_patch_and_no_frontier_is_not_done(explorer):
    # Like slam_toolbox's first scan with its default settings: 1.4 m2 of free space in the middle of unknown space,
    # far from any wall, so the explorer finds no frontier. That is a map that has not grown yet, not a finished one.
    grid = np.full((40, 40), -1, dtype=np.int8)
    grid[10:22, 10:22] = 0
    give_map(explorer, grid)

    for _ in range(5):
        explorer._explore(time.monotonic())

    assert explorer.status_text() == "waiting"
    assert explorer._waiting_for == "costmap"


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


def test_arriving_at_a_frontier_that_is_still_there_after_the_new_map_marks_it_explored(explorer):
    explorer._p.wait_for_rtabmap = False
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    goal = explorer._goal
    explorer._goal_handle, explorer._accepted_at = FakeHandle(), time.monotonic() - 30

    explorer._on_result(result_of(GoalStatus.STATUS_SUCCEEDED), explorer._goal_handle)
    assert explorer._explored == []  # not yet: the map may not show what the robot saw
    explorer._explore(time.monotonic() + 10)  # the wait for a new map is over, the frontier is still in it

    assert explorer._explored == [goal.target]
    assert explorer._failed == []


def test_arriving_at_a_frontier_that_is_gone_in_the_new_map_is_not_marked_explored(explorer):
    explorer._p.wait_for_rtabmap = False
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    explorer._goal_handle, explorer._accepted_at = FakeHandle(), time.monotonic() - 30
    explorer._on_result(result_of(GoalStatus.STATUS_SUCCEEDED), explorer._goal_handle)

    give_map(explorer, room_grid(door=False))  # the door is a wall now: nothing to explore there
    explorer._explore(time.monotonic() + 10)

    assert explorer._explored == []


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


def in_flight(explorer, started_s_ago=0.0, accepted_s_ago=None):
    """A trip that Nav2 accepted: the explorer has chosen a goal and the planner has answered."""
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    now = time.monotonic()
    explorer._goal_handle, explorer._goal_started = FakeHandle(), now - started_s_ago
    explorer._accepted_at = now - (started_s_ago if accepted_s_ago is None else accepted_s_ago)
    explorer._progress_time = explorer._accepted_at
    return explorer._goal, now


def feedback(distance, recoveries=0):
    return SimpleNamespace(feedback=SimpleNamespace(distance_remaining=distance, number_of_recoveries=recoveries))


def test_a_goal_without_feedback_and_without_robot_motion_fails_after_a_short_wait(explorer):
    goal, now = in_flight(explorer, started_s_ago=25)
    explorer._goal_timeout_s = 90.0
    explorer._last_motion_time = now - 25

    explorer._supervise_goal(now)

    assert explorer._goal is None
    assert explorer._failures == [[goal.target[0], goal.target[1], 1]]


def test_a_goal_without_feedback_that_is_still_moving_is_left_alone(explorer):
    goal, now = in_flight(explorer, started_s_ago=25)
    explorer._goal_timeout_s = 90.0
    explorer._last_motion_time = now - 1

    explorer._supervise_goal(now)

    assert explorer._goal is not None


def test_a_goal_whose_distance_does_not_shrink_is_abandoned_even_if_the_legs_move(explorer):
    # The trot moves the body even against a wall, so the robot's motion says nothing: Nav2's distance does
    goal, now = in_flight(explorer, started_s_ago=15)
    explorer._goal_timeout_s = 90.0
    explorer._last_motion_time = now  # moving all the time
    explorer._on_feedback(feedback(3.0), goal)
    explorer._progress_time = now - 11  # that was 11 s ago and the distance has not got smaller since

    explorer._supervise_goal(now)

    assert explorer._goal is None
    assert explorer._failures == [[goal.target[0], goal.target[1], 1]]


def test_a_goal_that_comes_closer_is_left_alone(explorer):
    goal, now = in_flight(explorer, started_s_ago=15)
    explorer._goal_timeout_s = 90.0
    explorer._on_feedback(feedback(3.0), goal)
    explorer._progress_time = now - 11
    explorer._on_feedback(feedback(2.5), goal)  # 0.5 m closer: more than progress_m

    explorer._supervise_goal(now)

    assert explorer._goal is not None
    assert explorer._best_distance == 2.5


def test_feedback_that_gets_only_a_little_closer_is_no_progress(explorer):
    goal, now = in_flight(explorer, started_s_ago=15)
    explorer._on_feedback(feedback(3.0), goal)
    explorer._progress_time = now - 11
    explorer._on_feedback(feedback(2.9), goal)  # 0.1 m: less than progress_m

    assert explorer._best_distance == 3.0
    assert explorer._progress_time == now - 11


def test_close_to_the_goal_no_progress_is_expected(explorer):
    # Only the turn on the spot is left: the distance can not shrink any more
    goal, now = in_flight(explorer, started_s_ago=15)
    explorer._goal_timeout_s = 90.0
    explorer._on_feedback(feedback(0.3), goal)
    explorer._progress_time = now - 11

    explorer._supervise_goal(now)

    assert explorer._goal is not None


def test_a_goal_after_two_recoveries_is_abandoned(explorer):
    goal, now = in_flight(explorer, started_s_ago=5)
    explorer._goal_timeout_s = 90.0
    explorer._on_feedback(feedback(3.0, recoveries=2), goal)

    explorer._supervise_goal(now)

    assert explorer._goal is None


def test_the_feedback_of_an_old_goal_is_ignored(explorer):
    goal, _ = in_flight(explorer)
    explorer._on_feedback(feedback(1.0, recoveries=5), SimpleNamespace())  # another goal

    assert explorer._recoveries == 0 and goal is explorer._goal


def test_the_time_for_a_trip_grows_with_its_path(explorer):
    near, far = FakePlanner(), FakePlanner()
    far.detour_for = lambda x, y: 6.0  # a path 6 m longer, there and back is more
    give_map(explorer, room_grid(door=True))

    explorer._path_planner = near
    explorer._explore(time.monotonic())
    short_time = explorer._goal_timeout_s
    explorer._drop_goal()

    explorer._path_planner = far
    explorer._explore(time.monotonic())

    assert short_time > explorer._p.goal_time_base_s
    assert explorer._goal_timeout_s > short_time + 40


# -- The planner is asked before driving


def two_door_room():
    """The room of room_grid with a second door in the west wall, which is nearer to the robot."""
    grid = room_grid(door=True)
    grid[13:18, 2] = 0
    grid[13:18, 1] = 0
    return grid


def test_the_planner_is_asked_about_the_nearest_frontiers_and_the_shortest_path_wins(explorer):
    explorer._path_planner.detour_for = lambda x, y: 20.0 if x < 2.0 else 0.0  # the west door: far to drive
    give_map(explorer, two_door_room())

    explorer._explore(time.monotonic())

    assert len(explorer._path_planner.requests) == 2
    assert explorer._goal.x > 3.0  # the east door, although the west one is nearer in a straight line


def test_a_frontier_without_a_path_is_skipped_without_driving(explorer):
    explorer._path_planner.no_path_for = lambda x, y: True
    give_map(explorer, room_grid(door=True))

    explorer._explore(time.monotonic())

    assert explorer._navigator.sent == []  # nothing was driven
    assert explorer._goal is None
    assert len(explorer._no_path) == 1
    assert explorer._failed == []  # not given up yet: the costmap may be behind
    assert explorer.status_text() == "exploring"  # and not "done"


def test_a_frontier_without_a_path_is_not_asked_about_again_until_the_wait_is_over(explorer):
    explorer._path_planner.no_path_for = lambda x, y: True
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    asked = len(explorer._path_planner.requests)

    for _ in range(5):
        explorer._explore(time.monotonic())

    assert len(explorer._path_planner.requests) == asked
    assert explorer._idle_checks == 0  # waiting for the retry is not "nothing left to explore"


def test_a_frontier_without_a_path_twice_is_given_up(explorer):
    explorer._path_planner.no_path_for = lambda x, y: True
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    target = (explorer._no_path[0][0], explorer._no_path[0][1])
    explorer._no_path[0][3] = time.monotonic() - 1  # the wait is over

    explorer._explore(time.monotonic())

    assert explorer._failed == [target]
    assert explorer._no_path == []


def test_a_frontier_that_got_a_path_later_is_driven_to(explorer):
    explorer._path_planner.no_path_for = lambda x, y: True
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    explorer._path_planner.no_path_for = lambda x, y: False  # the costmap has caught up
    explorer._no_path[0][3] = time.monotonic() - 1

    explorer._explore(time.monotonic())

    assert explorer._goal is not None
    assert explorer._failed == []


def test_a_planner_that_does_not_answer_is_not_the_frontiers_fault(explorer):
    explorer._path_planner.silent = True
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    assert explorer._path_check is not None
    assert explorer._navigator.sent == []  # no driving while the planner thinks

    explorer._path_check.sent_at = time.monotonic() - 10
    explorer._explore(time.monotonic())

    assert explorer._path_check is None
    assert explorer._no_path == [] and explorer._failed == [] and explorer._failures == []
    assert explorer._planner_failures == 1


def test_a_planner_that_keeps_failing_means_stuck(explorer):
    explorer._path_planner.silent = True
    give_map(explorer, room_grid(door=True))

    for _ in range(3):
        explorer._explore(time.monotonic())
        explorer._path_check.sent_at = time.monotonic() - 10
        explorer._explore(time.monotonic())

    explorer._continue_pausing(time.monotonic())
    assert explorer.status_text() == "stuck"


def test_an_error_of_the_planner_is_not_counted_against_the_frontier(explorer):
    explorer._path_planner.error_code = 202  # TF_ERROR: it doesn't know where the robot is
    give_map(explorer, room_grid(door=True))

    explorer._explore(time.monotonic())

    assert explorer._no_path == [] and explorer._failed == []
    assert explorer._planner_failures == 1


def test_an_answer_that_comes_after_a_pause_is_ignored(explorer):
    explorer._path_planner.silent = True
    give_map(explorer, room_grid(door=True))
    explorer._explore(time.monotonic())
    old_check = explorer._path_check

    explorer._on_command(String(data="pause"))
    explorer._path_planner.silent = False
    explorer._on_path_result(SimpleNamespace(result=lambda: explorer._path_planner._answer(3.0, 1.5)), old_check, 0)

    assert explorer._path_check is None
    assert explorer._goal is None and explorer._navigator.sent == []


# -- Frontiers the robot can not get to


def cut_off_room():
    """The room of room_grid with the door, and a wall across it: the robot (x = 1.0) is on the side without it."""
    grid = room_grid(door=True)
    grid[3:33, 20:22] = 100
    return grid


def test_frontiers_that_are_cut_off_are_not_done_but_waited_for(explorer):
    give_map(explorer, cut_off_room())

    for _ in range(10):  # far more than the three ticks of "nothing left"
        explorer._explore(time.monotonic())

    assert explorer.status_text() == "exploring"
    assert explorer._navigator.sent == [] and explorer._cut_off_since is not None
    assert explorer._selection.unreachable == 1


def test_frontiers_that_stay_cut_off_end_as_stuck_not_done(explorer):
    give_map(explorer, cut_off_room())
    explorer._explore(time.monotonic())

    explorer._cut_off_since = time.monotonic() - 31  # waited longer than cut_off_wait_s
    explorer._explore(time.monotonic())
    explorer._continue_pausing(time.monotonic())

    assert explorer.status_text() == "stuck"
    assert explorer._cut_off_since is None  # the next resume starts the waiting again


def fake_clear_clients(explorer):
    clients = [FakeCancelClient(), FakeCancelClient()]  # same two methods as a service client: ready, call_async
    explorer._clear_clients = clients
    return clients


def test_being_cut_off_clears_both_costmaps_once_per_wait(explorer):
    clients = fake_clear_clients(explorer)
    give_map(explorer, cut_off_room())

    for _ in range(5):
        explorer._explore(time.monotonic())

    assert [client.calls for client in clients] == [1, 1]  # not again on every tick of the same wait


def test_a_new_wait_clears_the_costmaps_again(explorer):
    clients = fake_clear_clients(explorer)
    give_map(explorer, cut_off_room())
    explorer._explore(time.monotonic())

    explorer._pause("stuck")  # a pause ends the wait
    explorer._mode = Mode.WAITING
    explorer._explore(time.monotonic())

    assert [client.calls for client in clients] == [2, 2]


def test_the_costmaps_are_not_cleared_when_nothing_is_cut_off(explorer):
    clients = fake_clear_clients(explorer)
    give_map(explorer, room_grid(door=True))

    explorer._explore(time.monotonic())

    assert [client.calls for client in clients] == [0, 0]


def test_a_service_that_is_not_ready_is_skipped(explorer):
    clients = fake_clear_clients(explorer)
    clients[0].service_is_ready = lambda: False
    give_map(explorer, cut_off_room())

    explorer._explore(time.monotonic())

    assert [client.calls for client in clients] == [0, 1]


def test_being_cut_off_saves_the_costmap_around_the_robot(explorer, tmp_path):
    fake_clear_clients(explorer)
    explorer._p.debug_dir = str(tmp_path)
    give_map(explorer, cut_off_room())

    explorer._explore(time.monotonic())

    [saved] = list(tmp_path.glob("cut_off_*.npz"))
    data = np.load(saved)
    assert set(data.files) >= {"costmap", "robot_cell", "start_cell", "crop_origin_cell", "resolution", "origin_xy"}
    assert tuple(data["robot_cell"]) == INFO.to_cell(1.0, 1.5)
    assert data["costmap"].shape[0] <= 61  # the robot's surroundings, not the whole costmap


def test_without_a_debug_folder_nothing_is_saved(explorer, tmp_path, monkeypatch):
    fake_clear_clients(explorer)
    monkeypatch.chdir(tmp_path)  # a file written by mistake would land here
    give_map(explorer, cut_off_room())

    explorer._explore(time.monotonic())

    assert list(tmp_path.iterdir()) == []


def test_the_costmap_opening_up_again_ends_the_waiting(explorer):
    give_map(explorer, cut_off_room())
    explorer._explore(time.monotonic())
    assert explorer._cut_off_since is not None

    give_map(explorer, room_grid(door=True))  # the wall is gone: a doorway that was only closed for a moment
    explorer._explore(time.monotonic())

    assert explorer._goal is not None
    assert explorer._cut_off_since is None
    assert explorer.status_text() == "exploring"


def test_the_status_says_how_many_frontiers_are_cut_off(explorer):
    published = []
    explorer._status_publisher = SimpleNamespace(publish=published.append)
    give_map(explorer, cut_off_room())
    explorer._explore(time.monotonic())

    explorer._publish_status(time.monotonic())

    status = json.loads(published[-1].data)
    assert status["cut_off"] == 1 and status["frontiers"] == 1 and status["goal"] is None


def test_resume_asks_the_planner_again_about_frontiers_it_found_no_path_to(explorer):
    explorer._no_path = [[3.0, 1.5, 1, time.monotonic() + 100]]
    explorer._on_command(String(data="pause"))
    explorer._continue_pausing(time.monotonic())

    explorer._on_command(String(data="resume"))

    assert explorer._no_path == []


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
