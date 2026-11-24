"""
Autonomous Mapping Node - the robot maps the floor by itself: it drives to the edge of the known map
(a frontier) again and again, until no frontier is left. slam_toolbox builds the map, Nav2 drives.

    /autonomous_mapping/command   (std_msgs/String) "pause", "resume" or "toggle". Pausing stops the robot,
                                  so it can be driven by hand (see autonomous_mapping_keys.py).
    /autonomous_mapping/status    (std_msgs/String, JSON; the last one is kept for late subscribers)
                                  {"status": "exploring", "frontiers": 4, "goal": {"x": 3.2, "y": -1.0}, ...}
                                  status: waiting, exploring, pausing, paused, done, stuck, time_limit
    /autonomous_mapping/frontiers (visualization_msgs/MarkerArray) frontiers (green), the goal (blue arrow), given-up
                                  frontiers (red) and frontiers the planner found no path to for now (orange) for rviz

"done" = no frontier left, "stuck" = frontiers are left that Nav2 failed to reach, or the robot is stuck and
doesn't get anywhere, "time_limit" = ran out of time. In all three the robot stands still and waits: the user
can save the map, or drive to missed areas (or free the robot) and send "resume". Frontiers given up on stay
given up after a resume (red in rviz). The node never stops the script by itself.

Before driving to a frontier, Nav2's planner (ComputePathToPose) is asked for a path to the nearest few. One without
a path is skipped for a minute without driving there, the second time it is given up. Of the others the one with the
shortest path is driven to. While driving, Nav2's feedback (distance to the goal, recoveries) tells if the robot gets
anywhere: no progress in 10 s, or two recoveries, and the goal is given up.

The frontier logic is in frontiers.py. This node only talks to ROS: it reads Nav2's global costmap, sends goals to Nav2
and reports what it does. It uses no simulation time: timeouts use the wall clock and the robot's position
is the latest transform.

It reads the global costmap and not /map on purpose: the costmap is what Nav2's planner plans on (walls inflated by the
robot's size, live obstacles added). A frontier or goal chosen on it is one the planner accepts, and a gap the planner
keeps closed is closed for the explorer too. The costmap is made from /map, so everything SLAM draws is still in it.
"""

import json
import math
import time
from enum import Enum
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from goto.mapping.frontiers import (
    GridInfo,
    choose_goal,
    classify_result,
    costmap_to_grid,
    costmap_wall_distance_m,
    find_frontier_clusters,
    frontier_still_exists,
)
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.planner.planner_node import quaternion_from_yaw

COMMAND_TOPIC = "/autonomous_mapping/command"
STATUS_TOPIC = "/autonomous_mapping/status"
MARKERS_TOPIC = "/autonomous_mapping/frontiers"
COSTMAP_TOPIC = "/global_costmap/costmap"
CLEAR_COSTMAP_SERVICES = (
    "/global_costmap/clear_entirely_global_costmap",  # the one the explorer reads
    "/local_costmap/clear_entirely_local_costmap",  # the one the controller drives on
)
DEBUG_CROP_M = 3.0  # how far around the robot the costmap is saved when the robot is cut off

TICK_PERIOD_S = 1.0  # how often the explorer re-checks what to do
STATUS_PERIOD_S = 2.0  # the status is repeated this often even if nothing changed
RTABMAP_SILENT_S = 5.0  # no /info for this long: RTAB-Map is busy, don't choose new goals
MIN_KNOWN_FREE_M2 = 5.0  # a map with less free area is just starting, not finished (slam_toolbox's first scan: ~10 m2)
STOPPED_AFTER_S = 1.0  # the robot counts as standing after this long without motion
PAUSE_TIMEOUT_S = 4.0  # pausing is reported after this long even if the robot still moves
MOVING_SPEED = 0.03  # m/s (or rad/s)
NEW_MAP_WAIT_S = 4.0  # after arriving, wait this long for a costmap that shows what the robot saw. What the robot sees
# reaches the costmap in steps: SLAM publishes /map (about 1 s), the costmap takes it over and publishes (1 s + 1 s)
MAX_SPURIOUS_RETRIES = 3  # per frontier, then an early abort counts as a failure
NO_PROGRESS_S = 20.0  # only used while Nav2 sends no feedback: a goal with no robot motion for this long has failed
MAX_RECOVERIES = 2  # Nav2 tries to free itself (clear the costmaps, spin, back up): this many times means it is stuck
EXPECTED_SPEED_MPS = 0.3  # what the simulated robot really does (CHAMP's top speed): how long may a trip take?
PATH_CHECK_TIMEOUT_S = 5.0  # the planner has this long to answer for one frontier
PLANNER_FAILURES_BEFORE_STUCK = 3  # planner timeouts or errors in a row (not "no path"): something is wrong with Nav2
# ComputePathToPose error codes that say "there is no way to this frontier" (and not "the planner is broken"):
# GOAL_OUTSIDE_MAP, GOAL_OCCUPIED, NO_VALID_PATH
NO_PATH_ERRORS = (204, 206, 208)
STUCK_FAILURES = 3  # this many failures within STUCK_RADIUS_M of each other mean the robot is stuck
STUCK_RADIUS_M = 0.7
FRONTIER_GONE_RADIUS_M = 1.0  # a frontier this near the goal's frontier is "the same one"

# Keep the last status for late subscribers (the keys node, the website)
STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# The global costmap is sent in full every second (always_send_full_costmap in nav2_params.yaml), so a node that starts
# later gets one soon. Asking for the last one to be kept (TRANSIENT_LOCAL) would be risky: a subscriber that wants more
# than the publisher offers gets nothing at all.
COSTMAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)

GREEN = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)
RED = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
ORANGE = ColorRGBA(r=1.0, g=0.6, b=0.1, a=1.0)


class Mode(Enum):
    WAITING = "waiting"  # not ready: no map, RTAB-Map silent, no Nav2 or no position yet
    EXPLORING = "exploring"
    PAUSING = "pausing"  # stop requested, waiting until the robot stands
    PAUSED = "paused"  # standing; _pause_reason says why: paused, done, stuck or time_limit


class AutonomousMappingNode(NodeWrapper):
    def __init__(self):
        # follow_state=False: there is no mission state for mapping yet (a team topic)
        super().__init__("autonomous_mapping", self._setup_channels, follow_state=False)

    def _setup_channels(self):
        node = self.node
        self._p = SimpleNamespace(
            min_frontier_m=node.declare_parameter("min_frontier_size_m", 0.5).value,
            max_wall_distance_m=node.declare_parameter("max_wall_distance_m", 1.0).value,
            # Gaps narrower than twice robot_clearance_m are skipped. Tried with 0.45 (0.9 m gaps): the 0.76 x 0.4 m
            # footprint barely fits those. preferred_clearance_m equal to robot_clearance_m = the goal is the nearest
            # passable cell to the frontier (the old behaviour); 0.9 = prefer open space. pullback_m: how far before the
            # frontier the goal may lie (1.5 was tried).
            clearance_m=node.declare_parameter("robot_clearance_m", 0.3).value,
            preferred_clearance_m=node.declare_parameter("preferred_clearance_m", 0.3).value,
            pullback_m=node.declare_parameter("pullback_m", 1.0).value,
            blacklist_radius_m=node.declare_parameter("blacklist_radius_m", 0.75).value,
            near_goal_m=node.declare_parameter("near_goal_m", 0.5).value,
            spurious_abort_s=node.declare_parameter("spurious_abort_s", 1.5).value,
            # A trip may take goal_time_base_s plus goal_time_factor times the time the path needs at the robot's speed
            goal_time_base_s=node.declare_parameter("goal_time_base_s", 20.0).value,
            goal_time_factor=node.declare_parameter("goal_time_factor", 3.0).value,
            # No path to a frontier: it is skipped for unreachable_retry_s (the costmap may not be up to date yet), the
            # second time it is given up. The planner is asked about the nearest max_candidates frontiers.
            unreachable_retry_s=node.declare_parameter("unreachable_retry_s", 60.0).value,
            max_candidates=node.declare_parameter("max_candidates", 3).value,
            # The trip counts as stuck if Nav2's distance to the goal gets no smaller than progress_m in
            # progress_window_s
            progress_m=node.declare_parameter("progress_m", 0.3).value,
            progress_window_s=node.declare_parameter("progress_window_s", 10.0).value,
            # Frontiers exist but the robot can't get to any of them (a doorway the costmap has closed for a moment, a
            # gap too narrow): wait this long for the costmap to open up again before pausing as "stuck"
            cut_off_wait_s=node.declare_parameter("cut_off_wait_s", 30.0).value,
            # How often a frontier may be reached and still be there (after waiting for a new map) until it is ignored
            visits_before_explored=node.declare_parameter("visits_before_explored", 1).value,
            time_limit_s=node.declare_parameter("time_limit_s", 1800.0).value,
            done_checks=node.declare_parameter("done_checks", 3).value,
            start_paused=node.declare_parameter("start_paused", False).value,
            # true only with RTAB-Map (build_run_sim_autonomous_mapping.sh): the explorer then waits while RTAB-Map is
            # busy rebuilding its map. slam_toolbox sends no /info, so waiting for it would never end.
            wait_for_rtabmap=node.declare_parameter("wait_for_rtabmap", False).value,
            # A folder: when the robot is cut off, the costmap around it is saved there (cut_off_<time>.npz)
            debug_dir=node.declare_parameter("debug_dir", "").value,
        )

        self._status_publisher = node.create_publisher(String, STATUS_TOPIC, STATUS_QOS)
        self._markers_publisher = node.create_publisher(MarkerArray, MARKERS_TOPIC, 1)
        node.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        node.create_subscription(OccupancyGrid, COSTMAP_TOPIC, self._on_costmap, COSTMAP_QOS)
        # Best effort connects to publishers of either kind, so a QoS mismatch can't silence these two
        node.create_subscription(Info, "/info", self._on_info, qos_profile_sensor_data)  # RTAB-Map's sign of life
        node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._navigator = ActionClient(node, NavigateToPose, "navigate_to_pose")
        # Nav2's planner, asked before driving: is there a way to this frontier, and how long is it?
        self._path_planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        # Cancelling the navigate goal alone isn't enough: after an abort the controller can keep driving
        # its path. A cancel request with an empty goal id cancels every goal of that action.
        self._cancel_clients = [
            node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal"),
            node.create_client(CancelGoal, "/follow_path/_action/cancel_goal"),
        ]
        # Clearing Nav2's costmaps throws away stale live obstacles, the same as Nav2's own recovery does
        self._clear_clients = [node.create_client(ClearEntireCostmap, name) for name in CLEAR_COSTMAP_SERVICES]

        # The costmap (called "map" below where it doesn't matter): _costmap as Nav2 sent it, _grid the same as
        # -1 / 0 / 100 for frontiers.py
        self._costmap = None
        self._grid = None
        self._grid_info = None
        self._wall_distance = None
        self._map_time = 0.0  # when the costmap last changed (time.monotonic)
        self._map_changed = False
        self._clusters = []  # the frontiers of the newest map
        self._clusters_time = 0.0
        self._info_time = -math.inf  # when RTAB-Map last sent /info
        self._last_motion_time = 0.0

        # The explorer
        self._mode = Mode.PAUSED if self._p.start_paused else Mode.WAITING
        self._pause_reason = "paused"
        self._pause_started = 0.0
        self._waiting_for = ""
        self._active_s = 0.0  # time spent exploring, for the time limit
        self._last_tick = time.monotonic()
        self._idle_checks = 0  # ticks in a row without a frontier to drive to
        self._wait_map_after = None  # after arriving: no new goal before a map newer than this
        self._goal = None  # the Goal being driven to, None = none
        self._goal_handle = None  # Nav2's handle of it, None until Nav2 accepted it
        self._goal_started = 0.0
        self._accepted_at = 0.0  # 0 = Nav2 hasn't accepted the goal yet
        self._goal_timeout_s = self._p.goal_time_base_s  # how long this trip may take, worked out from its path
        self._best_distance = math.inf  # the smallest distance to the goal Nav2 reported (its feedback) ...
        self._progress_time = 0.0  # ... and when it last got smaller by progress_m
        self._recoveries = 0  # how many recoveries Nav2 has tried on this trip
        # What happened per frontier: [x, y, count]. Reached and still there after the new map came in means nothing
        # more can be seen from there ("explored"); two failures mean Nav2 can't get there ("failed").
        self._visits, self._failures, self._spurious = [], [], []
        self._explored, self._failed = [], []
        self._visit_target = None  # the frontier the robot just reached: counted once the new map has come in
        # Frontiers the planner found no way to: [x, y, count, skipped until]. The second time they are given up.
        self._no_path = []
        self._cut_off_since = None  # since when frontiers exist that the robot can't get to, and no other goal
        self._path_check = None  # the planner questions in progress (see _start_path_check), None = none
        self._planner_failures = 0
        self._failure_places = []  # where the robot was at its latest failures, to notice it is stuck

        self._selection = None
        self._last_status = ""
        self._last_status_time = 0.0
        self._markers_dirty = True

        node.create_timer(TICK_PERIOD_S, self._tick)

    # --
    # -- Subscriptions
    # --

    def _on_costmap(self, message: OccupancyGrid) -> None:
        costmap = np.asarray(message.data, dtype=np.int8).reshape(message.info.height, message.info.width)
        info = GridInfo(
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
        )
        # The costmap is sent every second, changed or not: only look for frontiers again if it did change
        if self._costmap is not None and info == self._grid_info and np.array_equal(costmap, self._costmap):
            return
        self._map_time = time.monotonic()
        self._costmap, self._grid_info = costmap, info
        self._grid = costmap_to_grid(costmap)
        self._map_changed = True

    def _on_info(self, _message: Info) -> None:
        self._info_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        twist = message.twist.twist
        if math.hypot(twist.linear.x, twist.linear.y) > MOVING_SPEED or abs(twist.angular.z) > MOVING_SPEED:
            self._last_motion_time = time.monotonic()

    def _on_command(self, message: String) -> None:
        command = message.data.strip().lower()
        if command == "toggle":
            command = "resume" if self._mode == Mode.PAUSED else "pause"
        if command == "pause":
            if self._mode in (Mode.WAITING, Mode.EXPLORING):
                self._pause("paused")
        elif command == "resume":
            self._resume()
        else:
            self.node.get_logger().warning(f"Unknown command '{message.data}'. Use pause, resume or toggle.")

    # --
    # -- The explorer
    # --

    def _tick(self) -> None:
        now = time.monotonic()
        if self._mode in (Mode.WAITING, Mode.EXPLORING):
            self._active_s += now - self._last_tick
        self._last_tick = now

        if self._mode == Mode.PAUSING:
            self._continue_pausing(now)
        elif self._mode in (Mode.WAITING, Mode.EXPLORING):
            self._explore(now)

        self._publish_status(now)
        if self._markers_dirty:
            self._publish_markers()

    def _explore(self, now: float) -> None:
        if self._active_s > self._p.time_limit_s:
            self._pause("time_limit")
            return

        # An active goal is supervised even while RTAB-Map is silent: Nav2 keeps the last map
        problem = self._not_ready(need_rtabmap=self._goal is None, now=now)
        if problem:
            self._mode, self._waiting_for = Mode.WAITING, problem
            return
        self._mode, self._waiting_for = Mode.EXPLORING, ""

        if self._map_changed:
            self._update_frontiers(now)
        if self._goal is not None:
            self._supervise_goal(now)
        else:
            self._pick_next_goal(now)

    def _not_ready(self, need_rtabmap: bool, now: float) -> str:
        """What the explorer is still waiting for, "" if nothing."""
        if self._grid is None or self._known_free_m2() < MIN_KNOWN_FREE_M2:
            return "costmap"
        if need_rtabmap and self._p.wait_for_rtabmap and now - self._info_time > RTABMAP_SILENT_S:
            return "rtabmap"
        if not self._navigator.server_is_ready() or not self._path_planner.server_is_ready():
            return "nav2"
        if self._robot_xy() is None:
            return "position"
        return ""

    def _known_free_m2(self) -> float:
        """Free area of the map, to tell a map that is just starting from a finished one."""
        if self._grid is None:
            return 0.0
        return float(np.count_nonzero(self._grid == 0)) * self._grid_info.resolution**2

    def _update_frontiers(self, now: float) -> None:
        self._map_changed = False
        self._wall_distance = costmap_wall_distance_m(self._costmap, self._grid_info)
        self._clusters = find_frontier_clusters(
            self._grid, self._grid_info, self._wall_distance, self._p.min_frontier_m, self._p.max_wall_distance_m
        )
        self._clusters_time = now
        self._markers_dirty = True

    def _supervise_goal(self, now: float) -> None:
        problem = self._trip_problem(now)
        if problem:
            self._abandon_goal(problem)
        elif self._frontier_is_gone(self._goal):
            self._replace_goal(now)

    def _trip_problem(self, now: float) -> str:
        """What is wrong with the current trip, "" if nothing."""
        if now - self._goal_started > self._goal_timeout_s:
            return f"No arrival after {self._goal_timeout_s:.0f} s"
        if self._accepted_at == 0:
            return ""  # Nav2 has not accepted the goal yet
        if self._recoveries >= MAX_RECOVERIES:
            return f"Nav2 needed {self._recoveries} recoveries"
        if self._best_distance < math.inf:
            # Nav2 reports how far the goal still is. Close to the goal only the last turn is left: no check then
            if self._best_distance > self._p.near_goal_m and now - self._progress_time > self._p.progress_window_s:
                return f"Nav2 came no closer than {self._p.progress_m} m in {self._p.progress_window_s:.0f} s"
            return ""
        if now - max(self._accepted_at, self._last_motion_time) > NO_PROGRESS_S:
            # No feedback from Nav2 (yet): the robot's own motion has to do. It stands against something its map
            # doesn't show, for example.
            return f"The robot hasn't moved for {NO_PROGRESS_S:.0f} s"
        return ""

    def _frontier_is_gone(self, goal) -> bool:
        """The frontier of the current goal doesn't exist in the newest costmap any more."""
        return self._clusters_time > self._goal_started and not frontier_still_exists(
            self._clusters, goal.target, FRONTIER_GONE_RADIUS_M
        )

    def _replace_goal(self, now: float) -> None:
        """Seen from a distance already: no need to drive all the way. A new goal replaces this one in Nav2."""
        self._drop_goal()
        if not self._pick_next_goal(now):
            self._cancel_all_driving()

    def _pick_next_goal(self, now: float) -> bool:
        """Chooses the next frontier and has the planner check the way there. Returns True if a goal was sent."""
        # After arriving, the map may not show yet what the robot saw: the same frontier would look unchanged
        if self._wait_map_after is not None:
            if self._map_time < self._wait_map_after and now - self._wait_map_after < NEW_MAP_WAIT_S:
                return False
            self._wait_map_after = None
            self._count_visit()
        if self._path_check is not None:  # the planner is still answering
            self._continue_path_check(now)
            return self._goal is not None
        robot = self._robot_xy()
        if robot is None:
            return False

        selection = choose_goal(
            self._grid,
            self._grid_info,
            self._wall_distance,
            robot,
            self._clusters,
            self._explored + self._failed + self._skipped_for_now(now),
            clearance_m=self._p.clearance_m,
            preferred_clearance_m=self._p.preferred_clearance_m,
            pullback_m=self._p.pullback_m,
            blacklist_radius_m=self._p.blacklist_radius_m,
            max_candidates=self._p.max_candidates,
        )
        self._selection = selection
        if not selection.candidates:
            if self._skipped_for_now(now):
                return False  # the planner found no way to some frontiers a moment ago: ask again later, not "done"
            if selection.unreachable > 0:
                return self._wait_for_cut_off_frontiers(selection.unreachable, now)
            self._cut_off_since = None
            self._idle_checks += 1
            if self._idle_checks >= self._p.done_checks:
                # Frontiers left that Nav2 failed to reach are not "done": the user should look at them
                stuck = bool(self._failed) and selection.blacklisted > 0
                self._pause("stuck" if stuck else "done")
            return False

        self._idle_checks, self._cut_off_since = 0, None
        self._start_path_check(selection.candidates, now)
        return self._goal is not None  # with a quick planner the check is over already

    def _wait_for_cut_off_frontiers(self, count: int, now: float) -> bool:
        """
        There are frontiers but the robot can't get to any of them: they lie behind a wall, or the way to them is
        closed in the costmap. The costmap can close a doorway for a moment (a mark of a live scan, a door leaf that
        leaves too narrow a gap), so this is not "done" and not given up on at once: wait cut_off_wait_s, then stop as
        "stuck" so that the user can look at it (drive the robot out, or save the map).
        """
        if self._cut_off_since is None:
            self._cut_off_since = now
            self.node.get_logger().info(f"{count} frontiers can't be reached for now, waiting for the costmap")
            self._report_cut_off()
            self._clear_costmaps()
        if now - self._cut_off_since >= self._p.cut_off_wait_s:
            self.node.get_logger().warning(
                f"{count} frontiers can't be reached for {self._p.cut_off_wait_s:.0f} s (behind a wall or a gap too "
                "narrow for the robot)"
            )
            self._pause("stuck")
        return False

    def _report_cut_off(self) -> None:
        """Says where the robot stands in the costmap, so a robot stuck in the open can be explained afterwards."""
        robot, selection = self._robot_xy(), self._selection
        if robot is None or selection is None:
            return
        row, col = self._grid_info.to_cell(*robot)
        inside = 0 <= row < self._costmap.shape[0] and 0 <= col < self._costmap.shape[1]
        cost = int(self._costmap[row, col]) if inside else None
        moved_m = None
        if selection.start is not None:
            moved_m = math.dist((row, col), selection.start) * self._grid_info.resolution
        self.node.get_logger().info(
            f"Cut off at ({robot[0]:.1f}, {robot[1]:.1f}): cost of the robot's cell {cost}, "
            f"search started {'nowhere' if moved_m is None else f'{moved_m:.2f} m away'}, "
            f"robot's region {selection.reachable_m2:.1f} m2"
        )
        if self._p.debug_dir and inside:
            self._save_cut_off_crop(robot, row, col, selection)

    def _save_cut_off_crop(self, robot, row: int, col: int, selection) -> None:
        """Saves the costmap around the robot (numpy file) to look at it later, e.g. with imshow."""
        cells = math.ceil(DEBUG_CROP_M / self._grid_info.resolution)
        row_start, col_start = max(row - cells, 0), max(col - cells, 0)
        crop = self._costmap[row_start : row + cells + 1, col_start : col + cells + 1]
        name = Path(self._p.debug_dir) / f"cut_off_{time.strftime('%H-%M-%S')}.npz"
        np.savez(
            name,
            costmap=crop,
            crop_origin_cell=(row_start, col_start),  # cell of the crop's corner in the full costmap
            robot_cell=(row, col),
            start_cell=selection.start if selection.start is not None else (-1, -1),
            robot_xy=robot,
            resolution=self._grid_info.resolution,
            origin_xy=(self._grid_info.origin_x, self._grid_info.origin_y),
        )

    def _clear_costmaps(self) -> None:
        """Asks Nav2 to forget its live obstacles: a stale mark can wall in a robot standing in the open."""
        self.node.get_logger().info("Clearing Nav2's costmaps")
        for client in self._clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _count_visit(self) -> None:
        """
        Called once the new map has come in after arriving at a frontier. If the frontier is still there, nothing more
        can be seen from where the robot stood: it counts as a visit, and after visits_before_explored it is ignored.
        """
        target, self._visit_target = self._visit_target, None
        if target is None or not frontier_still_exists(self._clusters, target, FRONTIER_GONE_RADIUS_M):
            return  # no frontier left there: it was explored for real
        if self._bump(self._visits, target) >= self._p.visits_before_explored:
            self.node.get_logger().info(f"Visited ({target[0]:.1f}, {target[1]:.1f}), nothing more to see there")
            self._explored.append(target)

    # --
    # -- Asking the planner before driving
    # --

    def _skipped_for_now(self, now: float) -> list:
        """The frontiers the planner found no way to a short while ago (tried again after unreachable_retry_s)."""
        return [(entry[0], entry[1]) for entry in self._no_path if entry[3] > now]

    def _start_path_check(self, candidates: list, now: float) -> None:
        """
        Asks Nav2's planner for a path to each candidate, one after the other (a new request would replace a running
        one), and then drives to the one with the shortest path. The nearest in a straight line can be far to drive,
        and a frontier without a path is dropped at once, without driving to it first.
        """
        self._path_check = SimpleNamespace(candidates=list(candidates), index=0, results=[], handle=None, sent_at=now)
        self._send_path_request(self._path_check, now)

    def _send_path_request(self, check, now: float) -> None:
        goal, index = check.candidates[check.index], check.index
        request = ComputePathToPose.Goal()
        request.goal.header.frame_id = "map"
        request.goal.pose.position.x, request.goal.pose.position.y = goal.x, goal.y
        request.goal.pose.orientation.z, request.goal.pose.orientation.w = quaternion_from_yaw(goal.yaw)
        request.use_start = False  # from where the robot is now
        check.sent_at, check.handle = now, None
        sent = self._path_planner.send_goal_async(request)
        sent.add_done_callback(lambda future, c=check, i=index: self._on_path_response(future, c, i))

    def _on_path_response(self, future, check, index: int) -> None:
        handle = future.result()
        if check is not self._path_check or index != check.index:  # paused or replaced while the planner was answering
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._planner_problem("Nav2's planner refused the request")
            return
        check.handle = handle
        handle.get_result_async().add_done_callback(lambda done, c=check, i=index: self._on_path_result(done, c, i))

    def _on_path_result(self, future, check, index: int) -> None:
        if check is not self._path_check or index != check.index:
            return
        now = time.monotonic()
        answer = future.result()
        goal = check.candidates[index]
        poses = answer.result.path.poses
        if answer.status == GoalStatus.STATUS_SUCCEEDED and poses:
            length = sum(
                math.dist((a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y))
                for a, b in zip(poses, poses[1:])
            )
            check.results.append((goal, length))
        elif answer.result.error_code in NO_PATH_ERRORS or answer.status == GoalStatus.STATUS_SUCCEEDED:
            self._note_no_path(goal, now)
        else:
            self._planner_problem(f"Nav2's planner failed: {answer.result.error_code} {answer.result.error_msg}")
            return
        self._planner_failures = 0
        check.index += 1
        if check.index < len(check.candidates):
            self._send_path_request(check, now)
        else:
            self._finish_path_check(check, now)

    def _finish_path_check(self, check, now: float) -> None:
        self._path_check = None
        if not check.results:
            return  # no way to any of them: the next tick chooses again among the others
        goal, length = min(check.results, key=lambda result: result[1])
        self._send_goal(goal, now, length)

    def _continue_path_check(self, now: float) -> None:
        if now - self._path_check.sent_at > PATH_CHECK_TIMEOUT_S:
            self._planner_problem(f"Nav2's planner didn't answer in {PATH_CHECK_TIMEOUT_S:.0f} s")

    def _planner_problem(self, text: str) -> None:
        """The planner didn't answer or broke. That says nothing about the frontier, so it isn't counted against it."""
        self.node.get_logger().warning(text)
        self._cancel_path_check()
        self._planner_failures += 1
        if self._planner_failures >= PLANNER_FAILURES_BEFORE_STUCK:
            self._pause("stuck")

    def _cancel_path_check(self) -> None:
        if self._path_check is not None and self._path_check.handle is not None:
            self._path_check.handle.cancel_goal_async()
        self._path_check = None

    def _note_no_path(self, goal, now: float) -> None:
        """No path to the frontier: skip it for a while (the costmap may be behind), the second time give it up."""
        target = goal.target
        for entry in self._no_path:
            if math.dist((entry[0], entry[1]), target) < self._p.blacklist_radius_m:
                entry[2] += 1
                entry[3] = now + self._p.unreachable_retry_s
                break
        else:
            entry = [target[0], target[1], 1, now + self._p.unreachable_retry_s]
            self._no_path.append(entry)
        if entry[2] >= 2:
            self.node.get_logger().warning(f"Still no path to ({target[0]:.1f}, {target[1]:.1f}), giving up on it")
            self._no_path.remove(entry)
            self._failed.append(target)
        else:
            self.node.get_logger().info(
                f"No path to ({target[0]:.1f}, {target[1]:.1f}): skipped for {self._p.unreachable_retry_s:.0f} s"
            )
        self._markers_dirty = True

    # --
    # -- Driving
    # --

    def _send_goal(self, goal, now: float, path_length: float) -> None:
        self._goal, self._goal_handle, self._goal_started, self._accepted_at = goal, None, now, 0.0
        # Enough time for the path at the speed the robot really has, with a good margin for turns and recoveries
        self._goal_timeout_s = self._p.goal_time_base_s + self._p.goal_time_factor * path_length / EXPECTED_SPEED_MPS
        self._best_distance, self._progress_time, self._recoveries = math.inf, now, 0
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = "map"  # no time stamp: Nav2 then uses the newest data
        request.pose.pose.position.x, request.pose.pose.position.y = goal.x, goal.y
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = quaternion_from_yaw(goal.yaw)
        self.node.get_logger().info(
            f"Exploring: driving to ({goal.x:.1f}, {goal.y:.1f}), path {path_length:.1f} m, "
            f"{goal.clearance:.1f} m from the nearest wall, {len(self._clusters)} frontiers known"
        )
        sent = self._navigator.send_goal_async(
            request, feedback_callback=lambda message, sent_goal=goal: self._on_feedback(message, sent_goal)
        )
        sent.add_done_callback(lambda future, sent_goal=goal: self._on_goal_response(future, sent_goal))
        self._markers_dirty = True

    def _on_feedback(self, message, goal) -> None:
        """Nav2 says how far the goal still is and how often it had to recover. No progress is judged from this."""
        if goal is not self._goal:
            return
        feedback = message.feedback
        self._recoveries = feedback.number_of_recoveries
        if feedback.distance_remaining < self._best_distance - self._p.progress_m:
            self._best_distance, self._progress_time = feedback.distance_remaining, time.monotonic()

    def _on_goal_response(self, future, goal) -> None:
        handle = future.result()
        if goal is not self._goal:
            # Paused or replaced while Nav2 was answering. With no goal wanted any more, don't let it drive.
            if handle.accepted and self._goal is None:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._drop_goal()
            self._count_failure(goal)
            return
        self._goal_handle, self._accepted_at = handle, time.monotonic()
        self._progress_time = self._accepted_at
        handle.get_result_async().add_done_callback(
            lambda result, goal_handle=handle: self._on_result(result, goal_handle)
        )

    def _on_result(self, future, handle) -> None:
        if handle is not self._goal_handle:  # an old goal that was replaced or cancelled
            return
        goal = self._goal
        robot = self._robot_xy()
        outcome = classify_result(
            reached=future.result().status == GoalStatus.STATUS_SUCCEEDED,
            seconds_since_accepted=time.monotonic() - self._accepted_at,
            distance_to_goal_m=math.dist(robot, (goal.x, goal.y)) if robot else math.inf,
            near_goal_m=self._p.near_goal_m,
            spurious_s=self._p.spurious_abort_s,
        )
        self._drop_goal()

        if outcome == "reached":
            self._wait_map_after = time.monotonic()
            self._visit_target = goal.target  # counted when the new map is in (_count_visit)
            self._failure_places = []
        elif outcome == "spurious" and self._bump(self._spurious, goal.target) <= MAX_SPURIOUS_RETRIES:
            # Nav2 aborted at once but may still be driving: stop it, the next tick sends the goal again
            self.node.get_logger().info("Nav2 aborted right after accepting the goal - trying again")
            self._cancel_all_driving()
        else:
            self._count_failure(goal)

    def _abandon_goal(self, reason: str) -> None:
        """Stops driving to the current goal and counts it as a failure."""
        goal = self._goal
        self.node.get_logger().warning(f"{reason}, stopping the trip to ({goal.x:.1f}, {goal.y:.1f})")
        self._cancel_all_driving()
        self._drop_goal()
        self._count_failure(goal)

    def _count_failure(self, goal) -> None:
        """The first failure is retried by choosing again, the second gives the frontier up."""
        if self._bump(self._failures, goal.target) >= 2:
            self.node.get_logger().warning(f"Giving up on the frontier at ({goal.target[0]:.1f}, {goal.target[1]:.1f})")
            self._failed.append(goal.target)
            self._markers_dirty = True
        self._check_stuck()

    def _check_stuck(self) -> None:
        """Several failures with the robot staying where it is: not the frontiers' fault, the robot is stuck."""
        robot = self._robot_xy()
        if robot is None:
            return
        self._failure_places = (self._failure_places + [robot])[-STUCK_FAILURES:]
        if len(self._failure_places) == STUCK_FAILURES and all(
            math.dist(place, robot) < STUCK_RADIUS_M for place in self._failure_places
        ):
            self.node.get_logger().warning(f"{STUCK_FAILURES} failures in a row without getting anywhere")
            self._pause("stuck")

    def _bump(self, entries: list, target: tuple) -> int:
        """Counts one more event for the frontier near target and returns how many there are now."""
        for entry in entries:
            if math.dist((entry[0], entry[1]), target) < self._p.blacklist_radius_m:
                entry[2] += 1
                return entry[2]
        entries.append([target[0], target[1], 1])
        return 1

    def _drop_goal(self) -> None:
        """Forgets the goal. Its result is ignored from now on."""
        self._goal, self._goal_handle = None, None
        self._markers_dirty = True

    def _cancel_all_driving(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        for client in self._cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())  # empty goal id and time = every goal

    # --
    # -- Pause and resume
    # --

    def _pause(self, reason: str) -> None:
        self._cancel_all_driving()
        self._cancel_path_check()
        self._drop_goal()
        self._cut_off_since = None
        self._mode, self._pause_reason, self._pause_started = Mode.PAUSING, reason, time.monotonic()
        self.node.get_logger().info(f"Stopping ({reason})")

    def _continue_pausing(self, now: float) -> None:
        """Reports "paused" only when the robot stands, so manual driving doesn't fight Nav2 for /cmd_vel."""
        standing = now - self._last_motion_time > STOPPED_AFTER_S
        if standing or now - self._pause_started > PAUSE_TIMEOUT_S:
            self._mode = Mode.PAUSED

    def _resume(self) -> None:
        if self._mode != Mode.PAUSED:
            return
        if self._pause_reason == "time_limit":
            self._active_s = 0.0  # a new time window
        # Counters start again, but the frontiers already given up on or explored stay ignored: they were
        # unreachable (stairs, something the map doesn't show), and driving there again would get stuck again
        self._visits, self._failures, self._spurious, self._failure_places = [], [], [], []
        self._idle_checks, self._wait_map_after, self._visit_target = 0, None, None
        self._no_path, self._planner_failures = [], 0  # the planner is asked again about those
        self._map_changed = True
        self._mode, self._pause_reason = Mode.WAITING, "paused"
        self.node.get_logger().info("Exploring again")

    # --
    # -- Position, status and markers
    # --

    def _robot_xy(self):
        """The robot's position in the map, None if it isn't known yet. Latest transform: no clock needed."""
        try:
            translation = self._tf_buffer.lookup_transform("map", "base_footprint", Time()).transform.translation
        except TransformException:
            return None
        return translation.x, translation.y

    def status_text(self) -> str:
        return self._pause_reason if self._mode == Mode.PAUSED else self._mode.value

    def _publish_status(self, now: float) -> None:
        goal = self._goal
        text = json.dumps(
            {
                "status": self.status_text(),
                "waiting_for": self._waiting_for,
                "frontiers": len(self._clusters),
                "goal": {"x": round(goal.x, 2), "y": round(goal.y, 2)} if goal else None,
                "failed": len(self._failed),
                "explored": len(self._explored),
                "unreachable": len(self._no_path),  # no path found for now, tried again later
                "cut_off": self._selection.unreachable if self._selection else 0,  # no way there in the costmap
            }
        )
        if text != self._last_status or now - self._last_status_time > STATUS_PERIOD_S:
            self._status_publisher.publish(String(data=text))
            self._last_status, self._last_status_time = text, now
            self._markers_dirty = (
                True  # rviz doesn't keep markers: one that starts later sees them with the next heartbeat
            )

    def _publish_markers(self) -> None:
        self._markers_dirty = False

        def marker(namespace, kind, color, scale):
            item = Marker(ns=namespace, id=0, type=kind, action=Marker.ADD, color=color)
            item.header.frame_id = "map"
            item.scale.x = item.scale.y = item.scale.z = scale
            item.pose.orientation.w = 1.0
            return item

        frontiers = marker("frontiers", Marker.SPHERE_LIST, GREEN, 0.25)
        frontiers.points = [Point(x=x, y=y, z=0.1) for x, y in (c.centroid for c in self._clusters)]
        given_up = marker("given_up", Marker.SPHERE_LIST, RED, 0.35)
        given_up.points = [Point(x=x, y=y, z=0.1) for x, y in self._failed]

        no_path = marker("no_path", Marker.SPHERE_LIST, ORANGE, 0.3)
        no_path.points = [Point(x=entry[0], y=entry[1], z=0.1) for entry in self._no_path]

        goal = marker("goal", Marker.ARROW, BLUE, 0.15)
        if self._goal is not None:
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self._goal.x, self._goal.y, 0.2
            goal.pose.orientation.z, goal.pose.orientation.w = quaternion_from_yaw(self._goal.yaw)
            goal.scale.x = 0.7  # an arrow: x is its length
        else:
            goal.action = Marker.DELETE

        self._markers_publisher.publish(MarkerArray(markers=[frontiers, given_up, no_path, goal]))


# Main function - Called when executing file
def main():
    node_runner.run_node(lambda: AutonomousMappingNode())


if __name__ == "__main__":
    main()
