"""
Autonomous Mapping Node - the robot maps the floor by itself: it drives to the edge of the known map
(a frontier) again and again, until no frontier is left. RTAB-Map builds the map, Nav2 drives.

    /autonomous_mapping/command   (std_msgs/String) "pause", "resume" or "toggle". Pausing stops the robot,
                                  so it can be driven by hand (see autonomous_mapping_keys.py).
    /autonomous_mapping/status    (std_msgs/String, JSON; the last one is kept for late subscribers)
                                  {"status": "exploring", "frontiers": 4, "goal": {"x": 3.2, "y": -1.0}, ...}
                                  status: waiting, exploring, pausing, paused, done, stuck, time_limit
    /autonomous_mapping/frontiers (visualization_msgs/MarkerArray) frontiers, goal and given-up goals for rviz

"done" = no frontier left, "stuck" = frontiers are left that Nav2 failed to reach, or the robot is stuck and
doesn't get anywhere, "time_limit" = ran out of time. In all three the robot stands still and waits: the user
can save the map, or drive to missed areas (or free the robot) and send "resume". Frontiers given up on stay
given up after a resume (red in rviz). The node never stops the script by itself.

The frontier logic is in frontiers.py. This node only talks to ROS: it reads /map, sends goals to Nav2
and reports what it does. It uses no simulation time: timeouts use the wall clock and the robot's position
is the latest transform.
"""

import json
import math
import time
from enum import Enum
from types import SimpleNamespace

import numpy as np
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
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
    find_frontier_clusters,
    frontier_still_exists,
    wall_distance_m,
)
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.planner.planner_node import quaternion_from_yaw

COMMAND_TOPIC = "/autonomous_mapping/command"
STATUS_TOPIC = "/autonomous_mapping/status"
MARKERS_TOPIC = "/autonomous_mapping/frontiers"

TICK_PERIOD_S = 1.0  # how often the explorer re-checks what to do
STATUS_PERIOD_S = 2.0  # the status is repeated this often even if nothing changed
RTABMAP_SILENT_S = 5.0  # no /info for this long: RTAB-Map is busy, don't choose new goals
MIN_KNOWN_FREE_M2 = 5.0  # a map with less free area is just starting, not finished (slam_toolbox's first scan: ~10 m2)
STOPPED_AFTER_S = 1.0  # the robot counts as standing after this long without motion
PAUSE_TIMEOUT_S = 4.0  # pausing is reported after this long even if the robot still moves
MOVING_SPEED = 0.03  # m/s (or rad/s)
NEW_MAP_WAIT_S = 3.0  # after arriving, wait this long for a map that shows what the robot saw
MAX_SPURIOUS_RETRIES = 3  # per frontier, then an early abort counts as a failure
NO_PROGRESS_S = 20.0  # a goal with no robot motion for this long has failed (the goal timeout is far longer)
STUCK_FAILURES = 3  # this many failures within STUCK_RADIUS_M of each other mean the robot is stuck
STUCK_RADIUS_M = 0.7
FRONTIER_GONE_RADIUS_M = 1.0  # a frontier this near the goal's frontier is "the same one"

# Keep the last status for late subscribers (the keys node, the website)
STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# /map is published once per change, so a node that starts later needs the last one
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

GREEN = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)
RED = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)


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
            goal_timeout_s=node.declare_parameter("goal_timeout_s", 90.0).value,
            time_limit_s=node.declare_parameter("time_limit_s", 1800.0).value,
            done_checks=node.declare_parameter("done_checks", 3).value,
            start_paused=node.declare_parameter("start_paused", False).value,
            # false with slam_toolbox: it sends no /info, so waiting for it would never end
            wait_for_rtabmap=node.declare_parameter("wait_for_rtabmap", True).value,
        )

        self._status_publisher = node.create_publisher(String, STATUS_TOPIC, STATUS_QOS)
        self._markers_publisher = node.create_publisher(MarkerArray, MARKERS_TOPIC, 1)
        node.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        node.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        # Best effort connects to publishers of either kind, so a QoS mismatch can't silence these two
        node.create_subscription(Info, "/info", self._on_info, qos_profile_sensor_data)  # RTAB-Map's sign of life
        node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._navigator = ActionClient(node, NavigateToPose, "navigate_to_pose")
        # Cancelling the navigate goal alone isn't enough: after an abort the controller can keep driving
        # its path. A cancel request with an empty goal id cancels every goal of that action.
        self._cancel_clients = [
            node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal"),
            node.create_client(CancelGoal, "/follow_path/_action/cancel_goal"),
        ]

        # The map
        self._grid = None
        self._grid_info = None
        self._wall_distance = None
        self._map_time = 0.0  # when the last map arrived (time.monotonic)
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
        # What happened per frontier: [x, y, count]. Reached twice without the frontier going away means
        # nothing more can be seen from there ("explored"); two failures mean Nav2 can't get there ("failed").
        self._visits, self._failures, self._spurious = [], [], []
        self._explored, self._failed = [], []
        self._failure_places = []  # where the robot was at its latest failures, to notice it is stuck

        self._selection = None
        self._last_status = ""
        self._last_status_time = 0.0
        self._markers_dirty = True

        node.create_timer(TICK_PERIOD_S, self._tick)

    # --
    # -- Subscriptions
    # --

    def _on_map(self, message: OccupancyGrid) -> None:
        self._grid = np.asarray(message.data, dtype=np.int8).reshape(message.info.height, message.info.width)
        self._grid_info = GridInfo(
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
        )
        self._map_time = time.monotonic()
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
            return "map"
        if need_rtabmap and self._p.wait_for_rtabmap and now - self._info_time > RTABMAP_SILENT_S:
            return "rtabmap"
        if not self._navigator.server_is_ready():
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
        self._wall_distance = wall_distance_m(self._grid, self._grid_info)
        self._clusters = find_frontier_clusters(
            self._grid, self._grid_info, self._wall_distance, self._p.min_frontier_m, self._p.max_wall_distance_m
        )
        self._clusters_time = now
        self._markers_dirty = True

    def _supervise_goal(self, now: float) -> None:
        goal = self._goal
        if now - self._goal_started > self._p.goal_timeout_s:
            self._abandon_goal(f"No arrival after {self._p.goal_timeout_s:.0f} s")
        elif self._accepted_at > 0 and now - max(self._accepted_at, self._last_motion_time) > NO_PROGRESS_S:
            # Nav2 is trying but the robot doesn't move, e.g. it stands against something its map doesn't show
            self._abandon_goal(f"The robot hasn't moved for {NO_PROGRESS_S:.0f} s")
        elif self._clusters_time > self._goal_started and not frontier_still_exists(
            self._clusters, goal.target, FRONTIER_GONE_RADIUS_M
        ):
            # Seen from a distance already: no need to drive all the way. A new goal replaces this one in Nav2.
            self._drop_goal()
            if not self._pick_next_goal(now):
                self._cancel_all_driving()

    def _pick_next_goal(self, now: float) -> bool:
        """Chooses and sends the next goal. Returns True if one was sent."""
        # After arriving, the map may not show yet what the robot saw: the same frontier would look unchanged
        if self._wait_map_after is not None:
            if self._map_time < self._wait_map_after and now - self._wait_map_after < NEW_MAP_WAIT_S:
                return False
            self._wait_map_after = None
        robot = self._robot_xy()
        if robot is None:
            return False

        selection = choose_goal(
            self._grid,
            self._grid_info,
            self._wall_distance,
            robot,
            self._clusters,
            self._explored + self._failed,
            clearance_m=self._p.clearance_m,
            preferred_clearance_m=self._p.preferred_clearance_m,
            pullback_m=self._p.pullback_m,
            blacklist_radius_m=self._p.blacklist_radius_m,
        )
        self._selection = selection
        if selection.goal is None:
            self._idle_checks += 1
            if self._idle_checks >= self._p.done_checks:
                # Frontiers left that Nav2 failed to reach are not "done": the user should look at them
                stuck = bool(self._failed) and selection.blacklisted > 0
                self._pause("stuck" if stuck else "done")
            return False

        self._idle_checks = 0
        self._send_goal(selection.goal, now)
        return True

    def _send_goal(self, goal, now: float) -> None:
        self._goal, self._goal_handle, self._goal_started, self._accepted_at = goal, None, now, 0.0
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = "map"  # no time stamp: Nav2 then uses the newest data
        request.pose.pose.position.x, request.pose.pose.position.y = goal.x, goal.y
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = quaternion_from_yaw(goal.yaw)
        self.node.get_logger().info(
            f"Exploring: driving to ({goal.x:.1f}, {goal.y:.1f}), {goal.distance:.1f} m away, "
            f"{goal.clearance:.1f} m from the nearest wall, {len(self._clusters)} frontiers known"
        )
        sent = self._navigator.send_goal_async(request)
        sent.add_done_callback(lambda future, sent_goal=goal: self._on_goal_response(future, sent_goal))
        self._markers_dirty = True

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
            self._failure_places = []
            if self._bump(self._visits, goal.target) >= 2:
                self.node.get_logger().info(
                    f"Visited ({goal.target[0]:.1f}, {goal.target[1]:.1f}) twice, nothing more to see"
                )
                self._explored.append(goal.target)
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
        self._drop_goal()
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
        self._idle_checks, self._wait_map_after = 0, None
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

        goal = marker("goal", Marker.ARROW, BLUE, 0.15)
        if self._goal is not None:
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self._goal.x, self._goal.y, 0.2
            goal.pose.orientation.z, goal.pose.orientation.w = quaternion_from_yaw(self._goal.yaw)
            goal.scale.x = 0.7  # an arrow: x is its length
        else:
            goal.action = Marker.DELETE

        self._markers_publisher.publish(MarkerArray(markers=[frontiers, given_up, goal]))


# Main function - Called when executing file
def main():
    node_runner.run_node(lambda: AutonomousMappingNode())


if __name__ == "__main__":
    main()
