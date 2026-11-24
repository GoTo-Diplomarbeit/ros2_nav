"""
Drift Monitor - measures how far the robot's estimated position is from where the simulator really has it.

    /odom/ground_truth   (nav_msgs/Odometry)  the robot's real pose in Gazebo, bridged by the simulator
    /cmd_vel             (geometry_msgs/Twist) what Nav2 (or the keys) tells the robot to do
    TF map -> base_footprint                  the pose SLAM (RTAB-Map or slam_toolbox) believes
    TF odom -> base_footprint                 the pose of the odometry alone (what SLAM starts from)

The three poses are in different frames, so each of them is laid over the real pose at the first sample (same start
position and heading). From then on the difference to the real pose is the drift. Once per second a line is added to a
csv file, and a summary is written next to it (and printed when the node stops):

    time_s, travelled_m, slam_pos_err_m, slam_heading_err_deg, odom_pos_err_m, odom_heading_err_deg, tilt_deg

The same line also has the three positions themselves, each in the simulator's frame (the estimates laid over the real
pose as described above): real_x, real_y, slam_x, slam_y, odom_x, odom_y. The errors only say THAT an estimate is off;
the positions say which of the three moved. For example, a robot that wiggles in a tight spot moves in the simulator,
and whether the odometry saw that, or SLAM, shows in how far each of the three moved in the same seconds.
The last three columns are the three headings (real_yaw_deg, slam_yaw_deg, odom_yaw_deg), because wiggling is mostly
turning: a robot that turns in place hardly changes its position. Before them come cmd_vx and cmd_wz, the latest
command (m/s, rad/s) at the time of the sample: if the real robot creeps while the command is about zero, the gait is
at fault; if it creeps while a command is given, the path Nav2 plans is.

tilt_deg is how far the real robot is tipped over (the larger of roll and pitch, from the simulator's real pose): a
walking robot rocks by about 5 degrees, one that has fallen over is above 45. A fallen robot makes everything else
wrong (its lidar sees the floor as walls), so the summary also says how long the robot spent tipped over.

Why: slam_toolbox drifts and RTAB-Map doesn't, and the cause is not known. With the numbers one can see whether the
error is in the heading or in the distance, whether it grows while turning on the spot, and whether SLAM corrects
the odometry or makes it worse. Only works in the simulator (a real robot has no ground truth).

Limit: the real pose is the one of the robot's base link, the estimates are those of base_footprint. If the two are
not at the same place in the robot's body, the error has a constant part that changes with the heading.
"""

import csv
import math
import time
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper

SAMPLE_PERIOD_S = 1.0
SUMMARY_PERIOD_S = 10.0  # the summary file is made new this often, so it exists even if the node is killed
GROUND_TRUTH_TOPIC = "/odom/ground_truth"
COMMAND_TOPIC = "/cmd_vel"
CSV_HEADER = [
    "time_s",
    "travelled_m",
    "slam_pos_err_m",
    "slam_heading_err_deg",
    "odom_pos_err_m",
    "odom_heading_err_deg",
    "tilt_deg",
    "real_x",
    "real_y",
    "slam_x",
    "slam_y",
    "odom_x",
    "odom_y",
    "real_yaw_deg",
    "slam_yaw_deg",
    "odom_yaw_deg",
    "cmd_vx",
    "cmd_wz",
]
TIPPED_DEG = 30.0  # above this the robot counts as tipped over

Pose = tuple[float, float, float]  # x (m), y (m), heading (rad)


def yaw_of(x: float, y: float, z: float, w: float) -> float:
    """Heading (rotation around the vertical axis) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_of(x: float, y: float, z: float, w: float) -> float:
    """How far a quaternion tips the robot over, in degrees: the larger of its roll and its pitch."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(max(abs(roll), abs(pitch)))


def wrap_angle(angle: float) -> float:
    """The angle in (-pi, pi], so that 359 degrees and -1 degree are the same."""
    return math.atan2(math.sin(angle), math.cos(angle))


def align(estimate: Pose, real_start: Pose, estimate_start: Pose) -> Pose:
    """
    The estimate in the real frame. The estimate is in its own frame, so its start pose is laid over the real start
    pose: shifted so that both start at the same place, and turned by the difference of the start headings. Nothing
    else is adjusted.
    """
    turn = real_start[2] - estimate_start[2]
    shift_x, shift_y = estimate[0] - estimate_start[0], estimate[1] - estimate_start[1]
    return (
        real_start[0] + math.cos(turn) * shift_x - math.sin(turn) * shift_y,
        real_start[1] + math.sin(turn) * shift_x + math.cos(turn) * shift_y,
        wrap_angle(estimate[2] + turn),
    )


def align_and_error(real: Pose, estimate: Pose, real_start: Pose, estimate_start: Pose) -> tuple[float, float]:
    """
    How far an estimate is from the real pose: (position error in m, heading error in rad, positive = the estimate
    has turned too far left). The estimate is laid over the real pose first, see align.
    """
    aligned = align(estimate, real_start, estimate_start)
    return math.hypot(aligned[0] - real[0], aligned[1] - real[1]), wrap_angle(aligned[2] - real[2])


def summarize(rows: list[list[float]]) -> str:
    """A few lines about a run: the final and the largest error, and the error per 10 m driven."""
    if not rows:
        return "No samples: no ground truth or no TF received."
    last = rows[-1]
    travelled = last[1]
    lines = [f"samples: {len(rows)}, run time: {last[0]:.0f} s, driven: {travelled:.1f} m"]
    for name, position_column in (("slam", 2), ("odom", 4)):
        position = [row[position_column] for row in rows]
        heading = [abs(row[position_column + 1]) for row in rows]
        per_10_m = f"{position[-1] / travelled * 10.0:.2f} m" if travelled > 1.0 else "n/a (drove less than 1 m)"
        lines.append(
            f"{name}: position error at the end {position[-1]:.2f} m (largest {max(position):.2f} m, "
            f"{per_10_m} per 10 m driven), heading error at the end {rows[-1][position_column + 1]:.1f} deg "
            f"(largest {max(heading):.1f} deg)"
        )
    tilts = [row[6] for row in rows if len(row) > 6]
    if tilts:
        tipped = sum(1 for tilt in tilts if tilt > TIPPED_DEG)
        lines.append(
            f"tilt: largest {max(tilts):.0f} deg, tipped over (more than {TIPPED_DEG:.0f} deg) in {tipped} of "
            f"{len(tilts)} samples" + (" - THE ROBOT FELL OVER" if tipped else "")
        )
    return "\n".join(lines)


class DriftMonitorNode(NodeWrapper):
    def __init__(self):
        # follow_state=False: this is a measuring tool for the simulator, it has nothing to do with the mission
        super().__init__("drift_monitor", self._setup_channels, follow_state=False)

    def _setup_channels(self):
        node = self.node
        self._csv_path = Path(node.declare_parameter("csv_path", "drift.csv").value)
        self._summary_path = self._csv_path.with_name(self._csv_path.stem + "_summary.txt")

        node.create_subscription(Odometry, GROUND_TRUTH_TOPIC, self._on_ground_truth, qos_profile_sensor_data)
        node.create_subscription(Twist, COMMAND_TOPIC, self._on_command, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._command = (0.0, 0.0)  # the latest command: forward speed (m/s), turning speed (rad/s)
        self._real = None  # the latest real pose
        self._tilt_deg = 0.0  # how far the robot is tipped over (the latest)
        self._starts = None  # (real, slam, odom) at the first sample
        self._travelled_m = 0.0
        self._last_real = None
        self._started_at = time.monotonic()
        self._last_summary = 0.0
        self._rows = []

        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self._csv_path.open("w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._csv_file.flush()

        node.create_timer(SAMPLE_PERIOD_S, self._sample)

    def _on_ground_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        q = pose.orientation
        self._real = (pose.position.x, pose.position.y, yaw_of(q.x, q.y, q.z, q.w))
        self._tilt_deg = tilt_of(q.x, q.y, q.z, q.w)

    def _on_command(self, message: Twist) -> None:
        self._command = (message.linear.x, message.angular.z)

    def _lookup(self, parent: str) -> Pose | None:
        """The latest pose of base_footprint in the given frame, None if TF doesn't have it (yet)."""
        try:
            transform = self._tf_buffer.lookup_transform(parent, "base_footprint", Time()).transform
        except TransformException:
            return None
        q = transform.rotation
        return transform.translation.x, transform.translation.y, yaw_of(q.x, q.y, q.z, q.w)

    def _sample(self) -> None:
        slam, odom = self._lookup("map"), self._lookup("odom")
        if self._real is None or slam is None or odom is None:
            return  # waiting for all three
        real = self._real

        if self._last_real is not None:
            self._travelled_m += math.hypot(real[0] - self._last_real[0], real[1] - self._last_real[1])
        self._last_real = real
        if self._starts is None:
            self._starts = (real, slam, odom)

        slam_position, slam_heading = align_and_error(real, slam, self._starts[0], self._starts[1])
        odom_position, odom_heading = align_and_error(real, odom, self._starts[0], self._starts[2])
        slam_aligned = align(slam, self._starts[0], self._starts[1])
        odom_aligned = align(odom, self._starts[0], self._starts[2])
        row = [
            round(time.monotonic() - self._started_at, 1),
            round(self._travelled_m, 3),
            round(slam_position, 3),
            round(math.degrees(slam_heading), 2),
            round(odom_position, 3),
            round(math.degrees(odom_heading), 2),
            round(self._tilt_deg, 1),
            round(real[0], 3),
            round(real[1], 3),
            round(slam_aligned[0], 3),
            round(slam_aligned[1], 3),
            round(odom_aligned[0], 3),
            round(odom_aligned[1], 3),
            round(math.degrees(real[2]), 1),
            round(math.degrees(slam_aligned[2]), 1),
            round(math.degrees(odom_aligned[2]), 1),
            round(self._command[0], 3),
            round(self._command[1], 3),
        ]
        self._rows.append(row)
        self._csv.writerow(row)
        self._csv_file.flush()

        if time.monotonic() - self._last_summary > SUMMARY_PERIOD_S:
            self._write_summary()

    def _write_summary(self) -> None:
        self._last_summary = time.monotonic()
        self._summary_path.write_text(summarize(self._rows) + "\n")

    def destroy_node(self) -> None:
        self._write_summary()
        print(summarize(self._rows), flush=True)
        self._csv_file.close()
        super().destroy_node()


# Main function - Called when executing file
def main():
    node_runner.run_node(lambda: DriftMonitorNode())


if __name__ == "__main__":
    main()
