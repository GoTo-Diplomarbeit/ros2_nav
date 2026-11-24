"""
Removes the robot's own body from the 2D scan.

    pointcloud_to_laserscan -> /scan_raw -> scan_body_filter -> /scan -> Nav2, slam_toolbox

The Velodyne sits about 0.2 m ahead of the robot's centre and its minimum range is measured from the sensor, so the
rear legs (about 0.3 - 0.45 m from the centre) are in the 2D slice. In some leg poses 20 - 30 points lie inside the
footprint: the collision monitor then blocks every movement and Nav2 sees an obstacle under the robot.

Every point inside a box around the robot (in the scan's frame, base_footprint) becomes "no return" (inf).
A real obstacle can't be in that box without already touching the robot, and the lidar can't see that close anyway.
Parameters: half_length_m and half_width_m of the box, measured from the robot's centre (footprint: 0.38 x 0.20).
"""

import math

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper

INPUT_TOPIC = "/scan_raw"
OUTPUT_TOPIC = "/scan"
BODY_FRAME = "base_footprint"  # the box is in this frame; the scan is made in it (pointcloud_to_laserscan)


def remove_body_points(
    ranges: np.ndarray, angle_min: float, angle_increment: float, half_length_m: float, half_width_m: float
) -> np.ndarray:
    """A copy of ranges with every point inside the box |x| <= half_length_m, |y| <= half_width_m set to inf."""
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    # inf and nan stay as they are: they have no position (and a nan compares False, so it is never removed)
    with np.errstate(invalid="ignore"):
        inside = (np.abs(ranges * np.cos(angles)) <= half_length_m) & (np.abs(ranges * np.sin(angles)) <= half_width_m)
    filtered = ranges.copy()
    filtered[inside] = math.inf
    return filtered


class ScanBodyFilterNode(NodeWrapper):
    def __init__(self):
        # follow_state=False: the filter has to work in every mission state
        super().__init__("scan_body_filter", self._setup_channels, follow_state=False)

    def _setup_channels(self):
        node = self.node
        self._half_length_m = node.declare_parameter("half_length_m", 0.5).value
        self._half_width_m = node.declare_parameter("half_width_m", 0.3).value
        self._warned_frame = False
        # Same QoS as pointcloud_to_laserscan's /scan, so every subscriber that worked with it still connects
        self._publisher = node.create_publisher(LaserScan, OUTPUT_TOPIC, qos_profile_sensor_data)
        node.create_subscription(LaserScan, INPUT_TOPIC, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, message: LaserScan) -> None:
        if message.header.frame_id != BODY_FRAME:
            # The box is only right in the robot's frame: pass the scan on unchanged rather than cut the wrong points
            if not self._warned_frame:
                self._warned_frame = True
                self.node.get_logger().warning(
                    f"The scan is in frame '{message.header.frame_id}', not '{BODY_FRAME}': passed on unfiltered"
                )
        else:
            message.ranges = remove_body_points(
                np.asarray(message.ranges, dtype=np.float32),
                message.angle_min,
                message.angle_increment,
                self._half_length_m,
                self._half_width_m,
            ).tolist()
        self._publisher.publish(message)


# Main function - Called when executing file
def main():
    node_runner.run_node(lambda: ScanBodyFilterNode())


if __name__ == "__main__":
    main()
