"""
Frontier finding for autonomous mapping. Plain numpy/scipy, no ROS, so it can be tested on small grids.

A frontier is a known free cell next to an unknown cell: the edge of what the robot has seen. Driving
there makes the lidar reveal what lies behind it, and the frontier moves on.

Grid convention (nav_msgs/OccupancyGrid): grid[row, col], rows run in the y direction,
value -1 = unknown, 0 = free, 100 = occupied.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

OCCUPIED_VALUE = 50  # cells at or above this are walls (RTAB-Map publishes only 0, 100 and -1)
EIGHT_CONNECTED = np.ones((3, 3), dtype=bool)  # frontier cells touching diagonally belong together
MIN_NEAR_WALL_FRACTION = 0.5  # share of a frontier's cells that must be near a wall to count


@dataclass(frozen=True)
class GridInfo:
    """Where the grid lies in the world."""

    resolution: float  # metres per cell
    origin_x: float  # world position of the corner of cell (0, 0)
    origin_y: float

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        """(row, col) of the cell that contains the world point."""
        return (
            math.floor((y - self.origin_y) / self.resolution),
            math.floor((x - self.origin_x) / self.resolution),
        )

    def to_world(self, row: float, col: float) -> tuple[float, float]:
        """World position (x, y) of the centre of a cell."""
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )


@dataclass(frozen=True)
class Cluster:
    """Frontier cells that touch each other."""

    centroid: tuple[float, float]  # world position (x, y)
    size_m: float  # roughly the length of the frontier


@dataclass(frozen=True)
class Goal:
    """Where to drive to explore one frontier."""

    x: float
    y: float
    yaw: float  # facing the frontier
    target: tuple[float, float]  # centroid of that frontier, to recognise it again later
    distance: float  # straight-line distance from the robot
    clearance: float  # how far the goal cell is from the nearest wall, in metres


@dataclass(frozen=True)
class Selection:
    """The result of choosing a goal. goal is None if no frontier can be driven to."""

    goal: Goal | None
    clusters: int  # frontiers of at least the minimum size
    unreachable: int  # no way there through known free space
    blacklisted: int  # reachable, but given up on earlier


def wall_distance_m(grid: np.ndarray, info: GridInfo) -> np.ndarray:
    """Distance of every cell to the nearest wall in metres (infinite if the map has no wall yet)."""
    occupied = grid >= OCCUPIED_VALUE
    if not occupied.any():
        return np.full(grid.shape, np.inf)
    return ndimage.distance_transform_edt(~occupied) * info.resolution


def find_frontier_clusters(
    grid: np.ndarray,
    info: GridInfo,
    wall_distance: np.ndarray,
    min_size_m: float,
    max_wall_distance_m: float = math.inf,
) -> list[Cluster]:
    """
    All frontiers at least min_size_m long that lie next to walls (within max_wall_distance_m).

    Short ones are mostly lidar noise, and the robot would chase them forever. A real frontier is an
    opening between walls: a door, the end of a corridor, a gap behind a corner. The sparse fans of
    lidar rays that reach through a window or door lie in open space and are thousands of frontier
    cells long, so they would keep exploration from ever finishing. Measured on a finished map: real
    ones have all their cells within 0.4 m of a wall, fans mostly 1 m or more away.
    """
    unknown = grid < 0
    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]

    free = (grid >= 0) & (grid < OCCUPIED_VALUE)
    labels, count = ndimage.label(free & touches_unknown, structure=EIGHT_CONNECTED)
    if count == 0:
        return []

    # Group the frontier cells by label: sort them, then cut where the label changes
    rows, cols = np.nonzero(labels)
    order = np.argsort(labels[rows, cols], kind="stable")
    rows, cols = rows[order], cols[order]
    cuts = np.flatnonzero(np.diff(labels[rows, cols])) + 1

    clusters = []
    for cluster_rows, cluster_cols in zip(np.split(rows, cuts), np.split(cols, cuts)):
        size_m = len(cluster_rows) * info.resolution
        if size_m < min_size_m:
            continue
        near_wall = wall_distance[cluster_rows, cluster_cols] <= max_wall_distance_m
        if near_wall.mean() < MIN_NEAR_WALL_FRACTION:
            continue
        centroid = info.to_world(cluster_rows.mean(), cluster_cols.mean())
        clusters.append(Cluster(centroid=centroid, size_m=size_m))
    return clusters


def passable_mask(grid: np.ndarray, wall_distance: np.ndarray, clearance_m: float) -> np.ndarray:
    """Free cells at least clearance_m away from every wall, i.e. where the robot's centre may be."""
    free = (grid >= 0) & (grid < OCCUPIED_VALUE)
    return free & (wall_distance >= clearance_m)


def nearest_true(mask: np.ndarray, row: int, col: int, max_cells: int) -> tuple[int, int] | None:
    """The cell of mask that is True and closest to (row, col), searched within max_cells."""
    row_start, row_end = max(row - max_cells, 0), min(row + max_cells + 1, mask.shape[0])
    col_start, col_end = max(col - max_cells, 0), min(col + max_cells + 1, mask.shape[1])
    if row_start >= row_end or col_start >= col_end:  # the point is far outside the grid
        return None
    hits = np.argwhere(mask[row_start:row_end, col_start:col_end]) + (row_start, col_start)
    if len(hits) == 0:
        return None
    nearest = hits[((hits[:, 0] - row) ** 2 + (hits[:, 1] - col) ** 2).argmin()]
    return int(nearest[0]), int(nearest[1])


def pick_goal_cell(
    reachable: np.ndarray, wall_distance: np.ndarray, centre: tuple[int, int], radius_cells: int, preferred_m: float
) -> tuple[int, int] | None:
    """
    The cell to drive to for a frontier whose centre is `centre`: one of the reachable cells within
    radius_cells, preferring open space over a short trip.

    A frontier sits at a wall or a door frame, so the cell nearest to it is always right at the minimum
    wall distance. Instead take the nearest cell that is at least preferred_m from every wall; if there
    is none (a narrow corridor), the cell with the most room.
    """
    row, col = centre
    row_start, row_end = max(row - radius_cells, 0), min(row + radius_cells + 1, reachable.shape[0])
    col_start, col_end = max(col - radius_cells, 0), min(col + radius_cells + 1, reachable.shape[1])
    if row_start >= row_end or col_start >= col_end:  # the frontier is far outside the grid
        return None
    cells = np.argwhere(reachable[row_start:row_end, col_start:col_end]) + (row_start, col_start)
    if len(cells) == 0:
        return None
    distance_sq = (cells[:, 0] - row) ** 2 + (cells[:, 1] - col) ** 2
    within = distance_sq <= radius_cells**2  # a circle, not the square window
    cells, distance_sq = cells[within], distance_sq[within]
    if len(cells) == 0:
        return None

    room = wall_distance[cells[:, 0], cells[:, 1]]
    candidates = np.flatnonzero(room >= preferred_m)
    if len(candidates) == 0:
        candidates = np.flatnonzero(room >= room.max() - 1e-9)  # the roomiest cells; ties go to the nearest
    best = candidates[distance_sq[candidates].argmin()]
    return int(cells[best, 0]), int(cells[best, 1])


def choose_goal(
    grid: np.ndarray,
    info: GridInfo,
    wall_distance: np.ndarray,
    robot_xy: tuple[float, float],
    clusters: list[Cluster],
    blacklist: list[tuple[float, float]],
    clearance_m: float,
    preferred_clearance_m: float,
    pullback_m: float,
    blacklist_radius_m: float,
) -> Selection:
    """
    Picks the nearest frontier the robot can drive to.

    Nav2 plans only through known free space and keeps away from walls, so the goal is not the frontier
    cell itself (often right next to a wall or a door frame). The robot may only be where it is at least
    clearance_m from every wall, so a gap narrower than twice that is not passable. The goal cell is
    chosen by pick_goal_cell within pullback_m of the frontier, in open space where possible
    (preferred_clearance_m). A frontier with no such cell connected to the robot counts as unreachable.
    """
    passable = passable_mask(grid, wall_distance, clearance_m)
    robot_row, robot_col = info.to_cell(*robot_xy)
    start = nearest_true(passable, robot_row, robot_col, math.ceil(1.0 / info.resolution))
    if start is None:  # robot off the map or squeezed between walls
        return Selection(goal=None, clusters=len(clusters), unreachable=len(clusters), blacklisted=0)

    regions, _ = ndimage.label(passable)  # 4-connected: the robot can't squeeze through a diagonal gap
    reachable = regions == regions[start]

    best: Goal | None = None
    unreachable = blacklisted = 0
    pullback_cells = math.ceil(pullback_m / info.resolution)
    for cluster in clusters:
        cell = pick_goal_cell(
            reachable, wall_distance, info.to_cell(*cluster.centroid), pullback_cells, preferred_clearance_m
        )
        if cell is None:
            unreachable += 1
            continue
        if any(math.dist(cluster.centroid, given_up) < blacklist_radius_m for given_up in blacklist):
            blacklisted += 1
            continue

        goal_x, goal_y = info.to_world(*cell)
        distance = math.dist(robot_xy, (goal_x, goal_y))
        if best is None or distance < best.distance:
            # Face the frontier; if the goal is on top of it, face away from the robot instead
            if math.dist((goal_x, goal_y), cluster.centroid) > 0.05:
                yaw = math.atan2(cluster.centroid[1] - goal_y, cluster.centroid[0] - goal_x)
            else:
                yaw = math.atan2(goal_y - robot_xy[1], goal_x - robot_xy[0])
            best = Goal(
                x=goal_x,
                y=goal_y,
                yaw=yaw,
                target=cluster.centroid,
                distance=distance,
                clearance=float(wall_distance[cell]),
            )

    return Selection(goal=best, clusters=len(clusters), unreachable=unreachable, blacklisted=blacklisted)


def frontier_still_exists(clusters: list[Cluster], target: tuple[float, float], radius_m: float) -> bool:
    """Whether a frontier is still near target. A frontier shrinks and moves as the robot sees more."""
    return any(math.dist(cluster.centroid, target) < radius_m for cluster in clusters)


def classify_result(
    reached: bool, seconds_since_accepted: float, distance_to_goal_m: float, near_goal_m: float, spurious_s: float
) -> str:
    """
    What a finished Nav2 trip means for the explorer: "reached", "spurious" or "failed".

    Nav2's behaviour tree can abort a few ms after accepting a goal because the controller answered too
    slowly (default_server_timeout), while the robot drives on and arrives. So an abort this early is
    "spurious": not the frontier's fault, drive there again. An abort close to the goal is "reached".
    """
    if reached or distance_to_goal_m <= near_goal_m:
        return "reached"
    if seconds_since_accepted < spurious_s:
        return "spurious"
    return "failed"
