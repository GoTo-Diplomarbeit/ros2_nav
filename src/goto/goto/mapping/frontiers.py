"""
Frontier finding for autonomous mapping. Plain numpy/scipy, no ROS, so it can be tested on small grids.

A frontier is a known free cell next to an unknown cell: the edge of what the robot has seen. Driving
there makes the lidar reveal what lies behind it, and the frontier moves on.

Grid convention (nav_msgs/OccupancyGrid): grid[row, col], rows run in the y direction,
value -1 = unknown, 0 = free, 100 = occupied.

The explorer works on Nav2's global costmap, not on the SLAM map: that is the grid the planner really plans on
(inflated walls, live obstacles), so a goal chosen here is one the planner accepts. costmap_to_grid turns the costmap
into the grid above, and costmap_wall_distance_m measures the distance to the real obstacles in it.
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

OCCUPIED_VALUE = 50  # cells at or above this are walls (RTAB-Map publishes only 0, 100 and -1)
LETHAL_COST = 100  # costmap value of a cell with an obstacle in it
INSCRIBED_COST = 99  # costmap value of a cell where the robot's centre can't be: its body would touch an obstacle
EIGHT_CONNECTED = np.ones((3, 3), dtype=bool)  # frontier cells touching diagonally belong together
MIN_NEAR_WALL_FRACTION = 0.5  # share of a frontier's cells that must be near a wall to count
TOUCH_REACHABLE_M = 0.15  # a frontier counts as reachable if one of its cells is this near to where the robot can go


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
    # (rows, cols) of its cells, to check whether the robot can get next to them. Not part of ==, it is only a detail.
    cells: tuple[np.ndarray, np.ndarray] | None = field(default=None, compare=False, repr=False)


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

    goal: Goal | None  # the nearest candidate
    clusters: int  # frontiers of at least the minimum size
    unreachable: int  # no way there through known free space
    blacklisted: int  # reachable, but given up on earlier
    # The nearest few goals, nearest first, for a better check than "nearest in a straight line" (the planner's path
    # length). goal is the first of them.
    candidates: list[Goal] = field(default_factory=list)
    # For finding out why the robot sees no way anywhere: where the search for the robot's region started (the robot's
    # own cell if the robot may be there, else the nearest cell it may use; None = there is none) and how big it is
    start: tuple[int, int] | None = None
    reachable_m2: float = 0.0


def _distance_to_m(occupied: np.ndarray, info: GridInfo) -> np.ndarray:
    """Distance of every cell to the nearest True cell of the mask in metres (infinite if there is none)."""
    if not occupied.any():
        return np.full(occupied.shape, np.inf)
    return ndimage.distance_transform_edt(~occupied) * info.resolution


def wall_distance_m(grid: np.ndarray, info: GridInfo) -> np.ndarray:
    """Distance of every cell to the nearest wall in metres (infinite if the map has no wall yet)."""
    return _distance_to_m(grid >= OCCUPIED_VALUE, info)


def costmap_to_grid(costmap: np.ndarray) -> np.ndarray:
    """
    Turns a Nav2 costmap into the grid this file works on: -1 unknown, 0 free, 100 blocked.

    Nav2 gives every cell a cost: -1 unknown, 0 free, 1 - 98 free but near an obstacle (the closer, the higher),
    99 = the robot's centre can't be here (inscribed), 100 = an obstacle. The global planner refuses the cells from 99
    on, so they count as blocked here: a frontier or a goal in them could never be driven to.
    """
    grid = np.zeros(costmap.shape, dtype=np.int8)
    grid[costmap >= INSCRIBED_COST] = 100
    grid[costmap < 0] = -1
    return grid


def costmap_wall_distance_m(costmap: np.ndarray, info: GridInfo) -> np.ndarray:
    """
    Distance of every cell to the nearest real obstacle of the costmap in metres (infinite if there is none).

    Measured to the obstacle cells (100) and not to the blocked ring around them (99), so it is the distance to the wall
    itself, like wall_distance_m on a map.
    """
    return _distance_to_m(costmap >= LETHAL_COST, info)


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
        clusters.append(Cluster(centroid=centroid, size_m=size_m, cells=(cluster_rows, cluster_cols)))
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


def cells_with_line_of_sight(obstacles: np.ndarray, cells: np.ndarray, target: tuple[int, int]) -> np.ndarray:
    """
    Which of the cells ((N, 2) array of row, col) see the target cell: no obstacle cell lies on the straight line
    between them. The target cell itself is not looked at, a frontier is allowed to sit right at a wall.
    """
    if len(cells) == 0:
        return np.zeros(0, dtype=bool)
    # One sample per cell along the longest side of the longest line, so no cell on a line is skipped
    samples = int(np.abs(cells - np.array(target)).max()) + 1
    fractions = np.linspace(0.0, 1.0, samples, endpoint=False)[None, :, None]
    points = np.rint(cells[:, None, :] * (1.0 - fractions) + np.array(target) * fractions).astype(int)
    points[..., 0] = np.clip(points[..., 0], 0, obstacles.shape[0] - 1)
    points[..., 1] = np.clip(points[..., 1], 0, obstacles.shape[1] - 1)
    # Lines are sampled finer than their length needs, so a sample can round onto the target cell: that one is skipped
    on_target = (points[..., 0] == target[0]) & (points[..., 1] == target[1])
    return ~(obstacles[points[..., 0], points[..., 1]] & ~on_target).any(axis=1)


def pick_goal_cell(
    reachable: np.ndarray,
    wall_distance: np.ndarray,
    centre: tuple[int, int],
    radius_cells: int,
    preferred_m: float,
    obstacles: np.ndarray | None = None,
) -> tuple[int, int] | None:
    """
    The cell to drive to for a frontier whose centre is `centre`: one of the reachable cells within
    radius_cells, preferring open space over a short trip.

    A frontier sits at a wall or a door frame, so the cell nearest to it is always right at the minimum
    wall distance. Instead take the nearest cell that is at least preferred_m from every wall; if there
    is none (a narrow corridor), the cell with the most room.

    With obstacles (a mask of the wall cells) only cells that see the frontier centre are taken. A wall is thinner
    than the radius, so without this a frontier behind a wall gets a goal in front of the wall: the robot drives up
    to it and stares at it.
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
    if obstacles is not None:
        sees_frontier = cells_with_line_of_sight(obstacles, cells, centre)
        cells, distance_sq = cells[sees_frontier], distance_sq[sees_frontier]
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
    max_candidates: int = 3,
) -> Selection:
    """
    Picks the nearest frontier the robot can drive to (and the next nearest ones, up to max_candidates, as candidates).

    Nav2 plans only through known free space and keeps away from walls, so the goal is not the frontier
    cell itself (often right next to a wall or a door frame). The robot may only be where it is at least
    clearance_m from every wall, so a gap narrower than twice that is not passable. The goal cell is
    chosen by pick_goal_cell within pullback_m of the frontier, in open space where possible
    (preferred_clearance_m). A frontier counts as unreachable if the robot can't get next to it (it lies behind a wall,
    or behind a gap the planner keeps closed) or if no such cell sees it.
    """
    passable = passable_mask(grid, wall_distance, clearance_m)
    robot_row, robot_col = info.to_cell(*robot_xy)
    start = nearest_true(passable, robot_row, robot_col, math.ceil(1.0 / info.resolution))
    if start is None:  # robot off the map or squeezed between walls
        return Selection(goal=None, clusters=len(clusters), unreachable=len(clusters), blacklisted=0)

    regions, _ = ndimage.label(passable)  # 4-connected: the robot can't squeeze through a diagonal gap
    reachable = regions == regions[start]
    reachable_m2 = float(np.count_nonzero(reachable)) * info.resolution**2
    # Where the robot can get next to: a frontier cell sits at the edge of the known space, a little off the cells the
    # robot's centre may use. A frontier behind a wall is much farther away than this.
    touch_cells = max(1, math.ceil(TOUCH_REACHABLE_M / info.resolution))
    next_to_reachable = ndimage.binary_dilation(reachable, iterations=touch_cells)
    obstacles = wall_distance <= 0  # the wall cells themselves

    goals: list[Goal] = []
    unreachable = blacklisted = 0
    pullback_cells = math.ceil(pullback_m / info.resolution)
    for cluster in clusters:
        if cluster.cells is not None and not next_to_reachable[cluster.cells].any():
            unreachable += 1
            continue
        cell = pick_goal_cell(
            reachable,
            wall_distance,
            info.to_cell(*cluster.centroid),
            pullback_cells,
            preferred_clearance_m,
            obstacles=obstacles,
        )
        if cell is None:
            unreachable += 1
            continue
        if any(math.dist(cluster.centroid, given_up) < blacklist_radius_m for given_up in blacklist):
            blacklisted += 1
            continue

        goal_x, goal_y = info.to_world(*cell)
        # Face the frontier; if the goal is on top of it, face away from the robot instead
        if math.dist((goal_x, goal_y), cluster.centroid) > 0.05:
            yaw = math.atan2(cluster.centroid[1] - goal_y, cluster.centroid[0] - goal_x)
        else:
            yaw = math.atan2(goal_y - robot_xy[1], goal_x - robot_xy[0])
        goals.append(
            Goal(
                x=goal_x,
                y=goal_y,
                yaw=yaw,
                target=cluster.centroid,
                distance=math.dist(robot_xy, (goal_x, goal_y)),
                clearance=float(wall_distance[cell]),
            )
        )

    candidates = sorted(goals, key=lambda goal: goal.distance)[:max_candidates]
    return Selection(
        goal=candidates[0] if candidates else None,
        clusters=len(clusters),
        unreachable=unreachable,
        blacklisted=blacklisted,
        candidates=candidates,
        start=start,
        reachable_m2=reachable_m2,
    )


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
