import math

import numpy as np
import pytest

from goto.mapping.frontiers import (
    GridInfo,
    cells_with_line_of_sight,
    choose_goal,
    classify_result,
    costmap_to_grid,
    costmap_wall_distance_m,
    find_frontier_clusters,
    frontier_still_exists,
    passable_mask,
    pick_goal_cell,
    wall_distance_m,
)

# Cells are 0.1 m, so the grids below are small rooms of about 1 m. '#' wall, '.' free, '?' unknown.
INFO = GridInfo(resolution=0.1, origin_x=0.0, origin_y=0.0)
CLEARANCE_M = 0.25  # the robot's centre keeps this far from walls
# preferred = minimum: the plain "nearest cell" behaviour of the older tests; the open-space tests below raise it
CHOOSE = dict(clearance_m=CLEARANCE_M, preferred_clearance_m=CLEARANCE_M, pullback_m=0.5, blacklist_radius_m=0.5)


def make_grid(rows):
    values = {"#": 100, ".": 0, "?": -1}
    return np.array([[values[char] for char in row] for row in rows], dtype=np.int8)


def clusters_of(grid, min_size_m, max_wall_distance_m=math.inf):
    return find_frontier_clusters(grid, INFO, wall_distance_m(grid, INFO), min_size_m, max_wall_distance_m)


def goal_for(grid, robot_xy, clusters, blacklist=(), **overrides):
    return choose_goal(
        grid, INFO, wall_distance_m(grid, INFO), robot_xy, clusters, list(blacklist), **{**CHOOSE, **overrides}
    )


def cell_to_xy(row, col):
    return INFO.to_world(row, col)


# A closed room with a 3 cell wide door in the east wall, leading to unknown space
ROOM_WITH_DOOR = make_grid(
    [
        "????????????",
        "?##########?",
        "?#........#?",
        "?#.........?",
        "?#.........?",
        "?#.........?",
        "?#........#?",
        "?##########?",
        "????????????",
    ]
)

# The same room with doors in the west and the east wall
ROOM_WITH_TWO_DOORS = make_grid(
    [
        "????????????",
        "?##########?",
        "?#........#?",
        "??.........?",
        "??.........?",
        "??.........?",
        "?#........#?",
        "?##########?",
        "????????????",
    ]
)

# Two rooms with a wall between them. Only the right one has a door to unknown space.
CLOSED_ROOM_NEXT_TO_OPEN_ONE = make_grid(
    [
        "??????????????????????",
        "?####################?",
        "?#.........#.........?",
        "?#.........#.........?",
        "?#.........#.........?",
        "?#.........#.........?",
        "?#.........#.........?",
        "?####################?",
        "??????????????????????",
    ]
)


def test_grid_info_converts_between_cells_and_world():
    info = GridInfo(resolution=0.05, origin_x=-1.0, origin_y=2.0)

    x, y = info.to_world(10, 20)

    assert (x, y) == pytest.approx((0.025, 2.525))
    assert info.to_cell(x, y) == (10, 20)


def test_a_closed_room_has_no_frontier():
    closed = make_grid(["?????", "?###?", "?#.#?", "?###?", "?????"])

    assert clusters_of(closed, min_size_m=0.0) == []


def test_a_door_to_unknown_space_is_one_frontier():
    clusters = clusters_of(ROOM_WITH_DOOR, min_size_m=0.25)

    assert len(clusters) == 1
    assert clusters[0].size_m == pytest.approx(0.3)
    assert clusters[0].centroid == pytest.approx(cell_to_xy(4, 10))


def test_a_tiny_gap_is_ignored():
    grid = ROOM_WITH_DOOR.copy()
    grid[3, 10] = grid[5, 10] = 100  # leaves a one cell gap at row 4

    assert len(clusters_of(grid, min_size_m=0.05)) == 1
    assert clusters_of(grid, min_size_m=0.25) == []


def test_sparse_ray_fans_in_open_space_are_not_frontiers_but_the_same_next_to_a_wall_is():
    # Free space with unknown dots between lidar rays, like the fan behind a window
    open_space = np.zeros((60, 60), dtype=np.int8)
    open_space[0, :] = open_space[-1, :] = open_space[:, 0] = open_space[:, -1] = 100
    open_space[25:35:2, 25:35:2] = -1
    next_to_wall = open_space.copy()
    next_to_wall[25:35:2, 25:35:2] = 0
    next_to_wall[1:11:2, 1:11:2] = -1  # the same dots, in the corner

    assert len(clusters_of(open_space, 0.25)) == 1  # without the filter it counts
    assert clusters_of(open_space, 0.25, max_wall_distance_m=1.0) == []
    assert len(clusters_of(next_to_wall, 0.25, max_wall_distance_m=1.0)) == 1


def test_the_goal_is_pulled_back_from_the_door_and_faces_it():
    clusters = clusters_of(ROOM_WITH_DOOR, min_size_m=0.25)

    selection = goal_for(ROOM_WITH_DOOR, cell_to_xy(4, 5), clusters)

    goal = selection.goal
    door_x = cell_to_xy(4, 10)[0]
    assert goal is not None
    assert goal.x < door_x - 0.1  # in front of the door, not in the door frame
    assert goal.x > door_x - 0.5
    assert abs(goal.yaw) < 0.3  # looking east, at the door
    assert (selection.clusters, selection.unreachable, selection.blacklisted) == (1, 0, 0)


def test_the_nearest_frontier_is_chosen():
    clusters = clusters_of(ROOM_WITH_TWO_DOORS, min_size_m=0.25)
    assert len(clusters) == 2
    middle_x = cell_to_xy(4, 5)[0]

    west = goal_for(ROOM_WITH_TWO_DOORS, cell_to_xy(4, 3), clusters).goal
    east = goal_for(ROOM_WITH_TWO_DOORS, cell_to_xy(4, 8), clusters).goal

    assert west.x < middle_x < east.x


def test_a_frontier_behind_a_wall_is_unreachable():
    clusters = clusters_of(CLOSED_ROOM_NEXT_TO_OPEN_ONE, min_size_m=0.25)

    selection = goal_for(CLOSED_ROOM_NEXT_TO_OPEN_ONE, cell_to_xy(4, 5), clusters)  # robot in the closed room

    assert selection.goal is None
    assert (selection.clusters, selection.unreachable, selection.blacklisted) == (1, 1, 0)


def test_the_selection_reports_where_the_search_started_and_how_big_the_robots_region_is():
    clusters = clusters_of(CLOSED_ROOM_NEXT_TO_OPEN_ONE, min_size_m=0.25)
    robot = cell_to_xy(4, 5)

    selection = goal_for(CLOSED_ROOM_NEXT_TO_OPEN_ONE, robot, clusters)

    assert selection.start == INFO.to_cell(*robot)  # the robot's own cell is passable: no moving
    assert selection.reachable_m2 > 0.0


def test_a_robot_in_a_blocked_cell_starts_its_search_at_the_nearest_passable_one():
    grid = np.full((15, 15), 100, dtype=np.int8)
    grid[1:14, 1:14] = 0  # a room of 1.3 m, large enough for cells 0.25 m from every wall
    grid[7, 7] = 100  # a stray obstacle mark right where the robot stands
    robot = cell_to_xy(7, 7)

    selection = goal_for(grid, robot, [])

    assert selection.start != INFO.to_cell(*robot)
    assert selection.start is not None


def test_a_blacklisted_frontier_is_skipped_and_counted():
    clusters = clusters_of(ROOM_WITH_TWO_DOORS, min_size_m=0.25)
    given_up = [cluster.centroid for cluster in clusters]

    selection = goal_for(ROOM_WITH_TWO_DOORS, cell_to_xy(4, 5), clusters, blacklist=given_up)

    assert selection.goal is None
    assert (selection.clusters, selection.unreachable, selection.blacklisted) == (2, 0, 2)


def test_a_robot_squeezed_between_walls_reaches_nothing():
    grid = ROOM_WITH_DOOR.copy()
    grid[3:6, 2:9] = 100  # fills the room, only the door cells stay free
    clusters = clusters_of(grid, min_size_m=0.25)

    assert goal_for(grid, cell_to_xy(2, 3), clusters).goal is None


def test_passable_cells_keep_their_distance_to_walls():
    passable = passable_mask(ROOM_WITH_DOOR, wall_distance_m(ROOM_WITH_DOOR, INFO), clearance_m=0.25)

    assert not passable[2, 2]  # next to a wall
    assert passable[4, 5]  # middle of the room
    assert not passable[1, 5]  # a wall itself


def test_a_map_without_walls_has_infinite_wall_distance():
    assert np.isinf(wall_distance_m(np.zeros((3, 3), dtype=np.int8), INFO)).all()


def test_a_frontier_that_moved_slightly_still_exists_but_a_revealed_one_does_not():
    clusters = clusters_of(ROOM_WITH_DOOR, min_size_m=0.25)
    target = clusters[0].centroid

    assert frontier_still_exists(clusters, (target[0] + 0.3, target[1]), radius_m=1.0)
    assert not frontier_still_exists(clusters, (target[0] + 3.0, target[1]), radius_m=1.0)
    assert not frontier_still_exists([], target, radius_m=1.0)


@pytest.mark.parametrize(
    "reached, seconds, distance, expected",
    [
        (True, 20.0, 0.1, "reached"),
        (False, 20.0, 0.3, "reached"),  # aborted, but standing at the goal
        (False, 0.05, 4.0, "spurious"),  # aborted right after accepting: Nav2's timeout, not the frontier
        (False, 30.0, 4.0, "failed"),
    ],
)
def test_classify_result(reached, seconds, distance, expected):
    assert classify_result(reached, seconds, distance, near_goal_m=0.5, spurious_s=1.5) == expected


def test_the_yaw_is_finite_when_the_goal_lies_on_the_frontier():
    clusters = clusters_of(ROOM_WITH_DOOR, min_size_m=0.25)

    goal = goal_for(ROOM_WITH_DOOR, cell_to_xy(4, 9), clusters, clearance_m=0.0).goal

    assert goal is not None and math.isfinite(goal.yaw)


# -- Goals in open space, away from walls


def open_room(cols, rows, door_rows):
    """
    A closed room of cols x rows cells (0.1 m each) with a door in its east wall at the given interior rows,
    leading to unknown space. Array layout: unknown border, wall ring, interior.
    """
    grid = np.full((rows + 4, cols + 4), -1, dtype=np.int8)
    grid[1 : rows + 3, 1 : cols + 3] = 100
    grid[2 : rows + 2, 2 : cols + 2] = 0
    for row in door_rows:
        grid[2 + row, cols + 2] = 0
    return grid


def test_the_goal_is_in_open_space_not_at_the_wall():
    grid = open_room(cols=24, rows=15, door_rows=range(6, 9))
    clusters = clusters_of(grid, min_size_m=0.25)
    robot = cell_to_xy(9, 4)
    at_the_wall = goal_for(grid, robot, clusters, preferred_clearance_m=CLEARANCE_M, pullback_m=1.5).goal
    in_the_open = goal_for(grid, robot, clusters, preferred_clearance_m=0.6, pullback_m=1.5).goal

    assert at_the_wall.clearance < 0.35  # the nearest cell sits right at the minimum distance
    assert in_the_open.clearance >= 0.6 - 1e-9
    assert in_the_open.x < at_the_wall.x  # a bit farther back from the door, in return


def test_in_a_narrow_corridor_the_goal_is_in_the_middle():
    grid = open_room(cols=40, rows=12, door_rows=range(4, 8))
    clusters = clusters_of(grid, min_size_m=0.25)
    middle_y = cell_to_xy(2 + 5.5, 0)[1]  # the corridor is 1.2 m wide: no cell is 0.9 m from both walls

    goal = goal_for(grid, cell_to_xy(7, 4), clusters, preferred_clearance_m=0.9, pullback_m=1.5).goal

    assert goal is not None
    assert abs(goal.y - middle_y) <= 0.1
    assert goal.clearance == pytest.approx(0.6, abs=0.05)


def two_rooms(opening_rows):
    """Two rooms with a wall between them and an opening of the given rows; a door leads out of the right one."""
    grid = open_room(cols=30, rows=15, door_rows=range(6, 9))
    grid[2:17, 17] = 100
    for row in opening_rows:
        grid[2 + row, 17] = 0
    return grid


def test_a_gap_narrower_than_twice_the_minimum_distance_is_not_passable():
    grid = two_rooms(opening_rows=range(5, 11))  # 0.6 m wide
    clusters = clusters_of(grid, min_size_m=0.25)
    robot = cell_to_xy(9, 4)  # in the left room

    wide_enough = goal_for(grid, robot, clusters, clearance_m=0.25, preferred_clearance_m=0.25, pullback_m=1.5)
    too_narrow = goal_for(grid, robot, clusters, clearance_m=0.35, preferred_clearance_m=0.35, pullback_m=1.5)

    assert wide_enough.goal is not None
    assert too_narrow.goal is None
    assert too_narrow.unreachable == 1


def test_the_goal_reports_its_distance_to_the_nearest_wall():
    grid = open_room(cols=24, rows=15, door_rows=range(6, 9))
    wall_distance = wall_distance_m(grid, INFO)

    goal = goal_for(grid, cell_to_xy(9, 4), clusters_of(grid, min_size_m=0.25), preferred_clearance_m=0.6).goal

    assert goal.clearance == pytest.approx(wall_distance[INFO.to_cell(goal.x, goal.y)])


def room_shrinking_to_the_right():
    """A wall on the right: 1.0 m of room at column 0, none at column 10, like the free cells in front of a wall."""
    return np.ones((11, 11), dtype=bool), np.tile((10 - np.arange(11)) / 10, (11, 1))


def test_pick_goal_cell_takes_the_nearest_cell_that_is_roomy_enough():
    reachable, wall_distance = room_shrinking_to_the_right()

    # the frontier centre is at the wall (column 10); columns 0 to 5 have at least 0.5 m of room, column 5 is nearest
    assert pick_goal_cell(reachable, wall_distance, (5, 10), 10, preferred_m=0.5) == (5, 5)


def test_pick_goal_cell_falls_back_to_the_roomiest_cell_and_handles_no_cell():
    reachable, wall_distance = room_shrinking_to_the_right()

    # nothing has 5 m of room: the roomiest cells within 3 cells of (5, 2) are in column 0, the one on row 5 is nearest
    assert pick_goal_cell(reachable, wall_distance, (5, 2), 3, preferred_m=5.0) == (5, 0)
    assert pick_goal_cell(np.zeros((11, 11), dtype=bool), wall_distance, (5, 5), 3, preferred_m=0.5) is None
    assert pick_goal_cell(reachable, wall_distance, (500, 500), 3, preferred_m=0.5) is None


# -- The costmap as input


def test_costmap_to_grid_blocks_everything_the_robots_centre_can_not_use_and_keeps_unknown():
    costmap = np.array([[-1, 0, 50, 98, 99, 100]], dtype=np.int8)

    assert costmap_to_grid(costmap).tolist() == [[-1, 0, 0, 0, 100, 100]]


def test_costmap_wall_distance_is_measured_to_the_obstacle_and_not_to_the_blocked_ring():
    costmap = np.array([[100, 99, 99, 50, 0, 0]], dtype=np.int8)

    distance = costmap_wall_distance_m(costmap, INFO)

    assert distance[0] == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
    assert np.isinf(costmap_wall_distance_m(np.zeros((2, 2), dtype=np.int8), INFO)).all()


def test_a_frontier_in_the_blocked_ring_of_a_wall_is_no_frontier():
    # free cells next to unknown, but inside the ring of 99 around a wall: the robot can never stand there
    costmap = np.array([[-1, -1, -1], [-1, 99, 100], [-1, 0, 0], [-1, 0, 0]], dtype=np.int8)

    clusters = find_frontier_clusters(costmap_to_grid(costmap), INFO, costmap_wall_distance_m(costmap, INFO), 0.0)

    # Only the two cells of column 1 below the blocked one count. The blocked cell (row 1) would make it three cells
    # and move the centroid up by half a cell.
    assert len(clusters) == 1
    assert clusters[0].size_m == pytest.approx(0.2)
    assert clusters[0].centroid == pytest.approx((0.15, 0.3))


# -- A frontier behind a wall


def wall_with_pocket():
    """
    The robot's room (left, columns 2 - 15), a wall one cell thick with a one cell hole in it (column 16, row 10),
    and a pocket behind it (columns 17 - 28) that opens to unknown space in the east (column 29, rows 8 - 12).
    Like a wall a lidar ray went through.
    """
    grid = np.full((21, 40), -1, dtype=np.int8)
    grid[1:20, 1:30] = 100
    grid[2:19, 2:16] = 0
    grid[2:19, 17:29] = 0
    grid[10, 16] = 0
    grid[8:13, 29] = 0
    return grid


def test_a_frontier_behind_a_thin_wall_is_unreachable_even_when_a_goal_cell_is_in_range():
    grid = wall_with_pocket()
    clusters = clusters_of(grid, min_size_m=0.25)
    assert len(clusters) == 1
    robot = cell_to_xy(10, 8)

    # pullback 3 m: the robot's room lies within reach of the frontier, on the other side of the wall
    selection = goal_for(grid, robot, clusters, pullback_m=3.0)

    assert selection.goal is None
    assert (selection.clusters, selection.unreachable) == (1, 1)


def test_the_same_frontier_is_reachable_when_the_wall_has_a_real_gap():
    grid = wall_with_pocket()
    grid[7:14, 16] = 0  # a 0.7 m door instead of the hole
    clusters = clusters_of(grid, min_size_m=0.25)

    selection = goal_for(grid, cell_to_xy(10, 8), clusters, pullback_m=3.0)

    assert selection.goal is not None
    assert selection.goal.x > cell_to_xy(0, 16)[0]  # the goal is behind the door, near the frontier


def test_cells_with_line_of_sight_stop_at_walls_but_not_at_the_target():
    obstacles = np.zeros((11, 11), dtype=bool)
    obstacles[:, 5] = True
    cells = np.array([[5, 2], [5, 7], [2, 9], [5, 9]])

    visible = cells_with_line_of_sight(obstacles, cells, (5, 9))

    assert visible.tolist() == [False, True, True, True]  # the target in a wall is no reason to refuse
    obstacles[5, 9] = True
    assert cells_with_line_of_sight(obstacles, cells, (5, 9)).tolist() == [False, True, True, True]
    assert cells_with_line_of_sight(obstacles, np.zeros((0, 2), dtype=int), (5, 9)).tolist() == []


def test_pick_goal_cell_ignores_cells_that_can_not_see_the_frontier():
    reachable = np.ones((11, 11), dtype=bool)
    reachable[:, 5:] = False  # the robot's side of a wall in column 5
    wall_distance = np.ones((11, 11))
    obstacles = np.zeros((11, 11), dtype=bool)
    obstacles[:, 5] = True

    assert pick_goal_cell(reachable, wall_distance, (5, 8), 10, preferred_m=0.5) == (5, 4)  # through the wall
    assert pick_goal_cell(reachable, wall_distance, (5, 8), 10, preferred_m=0.5, obstacles=obstacles) is None


# -- Candidates for the planner


def test_the_candidates_are_the_nearest_goals_nearest_first():
    clusters = clusters_of(ROOM_WITH_TWO_DOORS, min_size_m=0.25)
    assert len(clusters) == 2

    selection = goal_for(ROOM_WITH_TWO_DOORS, cell_to_xy(4, 3), clusters)  # nearer to the west door

    assert len(selection.candidates) == 2
    assert selection.candidates[0].distance < selection.candidates[1].distance
    assert selection.goal is selection.candidates[0]


def test_the_number_of_candidates_is_capped():
    clusters = clusters_of(ROOM_WITH_TWO_DOORS, min_size_m=0.25)

    selection = goal_for(ROOM_WITH_TWO_DOORS, cell_to_xy(4, 3), clusters, max_candidates=1)

    assert len(selection.candidates) == 1
    # the one that was capped off still counts as a frontier
    assert (selection.clusters, selection.unreachable, selection.blacklisted) == (2, 0, 0)


def test_no_goal_means_no_candidates():
    clusters = clusters_of(CLOSED_ROOM_NEXT_TO_OPEN_ONE, min_size_m=0.25)

    selection = goal_for(CLOSED_ROOM_NEXT_TO_OPEN_ONE, cell_to_xy(4, 5), clusters)

    assert selection.goal is None and selection.candidates == []
