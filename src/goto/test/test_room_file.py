import math

import pytest

from goto.rooms.room_file import Room, load_rooms, save_rooms, yaw_between

# -- Setup code


@pytest.fixture
def two_rooms():
    # At most 3 decimals: the file rounds to millimetres (and about 0.06 degrees for yaw)
    return {
        "Room 1.12": Room("Room 1.12", [(2.0, 1.0), (6.0, 1.0), (6.0, 4.5), (2.0, 4.5)], 3.2, 0.6, 1.571),
        "Library": Room("Library", [(-3.0, -2.0), (-1.0, -2.0), (-2.0, 0.5)], -2.0, -2.5, -3.142),
    }


# -- Test code


def test_saved_rooms_load_back_the_same(tmp_path, two_rooms):
    path = tmp_path / "default_rooms.yaml"
    save_rooms(path, two_rooms)

    assert load_rooms(path) == two_rooms


def test_clicked_numbers_can_be_saved(tmp_path):
    # Clicks in the matplotlib window arrive as numpy numbers - the first real test crashed on them
    numpy = pytest.importorskip("numpy")
    click = numpy.float64
    room = Room(
        "Hall",
        [(click(-7.826), click(1.0)), (click(2.0), click(1.0)), (click(2.0), click(3.0))],
        click(0.5),
        click(1.5),
        click(1.571),
    )
    path = tmp_path / "rooms.yaml"
    save_rooms(path, {"Hall": room})

    assert load_rooms(path)["Hall"].area[0] == (-7.826, 1.0)


def test_missing_file_means_no_rooms(tmp_path):
    assert load_rooms(tmp_path / "does_not_exist.yaml") == {}


def test_all_rooms_deleted_by_hand_means_no_rooms(tmp_path):
    path = tmp_path / "rooms.yaml"
    path.write_text("rooms:\n")

    assert load_rooms(path) == {}


def test_non_ascii_room_names_are_kept(tmp_path):
    path = tmp_path / "rooms.yaml"
    rooms = {"Büro": Room("Büro", [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.5, 0.5, 0.0)}
    save_rooms(path, rooms)

    assert load_rooms(path) == rooms


@pytest.mark.parametrize(
    "content",
    [
        "rooms:\n  - {name: A, area: [[0, 0], [1, 0], [1, 1]]\n",  # broken YAML (bracket missing)
        "rooms:\n  - {name: A, area: [[0, 0], [1, 0], [1, 1]]}\n",  # no goal
        "rooms:\n  - just a text\n",  # not a room at all
        "rooms:\n  - {name: 1.10, area: [[0, 0], [1, 0], [1, 1]], goal: {x: 0, y: 0, yaw: 0}}\n",  # name w/o quotes
    ],
)
def test_broken_hand_edits_give_a_clear_error(tmp_path, content):
    # A ValueError the planner can catch - anything else would crash the node
    path = tmp_path / "rooms.yaml"
    path.write_text(content)

    with pytest.raises(ValueError):
        load_rooms(path)


def test_room_with_two_corners_is_rejected(tmp_path):
    path = tmp_path / "rooms.yaml"
    path.write_text("rooms:\n  - {name: A, area: [[0, 0], [1, 0]], goal: {x: 0, y: 0, yaw: 0}}\n")

    with pytest.raises(ValueError, match="at least 3 corners"):
        load_rooms(path)


def test_room_listed_twice_is_rejected(tmp_path):
    room = "  - {name: A, area: [[0, 0], [1, 0], [1, 1]], goal: {x: 0, y: 0, yaw: 0}}\n"
    path = tmp_path / "rooms.yaml"
    path.write_text("rooms:\n" + room + room)

    with pytest.raises(ValueError, match="listed twice"):
        load_rooms(path)


@pytest.mark.parametrize(
    "to_x, to_y, expected_yaw",
    [(1.0, 0.0, 0.0), (0.0, 2.0, math.pi / 2), (-3.0, 0.0, math.pi)],
)
def test_yaw_points_from_the_stop_point_to_the_second_click(to_x, to_y, expected_yaw):
    assert yaw_between(0.0, 0.0, to_x, to_y) == pytest.approx(expected_yaw)
