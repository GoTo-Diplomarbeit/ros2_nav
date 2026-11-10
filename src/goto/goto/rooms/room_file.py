"""
The room file of a map: which rooms exist, where they are, and where the robot stops for each one.

File maps/<world>_rooms.yaml, everything in metres on the map (frame "map"):

    rooms:
    - name: Room 1.12
      area:                                # corners of the outline, the app makes it clickable
      - [2.0, 1.0]
      - [6.0, 1.0]
      - [6.0, 4.5]
      goal: {x: 3.2, y: 0.6, yaw: 1.571}   # where the robot stops, yaw = heading in radians

Metres instead of pixels: when a map is continued, its image grows and pixel positions shift,
but positions in metres stay the same.

The rooms come from scripts/build_run_define_rooms.sh, or from the app (rooms_from_app + save_rooms).
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

FILE_HEADER = (
    "# Rooms of this map, in metres on the map. Made with scripts/build_run_define_rooms.sh;\n"
    "# editing by hand is fine (e.g. to delete a room). yaw = heading at the goal in radians.\n"
)


@dataclass
class Room:
    name: str
    area: list[tuple[float, float]]  # corners (x, y) of the room's outline, at least 3
    goal_x: float  # where the robot stops
    goal_y: float
    goal_yaw: float  # heading at the goal in radians, 0 = along the map's x axis


def load_rooms(path: str | Path) -> dict[str, Room]:
    """
    Reads a room file into {name: Room}. A missing file means: no rooms yet.
    Every problem in the file (it is edited by hand) is raised as a ValueError with a clear message.
    """
    path = Path(path)
    if not path.exists():
        return {}

    try:
        content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        raise ValueError(f"{path}: not valid YAML (check the indentation): {error}") from error
    if not isinstance(content, dict):
        raise ValueError(f"{path}: expected 'rooms:' at the top of the file")

    try:
        return rooms_from_entries(content.get("rooms") or [])  # "or []": "rooms:" with nothing under it
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from error


def rooms_from_app(text: str | bytes) -> dict[str, Room]:
    """
    Reads the rooms the app sends: a JSON list with the same fields as the room file, in metres:
        [{"name": "1.12", "area": [[2.0, 1.0], [6.0, 1.0], [6.0, 4.5]], "goal": {"x": 3.2, "y": 0.6, "yaw": 1.571}}]
    The list is all rooms of the map (save it with save_rooms). Bad data is raised as a ValueError.
    """
    try:
        entries = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f"not valid JSON: {error}") from error
    if not isinstance(entries, list):
        raise ValueError("expected a list of rooms")
    return rooms_from_entries(entries)


def rooms_from_entries(entries: list) -> dict[str, Room]:
    """Checks the room entries of the file or the app and turns them into {name: Room}."""
    rooms = {}
    for entry in entries:
        try:
            room = Room(
                name=entry["name"],
                area=[(float(x), float(y)) for x, y in entry["area"]],
                goal_x=float(entry["goal"]["x"]),
                goal_y=float(entry["goal"]["y"]),
                goal_yaw=float(entry["goal"]["yaw"]),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"room entry {entry!r} needs name, area and goal (x, y, yaw)") from error
        if not isinstance(room.name, str):
            # YAML reads  name: 1.10  as the number 1.1 - the app would send "1.10" and not find it
            raise ValueError(f"room name {room.name} must be text; in the YAML file: name: '{room.name}'")
        if not room.name or room.name != room.name.strip():
            # The planner strips what it receives, and an empty name means "stop the trip"
            raise ValueError(f"room name '{room.name}' must not be empty or start/end with spaces")
        if len(room.area) < 3:
            raise ValueError(f"room '{room.name}' needs at least 3 corners")
        if room.name in rooms:
            raise ValueError(f"room '{room.name}' is listed twice")
        rooms[room.name] = room
    return rooms


def save_rooms(path: str | Path, rooms: dict[str, Room]) -> None:
    """Writes the rooms as YAML, rounded to millimetres so the file stays readable."""

    def number(value) -> float:
        # float(): clicks from matplotlib are numpy numbers, which the YAML writer can't store
        return round(float(value), 3)

    entries = [
        {
            "name": room.name,
            "area": [[number(x), number(y)] for x, y in room.area],
            "goal": {"x": number(room.goal_x), "y": number(room.goal_y), "yaw": number(room.goal_yaw)},
        }
        for room in rooms.values()
    ]
    # default_flow_style=None writes each corner and the goal on one line, as in the docstring
    text = yaml.safe_dump({"rooms": entries}, sort_keys=False, allow_unicode=True, default_flow_style=None)
    # Write a temporary file first and then swap it in, so an interrupted save never leaves
    # a half-written room file behind. utf-8 so names like "Büro" always work.
    temporary_path = Path(path).with_suffix(".tmp")
    temporary_path.write_text(FILE_HEADER + text, encoding="utf-8")
    temporary_path.replace(path)


def yaw_between(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Heading (radians) of the direction from the first point to the second."""
    return math.atan2(to_y - from_y, to_x - from_x)
