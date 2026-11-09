"""
Names the rooms on a saved map, for the app's room selection.

    python3 -m goto.rooms.define_rooms <maps/world.yaml> <maps/world_rooms.yaml>

Normally started with scripts/build_run_define_rooms.sh [world]. Shows the map image (saved by the
mapping script) in a window. For each room: type its name in the terminal, click its corners in the
window, then click where the robot should stop and a second point in the direction it should face.
The room file is saved after every room. Needs no ROS and no simulator.
"""

import math
import sys
import warnings
from pathlib import Path

import yaml

# Harmless warning caused by the two matplotlib installs in the Docker image (only 3D plots are affected)
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib.pyplot as plt  # noqa: E402 - must come after the warning filter

from goto.rooms.room_file import Room, load_rooms, save_rooms, yaw_between  # noqa: E402

ARROW_LENGTH = 0.8  # metres, the arrow that shows the heading at the goal


def show_map(map_yaml_path: str):
    """Opens a window with the map image. Its axes are metres on the map, so clicks are metres too."""
    info = yaml.safe_load(Path(map_yaml_path).read_text())
    image = plt.imread(Path(map_yaml_path).parent / info["image"])
    height, width = image.shape[:2]
    resolution = info["resolution"]
    origin_x, origin_y = info["origin"][:2]  # map position of the image's bottom-left corner

    # Switch off matplotlib's keyboard shortcuts in the window ("q" would close it, "l" switch to a
    # log scale). Typing in the window still adds points while it waits for clicks.
    for setting in plt.rcParams:
        if setting.startswith("keymap."):
            plt.rcParams[setting] = []
    plt.ion()  # the window stays open while the terminal asks questions
    figure, axes = plt.subplots(figsize=(10, 8))
    axes.imshow(
        image,
        cmap="gray",
        extent=(origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution),
    )
    axes.set_xlabel("x (m)")
    axes.set_ylabel("y (m)")
    return figure, axes


def draw_rooms(axes, rooms: dict[str, Room]) -> None:
    """Draws every room: outline, name, and an arrow at the goal pointing in the goal's heading."""
    for artist in [*axes.lines, *axes.texts, *axes.patches]:
        artist.remove()
    for room in rooms.values():
        xs = [x for x, _ in room.area] + [room.area[0][0]]  # back to the first corner to close it
        ys = [y for _, y in room.area] + [room.area[0][1]]
        axes.plot(xs, ys, color="tab:blue")
        axes.text(sum(xs[:-1]) / len(room.area), sum(ys[:-1]) / len(room.area), room.name, ha="center")
        axes.arrow(
            room.goal_x,
            room.goal_y,
            ARROW_LENGTH * math.cos(room.goal_yaw),
            ARROW_LENGTH * math.sin(room.goal_yaw),
            width=0.05,
            color="tab:red",
        )
    axes.figure.canvas.draw_idle()
    plt.pause(0.1)  # lets the window redraw


def ask_for_room(figure, name: str) -> Room | None:
    """Collects one room's corners and goal by clicks. Returns None if it was cancelled."""
    print("  Click the corners in the window (right click = undo), then press Enter in the window.")
    corners = figure.ginput(n=-1, timeout=0)
    if len(corners) < 3:
        print("  A room needs at least 3 corners - not saved.")
        return None

    print("  Click where the robot should stop, then a point in the direction it should face.")
    points = figure.ginput(n=2, timeout=0)
    if len(points) < 2:
        print("  No stop point - not saved.")
        return None
    (stop_x, stop_y), (face_x, face_y) = points
    return Room(name, corners, stop_x, stop_y, yaw_between(stop_x, stop_y, face_x, face_y))


def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python3 -m goto.rooms.define_rooms <maps/world.yaml> <maps/world_rooms.yaml>")
        sys.exit(1)
    map_yaml_path, rooms_path = sys.argv[1:]

    rooms = load_rooms(rooms_path)
    figure, axes = show_map(map_yaml_path)
    draw_rooms(axes, rooms)
    print("Type room names here, click in the map window. An existing name is replaced.")
    print("(The window only reacts while it waits for clicks.)")

    try:
        while plt.fignum_exists(figure.number):
            print(f"\nRooms: {', '.join(rooms) or 'none yet'}")
            name = input("Room name (empty = quit): ").strip()
            if not name:
                break
            room = ask_for_room(figure, name)
            if room is not None:
                rooms[name] = room
                save_rooms(rooms_path, rooms)  # right away, so nothing is lost
                draw_rooms(axes, rooms)
                print(f"  Saved '{name}' in {rooms_path}")
    except (KeyboardInterrupt, EOFError):  # Ctrl+C, or the terminal closed
        print()
    print(f"{len(rooms)} rooms in {rooms_path}")
    plt.close("all")


if __name__ == "__main__":
    main()
