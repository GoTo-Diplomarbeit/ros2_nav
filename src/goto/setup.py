from glob import glob

from setuptools import find_packages, setup

package_name = "goto"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Globs, so a new launch file or config is installed without editing this list.
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Peter Promok, Kilian Schneider, Micah Labendia, Alex Pticek",
    maintainer_email=(
        "peter.promok07@htl-salzburg.ac.at, "
        "kilian.schneider07@htl-salzburg.ac.at, "
        "micah.labendia07@htl-salzburg.ac.at, "
        "alex.pticek07@htl-salzburg.ac.at"
    ),
    description="Go2 mapping and navigation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "movement_node = goto.movement.movement_node:main",
            "vision_node = goto.vision.vision_node:main",
            "planner_node = goto.planner.planner_node:main",
            "web_server_node = goto.communication.server.web_server_node:main",
            "state_node = goto.state.state_node:main",
        ],
    },
)
