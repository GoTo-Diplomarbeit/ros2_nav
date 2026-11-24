import copy
import os
from unittest.mock import ANY

import yaml

from goto.node_utils.merge_params import deep_merge, merge_files

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def load(name):
    with open(os.path.join(CONFIG_DIR, name)) as file:
        return yaml.safe_load(file)


def test_a_nested_override_replaces_only_that_value():
    base = {"node": {"ros__parameters": {"inflation": {"radius": 0.45, "scaling": 3.0}, "rate": 5.0}}}
    overrides = {"node": {"ros__parameters": {"inflation": {"radius": 0.9}}}}

    merged = deep_merge(base, overrides)

    assert merged["node"]["ros__parameters"]["inflation"] == {"radius": 0.9, "scaling": 3.0}
    assert merged["node"]["ros__parameters"]["rate"] == 5.0


def test_new_keys_are_added_and_lists_are_replaced_not_merged():
    merged = deep_merge({"a": {"plugins": ["x", "y"]}}, {"a": {"plugins": ["z"], "new": 1}, "b": 2})

    assert merged == {"a": {"plugins": ["z"], "new": 1}, "b": 2}


def test_the_inputs_are_not_changed():
    base = {"a": {"b": 1}}
    overrides = {"a": {"c": [1, 2]}}
    base_before, overrides_before = copy.deepcopy(base), copy.deepcopy(overrides)

    merged = deep_merge(base, overrides)
    merged["a"]["c"].append(3)  # changing the result must not reach back into the overrides

    assert base == base_before
    assert overrides == overrides_before


def test_merge_files_writes_the_merged_yaml(tmp_path):
    (tmp_path / "base.yaml").write_text(
        "node:\n  ros__parameters:\n    footprint: '[[0.38, 0.2], [-0.38, -0.2]]'\n    radius: 0.45\n"
    )
    (tmp_path / "overrides.yaml").write_text("node:\n  ros__parameters:\n    radius: 0.9\n")

    merge_files(str(tmp_path / "base.yaml"), str(tmp_path / "overrides.yaml"), str(tmp_path / "out.yaml"))

    result = yaml.safe_load((tmp_path / "out.yaml").read_text())
    assert result["node"]["ros__parameters"]["radius"] == 0.9
    assert result["node"]["ros__parameters"]["footprint"] == "[[0.38, 0.2], [-0.38, -0.2]]"  # still a string


def test_the_exploration_overrides_touch_only_the_inflation_radii_and_reversing():
    base = load("nav2_params.yaml")
    overrides = load("nav2_exploration_overrides.yaml")

    # The overrides name nothing but the two inflation radii (the radius value itself may be changed to experiment)
    # and that the controller may reverse
    assert overrides == {
        "local_costmap": {"local_costmap": {"ros__parameters": {"inflation_layer": {"inflation_radius": ANY}}}},
        "global_costmap": {"global_costmap": {"ros__parameters": {"inflation_layer": {"inflation_radius": ANY}}}},
        "controller_server": {"ros__parameters": {"FollowPath": {"allow_reversing": True}}},
    }

    merged = deep_merge(base, overrides)
    for costmap in ("local_costmap", "global_costmap"):
        layer = merged[costmap][costmap]["ros__parameters"]["inflation_layer"]
        assert (
            layer["inflation_radius"]
            == overrides[costmap][costmap]["ros__parameters"]["inflation_layer"]["inflation_radius"]
        )
        # must keep matching the controller's inflation_cost_scaling_factor (3.0)
        assert layer["cost_scaling_factor"] == 3.0
        # Everything else is the plain navigation file: put the radius back and nothing differs
        layer["inflation_radius"] = base[costmap][costmap]["ros__parameters"]["inflation_layer"]["inflation_radius"]
    assert merged["controller_server"]["ros__parameters"]["FollowPath"]["allow_reversing"] is True
    merged["controller_server"]["ros__parameters"]["FollowPath"]["allow_reversing"] = False
    assert merged == base


def test_the_slam_settings_for_small_movements_and_stricter_loop_closure():
    slam = load("slam_params.yaml")["slam_toolbox"]["ros__parameters"]

    # Small steps: a creep the odometry misses is matched in pieces well inside the matcher's +-0.25 m window
    assert slam["minimum_travel_distance"] <= 0.1
    assert slam["minimum_travel_heading"] <= 0.2
    # A loop closure moves the pose by at most half the loop search window: less than the 2 m of the wrong ones
    assert slam["loop_search_space_dimension"] / 2 < 2.0


def test_the_slam_exploration_overrides_touch_only_min_pass_through():
    base = load("slam_params.yaml")
    overrides = load("slam_exploration_overrides.yaml")

    assert overrides == {"slam_toolbox": {"ros__parameters": {"min_pass_through": ANY}}}

    merged = deep_merge(base, overrides)
    assert merged["slam_toolbox"]["ros__parameters"]["min_pass_through"] == 1
    # Everything else is the plain file (the team's bringup keeps using it)
    merged["slam_toolbox"]["ros__parameters"]["min_pass_through"] = base["slam_toolbox"]["ros__parameters"][
        "min_pass_through"
    ]
    assert merged == base
