"""
Puts a small overrides file on top of a ROS parameter file, so a run can change a few Nav2 settings
without keeping a second copy of the whole nav2_params.yaml.

    python3 -m goto.node_utils.merge_params <base.yaml> <overrides.yaml> <out.yaml>

Used by scripts/build_run_sim_autonomous_mapping.sh: the merged file is written to the run's log folder
and handed to Nav2, while build_run_sim_navigation.sh keeps using the unchanged nav2_params.yaml.
"""

import copy
import sys

import yaml


def deep_merge(base: dict, overrides: dict) -> dict:
    """
    The base with the overrides on top. Dictionaries are merged key by key; any other value (numbers,
    strings, lists) in the overrides replaces the base value. Neither input is changed.
    """
    merged = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def merge_files(base_path: str, overrides_path: str, out_path: str) -> None:
    """Reads two YAML files and writes the merged result."""
    with open(base_path) as base_file, open(overrides_path) as overrides_file:
        merged = deep_merge(yaml.safe_load(base_file), yaml.safe_load(overrides_file) or {})
    with open(out_path, "w") as out_file:
        yaml.safe_dump(merged, out_file, sort_keys=False)


def main() -> None:
    if len(sys.argv) != 4:
        sys.exit("Usage: python3 -m goto.node_utils.merge_params <base.yaml> <overrides.yaml> <out.yaml>")
    merge_files(*sys.argv[1:])


if __name__ == "__main__":
    main()
