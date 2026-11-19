"""Config loading and path resolution shared by every script.

Paths written in config.yaml belong to the repository, not to whatever
directory the script happens to be launched from. Everything goes through
`resolve_path()` so that `python /some/where/01.calibrate.py` writes its
results next to the code instead of into the caller's cwd.
"""
import os

import yaml

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.yaml")


def load_config(path=None):
    """Read config.yaml sitting next to this file."""
    with open(path or CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)


def resolve_path(path):
    """Turn a config path into an absolute one, anchored at the repo.

    Absolute paths and `~` are honoured as-is, so a user can point at weights
    or calibration files living anywhere.
    """
    expanded = os.path.expanduser(str(path))
    if os.path.isabs(expanded):
        return expanded
    return os.path.normpath(os.path.join(BASE_DIR, expanded))
