import json
import os
from vault import vault


def __parse_file(path: str) -> bool:
    """
    Parse the given config file. Override current config where new values are set.
    :param path: String containing the path to the config file.
    :return: All values included in the file that was parsed?
    """
    all_in = True
    conf = json.load(open(path))
    if "ros2_workspace" in conf:
        vault.directories.ros2_workspace = conf['ros2_workspace']
    else:
        all_in = False
    if "ignore_configs" in conf:
        vault.settings.ignore_configs = conf['ignore_configs']
    else:
        all_in = False
    if "rosdistro" in conf:
        vault.settings.rosdistro = conf['rosdistro']
    else:
        all_in = False
    if "always_yes" in conf:
        vault.settings.rosdistro = conf['always_yes']
    else:
        all_in = False
    return all_in


def parse_workspace_config():
    """
    Parse the workspace config file.
    :return: False if it doesn't exist.
    """
    srosc_ws_config = os.path.join(vault.directories.srosc_workspace, "config.json")
    if os.path.exists(srosc_ws_config):
        __parse_file(srosc_ws_config)
        return True
    else:
        return False


# load config from root config
__parse_file(vault.directories.CFG_FILE)
