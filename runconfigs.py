#!/usr/bin/python3
# system wide imports
import os
import subprocess
import shutil
# my lib imports
from lib.config import config
from lib.logadapter import logging, setup_logging
from lib.paths import paths
import lib.packageinfo as packageinfo
from lib import helpers
from lib import ros_api
from lib.entrypoint import EntryPoint


def run():
    """
    Triggers sub methods.
    """
    print_config()
    check_generate_ws()
    create_packages_from_config_files()
    ros_api.resolve_dep()


def print_config() -> None:
    """
    Print all config values and paths.
    """
    logging.debug("### CONFIG ###\n" + str(config))
    logging.debug("### PATHS ###\n" + str(paths))


def check_generate_ws() -> None:
    """
    Check for existence of the specified ROS-workspace.
    A new one will be generated if it doesn't exist.
    """
    try:
        paths.switch_to_ws_dir()
    except PermissionError:
        logging.critical("No permission to use the directory " + paths.ros_ws)
        logging.critical("Exiting!")
        exit(1)
    except NotADirectoryError:
        logging.critical("Workspace at " + paths.ros_ws + " does exist, but is not a directory")
        logging.critical("Exiting!")
        exit(1)
    except FileNotFoundError:
        logging.info("Specified ROS-workspace not found!")
        print("Shall I generate a new one?")
        if helpers.yes_or_no():
            logging.info("Generating new workspace...")
            os.makedirs(paths.ros_ws_src)
            logging.info("Done.")
            return
        else:
            logging.info("Please change config before running again.\nExit!")
            exit(1)


def create_packages_from_config_files():
    """
    Build all packages defined in the configs.
    """
    # get all filenames and remove the ones to be ignored
    packages = [j for j in os.listdir(paths.srosc_configs) if j.endswith(".json")]
    packages.remove(config.foreign_repos_config)
    for i in config.ignore_configs:
        packages.remove(i)
    logging.info("Found " + str(len(packages)) + " package(s) to build")
    pkg_counter = 0

    # build all packages in the list
    for p in packages:
        pkg_counter += 1
        logging.info("Building package "
                     + str(pkg_counter) + " of " + str(len(packages))
                     + " from file '" + p + "'")
        create_package(p)
    pass


def create_package(p):
    pkg_info = packageinfo.load_package_config_from_file(p)
    logging.info(str(pkg_info))
    pack_src_path = paths.get_package_src_path(pkg_info.package_name)
    logging.info("Package path: " + pack_src_path)
    try:
        # Build dir structure
        if os.path.isdir(pack_src_path):
            logging.warning("Update not yet implemented!")
            logging.warning("But it can be deleted and rebuilt.")
            logging.warning("Shall I do this?")
            if helpers.yes_or_no():
                logging.info("Deleting package...")
                shutil.rmtree(pack_src_path)
                raise FileNotFoundError
            else:
                logging.info("As I said, update not yet implemented...\nExit!")
                return
        else:
            raise FileNotFoundError
    except FileNotFoundError:
        ros_api.create_package(pkg_info)
        copy_files(pkg_info)
        ros_api.build_package(pkg_info)


def copy_files(pkg_info: packageinfo.PackageInfo):
    """
    TODO docs
    """
    # TODO subs
    for p in pkg_info.pubs:
        # build a line of python code for this publisher, that looks like this:
        # p = _Pub("NODENAME", MESSAGETYPE, "TOPIC")
        p_insertion = 'p = _Pub("' + p.node_name + '", ' + p.type + ', "' + p.topic + '")'
        ins = ("# !INSERT_PUBLISHER_DECLARATION_HERE!", p_insertion)
        helpers.modify_and_copy_python_file(pkg_info, "pub", p.node_name + "_pub", [ins])
        # copy runners if known
        if helpers.acceptable_answer_str(p.src, "-stdin-"):
            # use the -stdin- runner as an input for the publisher
            ins = ("# !INSERT_PUB_IMPORT_HERE!", "import " + p.node_name + "_pub as pub")
            filename = p.node_name + "_runner_stdin"
            helpers.modify_and_copy_python_file(pkg_info, "pub_runner_stdin", filename, [ins])
            entries = [EntryPoint(filename, pkg_info.package_name, filename)]
            helpers.register_entry_points(pkg_info, entries)
        elif helpers.acceptable_answer_str(p.src, "-button-"):
            # use the -gpio- runner as an input for the publisher
            ins = ("# !INSERT_PUB_IMPORT_HERE!", "import " + p.node_name + "_pub as pub")
            filename = p.node_name + "_runner_gpio"
            helpers.modify_and_copy_python_file(pkg_info, "pub_runner_gpio", filename, [ins])
            entries = [EntryPoint(filename, pkg_info.package_name, filename)]
            helpers.register_entry_points(pkg_info, entries)
        else:
            # TODO unknown runners
            pass
    pass


def main():
    """
    Checks for availability of ROS2 and the runs the app.
    :return: Nothing.
    """
    setup_logging("runconfigs.log")
    if not ros_api.probe_ros():
        logging.info("ROS was not found --> trying to source...")
        ret = os.system("bash -c \"source " + config.ros_source_path + "; python3 runconfigs.py\"")
        exit(ret)
    else:
        logging.info("Starting up.")
        run()
        logging.info("Finished.")


if __name__ == "__main__":
    main()

# TODO rest implementieren
"""
# colcon build --packages-select PACKAGE
# colcon build

# . install/setup.bash
# Punkt ist wichtig, oder

# ros2 run PACKAGE ENTRYPOINT
"""
