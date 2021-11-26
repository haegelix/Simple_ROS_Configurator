from lib.logadapter import logging
import os
import subprocess
import re


def create_package(paths, package_info) -> None:
    """
    Creates a new package by using the ros2 executables
    """
    paths.switch_to_ws_source_dir()
    command = ("ros2 pkg create --build-type ament_python"
               + " --description \"" + str(package_info.pkg_config.package_info.description) + "\""
               + " --license \"" + str(package_info.pkg_config.package_info.license) + "\""
               + " --dependencies " + str(package_info.pkg_config.package_info.exec_depends_str)
               + " --maintainer-email \"" + str(package_info.pkg_config.package_info.maintainer_mail)
               + "\""
               + " --maintainer-name \"" + str(package_info.pkg_config.package_info.maintainer) + "\""
               + " " + str(package_info.pkg_config.package_name))
    __runcommand(command, "ros2 pkg create")


def resolve_dep(paths):
    """
    Build all dependencies and all packages.
    :return: Nothing
    """
    logging.info("Resolving and installing dependencies...")
    cwd = os.getcwd()
    os.chdir(paths.ros_ws)
    command = "rosdep install -i --from-path src --rosdistro foxy -y"
    __runcommand(command, "rosdep install")
    os.chdir(cwd)
    logging.info("Done resolving dependencies.")


def __runcommand(command: str, shortname: str):
    # Split command without splitting nested Strings.
    pattern = re.compile(r'''((?:[^ "']|"[^"]*"|'[^']*')+)''')
    cmd = pattern.split(command)

    # delete al empty strings
    cmd = [c for c in cmd if c and c != ' ']
    logging.debug(cmd)

    # run the command and deal with output
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    rtrn = proc.wait()
    stdout, stderr = proc.communicate()
    logging.debug("OUTPUT of '" + shortname + "...':\n" + str(stdout))
    logging.info(shortname + " returned code " + str(rtrn))
