from lib.logadapter import logging
from lib.paths import paths
import lib.packageinfo as packageinfo
import os
import subprocess
import re


def create_package(pkg_info: packageinfo.PackageInfo) -> None:
    """
    Creates a new package by using the ros2 executables
    """
    cwd = os.getcwd()
    paths.switch_to_ws_source_dir()
    command = ("ros2 pkg create --build-type ament_python"
               + " --description \"" + str(pkg_info.package_info.description) + "\""
               + " --license \"" + str(pkg_info.package_info.license) + "\""
               + " --dependencies " + str(pkg_info.package_info.exec_depends_str)
               + " --maintainer-email \"" + str(pkg_info.package_info.maintainer_mail) + "\""
               + " --maintainer-name \"" + str(pkg_info.package_info.maintainer) + "\""
               + " " + str(pkg_info.package_name))
    __runcommand(command, "ros2 pkg create")
    os.chdir(cwd)


def resolve_dep():
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


def build_package(pkg_info: packageinfo.PackageInfo):
    logging.info("Building the new package...")
    cwd = os.getcwd()
    os.chdir(paths.ros_ws)
    command = "colcon build --packages-select " + pkg_info.package_name
    __runcommand(command, "colcon build")
    os.chdir(cwd)
    logging.info("Done building.")


def __runcommand(command: str, shortname: str):
    # Split command without splitting nested Strings.
    pattern = re.compile(r'''((?:[^ "']|"[^"]*"|'[^']*')+)''')
    cmd = pattern.split(command)

    # delete al empty strings
    cmd = [c for c in cmd if c and c != ' ']

    # unquote
    cmd = [c.replace("\"", "").replace("'", "") for c in cmd]
    logging.debug(cmd)

    # run the command and deal with output
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    rtrn = proc.wait()
    stdout, stderr = proc.communicate()
    logging.debug("OUTPUT of '" + shortname + "...':\n" + str(stdout))
    if str(stderr):
        logging.info("STDERR of '" + shortname + "...'" + str(stderr))
    logging.info(shortname + " returned code " + str(rtrn))
