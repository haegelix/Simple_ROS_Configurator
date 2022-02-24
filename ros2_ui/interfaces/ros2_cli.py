import shutil

from ros2_ui.domains.Project import Project
from ros2_ui.interfaces.cli_helper import runcommand
from ros2_ui.interfaces.Log import logging
from ros2_ui.settings import settings

import os
from os import path

ros2_ws_path = settings.ros2_ws_path
ros2_ws_src_path = settings.ros2_ws_src_path


def _exec_depends_str(exec_depends: [str]) -> str:
    """
    Assemble the exec_depends string. (space-seperated list)

    :param exec_depends: Array in.
    :return: String out.
    """
    s = ""
    for x in exec_depends:
        s += x
        if x != exec_depends[-1]:  # add spaces BETWEEN the entries
            s += " "
    return s


def create_package(project: Project, logger=logging) -> None:
    """
    Creates a new package by using the ros2 executables

    :param project: Project to be packed.
    :param logger: Logger to be used.
    :return: Nothing.
    """
    pkg_path = path.join(ros2_ws_src_path, project.project_info.package_name)
    if path.exists(pkg_path):
        logger.info("Existing package deleted.")
        shutil.rmtree(pkg_path)

    command = ("ros2 pkg create --build-type ament_python"
               + " --destination-directory \"" + ros2_ws_src_path + "\""
               + " --description \"" + project.project_info.description + "\""
               + " --license \"" + project.project_info.license + "\""
               + " --dependencies " + _exec_depends_str(project.project_dependencies.exec_depends)
               + " --maintainer-email \"" + project.project_info.maintainer_mail + "\""
               + " --maintainer-name \"" + project.project_info.maintainer + "\""
               + " " + project.project_info.package_name)
    runcommand(command, "ros2 pkg create", logger)
    os.mkdir(path.join(pkg_path, "launch"))


def workspace_exists() -> bool:
    """
    Find out if ROS2-workspace dir does exist or not.

    :return: True if ROS2-workspace already exists.
    """
    return path.exists(ros2_ws_path)


def create_workspace():
    """
    Create an empty new ROS2-workspace inside ros2_ui-workspace.

    :return: Nothing.
    """
    os.makedirs(path.join(ros2_ws_path, "src"))
    cwd = os.getcwd()
    os.chdir(ros2_ws_path)
    runcommand("rosdep update", "rosdep update")
    os.chdir(cwd)


def resolve_dep(project: Project):
    """
    Build all dependencies and all packages.

    :return: Nothing
    """
    cwd = os.getcwd()
    os.chdir(ros2_ws_path)
    command = "rosdep install -i --from-paths -y " + path.join("src", project.project_info.package_name)
    runcommand(command, "rosdep install")
    os.chdir(cwd)


def build_package(project: Project):
    cwd = os.getcwd()
    os.chdir(ros2_ws_path)
    command = "colcon build --packages-select " + project.project_info.package_name
    runcommand(command, "colcon build")
    os.chdir(cwd)


def get_package_py_dir(project: Project):
    return path.join(get_package_root_dir(project), project.project_info.package_name)


def get_package_root_dir(project: Project):
    return path.join(ros2_ws_src_path, project.project_info.package_name)
