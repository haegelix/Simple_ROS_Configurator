"""
Paths will store, manage and switch directories needed for SimpleRosConfigurator.
This class shall be used as singleton since it's instances values will not change after loading the configuration.
"""
import os
import lib.helpers as helpers
from vault import vault


def get_lib_path() -> str:
    """
    TODO doc
    :return:
    """
    return vault.directories.LIB_DIR


def get_lib_template_dir_path():
    """
    Return path to template dir.
    :return: Path.
    """
    return os.path.join(get_lib_path(), "template_files")


def get_lib_template_file_path(templatename):
    """
    Get the full path of a template-file by its name.
    :param templatename: Name of the template.
    :return: Path.
    """
    filename = helpers.add_filename_suffix_if_missing(templatename, "py")
    return os.path.join(get_lib_template_dir_path(), "template_files", filename)


def get_logfile_path(name: str) -> str:
    """
    Return the path to a logfile.
    :param name: name of the logfile
    :return: full path
    """
    filename = helpers.add_filename_suffix_if_missing(name, "log")
    return os.path.join(vault.directories.LOG_DIR, filename)


def get_ros_workspace_path():
    return vault.directories.ros2_workspace


def get_ros_workspace_src_package_file_path(packagename: str, filename: str) -> str:
    """
    Return path to file inside ROS_WS/src/PACKAGE/
    :param packagename: name of the package
    :param filename: filename of file inside this dir
    :return: full path
    """
    return os.path.join(get_ros_workspace_src_package_path(packagename), filename)


def get_ros_workspace_src_package_path(packagename: str) -> str:
    """
    Return path to ROS_WS/src/PACKAGE/
    :param packagename: name of the package
    :return: full path
    """
    return os.path.join(get_ros_workspace_src_path(), packagename)


def get_ros_workspace_src_package_pydir_path(packagename: str) -> str:
    """
    Return path to ROS_WS/src/PACKAGE/PACKAGE/
    :param packagename: name of the package
    :return: full path
    """
    return os.path.join(get_ros_workspace_src_package_path(packagename), packagename)


def get_ros_workspace_src_path() -> str:
    """
    Return the path to ROS_WS
    :return: full path
    """
    return os.path.join(get_ros_workspace_path(), "src")


def get_srosc_packagefiles_list() -> [str]:
    """
    Return list of all packagefiles in SROSC_WS/packages
    :return: list
    """
    packages = [j for j in os.listdir(get_srosc_workspace_packages_path()) if j.endswith(".json")]
    for i in vault.settings.ignore_configs:
        packages.remove(i)
    return packages


def get_srosc_workspace_package_file_path(packagename) -> str:
    """
    Return the path to a package file.
    :param packagename: name of the package
    :return: full path
    """
    filename = helpers.add_filename_suffix_if_missing(packagename, "json")
    return os.path.join(get_srosc_workspace_path(), "packages", filename)


def get_srosc_workspace_packages_path():
    """
    TODO doc
    """
    return os.path.join(get_srosc_workspace_path(), "packages")


def get_srosc_workspace_path() -> str:
    """
    Return the path to the current srosc workspace.
    :return: full path
    """
    return vault.directories.srosc_workspace


def get_srosc_ws_configfile_path():
    """
    TODO doc
    """
    return os.path.join(get_srosc_workspace_path(), "config.json")


def reallocate_srosc_ws() -> None:
    vault.directories.srosc_workspace = os.getcwd()


def switch_to_ros_workspace_dir() -> None:
    """
    Switch to the ros workspace directory.
    :return: does not return a value
    """
    os.chdir(get_ros_workspace_path())


def switch_to_srosc_lib_dir() -> None:
    """
    Switch to the root directory of srosc.
    """
    os.chdir(get_lib_path())


def switch_to_srosc_workspace_dir() -> None:
    """
    Switch to the current srosc workspace.
    """
    os.chdir(get_srosc_workspace_path())
