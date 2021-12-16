import os

# static directories
BIN_DIR = "/usr/local/bin/"
LIB_DIR = "/usr/local/lib/srosc/"
CFG_DIR = "/etc/srosc/"
LOG_DIR = "/var/log/srosc/"


def get_global_config_path():
    """
    Get the path of the global config file
    """
    return os.path.join(CFG_DIR, "config.json")


class Paths(object):
    """
    Paths will store, manage and switch directories needed for SimpleRosConfigurator.
    This class shall be used as singleton since it's instances values will not change after loading the configuration.
    """
    srosc_ws = os.getcwd()
    srosc_configs = ""
    ros_ws = ""
    ros_ws_src = ""

    def set_ros_ws(self, ros_ws: str):
        self.ros_ws = ros_ws
        self.ros_ws_src = os.path.join(ros_ws, "src")

    def __init__(self):
        self.srosc_configs = os.path.join(self.srosc_ws, "packages")

    def __str__(self):
        s = ""
        s += "SimpleROSConfigurator WD: " + self.srosc_ws + "\n"
        s += "SimpleROS Config Dir:     " + self.srosc_configs + "\n"
        s += "ROS workspace:            " + self.ros_ws + "\n"
        s += "ROS workspace src-dir:    " + self.ros_ws_src + "\n"
        return s

    def reallocate_srosc_ws(self):
        self.srosc_ws = os.getcwd()

    def switch_to_srosc_ws_dir(self) -> None:
        """
        Switch to the root directory of SimpleRosConfigurator
        :return: does not return a value
        """
        os.chdir(self.get_srosc_ws_path())

    def switch_to_srosc_lib_dir(self) -> None:
        """
        Switch to the root directory of SimpleRosConfigurator
        :return: does not return a value
        """
        os.chdir(LIB_DIR)

    def switch_to_package_dir(self, name) -> None:
        """
        Switch to the source dir of a specific package inside the ros workspace.
        :return: does not return a value
        """
        os.chdir(os.path.join(self.ros_ws_src, name))

    def switch_to_package_py_dir(self, name) -> None:
        """
        Switch to the python source dir of a specific package inside the ros workspace.
        :return: does not return a value
        """
        os.chdir(os.path.join(self.ros_ws_src, name, name))

    def switch_to_ws_source_dir(self) -> None:
        """
        Switch to the source directory inside the ros workspace directory
        """
        os.chdir(self.ros_ws_src)

    def switch_to_ws_dir(self) -> None:
        """
        Switch to the ros workspace directory.
        :return: does not return a value
        """
        os.chdir(self.ros_ws)

    def get_package_config_path(self, filename) -> str:
        """
        Path to a specific config file used by Simple_ROS_Configurator
        :param filename: filename
        :return: complete path
        """
        return os.path.join(self.srosc_configs, filename)

    def get_package_src_path(self, name: str, filename="") -> str:
        """
        Config code is directly inside package source directory.
        :param name: name of the package
        :param filename: filename of file inside this dir
        :return: path to the config-files directory of this project
        """
        base_path = os.path.join(self.ros_ws_src, name)
        if filename and len(filename) > 2:
            return os.path.join(base_path, filename)
        else:
            return base_path

    def get_package_py_src_path(self, name) -> str:
        """
        Python code is nested one dir-level deeper inside package source than config files.
        :param name: name of the package
        :return: path to the python-files directory of this project
        """
        return os.path.join(self.ros_ws_src, name, name)

    def get_template_path(self, name):
        """
        Get the full path of a template-file by its name.
        :param name: Name of the template.
        :return: Path.
        """
        return os.path.join(self.srosc_ws, "template_files", name + ".py")

    def get_runnable_path(self, filename: str):
        """
        TODO doc
        """
        return os.path.join(LIB_DIR, filename)

    def get_srosc_ws_configfile_path(self):
        """
        TODO doc
        """
        return os.path.join(self.srosc_ws, "config.json")

    def get_srosc_ws_packages_path(self):
        """
        TODO doc
        """
        return os.path.join(self.srosc_ws, "packages")

    def get_srosc_ws_package_file_path(self, name: str):
        """
        TODO doc
        """
        return os.path.join(self.srosc_ws, "packages", name + ".json")

    def get_srosc_ws_path(self):
        """
        TODO doc
        """
        return self.srosc_ws

    def get_logfile_path(self, filename: str):
        """
        TODO doc
        :param filename:
        """
        return os.path.join(LOG_DIR, filename)


paths = Paths()
