import os
from lib.config import config


class Paths(object):
    """
    Paths will store, manage and switch directories needed for SimpleRosConfigurator.
    This class be used as singleton.
    """
    srosc = os.getcwd()
    srosc_configs = ""
    ros_ws = ""
    ros_ws_src = ""

    def __init__(self):
        if config.wspath[-1] == "/" or config.wspath[-1] == "\\":
            self.ros_ws = config.wspath[:-1]
        else:
            self.ros_ws = config.wspath[:]
        self.ros_ws_src = os.path.join(self.ros_ws, "src")
        self.srosc_configs = os.path.join(self.srosc, "configs")

    def __str__(self):
        s = ""
        s += "SimpleROSConfigurator WD: " + self.srosc + "\n"
        s += "SimpleROS Config Dir:     " + self.srosc_configs + "\n"
        s += "ROS workspace:            " + self.ros_ws + "\n"
        s += "ROS workspace src-dir:    " + self.ros_ws_src + "\n"
        return s

    def switch_to_configurator_dir(self) -> None:
        """
        Switch to the root directory of SimpleRosConfigurator
        :return: does not return a value
        """
        os.chdir(self.srosc)

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
        return os.path.join(self.srosc, "py_templates", name + ".py")


paths = Paths()
