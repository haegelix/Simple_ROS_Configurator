import os
from lib.config import Config


class Paths(object):
    srosc = os.getcwd()
    srosc_configs = ""
    ros_ws = ""
    ros_ws_src = ""

    def __init__(self, conf: Config):
        if conf.wspath[-1] == "/" or conf.wspath[-1] == "\\":
            self.ros_ws = conf.wspath[:-1]
        else:
            self.ros_ws = conf.wspath[:]
        self.ros_ws_src = os.path.join(self.ros_ws, "src")
        self.srosc_configs = os.path.join(self.srosc, "configs")

    def __str__(self):
        s = ""
        s += "SimpleROSConfigurator WD: " + self.srosc + "\n"
        s += "SimpleROS Config Dir:     " + self.srosc_configs + "\n"
        s += "ROS workspace:            " + self.ros_ws + "\n"
        s += "ROS workspace src-dir:    " + self.ros_ws_src
        return s

    def switch_to_configurator_dir(self):
        os.chdir(self.srosc)

    def switch_to_package_dir(self, name):
        os.chdir(os.path.join(self.ros_ws_src, name))

    def switch_to_ws_source_dir(self):
        os.chdir(self.ros_ws_src)

    def switch_to_ws_dir(self):
        os.chdir(self.ros_ws)

    def get_package_config_path(self, filename):
        return os.path.join(self.srosc_configs, filename)

    def get_package_src_path(self, name):
        return os.path.join(self.ros_ws_src, name)
