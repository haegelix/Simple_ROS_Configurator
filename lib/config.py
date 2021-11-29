import json


class Config(object):
    mode: str
    wspath: str
    foreign_repos_config: str
    rosdistro: str
    ros_source_path: str
    ignore_configs: [str]

    def __init__(self, path="config.json"):
        js = json.load(open(path))
        self.mode = js["mode"]
        self.wspath = js["wspath"]
        self.foreign_repos_config = js["foreign_repos_config"]
        self.ignore_configs = js["ignore_configs"]
        self.rosdistro = js["rosdistro"]
        self.ros_source_path = js["ros_source_path"]

    def __str__(self):
        s = ""
        s += "ROS-workspace (wspath):       " + self.wspath + "\n"
        s += "Configurator-mode (mode):     " + self.mode + "\n"
        s += "ROS Distribution (rosdistro): " + self.rosdistro + "\n"
        s += "foreign_repos_config ():      " + self.foreign_repos_config + "\n"
        s += "Ignored config files ():      " + str(self.ignore_configs) + "\n"
        s += "ROS source-file path:         " + self.ros_source_path
        return s


config = Config()
