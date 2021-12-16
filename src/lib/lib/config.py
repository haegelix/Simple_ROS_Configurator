import json
import sys
import getopt
import re
from lib.paths import paths, get_global_config_path

ending_in_json = re.compile(r'[\w -]*[.]json')


class Config(object):
    wspath: str
    foreign_repos_config: str
    rosdistro: str
    ros_source_path: str
    ignore_configs: [str]
    always_yes: bool
    selected_pkg_config: str

    def __init__(self, path=get_global_config_path()):
        # load config from file
        self.selected_pkg_config = ""
        self.__parse_file__(path, fail_on_missing=True)
        self.__parse_argv__()
        paths.set_ros_ws(self.wspath)

    def __parse_file__(self, path: str, fail_on_missing=False):
        """
        Parse the given config file.
        :param path: String containing the path to the config file.
        """
        conf = json.load(open(path))
        try:
            self.wspath = self.conf_val(conf, "wspath", fail_on_missing)
            self.foreign_repos_config = self.conf_val(conf, "foreign_repos_config", fail_on_missing)
            self.ignore_configs = self.conf_val(conf, "ignore_configs", fail_on_missing)
            self.rosdistro = self.conf_val(conf, "rosdistro", fail_on_missing)
            self.ros_source_path = self.conf_val(conf, "ros_source_path", fail_on_missing)
            self.always_yes = self.conf_val(conf, "always_yes", fail_on_missing)
        except KeyError as k:
            if fail_on_missing:
                print("Affected config file:", path)
                raise k

    def __parse_argv__(self):
        """
        Checks for command line parameters and overrides config values.
        May exit the app.
        """
        try:
            argv = sys.argv[1:]  # name of this app shall not be searched for flags
            opts, args = getopt.getopt(argv, "hviys:", ["help", "version", "intercative", "yes", "select="])
        except getopt.GetoptError:
            print_help()
            sys.exit(2)

        try:
            o, a = zip(*opts)
        except ValueError:  # no command line parameters given
            return

        # search for version or help flags
        if "-h" in o or "--help" in o:
            print_help()
            sys.exit()
        if "-v" in o or "--version" in o:
            print_version()
            sys.exit()

        # search for all other valid flags and set config
        for opt, arg in opts:
            if opt in ("-i", "--interactive"):
                self.always_yes = False
                pass  # TODO more interactiveness?
            elif opt in ("-y", "--yes"):
                self.always_yes = True
            elif opt in ("-s", "--select"):
                sel = arg
                if not ending_in_json.match(sel):
                    sel = sel + ".json"
                print("Selected config file:", sel)
                self.selected_pkg_config = sel
            else:
                pass  # this branch is unreachable

    def __str__(self):
        selected_pkg_config = "None"
        if self.selected_pkg_config:
            selected_pkg_config = self.selected_pkg_config

        s = ""
        s += "ROS-workspace (wspath):       " + self.wspath + "\n"
        s += "ROS Distribution (rosdistro): " + self.rosdistro + "\n"
        s += "foreign_repos_config ():      " + self.foreign_repos_config + "\n"
        s += "Ignored config files ():      " + str(self.ignore_configs) + "\n"
        s += "ROS source-file path:         " + self.ros_source_path + "\n"
        s += "Always assume yes?            " + str(self.always_yes) + "\n"
        s += "Specific package selected?    " + str(selected_pkg_config)
        return s

    def parse_workspace_config(self):
        self.__parse_file__(paths.get_srosc_ws_configfile_path(), fail_on_missing=False)

    def conf_val(self, conf, key, fail_on_missing):
        try:
            return conf[key]
        except KeyError as k:
            if fail_on_missing:
                print("Key", key, "not found.")
                raise k
            else:
                return self.__getattribute__(key)


def print_help():
    print_version()
    print("")
    print("Usage:")
    print("srosc", sys.argv[0], ' [-h] [-i | -y] [-s <config_name>]')
    print("-h --help                Will produce this output and then exit.")
    print("-v --version             Will print version details and then exit.")
    print("-i --interactive         Force app to interact with you.")
    print("-y --yes                 Force app to run without interaction. USE WITH CAUTION!")
    print("-s <cfg> --select=<cfg>  Select specific package to be built from config. '.json' may be omitted.")


def print_version():
    print("Version (TODO)")  # TODO print version


config = Config()
