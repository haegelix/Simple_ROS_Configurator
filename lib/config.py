import json
import sys
import getopt
import re

ending_in_json = re.compile(r'[\w -]*[.]json')


class Config(object):
    mode: str
    wspath: str
    foreign_repos_config: str
    rosdistro: str
    ros_source_path: str
    ignore_configs: [str]
    always_yes: bool
    selected_pkg_config: str

    def __init__(self, path="config.json"):
        # load config from file
        self.__parse_file__(path)
        self.__parse_argv__()

    def __parse_file__(self, path: str):
        """
        Parse the given config file.
        :param path: String containing the path to the config file.
        """
        conf = json.load(open(path))
        self.mode = conf["mode"]
        self.wspath = conf["wspath"]
        self.foreign_repos_config = conf["foreign_repos_config"]
        self.ignore_configs = conf["ignore_configs"]
        self.rosdistro = conf["rosdistro"]
        self.ros_source_path = conf["ros_source_path"]
        self.always_yes = conf["always_yes"]

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
        s = ""
        s += "ROS-workspace (wspath):       " + self.wspath + "\n"
        s += "Configurator-mode (mode):     " + self.mode + "\n"
        s += "ROS Distribution (rosdistro): " + self.rosdistro + "\n"
        s += "foreign_repos_config ():      " + self.foreign_repos_config + "\n"
        s += "Ignored config files ():      " + str(self.ignore_configs) + "\n"
        s += "ROS source-file path:         " + self.ros_source_path + "\n"
        s += "Always assume yes?            " + str(self.always_yes)
        return s


def print_help():
    print_version()
    print("")
    print("Usage:")
    print(sys.argv[0], ' [-h] [-i | -y] [-s <config_name>]')
    print("-h --help                Will produce this output and then exit.")
    print("-v --version             Will print version details and then exit.")
    print("-i --interactive         Force app to interact with you.")
    print("-y --yes                 Force app to run without interaction. USE WITH CAUTION!")
    print("-s <cfg> --select=<cfg>  Select specific package to be built from config. '.json' may be omitted.")


def print_version():
    print("Version (TODO)")  # TODO print version


config = Config()
