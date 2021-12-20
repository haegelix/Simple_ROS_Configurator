import lib.myprettyprint
import os


class Vault(object):
    """
    Stores critical data to be accessible by all other modules. Storage only!
    """

    class __Directories(lib.myprettyprint.MyPrettyPrint):
        """
        Holds all relevant directory paths.
        Mainly used by the paths module.
        """
        DESCRIPTION = {
            'ros2_workspace': "ROS-workspace",
            'srosc_workspace': "ROS-workspace",
        }
        # path of the ros2 workspace in use
        ros2_workspace: str
        # path of the srosc workspace in use
        srosc_workspace: str
        # static paths
        BIN_DIR = "/usr/local/bin/"
        LIB_DIR = "/usr/local/lib/srosc/"
        CFG_DIR = "/etc/srosc/"
        LOG_DIR = "/var/log/srosc/"
        CFG_FILE = "/etc/srosc/config.json"

        def __init__(self):
            self.srosc_workspace = os.getcwd()

    class __Settings(lib.myprettyprint.MyPrettyPrint):
        """
        Holds all settings.
        Mainly used by the config module.
        """
        DESCRIPTION = {
            'rosdistro': "ROS2 Distribution",
            'ignore_configs': "Config files to be ignored",
            'always_yes': "quiet mode enabled",
            'selected_pkg_config': "single package to be run"
        }
        # holds codename of the ROS2 distro to be used (foxy, galactic, ...)
        rosdistro: str
        # config files to be ignored
        ignore_configs: [str]
        # quiet mode (always answer yes on interaction)
        always_yes: bool
        # name of single config file to be run
        selected_pkg_config: str

    directories = __Directories()
    settings = __Settings()


vault = Vault()
