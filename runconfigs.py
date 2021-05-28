import os
import subprocess
from lib.config import Config
from lib.paths import Paths
from lib.package_config import PackageInfo

# load config
_config: Config
# set paths
_paths: Paths
# set package_info
_package_info: PackageInfo


def clear_console():
    command = 'clear'
    if os.name in ('nt', 'dos'):  # If Machine is running on Windows, use cls
        command = 'cls'
    os.system(command)


class Runconfigs:

    def __init__(self):
        global _config, _paths, _package_info
        _config = Config()
        _paths = Paths(_config)
        _package_info = PackageInfo((_paths, _config))

    def run(self):
        # self.clear_console()
        self.shell_source()
        self.print_config()
        self.check_generate_ws()
        self.build_packages()
        self.resolve_dep()
        _package_info.load_packagexml("")

    def shell_source(self):
        try:
            r = subprocess.run(["ros2"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        except FileNotFoundError:
            print("Restarting using bash.")
            ret = os.system("bash -c \"source " + _config.ros_source_path + "; python3 runconfigs.py\"")
            exit(ret)
        else:
            print("ROS executables found.")

    def print_config(self):
        print("CONFIG:")
        print(str(_config))
        print("\n################################\n")
        print("PATHS:")
        print(str(_paths))
        print("\n################################\n")

    def check_generate_ws(self):
        try:
            _paths.switch_to_ws_dir()
        except PermissionError:
            print("No permission to use the directory " + _paths.ros_ws)
            print("Exiting!")
            exit(1)
        except NotADirectoryError:
            print("" + _paths.ros_ws + " is not a directory")
            print("Exiting!")
            exit(1)
        except FileNotFoundError:
            print("Specified ROS-workspace not found!")
            print("Shall I generate a new one?")
            answer = input("[Y]es or [N]o? ").upper()
            if answer == "YES" or answer == "Y":
                print("Generating...")
                os.makedirs(_paths.ros_ws_src)
                print("Done.")
                return
            elif answer == "NO" or answer == "N":
                print("Please change config before running again.\nExit!")
                exit(1)
            else:
                print("Answer not valid, please try again by re-running me.\nExit!")
                exit(1)

    def build_packages(self):
        configs_path = _paths.srosc_configs

        packages = [j for j in os.listdir(configs_path) if j.endswith(".json")]
        packages.remove(_config.foreign_repos_config)
        for p in packages:
            print("Building package from file '" + p + "'")
            _package_info.load_package_config(p)
            pack_src_path = _paths.get_package_src_path(_package_info.pkg_config.package_name)
            print("Package path: " + pack_src_path)
            try:
                # Build dir structure
                if os.path.isdir(pack_src_path):
                    print("Update not yet implemented!")
                    print("Skipping package " + _package_info.pkg_config.package_name)
                    # print("Delete " + pack_src_path + " and re-run!")
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                _paths.switch_to_ws_source_dir()
                os.system("ros2 pkg create --build-type ament_python " + _package_info.pkg_config.package_name)
                pass
            print("\n---\n")
            pass
        pass

    def resolve_dep(self):
        print("Resolving and installing dependencies...")
        cwd = os.getcwd()
        os.chdir(_paths.ros_ws)
        os.system("rosdep install -i --from-path src --rosdistro foxy -y")
        # os.system("pwd")
        os.chdir(cwd)
        print("Done.")


def main():
    Runconfigs().run()


if __name__ == "__main__":
    main()
