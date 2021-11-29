import os
import subprocess
from lib.config import config
from lib.logadapter import logging
from lib.paths import paths
from lib.packageinfo import pkg_info
from lib import helpers
from lib import ros_api


def clear_console():
    """
    Clears the terminal.
    :return: Nothing.
    """
    command = 'clear'
    if os.name in ('nt', 'dos'):  # If Machine is running on Windows, use cls
        command = 'cls'
    os.system(command)


class Runconfigs:
    def run(self):
        """
        Triggers sub methods.
        :return: Nothing.
        """
        self.print_config()
        self.check_generate_ws()
        self.create_packages()
        ros_api.resolve_dep(paths)
        # pkg_info.load_packagexml("")

    def print_config(self) -> None:
        """
        Print all config values.
        :return: Nothing
        """
        logging.debug("### CONFIG ###\n" + str(config))
        logging.debug("### PATHS ###\n" + str(paths))

    def check_generate_ws(self) -> None:
        """
        Check for existence of the specified ROS-workspace.
        A new one will be generated if it doesn't exist.
        :return: Nothing.
        """
        try:
            paths.switch_to_ws_dir()
        except PermissionError:
            logging.critical("No permission to use the directory " + paths.ros_ws)
            logging.critical("Exiting!")
            exit(1)
        except NotADirectoryError:
            logging.critical("Workspace at " + paths.ros_ws + " does exist, but is not a directory")
            logging.critical("Exiting!")
            exit(1)
        except FileNotFoundError:
            logging.info("Specified ROS-workspace not found!")
            print("Shall I generate a new one?")
            answer = input("[Y]es or [N]o? ")
            if helpers.acceptable_answer_str(answer, ["y", "yes"]):
                logging.info("Generating new workspace...")
                os.makedirs(paths.ros_ws_src)
                logging.info("Done.")
                return
            elif helpers.acceptable_answer_str(answer, ["n", "no"]):
                logging.info("Please change config before running again.\nExit!")
                exit(1)
            else:
                logging.warning("Answer not valid, please try again by re-running me.\nExit!")
                exit(1)

    def create_packages(self):
        """
        Build all packages defined in the configs.
        :return: Nothing.
        """
        # get all filenames and remove the ones to be ignored
        packages = [j for j in os.listdir(paths.srosc_configs) if j.endswith(".json")]
        packages.remove(config.foreign_repos_config)
        for i in config.ignore_configs:
            packages.remove(i)
        logging.info("Found " + str(len(packages)) + " package(s) to build")
        pkg_counter = 0

        # build all packages in the list
        for p in packages:
            pkg_counter += 1
            logging.info("Building package "
                         + str(pkg_counter) + " of " + str(len(packages))
                         + " from file '" + p + "'")
            pkg_info.load_package_config(p)
            logging.info(str(pkg_info))
            pack_src_path = paths.get_package_src_path(pkg_info.pkg_config.package_name)
            logging.info("Package path: " + pack_src_path)
            try:
                # Build dir structure
                if os.path.isdir(pack_src_path):
                    logging.warning("Update not yet implemented!")
                    logging.warning("Skipping package " + pkg_info.pkg_config.package_name)
                    # logging.warning("Delete " + pack_src_path + " and re-run!")
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                ros_api.create_package(paths, pkg_info)
                self.copy_files()
                pass
            pass
        pass

    def copy_files(self):
        """
        TODO docs
        :return:
        """
        # TODO subs
        for p in pkg_info.pkg_config.pubs:
            # build a line of python code for this publisher, that looks like this:
            # p = _Pub("NODENAME", MESSAGETYPE, "TOPIC")
            p_insertion = 'p = _Pub("' + p.node_name + '", ' + p.type + ', "' + p.topic + '")'
            ins = ("# !INSERT_PUBLISHER_DECLARATION_HERE!", p_insertion)
            helpers.modify_and_copy_python_file(paths, pkg_info, "pub", p.node_name + "_pub", [ins])
            if helpers.acceptable_answer_str(p.src, "-stdin-"):
                # use the -stdin- runner as an input for the publisher
                ins = ("# !INSERT_PUB_IMPORT_HERE!", "import " + p.node_name + "_pub as pub")
                helpers.modify_and_copy_python_file(paths, pkg_info, "pub_runner_stdin",
                                                    p.node_name + "_runner", [ins])
                pass
            else:
                pass
        pass

    def register_entry_points(self):
        """
        TODO implement & doc
        :return:
        """
        #   entry_points={
        #       'console_scripts': [
        #           'talker = pub_TEST.test:main',
        #           'pub = pub_TEST.pub'
        #       ]
        #   }
        pass


def probe_ros() -> bool:
    """
    Try to run ros2 framework.
    :return: True if this succeeded and False otherwise
    """
    try:
        r = subprocess.run(["ros2"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        return False
    else:
        return True


def main():
    """
    Checks for availability of ROS2 and the runs the app.
    :return: Nothing.
    """
    if not probe_ros():
        logging.info("ROS was not found --> trying to source...")
        ret = os.system("bash -c \"source " + config.ros_source_path + "; python3 runconfigs.py\"")
        exit(ret)
    else:
        logging.info("Starting up.")
        # try:
        Runconfigs().run()
        # except Exception:
        #     pass
        logging.info("Finished.")


if __name__ == "__main__":
    main()

# TODO rest implementieren
"""
# colcon build --packages-select PACKAGE
# colcon build

# . install/setup.bash
# Punkt ist wichtig, oder

# ros2 run PACKAGE ENTRYPOINT
"""
