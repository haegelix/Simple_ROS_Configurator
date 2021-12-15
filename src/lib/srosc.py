#!/usr/bin/python3
import sys
import os
import subprocess
from lib.paths import paths
import newworkspace
import newconfig
from lib.config import config


def run(command, argv):
    proc = subprocess.run([command, *argv])
    exit(proc.returncode)


def main(argv):
    if len(argv) < 1:
        print("Missing subcommand")
        print_help()
        exit(1)

    paths.switch_to_srosc_lib_dir()

    if argv[0] == "new-ws":
        newworkspace.main(argv[1:])
    elif argv[0] == "run":
        run(paths.get_runnable_path("runconfigs.py"), argv[1:])
    elif argv[0] == "new":
        config.__parse_file__(paths.get_srosc_ws_configfile_path())
        newconfig.main()
    elif argv[0] == "web":
        run(paths.get_runnable_path("runserver.py"), argv[1:])
    elif argv[0] == "stat":
        print("NOT YET IMPLEMENTED")
        run(paths.get_runnable_path("stats.py"), argv[1:])
    elif argv[0] == "help":
        if len(argv) > 1:
            print_sub_help(argv[1])
        else:
            print_help()
    else:
        print("Wrong subcommand")
        print_help()
        exit(1)
    exit(0)


def print_help() -> None:
    print_version()
    print("")
    print_usage()
    print("")
    print_subcommands()


def print_subcommands() -> None:
    print("Available subcommands:")
    print("new-ws        - will create a new workspace")
    print("run           - will run all config files found in this workspace")
    print("new           - will create a new config file in this workspace")
    print("web           - will run the web deamon")
    print("stat          - will output the current status")
    print("help          - will display this help")
    print("help <subcmd> - further help for a specific subcommand")


def print_usage() -> None:
    print("Usage:")
    print("srosc <subcommand> [command line flags]")


def print_version() -> None:
    print("SimpleRosConfigurator Version (TODO)")  # TODO print version


def print_sub_help(subcmd):
    # TODO implement
    print("Subcommand help:")
    print("TODO")


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print("\n\nKilled via KeyboardInterrupt (Ctrl+C). ByeBye!")
        exit(2)
