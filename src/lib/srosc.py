#!/usr/bin/python3
# external modules
import sys
import subprocess

# custom lib modules
from lib import config
from lib import paths

# custom app modules
import newworkspace
import newconfig
import runconfigs
import stats
from web import app as webserver


def run(command, argv):
    proc = subprocess.run([command, *argv])
    exit(proc.returncode)


def main(argv):
    if len(argv) < 1:
        print("Missing subcommand")
        print_help()
        exit(1)

    if argv[0] == "new-ws":
        newworkspace.main(argv[1:])
    elif argv[0] == "run":
        config.parse_workspace_config()
        runconfigs.parse_argv(argv[1:])
        runconfigs.main()
    elif argv[0] == "new":
        config.parse_workspace_config()
        newconfig.main()
    elif argv[0] == "web":
        config.parse_workspace_config()
        webserver.main()
    elif argv[0] == "stat":
        config.parse_workspace_config()
        stats.main(argv[1:])
    elif argv[0] == "help" or argv[0] == "--help":
        if len(argv) > 1:
            print_sub_help(argv[1])
        else:
            print_help()
    else:
        print("Wrong subcommand")
        print_help()
        exit(1)


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
    file = open(paths.get_lib_file_path(".version"), "r")
    v = file.read()
    print("You are using version", v, "of SimpleRosConfigurator.")
    file.close()


def print_sub_help(subcmd):
    print("Subcommand help:", "srosc", subcmd)
    print("")
    if subcmd == "new-ws":
        # newworkspace.print_help()  # TODO
        print("Not yet implemented this help entry.")
    elif subcmd == "run":
        runconfigs.print_help()
    elif subcmd == "new":
        # newconfig.print_help()  # TODO
        print("Not yet implemented this help entry.")
    elif subcmd == "web":
        # webserver.print_help()  # TODO
        print("Not yet implemented this help entry.")
    elif subcmd == "stat":
        # stats.print_help()  # TODO
        print("Not yet implemented this help entry.")
    print("")
    print_version()


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        exit("\n\nKilled via KeyboardInterrupt (Ctrl+C). ByeBye!")
    print("Thanks for using. ByeBye!")
