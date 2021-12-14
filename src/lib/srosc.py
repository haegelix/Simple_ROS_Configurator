#!/usr/bin/python3
import sys
import os
import subprocess


def run(command, argv):
    proc = subprocess.run([command, *argv])
    exit(proc.returncode)


def main(argv):
    if len(argv) < 1:
        print("Missing subcommand")
        print_help()
        exit(1)

    if argv[0] == "new-ws":
        print("NOT YET IMPLEMENTED")
        run("./new_ws.py", argv[1:])
    elif argv[0] == "run":
        run("./runconfigs.py", argv[1:])
    elif argv[0] == "new":
        run("./newconfig.py", argv[1:])
    elif argv[0] == "web":
        run("./runserver.py", argv[1:])
    elif argv[0] == "stat":
        run("./stats.py", argv[1:])
    else:
        print("Wrong subcommand")
        print_help()


def print_help():
    print_version()
    print("Usage:")
    print("srosc <subcommand> [command line flags]")
    print("")
    print("Available subcommands:")
    print("new-ws  - will create a new workspace")
    print("run     - will run all config files found in this workspace")
    print("new     - will create a new config file in this workspace")
    print("web     - will run the web deamon")
    print("stat    - will output the current status")


def print_version():
    print("Version (TODO)")  # TODO print version


if __name__ == "__main__":
    main(sys.argv[1:])
