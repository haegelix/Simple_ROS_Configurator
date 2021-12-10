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
        exit(1)

    if argv[0] == "run":
        run("./runconfigs.py", argv[1:])
    elif argv[0] == "new":
        run("./newconfig.py", argv[1:])
    elif argv[0] == "web":
        run("./runserver.py", argv[1:])
    else:
        print("Wrong subcommand")


if __name__ == "__main__":
    main(sys.argv[1:])
