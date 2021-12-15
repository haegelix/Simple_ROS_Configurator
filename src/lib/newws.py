#!/usr/bin/python3
import sys
import shutil
import os
import re
import errno
from lib.paths import paths


name_pattern = re.compile(r"[\w]+")


def main(argv) -> None:
    print("I'll generate a new workspace for you.")
    print("But you'll need to tell me a name for it.")
    print("Only use letters (a-z or A-Z), numbers (0-9) an unserscores (_).")

    paths.switch_to_srosc_ws_dir()

    ws_name = "srosc_ws"
    while True:
        new_ws_name = input("Please tell me a suitable name [" + ws_name + "]:")
        if new_ws_name != "":
            ws_name = new_ws_name
        if name_pattern.search(ws_name):
            break
        else:
            print("Please enter a valid name.")

    try:
        os.mkdir(ws_name)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            print("Some error just popped up.")
            raise
        else:
            print("This name does already exist. Cannot override.")
            exit(1)

    print("I managed to create your workspace.")
    print("The next step is to copy a standard config file to it.")
    shutil.copy2(paths.get_global_config_path(), os.path.join(".", ws_name))
    print("I'm done. Have fun!")
    exit(0)


if __name__ == "__main__":
    main(sys.argv[1:])
