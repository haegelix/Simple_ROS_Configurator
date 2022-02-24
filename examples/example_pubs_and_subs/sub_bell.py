from std_msgs.msg import *

switch_string = {
    "yes": True,
    "no": False,
    "on": True,
    "off": False,
    "": False  # default
}


def callback(out):
    if isinstance(out, String):
        if switch_string.get(out.data, False):
            print("RING!!")
