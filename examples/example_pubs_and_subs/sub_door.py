import sys
import threading
import time
from std_msgs.msg import String

door_closed = """
 _______________
|  ___________  |
| |   _ _ _   | |
| |  |     |  | |
| |  |     |  | |
| |  |_ _ _|  | |
| |           | |
| |        () | |
| |        || | |
| |        () | |
| |           | |
| |           | |
| |           | |
|_|___________|_|
"""

door_open = """
 _______________
|  ___________  |
| |  ,' | |   | |
| | |   | |   | |
| | |   | |   | |
| | |  ,' |   | |
| | |,' o |   | |
| |     H |   | |
| |     o |   | |
| |       |   | |
| |       |   | |
| |      ,'   | |
| |   ,'      | |
|_|,'_________|_|
"""

door_ring = """
                 /~~~~~~~~~\\
                < RING RING >
                 \\~~~~~~~~~/

 _______________
|  ___________  |
| |   _ _ _   | |
| |  |     |  | |
| |  |     |  | |
| |  |_ _ _|  | |
| |           | |
| |        () | |
| |        || | |
| |        () | |
| |           | |
| |           | |
| |           | |
|_|___________|_|
"""


def door_print(door: str):
    print("\n" * 50)
    print(door)


def init():
    door_print(door_closed)


t_open: threading.Thread


def callback(out: String):
    global t_open

    # if thread already ran, or is currently running: join it
    try:
        if t_open and t_open.is_alive():
            t_open.join()
    except NameError:
        pass

    # assemble new thread and run it
    t_open = threading.Thread(target=go_open_the_door, args=())
    if out.data == "yes":
        t_open.start()


def go_open_the_door():
    # get random time-to-open
    # sleep_time = random.randint(9, 15)
    sleep_time = 10

    # show ringing door
    door_print(door_ring)
    time.sleep(1)
    door_print(door_closed)
    time.sleep(1)
    door_print(door_ring)
    time.sleep(1)

    # show closed door (waiting for opening)
    door_print(door_closed)
    for i in range(sleep_time):
        time.sleep(1)
        print(".", end="")
        sys.stdout.flush()

    # show open door
    door_print(door_open)
    time.sleep(3)

    # show closed-again door
    door_print(door_closed)


if __name__ == '__main__':
    init()
    s = String()
    s.data = "yes"
    callback(s)
