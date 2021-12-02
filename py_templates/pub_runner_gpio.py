import sys
import os
from std_msgs.msg import String
from gpiozero import Button

from time import sleep

sys.path.append(os.path.dirname(os.path.abspath(__file__)))  # this line has to be above pub import!


class PubPlaceholder:
    def start_publisher(self):
        pass

    def publish(self, msg):
        pass

    def stop_publisher(self):
        pass


pub = PubPlaceholder()
port = 2


# !INSERT_PORT_IMPORT_HERE!
# !INSERT_PUB_IMPORT_HERE!


def main(args=None):
    # GPIO config
    b = Button(port)

    pub.start_publisher()
    old_val = None
    while True:
        try:
            new_val = b.is_pressed
            if new_val != old_val:
                msg = String()
                if new_val:
                    msg.data = "yes"
                else:
                    msg.data = "no"
                pub.publish(msg)
                old_val = new_val
                print("Sent: " + msg.data)
        except KeyboardInterrupt:
            print("\n")
            pub.stop_publisher()
            exit()


if __name__ == '__main__':
    main()
