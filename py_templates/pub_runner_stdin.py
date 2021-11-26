import sys
import os
from std_msgs.msg import String

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import pub as pub
# !INSERT_PUB_IMPORT_HERE!


def main(args=None):
    pub.start_publisher()
    for i in range(5):
        val = ""
        try:
            val = input("Input msg: ")
            msg = String()
            msg.data = val
            pub.publish(msg)
        except KeyboardInterrupt:
            print("\n")
            pub.stop_publisher()
            exit()


if __name__ == '__main__':
    main()
