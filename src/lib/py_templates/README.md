#How to use pub.py

<pre><code>import sys
sys.path.append(PATH TO APPEND)

import pub
from std_msgs.msg import String

def main(args=None):
    pub.start_publisher()
    for i in range(5):
        try:
            val = input("Input msg: ")
        except KeyboardInterrupt:
            print("\n")
            pub.stop_publisher()
            exit()
        msg = String()
        msg.data = val
        pub.publish(msg)

if __name__ == '__main__':
    main()
</code></pre>


Here the appended path is 
<code> /home/ubuntu/ros_ws/install/pub_TEST/lib/python3.8/site-packages/pub_TEST </code>