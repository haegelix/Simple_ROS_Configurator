import sys
import random
import time
from std_msgs.msg import String
from gpiozero import LED

port = 3
led: LED


def init():
    global led
    led = LED(port)
    led.off()


def callback(out: String):
    if out.data == "yes":
        # go blinkyblink
        led.on()
        time.sleep(3)
        led.off()


if __name__ == '__main__':
    init()
    s = String()
    s.data = "yes"
    callback(s)
