#!/usr/bin/python3
import json
import os

import lib.helpers as helpers
from lib.logadapter import logging, setup_logging
import re

# ^[\w.-]+@([\w-]+.)+[\w-]{2,4}$
#
mail_pattern = r'^[a-z0-9]([_.-][a-z0-9]|[a-z0-9])*[@][a-z0-9]([.-][a-z0-9]|[a-z0-9])+[.][a-z]{2,16}$'


class Obj(object):
    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=False, indent=4)


def main():
    setup_logging("newconfig.log")
    pkg = Obj()
    print("So you want to create a config file...")
    print("May I ask you some questions?")
    print("First some general questions about your new package:")
    pkg.package_name = input("Name for this package? ")
    pkg.package_info = Obj()
    pkg.package_info.version = input("Version? ")
    pkg.package_info.description = input("Description? ")
    pkg.package_info.license = input("What license? ")

    print("Next I'll need some personal information:")
    pkg.package_info.maintainer = input("Your full name / nickname? ")
    while True:
        pkg.package_info.maintainer_mail = input("Your email? ")
        if re.match(mail_pattern, pkg.package_info.maintainer_mail):  # email address is valid!
            break
        print("Mail address invalid. Scheme x@y.z must be used!\n"
              "- x must only contain alphanumeric characters, hyphens, dots and underscores\n"
              "- x must not end or begin with hyphen, dot nor underscore\n"
              "- x must contain at least one character\n"
              "- y must only contain alphanumeric characters, hyphens and dots\n"
              "- y must not end or begin with hyphen nor dot\n"
              "- y must not contain a dot followed by a hyphen or the other way round\n"
              "- y must contain at least one character\n"
              "- z must only contain characters and may have length between 2 and 16\n"
              "As RegEx: " + mail_pattern)

    # Auto-add some currently unchangeable values. # TODO make changeable
    pkg.package_info.test_depends = ["ament_copyright", "ament_flake8", "ament_pep257", "python3-pytest"]
    pkg.package_info.exec_depends = ["rclpy", "std_msgs"]
    pkg.additional_imports = []

    print("Shall there be sub(s)?")
    pkg.subs = []
    sub_counter = 1
    while True:
        if not helpers.yes_or_no(use_config=False):
            break
        print("I need some details about sub #" + str(sub_counter) + "...")
        sub = Obj()
        sub.node_name = input("Node name? ")
        sub.topic = input("Topic to sub? ")
        print("Available types:\n"
              "std_msgs.msg.{ Bool | Byte | ByteMultiArray | Char | ColorRGBA | Duration | Empty | Float32 "
              "| Float32MultiArray | Float64 | Float64MultiArray | Header | Int16 | Int16MultiArray | Int32 "
              "| Int32MultiArray | Int64 | Int64MultiArray | Int8 | Int8MultiArray | MultiArrayDimension "
              "| MultiArrayLayout | String | Time | UInt16 | UInt16MultiArray | UInt32 | UInt32MultiArray "
              "| UInt64 | UInt64MultiArray | UInt8 | UInt8MultiArray }")
        sub.type = input("What type? ")
        sub.callback = input("What to do on receive? ")
        sub_counter += 1
        pkg.subs.append(sub)
        print("Shall there be more subs?")
        pass

    print("Shall there be pub(s)?")
    pkg.pubs = []
    pub_counter = 1
    while True:
        if not helpers.yes_or_no(use_config=False):
            break
        print("I need some details about pub #" + str(pub_counter) + "...")
        pub = Obj()
        pub.node_name = input("Node name? ")
        pub.topic = input("Topic to pub? ")
        print("Available types:\n"
              "std_msgs.msg.{ Bool | Byte | ByteMultiArray | Char | ColorRGBA | Duration | Empty | Float32 "
              "| Float32MultiArray | Float64 | Float64MultiArray | Header | Int16 | Int16MultiArray | Int32 "
              "| Int32MultiArray | Int64 | Int64MultiArray | Int8 | Int8MultiArray | MultiArrayDimension "
              "| MultiArrayLayout | String | Time | UInt16 | UInt16MultiArray | UInt32 | UInt32MultiArray "
              "| UInt64 | UInt64MultiArray | UInt8 | UInt8MultiArray }")
        pub.type = input("What type? ")
        print("Available message sources:\n"
              "-stdin- This will send std_msgs.msg.String messages containing the content of stdin\n"
              "-button- This will send std_msgs.msg.String messages whenever button (GPIO2) is pressed/released")
        pub.src = input("Shall I generate a message-source from template? ")
        pub_counter += 1
        pkg.pubs.append(pub)
        print("Shall there be more pubs?")
        pass

    # write to file
    print("So this is what I'd write to a file:")
    s = pkg.to_json()
    print(s)
    print("\nShall I write the file?")
    if helpers.yes_or_no(use_config=False):  # yes
        filename = input("What shall be the filename (please omit the .json)? ")
        out_file = open("./configs/" + filename + ".json", "w")
        out_file.write(s)
        out_file.close()

        print("\nShall I directly build your package?")
        if helpers.yes_or_no(use_config=False):  # yes
            os.system("bash ./runserver.sh")
    else:  # no
        return


if __name__ == "__main__":
    main()

"""
{
  "package_name": "pub_button",
  "package_info": {
    "version": "0.0.1",
    "description": "Description of the pub-package",
    "maintainer_mail": "test@test.de",
    "maintainer": "First Name",
    "license": "None yet",
    "test_depends": [
      "ament_copyright",
      "ament_flake8",
      "ament_pep257",
      "python3-pytest"
    ],
    "exec_depends": [
      "rclpy",
      "std_msgs"
    ]
  },
  "subs": [
  ],
  "pubs": [
    {
      "node_name": "example",
      "topic": "topic",
      "type": "std_msgs.msg.String",
      "src": "-button-"
    }
  ],
  "additional_imports": [
  ]
}
"""
