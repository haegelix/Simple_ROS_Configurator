#!/usr/bin/bash

# source paths for ros2 suite
source /opt/ros/foxy/setup.bash

# set environment to enable dev-tools e.g. detection of changed files
export FLASK_ENV=development

python3 app.py