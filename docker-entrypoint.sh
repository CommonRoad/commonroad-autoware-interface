#!/bin/bash

# setup ros environment
cd "/root/workspace/reactive-planner"
pip install .
source "/opt/ros/galactic/setup.bash"
cd "/root/workspace/dfg-car"
colcon build
source "/root/workspace/dfg-car/install/setup.bash"
source "/root/swri_console/install/setup.bash"
source "/root/autoware_auto_msgs/install/setup.bash"
exec "$@" # run parameter