#!/bin/bash
source ~/catkin_build_ws/install/setup.bash --extend
PKG_ROOT=$(rospack find oit_navigation_microbot_01)
LAUNCH="${PKG_ROOT}/launch/real/detect_collision.launch"
roslaunch ${LAUNCH} python3:=true

