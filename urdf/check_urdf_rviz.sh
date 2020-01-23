#!/bin/bash
rosrun xacro xacro --inorder microbot_01_jetbot_camera.xacro -o /tmp/robot.urdf
roslaunch urdf_tutorial display.launch model:=/tmp/robot.urdf
