#!/bin/bash
rosrun xacro xacro --inorder microbot_01.xacro -o /tmp/robot.urdf
check_urdf /tmp/robot.urdf
