#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import smach
import smach_ros
from state.get_image__state import GetImageState


def main():
    rospy.init_node('smach_example_state_machine')
    rospy.sleep(0.5)  # 起動直後は rospy.Time.now() がゼロを返す．
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('get_image', GetImageState('/jetbot_camera_image_modifier/mod_image', '/tmp/01.jpg'),
                               transitions={'ok': 'OK', 'ng': 'NG'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
