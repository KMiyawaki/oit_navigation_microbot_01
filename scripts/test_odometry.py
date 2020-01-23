#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import smach
import smach_ros
from state.go_straight import GoStraightOdomByDistance
from state.turn import TurnOdomByAngle

def main():
    rospy.init_node('smach_example_state_machine')
    rospy.sleep(0.5)  # 起動直後は rospy.Time.now() がゼロを返す．
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    sm.userdata.linear_x = 0.3
    sm.userdata.linear_y = 0
    sm.userdata.distance = 0.5
    sm.userdata.turn_angle = math.radians(90)
    timeout = 20
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('straight_1', GoStraightOdomByDistance('/odom', timeout),
                               transitions={'ok': 'turn_1', 'ng': 'NG', 'timeout': 'straight_2'})
        smach.StateMachine.add('turn_1', TurnOdomByAngle('/odom', timeout),
                               transitions={'ok': 'straight_2', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('straight_2', GoStraightOdomByDistance('/odom', timeout),
                               transitions={'ok': 'turn_2', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('turn_2', TurnOdomByAngle('/odom', timeout),
                               transitions={'ok': 'straight_3', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('straight_3', GoStraightOdomByDistance('/odom', timeout),
                               transitions={'ok': 'turn_3', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('turn_3', TurnOdomByAngle('/odom', timeout),
                               transitions={'ok': 'straight_4', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('straight_4', GoStraightOdomByDistance('/odom', timeout),
                               transitions={'ok': 'turn_4', 'ng': 'NG', 'timeout': 'NG'})
        smach.StateMachine.add('turn_4', TurnOdomByAngle('/odom', timeout),
                               transitions={'ok': 'straight_1', 'ng': 'NG', 'timeout': 'NG'})

    sis = smach_ros.IntrospectionServer('state_machine_simple', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
