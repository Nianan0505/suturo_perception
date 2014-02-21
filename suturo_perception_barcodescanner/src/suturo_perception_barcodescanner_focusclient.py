#! /usr/bin/env python
import rospy

import actionlib
import suturo_perception_msgs.msg
import sys

def focus_client(order_value):
    rospy.loginfo('starting FocusClient')
    client = actionlib.SimpleActionClient('FocusDispatcher', suturo_perception_msgs.msg.ScannerFocusAction)
    rospy.loginfo('created actionclient')
    client.wait_for_server()

    goal = suturo_perception_msgs.msg.ScannerFocusGoal(focusValue=int(order_value), videoDevice='/dev/video1')
    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo('completed successfully')
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('scannerFocusClient')
        feedback = focus_client(sys.argv[1])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"