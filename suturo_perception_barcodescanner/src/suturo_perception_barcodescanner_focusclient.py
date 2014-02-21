#! /usr/bin/env python
import rospy

import suturo_perception_msgs.srv
import sys

def focus_client(req_value):
    rospy.wait_for_service('/suturo/ScannerFocus')
    
    try:
        refocus = rospy.ServiceProxy('/suturo/ScannerFocus', suturo_perception_msgs.srv.ScannerFocus)
        resp = refocus(focusValue=int(req_value), videoDevice='/dev/video1')
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    rospy.loginfo('completed successfully')

if __name__ == '__main__':
    rospy.loginfo('starting FocusClient')
    focus_client(sys.argv[1])