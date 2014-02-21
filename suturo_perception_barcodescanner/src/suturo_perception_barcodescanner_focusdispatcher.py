#! /usr/bin/env python

import rospy
import suturo_perception_msgs.srv
import subprocess

''' 
This service server is a wrapper around uvcdynctrl. 
The code of uvcdynctrl is so horrendously ugly and unsafe that we will rather call it via shell 
using this service than to fumble in the code of this pile of dirt.
'''

def handle_refocus(req):
  rospy.loginfo('executing focus call')
  
  # execute uvcdynctrl to adjust the focus on the given video device
  # with the given focusValue

  # first: deactivate autofocus on cam
  print ['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus, Auto', '0']
  subprocess.call(['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus, Auto', '0'])
  print ['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus (absolute)', str(req.focusValue)]
  subprocess.call(['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus (absolute)', str(req.focusValue)])
  return suturo_perception_msgs.srv.ScannerFocusResponse()
      
if __name__ == '__main__':
  rospy.init_node('FocusDispatcherServer')
  s = rospy.Service('/suturo/ScannerFocus', suturo_perception_msgs.srv.ScannerFocus, handle_refocus)
  rospy.loginfo('FocusDispatcher ready')
  rospy.spin()
