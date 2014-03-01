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
  ''' 
  execute uvcdynctrl to adjust the focus on the given video device
  with the given focusValue
  '''
  dev_null = open('/dev/null', 'w') # push uvcdynctrl garbage to /dev/null

  # first: deactivate autofocus on cam
  subprocess.call(['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus, Auto', '0'], 
    stdout=dev_null, stderr=dev_null)
  # set the focusvalue from the request
  subprocess.call(['uvcdynctrl', '-d', str(req.videoDevice), '-s', 'Focus (absolute)', str(req.focusValue)],
    stdout=dev_null, stderr=dev_null)
  rospy.loginfo('[focusdispatcher] Executed focus call. New focus value is: %s', str(req.focusValue))
  return suturo_perception_msgs.srv.ScannerFocusResponse()
      
if __name__ == '__main__':
  rospy.init_node('FocusDispatcherServer')
  s = rospy.Service('/suturo/ScannerFocus', suturo_perception_msgs.srv.ScannerFocus, handle_refocus)
  rospy.loginfo('FocusDispatcher ready')
  rospy.spin()
