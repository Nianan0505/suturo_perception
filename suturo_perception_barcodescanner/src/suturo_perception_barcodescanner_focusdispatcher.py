#! /usr/bin/env python

import rospy
import actionlib
import suturo_perception_msgs.msg
import subprocess
import shlex

class ScannerFocusAction(object):
  ''' 
  This action server is a wrapper around uvcdynctrl. 
  The code of uvcdynctrl is so horrendously ugly and unsafe that we will rather call it via shell 
  using this action server then to fumble in the code of this pile of dirt.
  '''
  # create messages that are used to publish feedback/result
  _feedback = suturo_perception_msgs.msg.ScannerFocusFeedback()
  _result   = suturo_perception_msgs.msg.ScannerFocusResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, suturo_perception_msgs.msg.ScannerFocusAction, 
      execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    rospy.loginfo('FocusDispatcher ready')

  def execute_cb(self, goal):
    # helper variables
    success = True
    
    self._feedback.feedbackValue = goal.focusValue

    rospy.loginfo('executing FocusAction')
    
    # execute uvcdynctrl to adjust the focus on the given video device
    # with the given focusValue
    subprocess.call(['uvcdynctrl', '-d', str(goal.videoDevice), '-s', 'Focus (absolute)', str(goal.focusValue)])

    # start executing the action
    rospy.loginfo('focusValue: %s' % goal.focusValue)
    for i in xrange(1, goal.focusValue):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break
      self._as.publish_feedback(self._feedback)
      
    if success:
      #self._result.feedbackValue = self._feedback.feedbackValue
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('FocusDispatcher')
  ScannerFocusAction(rospy.get_name())
  rospy.spin()
