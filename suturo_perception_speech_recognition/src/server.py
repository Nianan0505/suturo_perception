#! /usr/bin/env python

import rospy
import suturo_perception_msgs.msg
import BaseHTTPServer
import time

''' 
This server handles incoming requests from the speech recognition 
android app and returns a status to it. 
If a valid command was found, it will be sent via ros.
'''

HOST_NAME = '0.0.0.0'
PORT_NUMBER = 8000
COMMAND_WORDS = set(['cleanup', 'clean', 'remove'])
OBJECT_WORDS = set(['corny', 'cafetfilter'])

pub = rospy.Publisher('/suturo/SpeechRecognitionCommand', suturo_perception_msgs.msg.SpeechRecognitionCommand)
rospy.init_node('talker', anonymous=True)
r = rospy.Rate(10) # 10hz

def publish_command(command, obj):
    ''' publish the recognized command '''    
    rospy.loginfo('publishing...')
    s = suturo_perception_msgs.msg.SpeechRecognitionCommand()
    s.object = obj
    s.command = command
    pub.publish(s)
    r.sleep()

class HttpHandler(BaseHTTPServer.BaseHTTPRequestHandler):
  def do_HEAD(s):
    s.send_response(200)
    s.send_header("Content-type", "text/plain")
    s.end_headers()
  
  def do_GET(s):
    ''' Respond to a HTTP GET request. '''
    s.send_response(200)
    s.send_header("Content-type", "text/plain")
    s.end_headers()
    
    print 'request: ' + s.path
    request = set(s.path.split('/')[1].split('_'))
    
    if request.intersection(COMMAND_WORDS) and request.intersection(OBJECT_WORDS):
      s.wfile.write("OK")
      try:
        publish_command(request.intersection(COMMAND_WORDS).pop(), 
                        request.intersection(OBJECT_WORDS).pop()) # use one command and object from the set
      except rospy.ROSInterruptException: pass
    else: s.wfile.write("NOK")

    
      
if __name__ == '__main__':
  server_class = BaseHTTPServer.HTTPServer
  httpd = server_class((HOST_NAME, PORT_NUMBER), HttpHandler)
  print time.asctime(), "Server started - %s:%s" % (HOST_NAME, PORT_NUMBER)
  try:
      httpd.serve_forever()
  except KeyboardInterrupt:
      pass
  httpd.server_close()
