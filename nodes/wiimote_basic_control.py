#!/usr/bin/env python

import roslib; roslib.load_manifest('sg90dog_ros')
import rospy
from std_msgs.msg import * #import all of the standard message types
from sensor_msgs.msg import Joy
from numpy import *
import time;


class Node():
  def __init__(self):

    #set up your subscribers
    self.pub1 = rospy.publisher("/action",String,queue_size=1)
    self.sub2 = rospy.Subscriber("/joy",Joy,self.sub1Callback)
    #initialize any variables that the class "owns. these will be available in any function in the class.

    self.action = 'stand'


  def sub1Callback(self,data):
    #the actual string is called by data.data. update the appropriate class-owned variable.
    

    outmsg = String()
    if data.buttons[0]:
      outmsg.data = 'stand'
    elif data.buttons[1]:
      outmsg.data='sit'
    elif data.buttons[5]:
      outmsg.data='bump'
    elif data.buttons[6]:
      outmsg.data='walk'
    else:
      outmsg.data='down'
    self.pub1.publish(outmsg)
    
    
      
#main function
def main(args):
  rospy.init_node('dog_control_node', anonymous=True)
  my_node = Node()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 
