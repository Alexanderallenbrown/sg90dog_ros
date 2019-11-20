#!/usr/bin/env python

import roslib; roslib.load_manifest('sg90dog_ros')
import rospy
from std_msgs.msg import * #import all of the standard message types
from numpy import *
import time;

from SG90Dog import SG90Dog


#this node subscribes to a float and a string. The float represents the input to a first order system. The string represents a state.

class Node():
  def __init__(self):

    self.dog = SG90Dog() 
    self.timenow = rospy.Time.now()#in case you need this
    self.frequency = 6.0
    self.amplitude = 0.01    
    #set up your publishers with appropriate topic types

    #set up your subscribers
    self.sub1 = rospy.Subscriber("/action",String,self.sub1Callback)
    self.sub2 = rospy.Subscriber("/frequency",Float32,self.sub2Callback)
    self.sub3 = rospy.Subscriber("/amplitude",Float32,self.sub3Callback)
    #initialize any variables that the class "owns. these will be available in any function in the class.

    self.action = 'stand'

    self.dt = 0.01
    #set up timed loop to run like an arduino's "void loop" at a particular rate (100Hz)
    rospy.Timer(rospy.Duration(self.dt),self.loop,oneshot=False) 


  def sub1Callback(self,data):
    #the actual string is called by data.data. update the appropriate class-owned variable.
    self.action = data.data
    
  def sub2Callback(self,data):
    self.frequency = data.data
  def sub3Callback(self,data):
    self.amplitude = data.data


  def loop(self,event):
    #this function runs over and over again at dt.
    #do stuff based on states. 

    self.dog.update(self.dt,self.action,self.frequency,self.amplitude)


    
      
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
