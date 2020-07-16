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
    self.timenow = rospy.Time.now() #in case you need this
    self.frequency = 9.0
    self.amplitude = 0.01
    self.footforce1 = 0
    self.footforce2 = 0
    self.footforce3 = 0
    self.footforce4 = 0
    #set up your publishers with appropriate topic types

    #set up your subscribers
    self.sub1 = rospy.Subscriber("/action",String,self.sub1Callback)
    self.sub2 = rospy.Subscriber("/frequency",Float32,self.sub2Callback)
    self.sub3 = rospy.Subscriber("/amplitude",Float32,self.sub3Callback)
    self.footsub1 = rospy.Subscriber("/force_sensor1",Float64,self.foot1Callback)
    self.footsub2 = rospy.Subscriber("/force_sensor2",Float64,self.foot2Callback)
    self.footsub3 = rospy.Subscriber("/force_sensor3",Float64,self.foot3Callback)
    self.footsub4 = rospy.Subscriber("/force_sensor4",Float64,self.foot4Callback)

    #initialize any variables that the class "owns. these will be available in any function in the class.

    self.action = 'stand'

    self.dt = 0.01 #0.1
    #set up timed loop to run like an arduino's "void loop" at a particular rate (100Hz)
    rospy.Timer(rospy.Duration(self.dt),self.loop,oneshot=False)
    self.dT = self.dt
    self.time = time.time()
    self.oldtime = self.time-self.dt

  def sub1Callback(self,data):
    #the actual string is called by data.data. update the appropriate class-owned variable.
    self.action = data.data

  def sub2Callback(self,data):
    self.frequency = data.data
  def sub3Callback(self,data):
    self.amplitude = data.data

  def foot1Callback(self, data):
    self.footforce1 = data.data
  def foot2Callback(self, data):
    self.footforce2 = data.data
  def foot3Callback(self, data):
    self.footforce3 = data.data
  def foot4Callback(self, data):
    self.footforce4 = data.data


  def loop(self,event):
    #this function runs over and over again at dt.
    #do stuff based on states.
    self.time = time.time()
    self.dT = self.time-self.oldtime
    self.oldtime = self.time
    self.dog.update(self.dT,self.action,self.frequency,self.amplitude,self.footforce1,self.footforce2,self.footforce3,self.footforce4)



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
