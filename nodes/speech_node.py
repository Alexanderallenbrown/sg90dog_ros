#!/usr/bin/env python

import roslib; roslib.load_manifest('sg90dog_ros')
import rospy
from std_msgs.msg import * #import all of the standard message types
from sensor_msgs.msg import Joy
from numpy import *
import time;
import subprocess
import speech_recognition as sr


class Node():
  def __init__(self):
    ospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    self.package_path=rospack.get_path('sg90dog_ros')
    #set up your subscribers
    self.pub1 = rospy.Publisher("/action",String,queue_size=1)
    #initialize any variables that the class "owns. these will be available in any function in the class.
    self.process = os.system(['arecord -d 2 -D plughw:1 -c1 -r 48000 -f S32_LE -t wav -V mono -v '+self.package_path+'/wav/file.wav'])
    self.action = 'stand'
    self.r = sr.Recognizer()

    

    self.rectimer =  rospy.Timer(rospy.Duration(2),self.recloop,oneshot=False) #timer callback (math) allows filter to run at constant time
    self.proctimer = rospy.Timer(rospy.Duration(2),self.procloop,oneshot=False)
    self.afile = self.package_path+'/wav/file.wav'
  def recloop(self,event):
    self.process = os.system(['arecord -d 2 -D plughw:1 -c1 -r 48000 -f S32_LE -t wav -V mono -v '+self.package_path+'/wav/file.wav'])
  
  def procloop(self,event):
    with sr.AudioFile(afile) as source:
        audio = self.r.record(source)  # read the entire audio file
    try:
        print("Sphinx thinks you said " + r.recognize_sphinx(audio))
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))

      
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
