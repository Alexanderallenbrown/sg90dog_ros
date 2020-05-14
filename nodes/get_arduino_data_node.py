#!/usr/bin/env python

from smbus import SMBus
from std_msgs.msg import Float64
import time
import rospy

bus = SMBus(3) # Raspberry Pi revision 2
address = 0x64

def pubForceSensor():
    pub1 = rospy.Publisher('/force_sensor1', Float64,queue_size = 1)
    pub2 = rospy.Publisher('/force_sensor2', Float64,queue_size = 1)
    pub3 = rospy.Publisher('/force_sensor3', Float64,queue_size = 1)
    pub4 = rospy.Publisher('/force_sensor4', Float64,queue_size = 1)
    rospy.init_node('pubForceReading',anonymous = True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
	forces = bus.read_i2c_block_data(address, 0)
	print forces[0:4]
	rospy.loginfo(forces)
	f1 = Float64()
	f2 = Float64()
	f3 = Float64()
	f4 = Float64()
	f1.data = float(forces[0])
	f2.data = float(forces[1])
	f3.data = float(forces[2])
	f4.data = float(forces[3])
	pub1.publish(f1)
	pub2.publish(f2)
	pub3.publish(f3)
	pub4.publish(f4)
	rate.sleep()

#while True:
#    data = bus.read_i2c_block_data(address,0)
#    print data[0:4]
#    time.sleep(0.1)

if __name__ == '__main__':
    try:
	pubForceSensor()
    except rospy.ROSInterruptException:
	pass
