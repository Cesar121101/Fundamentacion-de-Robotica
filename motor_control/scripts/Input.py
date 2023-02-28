#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from motor_control.msg import motor_input

# Setup Variables, parameters and messages to be used

#Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Input Genertor is Running")

    #Initialise and Setup node
    rospy.init_node("input")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publisher
    pwm_pub = rospy.Publisher("cmd_pwm", motor_input , queue_size=1) 

	#Run the node
    while not rospy.is_shutdown():
        msg = motor_input()
        msg.input = np.sin(rospy.get_time()*0.8)*4
        msg.time = rospy.get_time()

		#Write your code here
        pwm_pub.publish(msg)
        rate.sleep()