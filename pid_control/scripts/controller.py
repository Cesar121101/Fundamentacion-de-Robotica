#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)
def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.data)
    rospy.get_param("Setpoint", "No param found")

def motor_output_callback(msg):
    rospy.loginfo("Motor output: %s", msg.output)

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    rospy.Subscriber("set_point", set_point, set_point_callback)
    rospy.Subscriber("motor_output", motor_output, motor_output_callback)
    pub = rospy.Publisher("motor_input", motor_input , queue_size=1) 

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Write your code here

        rate.sleep()