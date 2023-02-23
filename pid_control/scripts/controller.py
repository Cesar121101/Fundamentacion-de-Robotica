#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)
setpoint = 0.0
output = motor_input()

def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.setpoint)

    global setpoint
    setpoint = msg.setpoint

def motor_output_callback(msg):
    rospy.loginfo("Motor output: %s", msg.output)
    
    global setpoint
    global output
    Kp = rospy.get_param("Kp", "No param found")
    e = setpoint - msg.output
    output.input = Kp * e
    output.time = rospy.get_time()

    rospy.loginfo("Error: %s", e)
    rospy.loginfo("Kp: %s", Kp)
    rospy.loginfo("Motor input: %s", output)

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")

    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    rospy.Subscriber("set_point", set_point, set_point_callback)
    rospy.Subscriber("motor_output", motor_output, motor_output_callback)
    input_pub = rospy.Publisher("motor_input", motor_input , queue_size=1) 

    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
        input_pub.publish(output)
        rate.sleep()