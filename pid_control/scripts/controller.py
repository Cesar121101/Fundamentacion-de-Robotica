#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)
setpoint = 0.0
output = motor_input()
superError = 0.0
currentTime = 0.0
step = 0
prev_time = -100.0

def PID(error):
    global currentTime
    global step
    global superError
    
    # P
    Kp = rospy.get_param("Kp", "No param found")
    P = Kp*error

    # I
    Ts = 1/100
    superError += error
    Ki = rospy.get_param("Ki", "NO param found")
    I = superError*Ki*Ts

    # D
    D = 0


    return (P + I + D)

def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.setpoint)

    global setpoint
    setpoint = msg.setpoint

def motor_output_callback(msg):
    global currentTime
    global step
    step += 1
    rospy.loginfo("Motor output: %s", msg.output)
    
    global setpoint
    global output
    
    error = setpoint - msg.output
    currentTime = rospy.get_time()
    control = PID(error)

    # Throw out
    output.input = control
    output.time = currentTime

    rospy.loginfo("Error: %s", error)
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