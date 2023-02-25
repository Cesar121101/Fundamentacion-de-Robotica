#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point
from std_msgs.msg import Float32

#Setup parameters, vriables and callback functions here (if required)
setpoint = 0.0
output = motor_input()
superError = 0.0
currentTime = 0.0
prevError = 0.0
errorGlob = 0.0

def PID(error):
    global currentTime
    global superError
    global prevError
    
    # P
    Kp = rospy.get_param("Kp", "No param found")
    P = Kp*error

    # I
    Ts = 1/100
    superError += error
    Ki = rospy.get_param("Ki", "NO param found")
    I = superError*Ki*Ts

    # D
    Kd = rospy.get_param("Kd", "NO param found")
    D = Kd*(error-prevError)*0.001

    prevError = error

    return (P + I + D)

def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.setpoint)

    global setpoint
    setpoint = msg.setpoint

def motor_output_callback(msg):
    global currentTime
    global errorGlob
    rospy.loginfo("Motor output: %s", msg.output)
    
    global setpoint
    global output
    
    error = setpoint - msg.output
    errorGlob = error
    currentTime = rospy.get_time()
    control = PID(error)

    if control > 1:
        control = 1
    elif control < -1:
        control = -1

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
    error_pub = rospy.Publisher("error", Float32 , queue_size=1) 

    #Run the node
    while not rospy.is_shutdown():

        #Write your code here
        error_pub.publish(errorGlob)
        input_pub.publish(output)
        rate.sleep()