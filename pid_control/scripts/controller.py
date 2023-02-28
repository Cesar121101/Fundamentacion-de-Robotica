#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point
from std_msgs.msg import Float32

#Setup parameters, vriables and callback functions here (if required)
setpoint = 0.0
out = motor_input()
superError = 0.0
currentTime = 0.0
prevTime = 0.0
prevError = 0.0
errorGlob = 0.0
motorOut = motor_output()

def PID(error):
    global currentTime
    global prevTime
    global superError
    global prevError

    dt = currentTime-prevTime
    
    # P
    Kp = rospy.get_param("Kp", "No param found")
    P = Kp*error

    # I
    superError += error * dt
    Ki = rospy.get_param("Ki", "NO param found")
    I = superError*Ki

    # D
    Kd = rospy.get_param("Kd", "NO param found")
    D = Kd*((error-prevError)/dt)

    prevError = error

    return (P + I + D)
    

def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.setpoint)

    global setpoint
    setpoint = msg.setpoint

def motor_output_callback(msg):
    global motor_output
    motorOut.output = msg.output
    motorOut.time = msg.time
    motorOut.status = msg.status

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

    #Set the current time
    currentTime = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():
        prevTime = currentTime
        currentTime = rospy.get_time()
        error = setpoint - motorOut.output
        out.input = PID(error)
        out.time = motorOut.time

        rospy.loginfo("Motor output: %s", motorOut.output)
        rospy.loginfo("Error: %s", error)
        rospy.loginfo("Motor input: %s", out)
        
        if(out.input > 1):
            out.input = 1
        elif(out.input < -1):
            out.input = -1

        #Write your code here
        error_pub.publish(error)
        input_pub.publish(out)
        rate.sleep()