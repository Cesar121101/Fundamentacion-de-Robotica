#!/usr/bin/env python
import rospy
import numpy as np
from challengeFinal.msg import set_point
from std_msgs.msg import Float32

#Setup parameters, vriables and callback functions here (if required)
setpoint = set_point()
#out = motor_input()
out = 0.0
superError = 0.0
currentTime = 0.0
prevTime = 0.0
prevError = 0.0
errorGlob = 0.0
#motorOut = motor_output()
motorOut = 0.0

# PID function 
def PID(error):
    global currentTime
    global prevTime
    global superError
    global prevError

    dt = currentTime-prevTime
    
    # P
    Kp = rospy.get_param("/Kp", "No param found")
    P = Kp*error

    # I
    superError += error * dt
    Ki = rospy.get_param("/Ki", "No param found")
    I = superError*Ki

    # D
    Kd = rospy.get_param("/Kd", "No param found")
    D = Kd*((error-prevError)/dt)

    prevError = error

    return (P + I + D)
    

def set_point_callback(msg):
    rospy.loginfo("Setpoint: %s", msg.setpoint)

    global setpoint
    setpoint.setpoint = msg.setpoint
    setpoint.type = msg.type

def motor_output_callback(msg):
    global motorOut
    motorOut = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")

    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    rospy.Subscriber("set_point", set_point, set_point_callback)
    rospy.Subscriber("motor_output", Float32, motor_output_callback)
    input_pub = rospy.Publisher("motor_input", Float32 , queue_size=1)
    error_pub = rospy.Publisher("error", Float32 , queue_size=1) 

    #Set the current time
    currentTime = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():
        prevTime = currentTime
        currentTime = rospy.get_time()

        error = setpoint.setpoint - motorOut
        out = PID(error)/10

        # Log info about Controller
        if setpoint.type == 1.0:
            rospy.loginfo("Type Input: Step")
        elif setpoint.type == 2.0:
            rospy.loginfo("Type Input: Square")
        else:
            rospy.loginfo("Type Input: Sine")

        rospy.loginfo("Input Value: %s", setpoint.setpoint)
        
        rospy.loginfo("Motor output: %s", motorOut)
        rospy.loginfo("Error: %s", error)
        rospy.loginfo("Motor input: %s", out)

        # Publish error and motor_input
        error_pub.publish(error)
        input_pub.publish(out)
        rate.sleep()