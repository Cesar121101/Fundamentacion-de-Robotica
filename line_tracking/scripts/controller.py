#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#Setup global variables
msgRobot = Twist()
color_light = 2.0
error = 500
linearVelocity = 0.0
angularVelocity = 0.0
currentTime = 0.0
prevTime = 0.0
superError = 0.0
prevError = 0.0

# PID function 
def PID(error):
    global currentTime
    global prevTime
    global superError
    global prevError

    dt = currentTime-prevTime
    
    # P
    P = 0.0012*error

    # I
    superError += error * dt
    I = superError*0

    # D
    D = 0.0001*((error-prevError)/dt)

    prevError = error

    print("PID: " + str(P+I+D))

    return (P + I + D)


# Callback function of motor output
def colors_callback(msg):
    global color_light
    color_light = msg.data

def instruction_callback(msg):
    global error
    error = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("colors", Float32, colors_callback)
    rospy.Subscriber("Instructor", Float32 , instruction_callback)

    #Run the node
    while not rospy.is_shutdown():
        prevTime = currentTime
        currentTime = rospy.get_time()
        #Forward
        if error < 25 and error > -25:
            linearVelocity = 0.15
            angularVelocity = 0.0
        elif error > 499: 
            linearVelocity = 0.0
        else:
            linearVelocity = 0.15 #0.08
            angularVelocity = PID(error)
            # angularVelocity = PID(error)
        
        # Print information
        print("Linear Velocity: " + str(linearVelocity))
        print("Angular Velocity: " + str(angularVelocity))
        print(" ")
        print("Error: " + str(error))

        msgRobot.linear.x = linearVelocity
        msgRobot.angular.z = angularVelocity

        # Publish all the topics
        cmd_vel.publish(msgRobot)
        rate.sleep()