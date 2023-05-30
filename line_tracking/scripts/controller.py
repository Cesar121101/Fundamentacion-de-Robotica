#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#Setup global variables
msgRobot = Twist()
color_light = 2.0
instruction = ""

# Callback function of motor output
def colors_callback(msg):
    global color_light
    color_light = msg.data

def instruction_callback(msg):
    global instruction
    instruction = msg.data

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
    rospy.Subscriber("Instructor", String , instruction_callback)

    #Run the node
    while not rospy.is_shutdown():
            
        if instruction == "forward":
            linearVelocity = 0.1
            angularVelocity = 0.0
        elif instruction == "left":
            linearVelocity = 0.05
            angularVelocity = 0.07
        elif instruction == "right":
            linearVelocity = 0.05
            angularVelocity = -0.07
        else: 
            linearVelocity = 0.0
            angularVelocity = 0.0

        # Print information
        print("Linear Velocity: " + str(linearVelocity))
        print("Angular Velocity: " + str(angularVelocity))
        print(" ")

        msgRobot.linear.x = linearVelocity
        msgRobot.angular.z = angularVelocity

        # Publish all the topics
        cmd_vel.publish(msgRobot)
        rate.sleep()