#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#Setup parameters, variables and callback functions
out = 0.0
superError = 0.0
currentTime = 0.0
prevTime = 0.0
prevError = 0.0
errorGlob = 0.0
motorOut = 0.0
msgRobot = Twist()
r = 0.05
l = 0.18
distance = 2
startTime = 0.0
isFinished = 0

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

# This function handles the info inside of the 'motor_output' topic
def motor_output_callback(msg):
    global motorOut
    motorOut = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")
    
    #Initialise and Setup node at 100Hz
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    rospy.Subscriber("motor_output", Float32, motor_output_callback)
    input_pub = rospy.Publisher("motor_input", Float32 , queue_size=1)
    error_pub = rospy.Publisher("error", Float32 , queue_size=1) 
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    #Set the current time
    startTime = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():
        prevTime = currentTime
        currentTime = rospy.get_time()

        # error = distance - msgRobot.linear.x
        # out = PID(error)

        dt = currentTime-startTime

        d_sim = msgRobot.linear.x*dt
        omega_sim = r*(msgRobot.angular.z/l)*dt
        error = distance - d_sim

        if error > 0 and isFinished == 0:
            msgRobot.linear.x = 1
        else:
            msgRobot.linear.x = 0
            isFinished = 1

        #msgRobot.linear.x = out
        msgRobot.angular.z = 0

        # Print to terminal the setpoint, the motor output, the error and the motor input which is the real value that gets sent to the PWM
        # rospy.loginfo("Input Value: %s", setpoint.setpoint)
        rospy.loginfo("Motor output: %s", motorOut)
        rospy.loginfo("Error: %s", error)
        rospy.loginfo("Motor input: %s", out)
        rospy.loginfo("Linear x: %s", msgRobot.linear.x)

        # Speed regulation to prevent data over 1 or under -1
        # if out > 1.0:
        #     out = 1.0
        # elif out < -1.0:
        #     out = -1.0

        # Handle friction and minimal output, for when the motor can't overpower friction
        # if out <= 0.05 and out >= -0.05:
        #     out = 0.0

        # Publish error, motor_input, and velocity for gazebo
        # We handle the error as a topic in order to able to plot it
        error_pub.publish(error)
        input_pub.publish(out)
        cmd_vel.publish(msgRobot)
        rate.sleep()