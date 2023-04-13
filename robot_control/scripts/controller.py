#!/usr/bin/env python
import rospy
import math
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
distance = 0
rotation = 1
startTime = 0.0
isFinished = 0
isFinishedR = 0
points = [(2.0,2.0), (0.0,2.0),(0.0,0.0)]

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

    # Manages which point we will focus on
    point = 0

    # Calculates distances and angles between all of them
    commands = []
    #commands.append((0.0, 1))
    # Hardcoded values to make a square
    commands.append((2.0, 0.0)) # Move 2 m
    commands.append((0.0, 0.5)) # Turn 90 deg, 360 deg aprox 2, 90 aprox 0.5
    commands.append((2.0, 0.0)) # ...
    commands.append((0.0, 0.5))
    commands.append((2.0, 0.0))
    commands.append((0.0, 0.5))
    commands.append((2.0, 0.0))
    commands.append((0.0, 0.5))
    #commands.append((0.0, 0.5))
    # Future code to create a similar list of points but for given points
    #for i in range(len(points)):
        # Distance
    #    x = points[i+1][0] - points[i][0]
    #    y = points[i+1][1] - points[i][1]
    #    dist = math.sqrt(x*x+y*y)

        # Rotation
        # For square is always 0.5
    #    commands.append((dist, 0.5))

    #Run the node
    while not rospy.is_shutdown():

        #prevTime = currentTime
        currentTime = rospy.get_time()

        # Get the working Distance and Rotation for each command (point)
        distance = commands[point][0]
        rotation = commands[point][1]
        print ("DISTANCE: " + str(distance))
        print ("ROTATION: " + str(rotation))
        print ("POINT: " + str(point))

        # error = distance - msgRobot.linear.x
        # out = PID(error)

        dt = currentTime-startTime

        d_sim = msgRobot.linear.x*dt
        omega_sim = r*(msgRobot.angular.z/l)*dt
        print("OMEGASIM:" + str(omega_sim))
        error = distance - d_sim
        errorR = rotation - omega_sim
        #msgRobot.linear.x = 1
            # Restart parameters
            #point += 1
            #startTime = rospy.get_time()
        
        # Handle rotations first (one must be first to get straight lines, or else we get curves)
        if errorR > 0 and isFinishedR == 0:
            msgRobot.angular.z = 0.5
            msgRobot.linear.x = 0
        else:
            msgRobot.linear.x = 0
            msgRobot.angular.z = 0
            isFinishedR = 1
            
            # Handle translation
            if error > 0 and isFinished == 0:
                msgRobot.linear.x = 1
                msgRobot.angular.z = 0
            else:
                msgRobot.linear.x = 0
                msgRobot.angular.z = 0
                isFinished = 1
                # Restart parameters
                point += 1
                startTime = rospy.get_time()
                isFinished = 0
                isFinishedR = 0

        

        #msgRobot.linear.x = 
        #msgRobot.angular.z = 0

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