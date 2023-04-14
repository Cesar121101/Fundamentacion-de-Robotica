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
startTime = 0.0
msgRobot = Twist()
r = 0.05
l = 0.18
distance = 0
rotation = 1
startTime = 0.0
isFinished = 0
isFinishedR = 0
# points = [(0.0, 0.0), (2.0,2.0), (0.0,2.0),(0.0,0.0)]
points = [(0.0, 0.0), (2.0,2.0)]

robot_angle = 0.0

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

    #Set initial error
    error = 2

    msgRobot.linear.x = 0
    rate.sleep()
    #Set the current time
    startTime = rospy.get_time()

    # Manages which point we will focus on
    point = 0

    # Calculates distances and angles between all of them
    commands = []

    # --- Hardcoded values to make a square ---
    # commands.append((2.0, 0.0)) # Move 2 m
    # commands.append((0.0, 0.5)) # Turn 90 deg, 360 deg aprox 2, 90 aprox 0.5
    # commands.append((2.0, 0.0)) # ...
    # commands.append((0.0, 0.5))
    # commands.append((2.0, 0.0))
    # commands.append((0.0, 0.5))
    # commands.append((2.0, 0.0))
    # commands.append((0.0, 0.5))

    # --- Follow a set of points ---
    for i in range(len(points)-1):
        # Distance
       x = points[i+1][0] - points[i][0]
       y = points[i+1][1] - points[i][1]
       dist = np.sqrt(x*x+y*y)
       angle = np.arctan(y/x)
       commands.append((0.0, angle))
       commands.append((dist, 0.0))

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

        dt = currentTime-startTime

        d_sim = msgRobot.linear.x*dt
        omega_sim = r*(msgRobot.angular.z/l)*dt
        print("OMEGASIM:" + str(omega_sim))
        error = distance - d_sim
        errorR = rotation - omega_sim
        
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
                robot_angle += rotation

        # Print to terminal the setpoint, the motor output, the error and the motor input which is the real value that gets sent to the PWM
        # rospy.loginfo("Input Value: %s", setpoint.setpoint)
        rospy.loginfo("\n")
        rospy.loginfo("Angle: %s", angle)
        rospy.loginfo("Error: %s", error)
        rospy.loginfo("Linear x: %s", msgRobot.linear.x)
        rospy.loginfo("Initial time: %s", startTime)
        rospy.loginfo("Currrent time: %s", currentTime)
        rospy.loginfo("DT: %s", dt)
        rospy.loginfo("d_sim: %s", d_sim)
    

        # Speed regulation to prevent data over 1 or under -1
        # if out > 1.0:
        #     out = 1.0
        # elif out < -1.0:d_sim
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