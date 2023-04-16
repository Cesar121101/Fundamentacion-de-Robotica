#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#Setup global variables
out = 0.0
currentTime = 0.0
startTime = 0.0
msgRobot = Twist()
r = 0.05
l = 0.18
distance = 0
rotation = 0
startTime = 0.0
commands = [(0.0, 0.0)]
isPoints = False
velocity = 0.0
error = 0.0
robot_angle = 0.0
# points = [(0.0, 0.0), (2.0,2.0), (0.0,2.0),(0.0,0.0)]
points = [(0.0, 0.0), (2.0,2.0)]

# Make square points
def calculate_points():
    # Get global variables
    global isPoints
    global velocity

    # If the user defined the distance
    if user_dist > 0.0 and user_time == 0.0:
        # Make a square with the distance of the user
        commands.append((float(user_dist), 0.0)) # Move 
        commands.append((0.0, 0.5))              # Turn 90 deg
        commands.append((float(user_dist), 0.0)) # ...
        commands.append((0.0, 0.5))
        commands.append((float(user_dist), 0.0))
        commands.append((0.0, 0.5))
        commands.append((float(user_dist), 0.0))
        commands.append((0.0, 0.5))
        velocity = 1.0

    # If the user defined the time
    else: 
        # Make a square of 2m
        commands.append((2.0, 0.0)) # Move 2 m
        commands.append((0.0, 0.5)) # Turn 90 deg
        commands.append((2.0, 0.0)) # ...
        commands.append((0.0, 0.5))
        commands.append((2.0, 0.0))
        commands.append((0.0, 0.5))
        commands.append((2.0, 0.0))
        commands.append((0.0, 0.5))

        # Calculate the velocity
        velocity = 8.0/user_time

        # If the velocity exceeds the max velocity
        if velocity > 1:
            velocity = 1

    isPoints = True

# This function handles the info inside of the 'motor_output' topic
def motor_output_callback(msg):
    global motorOut
    motorOut = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")
    
    # Initialise and Setup node at 100Hz
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers
    input_pub = rospy.Publisher("motor_input", Float32 , queue_size=1)
    error_pub = rospy.Publisher("error", Float32 , queue_size=1) 
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # Set the current time
    startTime = rospy.get_time()

    # --- Follow a set of points ---
    # for i in range(len(points)-1):
    #     # Distance
    #    x = points[i+1][0] - points[i][0]
    #    y = points[i+1][1] - points[i][1]
    #    dist = np.sqrt(x*x+y*y)
    #    angle = np.arctan(y/x)
    #    commands.append((0.0, angle))
    #    commands.append((dist, 0.0))

    #Run the node
    while not rospy.is_shutdown():

        # Get global parameters
        user_finish = rospy.get_param("/user_finish", "No param found")
        user_dist = rospy.get_param("/user_dist", "No param found")
        user_time = rospy.get_param("/user_time", "No param found")

        # If the user already define the distance or time
        if user_finish == 1.0 and isPoints == False:
            calculate_points()      # Calculate the points
            isPoints = True         # Flag to calculate only one time
            point = 0               # Manages which point we will focus on
            error = user_dist       # Set initial error
            msgRobot.linear.x = 0   # Set initial velocity
            rate.sleep()            # Time for all the variables to change

        # If the calculated points exists
        if isPoints:
            currentTime = rospy.get_time()  # Obtain the time
            isFinished = False              # Flag of finish traslation
            isFinishedR = False             # Flag of finish rotation

            # Get the working Distance and Rotation for each command (point)
            distance = commands[point][0]
            rotation = commands[point][1]
            # print ("DISTANCE: " + str(distance))
            # print ("ROTATION: " + str(rotation))
            # print ("POINT: " + str(point))

            dt = currentTime-startTime

            d_sim = msgRobot.linear.x*dt
            omega_sim = r*(msgRobot.angular.z/l)*dt
            #print("OMEGASIM:" + str(omega_sim))
            error = distance - d_sim
            errorR = rotation - omega_sim
            
            # Handle rotations first (one must be first to get straight lines, or else we get curves)
            if errorR > 0 and isFinishedR == False:
                msgRobot.angular.z = 0.5
                msgRobot.linear.x = 0
            else:
                msgRobot.linear.x = 0
                msgRobot.angular.z = 0
                isFinishedR = True
                
                # Handle translation
                if error > 0 and isFinished == False and isFinishedR == True:
                    msgRobot.linear.x = velocity
                    msgRobot.angular.z = 0
                else:
                    msgRobot.linear.x = 0
                    msgRobot.angular.z = 0
                    isFinished = True
                    # Restart parameters
                    point += 1
                    startTime = rospy.get_time()
                    isFinished = True
                    robot_angle += rotation

        # Print to terminal
        # rospy.loginfo("\n")
        # rospy.loginfo("Angle: %s", angle)
        # rospy.loginfo("Error: %s", error)
        # rospy.loginfo("Linear x: %s", msgRobot.linear.x)
        # rospy.loginfo("Initial time: %s", startTime)
        # rospy.loginfo("Currrent time: %s", currentTime)
        # rospy.loginfo("DT: %s", dt)
        # rospy.loginfo("d_sim: %s", d_sim)
    

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