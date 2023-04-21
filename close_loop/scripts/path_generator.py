#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from close_loop.msg import goals

#Setup global variables
distance = 0
rotation = 0
commands = []
isPoints = False
robot_angle = 0.0
vectorL = 1
user_finish = 0
type = 0
points = [[0.0,0.0]]
isFinish = 0.0
msg_goals = goals()

def get_inputs():
    # Global variables
    global user_finish
    global user_dist
    global type
    global points
    selection = False   # To know if a selection was made

    print("-- User input --")

    while (selection == False):
        # The user choose square or points
        print("Choose 0 for square or 1 for points: ")
        type = input("Your choice: ")
        type = int(type)

        # If the user wants square
        if type == 0:
            user_dist = input("\nWrite the distance: ") # Get distance from user
            user_dist = float(user_dist) # Set the distance

            # Set the points for the square
            points.append((user_dist, 0.0))
            points.append((user_dist, user_dist))
            points.append((0.0, user_dist))
            points.append((0.0, 0.0))

            selection = True # The user already set a value
        
        # If the user wants points
        elif type == 1:

            # Get the number of points
            num_points = input("\nHow many points do you want to set: ")

            # Ask the user for each point
            for i in range(int(num_points)):
                print("Write the point " + str(i))
                x = input("X coordinate: ")     # Get X coordinate
                y = input("Y coordinate: ")     # Get Y coordinate
                points.append((float(x), float(y))) # Add values to array of points
                
            selection = True    # The user already set a value

        # Validate the value of the parameters
        print("User distance: " + str(user_dist))
        print("Points: ")
        print(points)

        # Ask the user if he/she wants to provide another selection
        print("\n\nHave you finish setting the parameters? 0 for No, 1 for Yes")
        another = input("Your choice: ")

        # If the user wants to provide another selection
        if another == 0:
            selection = False   # Go to the beginning

        # If the user does not want to provide another selection
        elif another == 1:
            user_finish = 1.0
            selection = True

# Make points
def calculate_points():
    # Get global variables
    global isPoints
    global type
    global user_dist
    global points
    global robot_angle

    print("POINTS")
    print(points)

    # --- Follow a set of points ---
    for i in range(len(points)-1):
        # Distance
        distx = points[i+1][0] - points[i][0]
        disty = points[i+1][1] - points[i][1]
        b = np.sqrt(distx*distx+disty*disty)

        robotx = points[i][0]
        roboty = points[i][1]

        newpointx = vectorL*np.cos(robot_angle) + robotx
        newpointy = vectorL*np.sin(robot_angle) + roboty

        print("New point: x(" + str(newpointx) + "), y(" + str(newpointy)+ ")")

        distx2 = newpointx-points[i+1][0]
        disty2 = newpointy-points[i+1][1]
        a = np.sqrt(distx2*distx2 + disty2*disty2)

        print("a: "+str(a))
        c = vectorL
        print("c: " + str(c))
        print("b: " + str(b))

        op = (b*b+c*c-a*a)/(2*b*c)
        angle = np.arccos(op)
        print("angle: "+ str(angle))
        print("robotangle: "+str(robot_angle))

        # Check which side to choose
        testPointX = b*np.cos(robot_angle)*np.cos(angle) - b*vectorL*np.sin(robot_angle)*np.sin(angle) + robotx
        testPointY = b*np.cos(robot_angle)*np.sin(angle) + b*vectorL*np.sin(robot_angle)*np.cos(angle) + roboty

        print("Test point: x(" + str(testPointX) + "), y(" + str(testPointY)+ ")")

        if (testPointX < points[i+1][0]+1 and testPointX > points[i+1][0]-1 )and (testPointY < points[i+1][1]+1 and testPointY > points[i+1][1]-1):
            print("First try :)")      
        else:
            print("Not right one")
            angle = 2*np.pi-angle
            
        commands.append((0.0,((angle)/np.pi)))
        commands.append((b, 0.0))
        print("Sent Angle: " + str(angle/np.pi))
        print("Sent Distance: " + str(b))
        print("")
        robot_angle += angle

        print(commands)

    isPoints = True

# Callback function to get state of the robot
def isFinish_callback(msg):
    global isFinish
    isFinish = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Path Generator is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("path_generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers and Subscribers
    goals_pub = rospy.Publisher("goals", goals, queue_size=1)
    isFinish_pub = rospy.Publisher("isFinish", Float32, queue_size=1)
    rospy.Subscriber("isFinish", Float32, isFinish_callback)

    # Get information from the user
    get_inputs()

    #Run the node
    while not rospy.is_shutdown():
        
        # If we have information from the user
        if user_finish == 1.0 and isPoints == False:
            calculate_points()      # Calculate the points
            isPoints = True         # Flag to calculate only one time
            point = 0               # Manages which point we will focus on  
            rate.sleep()            # Time for all the variables to change

        # If the calculated points exists
        if isPoints:

            # If the robot has not finish
            if isFinish == 0.0:

                # Get the working Distance and Rotation for each command (point)
                distance = commands[point][0]
                rotation = commands[point][1]

                # Create the message
                msg_goals.point = point         # Point we are doing
                msg_goals.distance = distance   # Distance goal
                msg_goals.rotation = rotation   # Rotation goal

                goals_pub.publish(msg_goals)    # Publish the message to topic 'goals'

            # When the robot finish
            else:
                point += 1                      # Do the next point
                isFinish = 0.0                  # Flag that is not finish
                isFinish_pub.publish(isFinish)  # Publish flag of ifFinish

        rate.sleep()